#pragma once

// Bearing tracker (docs/dev/doa-tracking.md): one per VFO. Follows a MOVING
// emitter seen from a stationary array - a drone at a few hundred metres
// crosses tens of degrees a second - with a constant-angular-velocity Kalman
// filter on the MUSIC peak:
//   state  x = [bearing (deg, unwrapped), rate (deg/s)]
//   model  white angular acceleration, spectral density q (deg^2/s^3)
//   input  each NEW published MUSIC frame (deduplicated by its stamp): peak
//          azimuth + the frame's eigenvalue ratio (signal presence and the
//          measurement noise)
// Frames whose ratio is below the gate carry no signal and are skipped; a
// peak outside the innovation gate is an outlier (multipath, a stray noise
// peak) and is rejected, and REACQUIRE_MISSES rejects in a row re-start the
// track on the new bearing. Between frames (a burst ended, squelch closed,
// calibration hold) the track coasts on its rate, its uncertainty growing,
// and is LOST after coast_ms.
//
// Azimuths are MUSIC's (world frame: array offset applied, counter-clockwise
// from ANT0 / North, as the published pseudospectrum); the page converts to
// its display convention. Times are system_clock ms (getResultStampMs()).
//
// Thread safety: update() runs on the VFO's pipeline thread, snapshot() on
// the uWS loop - every method takes the tracker's mutex.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>

namespace doatrack {

// Agility profile (DOA_TRACK:). OFF = no tracking, nothing published.
enum class Profile : int { OFF = 0, SLOW = 1, NORMAL = 2, FAST = 3 };

// Process noise per profile, deg^2/s^3. sqrt(q * 1 s) is about how much the
// bearing rate may change in a second: SLOW ~3 deg/s (a walker, a car far
// off), NORMAL ~10 deg/s, FAST ~25 deg/s (a drone manoeuvring close in).
inline double profile_q(Profile p) {
    switch (p) {
        case Profile::SLOW:   return 10.0;
        case Profile::NORMAL: return 100.0;
        case Profile::FAST:   return 625.0;
        default:              return 0.0;
    }
}

inline const char* profile_name(Profile p) {
    switch (p) {
        case Profile::SLOW:   return "slow";
        case Profile::NORMAL: return "normal";
        case Profile::FAST:   return "fast";
        default:              return "off";
    }
}

// "off" / "slow" / "normal" / "fast" (or 0..3); false = not one of them
inline bool parse_profile(const char* s, Profile& out) {
    struct { const char* n; Profile p; } const names[] = {
        {"off", Profile::OFF}, {"slow", Profile::SLOW},
        {"normal", Profile::NORMAL}, {"fast", Profile::FAST},
        {"0", Profile::OFF}, {"1", Profile::SLOW},
        {"2", Profile::NORMAL}, {"3", Profile::FAST},
    };
    for (const auto& e : names) {
        const char* a = e.n; const char* b = s;
        while (*a && *b && (*a == *b || *a == (*b | 0x20))) { a++; b++; }
        if (!*a && !*b) { out = e.p; return true; }
    }
    return false;
}

// Shared settings (every VFO's tracker reads them on each update)
inline std::atomic<int>   g_profile{static_cast<int>(Profile::OFF)};
inline std::atomic<float> g_min_ratio{3.0f};   // eigenvalue-ratio gate (linear, ~1 = noise)

constexpr float  MIN_RATIO_LO = 1.2f, MIN_RATIO_HI = 100.0f;
constexpr double GATE_SIGMA = 4.0;            // innovation gate, standard deviations
constexpr double GATE_FLOOR_DEG = 8.0;        // ...but never narrower than this
constexpr int    REACQUIRE_MISSES = 4;        // rejects in a row -> new track
constexpr int    CONFIRM_UPDATES = 3;         // accepted frames to leave ACQUIRING
constexpr double INIT_RATE_SIGMA = 30.0;      // deg/s, a new track's rate uncertainty
constexpr double MAX_RATE = 180.0;            // deg/s, clamp (a target 10 m away at 30 m/s)
constexpr int64_t COAST_AFTER_MS_MIN = 300;   // no frame for this long -> COASTING
constexpr int64_t LOST_AFTER_MS = 5000;       // coasting this long -> LOST (the next frame starts a new track)

// Measurement noise from the eigenvalue ratio: bearing error falls roughly as
// 1/sqrt(SNR), and ratio - 1 tracks the SNR. Clamped to [0.5, 15] deg.
inline double meas_sigma_deg(float ratio) {
    const double snr = std::max(0.05, static_cast<double>(ratio) - 1.0);
    return std::clamp(6.0 / std::sqrt(snr), 0.5, 15.0);
}

inline double wrap180(double d) {
    d = std::fmod(d + 180.0, 360.0);
    if (d < 0) d += 360.0;
    return d - 180.0;
}
inline double wrap360(double d) {
    d = std::fmod(d, 360.0);
    if (d < 0) d += 360.0;
    return d >= 360.0 ? 0.0 : d;
}

enum class State : int { IDLE = 0, ACQUIRING = 1, TRACKING = 2, COASTING = 3, LOST = 4 };

inline const char* state_name(State s) {
    switch (s) {
        case State::ACQUIRING: return "acquiring";
        case State::TRACKING:  return "tracking";
        case State::COASTING:  return "coasting";
        case State::LOST:      return "lost";
        default:               return "idle";
    }
}

struct Snapshot {
    State   state = State::IDLE;
    double  bearing_deg = 0;     // predicted to the snapshot time, [0, 360)
    double  rate_dps = 0;        // deg/s, + = counter-clockwise (MUSIC azimuth)
    double  sigma_deg = 0;       // 1-sigma bearing uncertainty at the snapshot time
    double  meas_deg = 0;        // the last ACCEPTED measurement
    float   ratio = 0;           // its eigenvalue ratio
    int64_t age_ms = 0;          // since the last accepted measurement
    uint32_t updates = 0;        // accepted measurements in this track
    uint32_t rejects = 0;        // outliers rejected in this track
};

class BearingTracker {
public:
    void reset() {
        std::lock_guard<std::mutex> lk(mu_);
        resetLocked();
    }

    // false = this frame was already offered (skip computing its peak)
    bool isNew(int64_t stamp_ms) const {
        std::lock_guard<std::mutex> lk(mu_);
        return stamp_ms != last_stamp_;
    }

    // A published MUSIC frame. stamp_ms dedups (the same frame is offered on
    // every packet until the next one is published). Returns true when the
    // measurement was used.
    bool update(int64_t stamp_ms, float azimuth_deg, float eig_ratio) {
        const Profile prof = static_cast<Profile>(g_profile.load(std::memory_order_relaxed));
        std::lock_guard<std::mutex> lk(mu_);
        if (prof == Profile::OFF) { resetLocked(); return false; }
        if (stamp_ms == last_stamp_) return false;
        last_stamp_ = stamp_ms;
        if (!std::isfinite(azimuth_deg) || azimuth_deg < 0 || !std::isfinite(eig_ratio)) return false;
        if (eig_ratio < g_min_ratio.load(std::memory_order_relaxed)) return false;   // no signal

        // frame spacing (an EMA) sets when a quiet track counts as coasting.
        // Only spacings within a burst count: the gaps between bursts would
        // pull it up until a quiet track never showed as coasting.
        if (last_frame_ms_ > 0 && stamp_ms > last_frame_ms_) {
            const double gap = static_cast<double>(stamp_ms - last_frame_ms_);
            if (frame_ms_ <= 0) frame_ms_ = gap;
            else if (gap <= 2.0 * frame_ms_) frame_ms_ = 0.8 * frame_ms_ + 0.2 * gap;
            else frame_ms_ = std::min(frame_ms_ * 1.05, 1000.0);   // slower frames: adapt slowly
        }
        last_frame_ms_ = stamp_ms;

        const double z = azimuth_deg;
        const double r = meas_sigma_deg(eig_ratio);
        const double R = r * r;

        if (state_ == State::IDLE || state_ == State::LOST ||
            stamp_ms <= t_ms_ || stamp_ms - t_ms_ > LOST_AFTER_MS) {
            startLocked(stamp_ms, z, R, eig_ratio);
            return true;
        }

        // predict to the measurement
        const double dt = static_cast<double>(stamp_ms - t_ms_) * 1e-3;
        double th = th_ + om_ * dt;
        double om = om_;
        const double q = profile_q(prof);
        double P00 = P00_ + dt * (2.0 * P01_ + dt * P11_) + q * dt * dt * dt / 3.0;
        double P01 = P01_ + dt * P11_ + q * dt * dt / 2.0;
        double P11 = P11_ + q * dt;

        // innovation gate
        const double y = wrap180(z - th);
        const double S = P00 + R;
        const double gate = std::max(GATE_FLOOR_DEG, GATE_SIGMA * std::sqrt(S));
        if (std::fabs(y) > gate) {
            rejects_++;
            if (++misses_ >= REACQUIRE_MISSES) {   // it really moved (or a new emitter)
                startLocked(stamp_ms, z, R, eig_ratio);
                return true;
            }
            return false;
        }
        misses_ = 0;

        // update
        const double K0 = P00 / S, K1 = P01 / S;
        th += K0 * y;
        om = std::clamp(om + K1 * y, -MAX_RATE, MAX_RATE);
        const double nP00 = (1.0 - K0) * P00;
        const double nP01 = (1.0 - K0) * P01;
        const double nP11 = P11 - K1 * P01;

        th_ = th; om_ = om;
        P00_ = nP00; P01_ = nP01; P11_ = std::max(nP11, 1e-6);
        t_ms_ = stamp_ms;
        meas_ = z; ratio_ = eig_ratio;
        updates_++;
        if (state_ == State::ACQUIRING && updates_ >= CONFIRM_UPDATES) state_ = State::TRACKING;
        if (state_ == State::COASTING) state_ = State::TRACKING;
        return true;
    }

    // The track predicted to now_ms (system_clock ms)
    Snapshot snapshot(int64_t now_ms) const {
        const Profile prof = static_cast<Profile>(g_profile.load(std::memory_order_relaxed));
        std::lock_guard<std::mutex> lk(mu_);
        Snapshot s;
        if (state_ == State::IDLE || prof == Profile::OFF) return s;
        const int64_t age = std::max<int64_t>(0, now_ms - t_ms_);
        const int64_t coast_after = std::max<int64_t>(
            COAST_AFTER_MS_MIN, static_cast<int64_t>(3.0 * frame_ms_));
        s.state = state_;
        if (age > LOST_AFTER_MS) s.state = State::LOST;
        else if (age > coast_after && state_ == State::TRACKING) s.state = State::COASTING;

        // a lost track stays where it was last seen (no runaway extrapolation)
        const double dt = static_cast<double>(std::min<int64_t>(age, LOST_AFTER_MS)) * 1e-3;
        const double q = profile_q(prof);
        // The prediction trusts the rate only as far as it is known: shrunk by
        // om^2 / (om^2 + P11). A few frames per burst leave the rate noisy, and
        // extrapolating a noisy rate through a 1 s gap did worse than holding
        // the last bearing (measured, docs/dev/doa-tracking.md).
        const double shrink = (om_ * om_) / (om_ * om_ + P11_);
        s.bearing_deg = wrap360(th_ + shrink * om_ * dt);
        s.rate_dps = om_;
        const double P00 = P00_ + dt * (2.0 * P01_ + dt * P11_) + q * dt * dt * dt / 3.0;
        s.sigma_deg = std::sqrt(std::max(0.0, P00));
        s.meas_deg = wrap360(meas_);
        s.ratio = ratio_;
        s.age_ms = age;
        s.updates = updates_;
        s.rejects = rejects_;
        return s;
    }

private:
    void resetLocked() {
        state_ = State::IDLE;
        last_stamp_ = -1; last_frame_ms_ = 0; frame_ms_ = 0;
        misses_ = 0; updates_ = 0; rejects_ = 0;
    }

    void startLocked(int64_t t, double z, double R, float ratio) {
        th_ = z; om_ = 0;
        P00_ = R; P01_ = 0; P11_ = INIT_RATE_SIGMA * INIT_RATE_SIGMA;
        t_ms_ = t; meas_ = z; ratio_ = ratio;
        misses_ = 0; updates_ = 1; rejects_ = 0;
        state_ = State::ACQUIRING;
    }

    mutable std::mutex mu_;
    State   state_ = State::IDLE;
    double  th_ = 0, om_ = 0;                // unwrapped bearing (deg), rate (deg/s)
    double  P00_ = 0, P01_ = 0, P11_ = 0;    // covariance
    int64_t t_ms_ = 0;                       // time of the state
    int64_t last_stamp_ = -1;                // dedup of offered frames
    int64_t last_frame_ms_ = 0;              // last frame with signal
    double  frame_ms_ = 0;                   // EMA of the spacing of those frames
    double  meas_ = 0;
    float   ratio_ = 0;
    int     misses_ = 0;
    uint32_t updates_ = 0, rejects_ = 0;
};

} // namespace doatrack
