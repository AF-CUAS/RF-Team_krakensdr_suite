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
// peak) and is rejected, and `reacquire` rejects in a row re-start the
// track on the new bearing. Between frames (a burst ended, squelch closed,
// calibration hold) the track coasts on its rate, its uncertainty growing,
// and is LOST after `lost_ms`.
//
// EVERY tuning value is a runtime parameter (PARAM_SPECS below; the
// DOA_TRACK_PARAMS: command, persisted, the page's Tuning panel) so the
// filter can be tuned in the field without a rebuild. The page's DTP_SPECS
// mirrors PARAM_SPECS (labels, help, steps): keep the keys and ranges alike.
//
// Azimuths are MUSIC's (world frame: array offset applied, counter-clockwise
// from ANT0 / North, as the published pseudospectrum); the page converts to
// its display convention. Times are system_clock ms (getResultStampMs()).
//
// Thread safety: update() runs on the VFO's pipeline thread, snapshot() on
// the uWS loop - every method takes the tracker's mutex. The parameters are
// copied under their own mutex first (never held together).

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <string>
#include <string_view>

namespace doatrack {

// Agility profile (DOA_TRACK:). OFF = no tracking, nothing published.
enum class Profile : int { OFF = 0, SLOW = 1, NORMAL = 2, FAST = 3 };

// Process noise per profile, deg^2/s^3. sqrt(q * 1 s) is about how much the
// bearing rate may change in a second: SLOW ~3 deg/s (a walker, a car far
// off), NORMAL ~10 deg/s, FAST ~25 deg/s. The `q` parameter overrides it.
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
constexpr float MIN_RATIO_LO = 1.2f, MIN_RATIO_HI = 100.0f;

// ---- The tuning parameters (DOA_TRACK_PARAMS:key=value,...) ----
enum Param : int {
    P_Q,            // process noise, deg^2/s^3; 0 = the profile's
    P_GATE_SIGMA,   // innovation gate, standard deviations
    P_GATE_FLOOR,   // ...but never narrower than this, deg
    P_REACQUIRE,    // rejects in a row -> a new track at the new bearing
    P_CONFIRM,      // accepted frames to leave ACQUIRING
    P_INIT_RATE,    // a new track's rate uncertainty (1 sigma), deg/s
    P_MAX_RATE,     // rate clamp, deg/s
    P_COAST_MS,     // no frame for max(this, 3 x frame spacing) -> COASTING
    P_LOST_MS,      // no frame for this long -> LOST; the next frame starts a new track
    P_MEAS_K,       // measurement sigma = meas_k / sqrt(ratio - 1), deg ...
    P_MEAS_MIN,     // ...clamped to [meas_min, meas_max]
    P_MEAS_MAX,
    P_SHRINK,       // 1: coasting trusts the rate only as far as it is known; 0: plain prediction
    P_COUNT
};

struct ParamSpec { const char* key; double lo, hi, def; int decimals; };
inline constexpr ParamSpec PARAM_SPECS[P_COUNT] = {
    {"q",          0.0,  100000.0, 0.0,    1},
    {"gate_sigma", 1.0,  20.0,     4.0,    1},
    {"gate_floor", 0.0,  90.0,     8.0,    1},
    {"reacquire",  1.0,  50.0,     4.0,    0},
    {"confirm",    1.0,  50.0,     3.0,    0},
    {"init_rate",  1.0,  360.0,    30.0,   1},
    {"max_rate",   1.0,  720.0,    180.0,  1},
    {"coast_ms",   50.0, 10000.0,  300.0,  0},
    {"lost_ms",    100.0, 120000.0, 5000.0, 0},
    {"meas_k",     0.1,  100.0,    6.0,    2},
    {"meas_min",   0.01, 90.0,     0.5,    2},
    {"meas_max",   0.1,  90.0,     15.0,   2},
    {"shrink",     0.0,  1.0,      1.0,    0},
};

using Params = std::array<double, P_COUNT>;

inline Params default_params() {
    Params p{};
    for (int i = 0; i < P_COUNT; i++) p[i] = PARAM_SPECS[i].def;
    return p;
}

inline std::mutex g_params_mu;
inline Params g_params = default_params();

inline Params params() {
    std::lock_guard<std::mutex> lk(g_params_mu);
    return g_params;
}

// Clamp to the ranges, round the integer ones, keep meas_min <= meas_max
inline void normalise(Params& p) {
    for (int i = 0; i < P_COUNT; i++) {
        double v = std::clamp(p[i], PARAM_SPECS[i].lo, PARAM_SPECS[i].hi);
        if (PARAM_SPECS[i].decimals == 0) v = std::round(v);
        p[i] = v;
    }
    if (p[P_MEAS_MAX] < p[P_MEAS_MIN]) p[P_MEAS_MAX] = p[P_MEAS_MIN];
}

// "key=value,key=value" (any subset, any order; spaces allowed) or "default"
// / "defaults". All or nothing: an unknown key or a value that isn't a number
// changes nothing and returns false with *err set. "" = no change (the
// settings replay of a never-saved value).
inline bool apply_params(std::string_view s, std::string* err) {
    auto trim = [](std::string_view v) {
        while (!v.empty() && (v.front() == ' ' || v.front() == '\t')) v.remove_prefix(1);
        while (!v.empty() && (v.back() == ' ' || v.back() == '\t' || v.back() == '\r' || v.back() == '\n')) v.remove_suffix(1);
        return v;
    };
    s = trim(s);
    Params p = params();
    if (s == "default" || s == "defaults") {
        p = default_params();
    } else {
        while (!s.empty()) {
            const size_t comma = s.find(',');
            std::string_view item = trim(s.substr(0, comma));
            s = comma == std::string_view::npos ? std::string_view() : s.substr(comma + 1);
            if (item.empty()) continue;
            const size_t eq = item.find('=');
            if (eq == std::string_view::npos) { if (err) *err = "expected key=value: " + std::string(item); return false; }
            const std::string_view key = trim(item.substr(0, eq));
            const std::string val(trim(item.substr(eq + 1)));
            int idx = -1;
            for (int i = 0; i < P_COUNT; i++) if (key == PARAM_SPECS[i].key) { idx = i; break; }
            if (idx < 0) { if (err) *err = "unknown tracker parameter: " + std::string(key); return false; }
            char* end = nullptr;
            const double v = std::strtod(val.c_str(), &end);
            if (val.empty() || end == val.c_str() || *end != '\0' || !std::isfinite(v)) {
                if (err) *err = "not a number: " + std::string(key) + "=" + val;
                return false;
            }
            p[idx] = v;
        }
    }
    normalise(p);
    std::lock_guard<std::mutex> lk(g_params_mu);
    g_params = p;
    return true;
}

// Every parameter, "q=0.0,gate_sigma=4.0,..." (the applied / saved form)
inline std::string params_string(const Params& p) {
    std::string out;
    char buf[64];
    for (int i = 0; i < P_COUNT; i++) {
        std::snprintf(buf, sizeof buf, "%s%s=%.*f", i ? "," : "", PARAM_SPECS[i].key,
                      PARAM_SPECS[i].decimals, p[i]);
        out += buf;
    }
    return out;
}
inline std::string params_string() { return params_string(params()); }

// Measurement noise from the eigenvalue ratio: bearing error falls roughly as
// 1/sqrt(SNR), and ratio - 1 tracks the SNR.
inline double meas_sigma_deg(float ratio, const Params& p) {
    const double snr = std::max(0.05, static_cast<double>(ratio) - 1.0);
    return std::clamp(p[P_MEAS_K] / std::sqrt(snr), p[P_MEAS_MIN], p[P_MEAS_MAX]);
}

inline double effective_q(Profile prof, const Params& p) {
    return p[P_Q] > 0 ? p[P_Q] : profile_q(prof);
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
    // tuning diagnostics
    double  innov_deg = 0;       // the last innovation (measurement - prediction), deg
    double  nis = 0;             // normalised innovation squared, an average: ~1 = consistent
    double  frame_ms = 0;        // the frame spacing the coast threshold uses
    double  q = 0;               // the process noise in use
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
        const Params p = params();
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
        const double r = meas_sigma_deg(eig_ratio, p);
        const double R = r * r;
        const int64_t lost_ms = static_cast<int64_t>(p[P_LOST_MS]);

        if (state_ == State::IDLE || state_ == State::LOST ||
            stamp_ms <= t_ms_ || stamp_ms - t_ms_ > lost_ms) {
            startLocked(stamp_ms, z, R, eig_ratio, p);
            return true;
        }

        // predict to the measurement
        const double dt = static_cast<double>(stamp_ms - t_ms_) * 1e-3;
        double th = th_ + om_ * dt;
        double om = om_;
        const double q = effective_q(prof, p);
        double P00 = P00_ + dt * (2.0 * P01_ + dt * P11_) + q * dt * dt * dt / 3.0;
        double P01 = P01_ + dt * P11_ + q * dt * dt / 2.0;
        double P11 = P11_ + q * dt;

        // innovation gate
        const double y = wrap180(z - th);
        const double S = P00 + R;
        innov_ = y;
        const double gate = std::max(p[P_GATE_FLOOR], p[P_GATE_SIGMA] * std::sqrt(S));
        if (std::fabs(y) > gate) {
            rejects_++;
            if (++misses_ >= static_cast<int>(p[P_REACQUIRE])) {   // it really moved (or a new emitter)
                startLocked(stamp_ms, z, R, eig_ratio, p);
                return true;
            }
            return false;
        }
        misses_ = 0;
        const double nis1 = y * y / S;
        nis_ = nis_n_ == 0 ? nis1 : 0.95 * nis_ + 0.05 * nis1;
        nis_n_++;

        // update
        const double K0 = P00 / S, K1 = P01 / S;
        th += K0 * y;
        om = std::clamp(om + K1 * y, -p[P_MAX_RATE], p[P_MAX_RATE]);
        const double nP00 = (1.0 - K0) * P00;
        const double nP01 = (1.0 - K0) * P01;
        const double nP11 = P11 - K1 * P01;

        th_ = th; om_ = om;
        P00_ = nP00; P01_ = nP01; P11_ = std::max(nP11, 1e-6);
        t_ms_ = stamp_ms;
        meas_ = z; ratio_ = eig_ratio;
        updates_++;
        if (state_ == State::ACQUIRING && updates_ >= static_cast<uint32_t>(p[P_CONFIRM])) state_ = State::TRACKING;
        if (state_ == State::COASTING) state_ = State::TRACKING;
        return true;
    }

    // The track predicted to now_ms (system_clock ms)
    Snapshot snapshot(int64_t now_ms) const {
        const Profile prof = static_cast<Profile>(g_profile.load(std::memory_order_relaxed));
        const Params p = params();
        std::lock_guard<std::mutex> lk(mu_);
        Snapshot s;
        if (state_ == State::IDLE || prof == Profile::OFF) return s;
        const int64_t lost_ms = static_cast<int64_t>(p[P_LOST_MS]);
        const int64_t age = std::max<int64_t>(0, now_ms - t_ms_);
        const int64_t coast_after = std::max<int64_t>(
            static_cast<int64_t>(p[P_COAST_MS]), static_cast<int64_t>(3.0 * frame_ms_));
        s.state = state_;
        if (age > lost_ms) s.state = State::LOST;
        else if (age > coast_after && state_ == State::TRACKING) s.state = State::COASTING;

        // a lost track stays where it was last seen (no runaway extrapolation)
        const double dt = static_cast<double>(std::min<int64_t>(age, lost_ms)) * 1e-3;
        const double q = effective_q(prof, p);
        // The prediction trusts the rate only as far as it is known: shrunk by
        // om^2 / (om^2 + P11). A few frames per burst leave the rate noisy, and
        // extrapolating a noisy rate through a 1 s gap did worse than holding
        // the last bearing (measured, docs/dev/doa-tracking.md). shrink=0: off.
        const double shrink = p[P_SHRINK] >= 0.5 ? (om_ * om_) / (om_ * om_ + P11_) : 1.0;
        s.bearing_deg = wrap360(th_ + shrink * om_ * dt);
        s.rate_dps = om_;
        const double P00 = P00_ + dt * (2.0 * P01_ + dt * P11_) + q * dt * dt * dt / 3.0;
        s.sigma_deg = std::sqrt(std::max(0.0, P00));
        s.meas_deg = wrap360(meas_);
        s.ratio = ratio_;
        s.age_ms = age;
        s.updates = updates_;
        s.rejects = rejects_;
        s.innov_deg = innov_;
        s.nis = nis_;
        s.frame_ms = frame_ms_;
        s.q = q;
        return s;
    }

private:
    void resetLocked() {
        state_ = State::IDLE;
        last_stamp_ = -1; last_frame_ms_ = 0; frame_ms_ = 0;
        misses_ = 0; updates_ = 0; rejects_ = 0;
        innov_ = 0; nis_ = 0; nis_n_ = 0;
    }

    void startLocked(int64_t t, double z, double R, float ratio, const Params& p) {
        th_ = z; om_ = 0;
        P00_ = R; P01_ = 0; P11_ = p[P_INIT_RATE] * p[P_INIT_RATE];
        t_ms_ = t; meas_ = z; ratio_ = ratio;
        misses_ = 0; updates_ = 1; rejects_ = 0;
        innov_ = 0; nis_ = 0; nis_n_ = 0;
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
    double  innov_ = 0, nis_ = 0;            // diagnostics (Snapshot)
    uint32_t nis_n_ = 0;
};

} // namespace doatrack
