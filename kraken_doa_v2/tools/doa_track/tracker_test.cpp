// Offline check of doatrack::BearingTracker: a drone passing a stationary array
// (simulated bearings, no pipeline). From kraken_doa_v2:
//   g++ -std=c++20 -O2 -Wall -Wextra -Iinclude tools/doa_track/tracker_test.cpp -o /tmp/tracker_test && /tmp/tracker_test
#include "signal_processing/bearing_tracker.hpp"
#include <cstdio>
#include <random>
#include <vector>

using namespace doatrack;

struct Result { double raw_rms, trk_rms, coast_rms, max_err; int lost_flags; double hold_rms; };

// Drone on a straight line: closest approach d0 (m), speed v (m/s).
// Frames every frame_ms; bursts: on_ms on / off_ms off (off_ms 0 = continuous).
// noise_deg: MUSIC bearing noise; outlier_p: chance a frame's peak is random.
static Result run(Profile prof, double d0, double v, int frame_ms, int on_ms, int off_ms,
                  double noise_deg, double outlier_p, float ratio, unsigned seed, bool verbose = false) {
    g_profile = static_cast<int>(prof);
    g_min_ratio = 3.0f;
    BearingTracker t;
    std::mt19937 rng(seed);
    std::normal_distribution<double> n(0, noise_deg);
    std::uniform_real_distribution<double> u(0, 1);
    const double T = 2.0 * 300.0 / v;              // from -300 m to +300 m along track
    double se_raw = 0, se_trk = 0, se_coast = 0, se_hold = 0, maxe = 0, last_z = -1; int nr = 0, nt = 0, nc = 0, lost = 0;
    const int64_t t0 = 1'000'000;
    auto truth = [&](double ts) {
        const double x = -300.0 + v * ts;
        return wrap360(std::atan2(d0, x) * 180.0 / M_PI);   // azimuth, CCW
    };
    for (int64_t ms = 0; ms <= static_cast<int64_t>(T * 1000); ms += frame_ms) {
        const double ts = ms * 1e-3;
        const bool on = off_ms == 0 || (ms % (on_ms + off_ms)) < on_ms;
        const double tr = truth(ts);
        if (on) {
            double z = u(rng) < outlier_p ? u(rng) * 360.0 : wrap360(tr + n(rng));
            t.update(t0 + ms, static_cast<float>(z), ratio);
            const double er = wrap180(z - tr); se_raw += er * er; nr++; last_z = z;
        } else {
            t.update(t0 + ms, static_cast<float>(u(rng) * 360.0), 1.1f);   // noise frame, below the gate
        }
        Snapshot s = t.snapshot(t0 + ms);
        if (ms > 2000 && s.state != State::IDLE) {
            const double e = wrap180(s.bearing_deg - tr);
            if (s.state == State::COASTING) { se_coast += e * e; nc++; const double eh = wrap180(last_z - tr); se_hold += eh * eh; }
            else { se_trk += e * e; nt++; }
            if (std::fabs(e) > maxe) maxe = std::fabs(e);
            if (s.state == State::LOST) lost++;
            if (verbose && ms % 1000 == 0)
                std::printf("  t=%5.1fs truth=%6.1f trk=%6.1f rate=%6.1f sig=%4.1f %s age=%lld\n",
                            ts, tr, s.bearing_deg, s.rate_dps, s.sigma_deg, state_name(s.state),
                            static_cast<long long>(s.age_ms));
        }
    }
    return {std::sqrt(se_raw / std::max(1, nr)), std::sqrt(se_trk / std::max(1, nt)),
            nc ? std::sqrt(se_coast / nc) : 0.0, maxe, lost, nc ? std::sqrt(se_hold / nc) : 0.0};
}

int main() {
    int fails = 0;
    auto check = [&](const char* name, bool ok) { std::printf("%s %s\n", ok ? "PASS" : "FAIL", name); if (!ok) fails++; };

    // 1. continuous, 100 m pass at 20 m/s (peak rate 11.5 deg/s), 3 deg noise, 5 % outliers
    for (Profile p : {Profile::SLOW, Profile::NORMAL, Profile::FAST}) {
        Result r = run(p, 100, 20, 50, 0, 0, 3.0, 0.05, 10.0f, 1);
        std::printf("continuous 100m 20m/s %-6s raw %.2f  trk %.2f  max %.1f\n",
                    profile_name(p), r.raw_rms, r.trk_rms, r.max_err);
    }
    Result c = run(Profile::NORMAL, 100, 20, 50, 0, 0, 3.0, 0.05, 10.0f, 1);
    check("continuous: tracked error well below raw (outliers rejected)", c.trk_rms < 0.5 * c.raw_rms);
    check("continuous: never lost", c.lost_flags == 0);

    // 2. close fast pass: 30 m at 30 m/s (peak 57 deg/s)
    Result f = run(Profile::FAST, 30, 30, 50, 0, 0, 3.0, 0.05, 10.0f, 2, true);
    std::printf("fast 30m 30m/s fast: raw %.2f trk %.2f max %.1f\n", f.raw_rms, f.trk_rms, f.max_err);
    Result fs = run(Profile::SLOW, 30, 30, 50, 0, 0, 3.0, 0.05, 10.0f, 2);
    std::printf("fast 30m 30m/s slow: raw %.2f trk %.2f max %.1f\n", fs.raw_rms, fs.trk_rms, fs.max_err);
    check("close pass: FAST profile keeps up (rms < 3 deg)", f.trk_rms < 3.0);
    check("close pass: FAST beats SLOW", f.trk_rms < fs.trk_rms);

    // 3. bursty: 200 ms on / 1000 ms off, 100 m at 20 m/s
    Result b = run(Profile::NORMAL, 100, 20, 50, 200, 1000, 3.0, 0.0, 10.0f, 3, true);
    std::printf("bursty 200/1000: raw %.2f trk %.2f coast %.2f (hold-last %.2f) max %.1f\n", b.raw_rms, b.trk_rms, b.coast_rms, b.hold_rms, b.max_err);
    check("bursty: coasting reported in the gaps", b.coast_rms > 0);
    // 30 m crossing at 20 m/s, bursty: rates up to 38 deg/s
    for (Profile p : {Profile::NORMAL, Profile::FAST}) {
        Result bc = run(p, 30, 20, 50, 200, 1000, 3.0, 0.0, 10.0f, 4);
        std::printf("bursty close 30m %-6s: trk %.2f coast %.2f (hold-last %.2f) max %.1f lost %d\n",
                    profile_name(p), bc.trk_rms, bc.coast_rms, bc.hold_rms, bc.max_err, bc.lost_flags);
        if (p == Profile::FAST) check("bursty close pass: coasting beats holding the last bearing", bc.coast_rms < bc.hold_rms);
    }
    check("bursty: never lost on 1 s gaps", b.lost_flags == 0);

    // 4. long gap -> LOST, then a new bearing re-acquires
    {
        g_profile = static_cast<int>(Profile::NORMAL);
        BearingTracker t;
        for (int i = 0; i < 20; i++) t.update(1000 + i * 50, 90.0f, 20.0f);
        Snapshot s = t.snapshot(1000 + 19 * 50 + 6000);
        check("gap > 5 s reports LOST", s.state == State::LOST);
        check("LOST holds the last bearing", std::fabs(wrap180(s.bearing_deg - 90.0)) < 1.0);
        t.update(1000 + 19 * 50 + 6000, 200.0f, 20.0f);
        s = t.snapshot(1000 + 19 * 50 + 6000);
        check("re-acquires a new bearing after LOST", s.state == State::ACQUIRING && std::fabs(wrap180(s.bearing_deg - 200.0)) < 0.5);
    }
    // 5. jump: emitter switched -> re-acquired after REACQUIRE_MISSES
    {
        BearingTracker t;
        int64_t ms = 0;
        for (int i = 0; i < 20; i++, ms += 50) t.update(ms + 1, 10.0f, 20.0f);
        for (int i = 0; i < REACQUIRE_MISSES; i++, ms += 50) t.update(ms + 1, 180.0f, 20.0f);
        Snapshot s = t.snapshot(ms);
        check("jump to a new bearing re-acquires", std::fabs(wrap180(s.bearing_deg - 180.0)) < 2.0);
    }
    // 6. wrap-around: crossing 0/360
    {
        BearingTracker t;
        double az = 340;
        for (int i = 0; i < 100; i++, az += 0.5) t.update(i * 50 + 1, static_cast<float>(wrap360(az)), 20.0f);
        Snapshot s = t.snapshot(99 * 50 + 1);
        check("wraps through 0 deg", std::fabs(wrap180(s.bearing_deg - wrap360(az - 0.5))) < 1.0 && std::fabs(s.rate_dps - 10.0) < 1.5);
    }
    // 7. dedup, gate, off
    {
        BearingTracker t;
        t.update(5, 50.0f, 20.0f);
        check("same stamp offered twice is used once", !t.update(5, 51.0f, 20.0f));
        check("below the ratio gate is ignored", !t.update(6, 51.0f, 2.0f));
        g_profile = static_cast<int>(Profile::OFF);
        check("OFF: no snapshot", t.snapshot(10).state == State::IDLE);
        check("OFF: no update", !t.update(7, 51.0f, 20.0f));
    }
    Profile p;
    check("parse_profile", parse_profile("FAST", p) && p == Profile::FAST && parse_profile("off", p) && !parse_profile("quick", p));

    std::printf("%s\n", fails ? "SOME FAILED" : "ALL PASSED");
    return fails ? 1 : 0;
}
