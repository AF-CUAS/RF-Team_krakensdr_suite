# Bearing tracker (moving emitters)

Added 2026-11-20 for a drone transmitting on 915.1 MHz, seen from a
stationary KrakenSDR: continuous for now, bursty later. *Italic* names are
other topics in this folder.

## Status and how to resume

Last worked on 2026-11-20. Done: the tracker (receiver), `DOA_TRACK:` /
`DOA_TRACK_GATE:`, `{"doa_track"}` at 10 Hz, the DoA plot's track and
the VFO card's Track line, the map's Track layer, these docs, and the
tests below (offline, the real pipeline against a fake heimdall, a
headless-browser check). Committed to main as 82a08c6 (2026-11-20):
`include/signal_processing/bearing_tracker.hpp` (new),
`decimator_manager.hpp`, `message_builders.hpp/.cpp`, `control_handler.cpp`,
`settings_store.cpp`, `networking/data_receiver.cpp`,
`networking/websocket_server.cpp`, `kraken_doa.html`, `tools/doa_track/`
(new), this file and the CLAUDE.md / README.md index rows. Not done:
a Windows (MinGW) syntax check (no MinGW on the machine it was written
on), real hardware, really weak signals, multipath, several emitters.

Next steps (none started):
- a real-hardware test: the drone at 915.1 MHz, Tracking Normal, Averaging
  off, a UCA radius a little under 140 mm (the page warns at 140 mm: 0.502
  lambda spacing at 915 MHz), the VFO as wide as the signal
- the track in the DoA logger, the web mapper, the 8081 page, drive mode
  (they still use the raw MUSIC peak)
- elevation for close / high drones (UCA steering assumes 0 deg)
- coasting at a close pass lags up to ~20-30 deg through a 1 s gap: a
  better coast model (e.g. the geometry of a straight pass) if bursts
  stay that sparse

Tests to re-run after a change (`tools/doa_track/`; Windows dev machine,
WSL Ubuntu, no sudo needed):
1. `wsl -e bash kraken_doa_v2/tools/doa_track/wsl_setup.sh` once (deps
   into ~/kdt, a copy of kraken_doa_v2 with the ports moved to 2808x /
   2809x, built as ~/kdt/run/kraken_doa_test); `--build-only` after
   C++ changes. From Git Bash prefix `MSYS_NO_PATHCONV=1`, or the
   /mnt/c path is rewritten
2. offline tracker: `tracker_test.cpp` (the g++ line at its top)
3. pipeline matrix: `run_tests.sh pass100:normal pass30:normal
   bursty100:normal bursty30:normal ...` (about a minute a case; table
   from `summarize.py`, compare with the table below)
4. page: `jscheck.js` (parse), then `stack.sh bursty30 25` in the
   background and `python browser_check.py` (Windows Python + Edge) -
   read the screenshots in `tools/doa_track/results/shots/`
Never run git from WSL in the Windows checkout (no autocrlf there: it sees
every file as changed, and a stash / checkout rewrites them as LF).

## What it does

One `doatrack::BearingTracker` per VFO
(`include/signal_processing/bearing_tracker.hpp`, header-only), a
constant-angular-velocity Kalman filter on the MUSIC peak:

- state: bearing (deg, unwrapped) and bearing rate (deg/s); white angular
  acceleration with spectral density q per profile (`DOA_TRACK:`):
  slow 10, normal 100, fast 625 deg^2/s^3 (sqrt(q x 1 s) = about how much
  the rate may change in a second: ~3 / 10 / 25 deg/s)
- input: each NEW published MUSIC frame (deduplicated by
  `getResultStampMs()`), its peak (`getPeakAngleWithConfidence()`) and
  eigenvalue ratio (`getEigenvalueRatio()`). Frames below the gate
  (`DOA_TRACK_GATE:`, linear ratio, default 3; ~1 = noise) carry no signal
  and are skipped. Measurement sigma = clamp(6 / sqrt(ratio - 1), 0.5, 15)
  deg (bearing error ~ 1/sqrt(SNR); a heuristic, not measured on hardware)
- outliers: an innovation beyond max(8 deg, 4 sigma) is rejected;
  `REACQUIRE_MISSES` (4) rejects in a row restart the track at the new
  bearing (the emitter really jumped, or another one took over)
- states: acquiring (until 3 accepted frames), tracking, coasting (no
  frame for max(300 ms, 3 x the in-burst frame spacing)), lost (no frame
  for 5 s; held at the last bearing, the next frame starts a new track)
- coasting predicts on the rate, shrunk by w^2 / (w^2 + P_ww): extrapolating
  a noisy rate (4 frames per 200 ms burst) through a 1 s gap did worse than
  holding the last bearing; with the shrink it does better (below)
- a retune or VFO move (`isResultStale()`) and any `DOA_TRACK:` change
  reset the tracks. Wideband / independent modes: no MUSIC frames, so the
  tracks go lost

Fed from the VFO pipeline (`update_bearing_tracker` in
`src/networking/data_receiver.cpp`, after MUSIC and the talker DoA - so
the Digital squelch's confirmed-samples publish is tracked too). The
tracker locks itself; the pipeline thread updates, the uWS loop reads.

## Protocol

- `DOA_TRACK:off|slow|normal|fast` (persisted `doa_track`, replayed;
  default off - nothing changes for existing setups)
- `DOA_TRACK_GATE:<1.2..100>` (persisted `doa_track_gate`, replayed)
- `{"doa_track":{"profile","gate","tracks":[{"id","state","az","rate",
  "sigma","meas","ratio","age_ms","updates","rejects"}]}}` on the DoA
  topic at 10 Hz (its own timer in `websocket_server.cpp`), predicted to
  the send time; nothing while the tracker or DoA is off. `az`, `meas` and
  `rate` are MUSIC azimuth (counter-clockwise, array offset applied); the
  page turns them to its convention (compass: rate sign flipped). A new
  key: older pages ignore it; the binary type 3 DoA message is unchanged.

## Page

MUSIC box: Tracking (select) and Track gate (slider). Each VFO's DoA plot
draws the track in amber (`--warn` #e3a842): a +-2 sigma wedge (capped at
45 deg) and a line to the rim, dashed while coasting, grey (`--t5`) once
lost; a track not refreshed for 1.5 s is not drawn. The VFO card gets a
Track line (bearing, rate; the title has state, sigma, age, counts).

Map (the DF section's drawing, `rdfDrawTracks`): layer **Track**
(`mapState.rdfTrack`, Layers popover under Direction finding, on by
default) - per VFO selected in the DF card, from the station: the tracked
COMPASS bearing (`doaToCompass(az, station heading)`, like the lobes; no
heading -> nothing drawn) as an amber line to the map edge with a dark
casing, a +-2 sigma wedge out to the lobe radius, dashed while coasting,
grey once lost, a label (`D0 294deg +33.4deg/s (coasting)`, the rate in
compass sense) and the last 10 s as fading lines (`doaTrackHist`, a
bearing every 0.5 s). Not with a talker selected in the DF card. The
tracks are kept in `doaTracks` (globals block) by `dtOnMessage`, which
also calls `mapDraw()` (rAF-coalesced, nothing while the map is hidden).

Not done yet: the DoA logger, the web mapper, the 8081 page and drive mode
still use the raw MUSIC peak.

## Measurements (simulation only)

`tracker_test.cpp` (job scratch, not committed): a straight pass, frames
every 50 ms, 3 deg Gaussian bearing noise, ratio 10. RMS bearing error
after the first 2 s:

| Scenario | raw MUSIC | slow | normal | fast | hold last |
|---|---|---|---|---|---|
| 100 m pass, 20 m/s, continuous, 5 % random-peak outliers | 24.9 | 1.36 | 1.73 | 2.05 | |
| 30 m pass, 30 m/s (peak 57 deg/s), continuous, 5 % outliers | 21.2 | 2.47 | | 1.75 | |
| 100 m, 20 m/s, 200 ms on / 1 s off: coasting error | | | 4.05 | | 4.62 |
| 30 m, 20 m/s, 200 ms on / 1 s off: coasting error | | | 3.89 | 6.01 | 7.49 |

So: fast for a close continuous signal, normal for bursts (the rate needs
several bursts to settle, and fast lets it wander between them).

## Measurements (the real pipeline, fake heimdall)

2026-11-20: the TEST build (ports 2808x / 2809x, `kraken_doa_test`) fed
by a fake heimdall (job scratch `fake_heimdall.py`: 5 channels, 2.4 MS/s
real-time paced, 16384 samples a packet, a carrier at 915.1 MHz on a UCA r
= 140 mm, the uca_scene.py steering; noise 8 LSB rms per I / Q, carrier 20
LSB - about 25 dB in the 25 kHz VFO) and a WebSocket client
(`track_client.py`) that sets the receiver up, records `{"doa_track"}`
(10 Hz) and the type 3 DoA frames (5 Hz, peak as the page computes it), and
scores both against the flight's truth (by receive time, after the first
2 s of the flight). Squelch off. RMS / max error in deg, rate error in deg/s:

| Case | raw MUSIC | track | track max | rate |
|---|---|---|---|---|
| 100 m pass, 20 m/s, continuous, slow | 0.83 | 0.44 | 0.89 | 0.40 |
| same, normal | 0.85 | 0.51 | 1.07 | 0.56 |
| same, fast | 0.79 | 0.59 | 1.31 | 1.03 |
| 30 m pass, 30 m/s (57 deg/s), normal | 3.45 | 1.80 | 4.65 | 5.63 |
| same, fast | 3.56 | 1.81 | 4.28 | 4.72 |
| 100 m, 20 m/s, 200 ms on / 1 s off, normal | 90.2 | 1.81 | 6.15 | 1.20 |
| same, fast | 94.7 | 2.62 | 8.20 | 1.65 |
| 30 m, 20 m/s, 200 ms on / 1 s off, normal | 97.6 | 4.74 | 19.2 | 6.27 |
| same, fast | 97.7 | 5.17 | 19.0 | 6.36 |

- Bursty, squelch off: between bursts MUSIC keeps publishing frames of
  noise (random bearings - the raw column); the ratio gate keeps them out
  of the track. The track's worst error is coasting through a 1 s gap at
  the closest approach of a 30 m pass (~20 deg; one browser-check sample
  30 deg).
- Fast did not beat normal on the close continuous pass here (the
  simulation said it would); normal is the better default, fast only a
  slightly better rate.
- A carrier of 6 LSB (-10 dB) scored the same as 20: still ~15 dB in the
  25 kHz VFO, so it was not a weak-signal test. Not tested: really weak
  signals, multipath, several emitters, real hardware.
- Browser check (headless Edge over CDP, the bursty 30 m pass, static
  station): DoA plot and map track, the card line, coasting / tracking
  states, the Track layer row and switch; no page errors.

## Limits for a drone

- Elevation: a UCA's steering vectors assume 0 deg elevation; a close,
  high drone shrinks the effective aperture by cos(elevation), widening
  the peak and adding error. Only custom 3D arrays scan elevation.
- Narrow VFOs: below a few kHz a MUSIC frame can take up to 1 s
  (`MAX_FRAME_SECONDS`), which smears a fast mover; keep the VFO wide
  enough for the signal.
- Covariance Averaging lags a moving bearing: keep it off with tracking.
- Frequency hopping (many 902-928 MHz links): one VFO only sees the hops
  that land in it.
