#!/usr/bin/env python3
"""Test client for the bearing tracker: sets kraken_doa up like a user would
(915.0 MHz centre, VFO 0 at +100 kHz / 25 kHz, UCA r = 140 mm, DoA on,
averaging off, DOA_TRACK:<profile>), records every {"doa_track"} message and
every raw MUSIC frame (binary type 3, peak as the page computes it), then
scores both against the fake heimdall's truth.

  track_client.py --truth truth.json --profile fast --out run.json [--port 28080]

Run by run_tests.sh. The raw score is the page's view (the latest frame by
receive time); between bursts with squelch off it is noise. hold_last_gap_rms
(holding the last in-burst raw frame) is rough: raw frames come at 5 Hz and
the burst edges are judged by receive time, so it can pick a noise frame.
"""
import argparse, asyncio, json, math, ssl, struct, time
import numpy as np
import websockets

import importlib.util, os
spec = importlib.util.spec_from_file_location('fh', os.path.join(os.path.dirname(__file__), 'fake_heimdall.py'))
fh = importlib.util.module_from_spec(spec); spec.loader.exec_module(fh)


def wrap180(d):
    return (d + 180.0) % 360.0 - 180.0


def page_peak(spec_v, res):
    """PolarPlot.updateData's peak: argmax + 3-point parabolic fit on the log spectrum."""
    n = len(spec_v); i = int(np.argmax(spec_v)); mx = spec_v[i]
    delta = 0.0
    pl, pr = spec_v[(i - 1) % n], spec_v[(i + 1) % n]
    if pl > 0 and pr > 0 and mx > 0:
        y1, y2, y3 = math.log(pl), math.log(mx), math.log(pr)
        den = y1 - 2 * y2 + y3
        if den < -1e-12:
            delta = max(-0.5, min(0.5, 0.5 * (y1 - y3) / den))
    return ((i + delta) * res + 360.0) % 360.0


async def run(a):
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT); ctx.check_hostname = False; ctx.verify_mode = ssl.CERT_NONE
    tracks, raw, sync = [], [], []
    async with websockets.connect(f'wss://localhost:{a.port}/', ssl=ctx, max_size=None) as ws:
        for cmd in ['FREQ:915.000', 'DOA:1', 'TOPOLOGY:UCA', 'RADIUS:140', 'ARRAY_OFFSET:0',
                    'MUSIC_COVARIANCE_ALPHA:1.00', 'MUSIC_SIGNAL_SOURCES:1',
                    'SET_DECIMATOR_FREQ:0:100', 'SET_DECIMATOR_BW_HZ:0:25000',
                    f'DOA_TRACK_GATE:{a.gate}', f'DOA_TRACK:{a.profile}', 'GET_DECIMATOR_INFO']:
            await ws.send(cmd)
        truth = None
        t_end = None
        while True:
            if truth is None and os.path.exists(a.truth):
                try:
                    truth = json.load(open(a.truth))
                    t_end = truth['flight_start_wall'] + truth['flight_s'] + 1.0
                except Exception:
                    truth = None
            if t_end and time.time() > t_end:
                break
            try:
                m = await asyncio.wait_for(ws.recv(), 1.0)
            except asyncio.TimeoutError:
                continue
            now = time.time()
            if isinstance(m, str):
                if '"doa_track"' in m:
                    tracks.append({'t': now, **json.loads(m)['doa_track']})
                elif 'DOA_TRACK' in m:
                    sync.append(m[:200])
                continue
            v = memoryview(m)
            if len(v) < 8 or struct.unpack_from('<I', v, 0)[0] != 3:
                continue
            nd = struct.unpack_from('<I', v, 4)[0]; o = 8
            for _ in range(nd):
                did, foff, bwi, na = struct.unpack_from('<IfII', v, o); o += 16
                rad, blocks, res = struct.unpack_from('<fIf', v, o); o += 12
                sp = np.frombuffer(v[o:o + 4 * na], dtype='<f4'); o += 4 * na
                is3d = struct.unpack_from('<I', v, o)[0]; o += 4
                if is3d:
                    ne = struct.unpack_from('<I', v, o)[0]; o += 12 + 4 * ne
                if did == 0 and na > 0:
                    raw.append({'t': now, 'az': page_peak(sp, res), 'blocks': blocks})
    return truth, tracks, raw, sync


def score(truth, tracks, raw):
    sc = {k: truth[k] for k in ('d0', 'v', 'L')}
    bursts = truth.get('bursts')
    t0 = truth['flight_start_wall']
    def tb(t): return fh.truth_bearing(sc, t - t0)
    def is_on(t):
        tf = t - t0
        if not bursts or tf < 0: return True
        return (tf % (bursts[0] + bursts[1])) < bursts[0]
    out = {}
    # raw MUSIC: what the page shows today (it holds the last frame between bursts)
    re_ = [wrap180(r['az'] - tb(r['t'])) for r in raw if t0 + 2 <= r['t'] <= t0 + truth['flight_s']]
    out['raw_rms'] = float(np.sqrt(np.mean(np.square(re_)))) if re_ else None
    out['raw_n'] = len(re_)
    by = {}
    for m in tracks:
        if not (t0 + 2 <= m['t'] <= t0 + truth['flight_s']): continue
        for tr in m['tracks']:
            if tr['id'] != 0: continue
            e = wrap180(tr['az'] - tb(m['t']))
            key = tr['state'] if is_on(m['t']) or not bursts else tr['state'] + '_gap'
            by.setdefault(key, []).append(e)
            by.setdefault('all', []).append(e)
            if bursts and not is_on(m['t']):
                by.setdefault('gap', []).append(e)
    for k, es in by.items():
        out[f'trk_{k}_rms'] = float(np.sqrt(np.mean(np.square(es))))
        out[f'trk_{k}_n'] = len(es)
    out['trk_max'] = float(max(abs(e) for e in by.get('all', [0])))
    # holding the last raw bearing through the gaps (today's display) vs the track's prediction
    if bursts:
        he = []
        for m in tracks:
            if not (t0 + 2 <= m['t'] <= t0 + truth['flight_s']) or is_on(m['t']): continue
            prev = [r for r in raw if r['t'] <= m['t']]
            # the last raw frame that was taken during a burst
            prev_on = [r for r in prev if is_on(r['t'] - 0.05)]
            if prev_on:
                he.append(wrap180(prev_on[-1]['az'] - tb(m['t'])))
        out['hold_last_gap_rms'] = float(np.sqrt(np.mean(np.square(he)))) if he else None
    # the track's estimated rate against the true rate (centre of the pass)
    rate_err = []
    for m in tracks:
        tf = m['t'] - t0
        if not (2 <= tf <= truth['flight_s'] - 2): continue
        for tr in m['tracks']:
            if tr['id'] == 0 and tr['state'] == 'tracking':
                true_rate = wrap180(tb(m['t'] + 0.05) - tb(m['t'] - 0.05)) / 0.1
                rate_err.append(tr['rate'] - true_rate)
    out['rate_rms'] = float(np.sqrt(np.mean(np.square(rate_err)))) if rate_err else None
    states = {}
    for m in tracks:
        for tr in m['tracks']:
            if tr['id'] == 0 and t0 + 2 <= m['t'] <= t0 + truth['flight_s']:
                states[tr['state']] = states.get(tr['state'], 0) + 1
    out['states'] = states
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--truth', required=True)
    ap.add_argument('--profile', default='fast')
    ap.add_argument('--gate', default='3.0')
    ap.add_argument('--out', required=True)
    ap.add_argument('--port', type=int, default=28080)
    a = ap.parse_args()
    truth, tracks, raw, sync = asyncio.run(run(a))
    res = score(truth, tracks, raw)
    res.update(scene=truth['scene'], profile=a.profile, n_track_msgs=len(tracks), n_raw=len(raw), sync=sync[:3])
    json.dump({'result': res, 'truth': truth, 'tracks': tracks, 'raw': raw}, open(a.out, 'w'))
    print(json.dumps(res, indent=1))


if __name__ == '__main__':
    main()
