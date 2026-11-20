#!/usr/bin/env python3
"""Fake heimdall for the bearing-tracker test: a 5-channel coherent KrakenSDR
stream (8091 protocol, real-time paced at 2.4 MS/s) with ONE emitter - a
carrier at RF_HZ - flying a straight pass past the array, and a minimal 8092
control port (set_frequency honoured; every command answered).

UCA as kraken_doa expects it (plugins/sdk/tools/uca_scene.py): ANT0 on +x,
the others CLOCKWISE, phase_k = 2 pi r / lambda cos(b - phi_k), phi_k = -2 pi k / M;
b = the bearing as the MUSIC DoA plot shows it (CCW from ANT0).

  fake_heimdall.py --scene NAME --truth truth.json [--port-base 28090]

The emitter sits at its start bearing for --warmup s (DoA settles), then
flies; truth.json gets the wall-clock time the flight starts, for the client.
"""
import argparse, json, math, socket, struct, threading, time
import numpy as np

FS = 2.4e6
M = 5
NS = 16384                     # samples per channel per packet
MAGIC = 0x4D434851
C = 299792458.0

SCENES = {
    # d0: closest approach (m), v: m/s, L: half track length (m); bursts: (on_s, off_s) or None
    'pass100':      dict(d0=100.0, v=20.0, L=300.0, bursts=None),
    'pass30':       dict(d0=30.0,  v=30.0, L=150.0, bursts=None),
    'bursty100':    dict(d0=100.0, v=20.0, L=300.0, bursts=(0.2, 1.0)),
    'bursty30':     dict(d0=30.0,  v=20.0, L=150.0, bursts=(0.2, 1.0)),
}


def truth_bearing(sc, t_flight):
    """Bearing (deg, CCW from ANT0) at t_flight s after the flight starts (< 0: start point)."""
    x = -sc['L'] + sc['v'] * max(0.0, t_flight)
    return math.degrees(math.atan2(sc['d0'], x)) % 360.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scene', required=True, choices=sorted(SCENES))
    ap.add_argument('--truth', required=True)
    ap.add_argument('--port-base', type=int, default=28090)
    ap.add_argument('--rf', type=float, default=915.1e6)
    ap.add_argument('--radius', type=float, default=0.14)
    ap.add_argument('--warmup', type=float, default=20.0)
    ap.add_argument('--amp', type=float, default=20.0, help='carrier amplitude, LSB of 127.5')
    ap.add_argument('--noise', type=float, default=8.0, help='noise std per I/Q, LSB')
    a = ap.parse_args()
    sc = SCENES[a.scene]
    flight_s = 2 * sc['L'] / sc['v']

    state = {'center': 915.0e6, 'fcc': 0}
    lock = threading.Lock()

    # ---- 8092 control
    def control_client(conn):
        buf = b''
        conn.settimeout(0.5)
        last_status = 0
        try:
            while True:
                now = time.time()
                if now - last_status > 0.5:
                    last_status = now
                    conn.sendall(b'{"settings":{"frequency":%d},"version":"fake-heimdall"}\n' % int(state['center']))
                try:
                    d = conn.recv(65536)
                except socket.timeout:
                    continue
                if not d:
                    return
                buf += d
                while True:   # every complete top-level {...}
                    s = buf.find(b'{')
                    if s < 0: buf = b''; break
                    depth, e = 0, -1
                    for i in range(s, len(buf)):
                        if buf[i:i+1] == b'{': depth += 1
                        elif buf[i:i+1] == b'}':
                            depth -= 1
                            if depth == 0: e = i; break
                    if e < 0: buf = buf[s:]; break
                    obj = buf[s:e+1].decode(errors='replace'); buf = buf[e+1:]
                    reply = {'status': 'success'}
                    if '"set_frequency"' in obj:
                        try:
                            j = json.loads(obj)
                            f = j.get('frequency') or j.get('set_frequency', {}).get('frequency')
                            if f:
                                with lock:
                                    if abs(float(f) - state['center']) > 1:
                                        state['center'] = float(f); state['fcc'] += 1
                                reply['frequency'] = int(f)
                        except Exception as ex:
                            reply = {'status': 'error', 'message': str(ex)}
                    conn.sendall((json.dumps(reply) + '\n').encode())
        except OSError:
            return
        finally:
            conn.close()

    def control_server():
        srv = socket.socket(); srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(('127.0.0.1', a.port_base + 2)); srv.listen(4)
        while True:
            c, _ = srv.accept()
            threading.Thread(target=control_client, args=(c,), daemon=True).start()
    threading.Thread(target=control_server, daemon=True).start()

    # ---- 8091 data
    srv = socket.socket(); srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('127.0.0.1', a.port_base + 1)); srv.listen(1)
    print(f'fake heimdall: scene {a.scene} ({flight_s:.0f} s flight after {a.warmup:.0f} s warm-up), '
          f'waiting on {a.port_base + 1}', flush=True)
    conn, _ = srv.accept()
    conn.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4 << 20)

    rng = np.random.default_rng(1)
    POOL = 48
    noise_pool = [(rng.normal(0, a.noise, (M, NS)) + 1j * rng.normal(0, a.noise, (M, NS))).astype(np.complex64)
                  for _ in range(POOL)]
    phi = -2 * np.pi * np.arange(M) / M
    lam = C / a.rf
    n_idx = np.arange(NS)

    t_start = time.time() + 0.05
    t_flight0 = t_start + a.warmup
    json.dump({'scene': a.scene, **sc, 'flight_start_wall': t_flight0, 'flight_s': flight_s,
               'warmup': a.warmup, 'rf': a.rf, 'radius': a.radius}, open(a.truth, 'w'))
    sent = 0
    pkt = 0
    end_wall = t_flight0 + flight_s + 3.0
    while True:
        due = t_start + sent / FS
        now = time.time()
        if due > now:
            time.sleep(due - now)
        if due > end_wall:
            break
        ts = sent / FS                       # stream time of this packet's first sample
        wall = t_start + ts
        tf = wall - t_flight0
        b = math.radians(truth_bearing(sc, tf))
        on = True
        if sc['bursts'] and tf >= 0:
            on_s, off_s = sc['bursts']
            on = (tf % (on_s + off_s)) < on_s
        with lock:
            center, fcc = state['center'], state['fcc']
        x = noise_pool[pkt % POOL].copy()
        f_off = a.rf - center
        if on and abs(f_off) < FS / 2:
            steer = np.exp(1j * 2 * np.pi * a.radius / lam * np.cos(b - phi)).astype(np.complex64)
            tone = (a.amp * np.exp(1j * (2 * np.pi * f_off * (sent + n_idx) / FS))).astype(np.complex64)
            x += steer[:, None] * tone[None, :]
        iq = np.empty((M, NS, 2), np.float32)
        iq[:, :, 0] = x.real; iq[:, :, 1] = x.imag
        u8 = np.clip(np.rint(iq + 127.5), 0, 255).astype(np.uint8)
        hdr = struct.pack('>8I', MAGIC, M, NS, 4, 0, fcc, 0, 0)
        hdr += b''.join(struct.pack('<ff', center, 30.0) for _ in range(M))
        try:
            conn.sendall(hdr + u8.tobytes())
        except OSError:
            print('fake heimdall: client went away', flush=True)
            return
        sent += NS
        pkt += 1
        if pkt % 1460 == 0:
            lag = time.time() - (t_start + sent / FS)
            print(f'  t={ts:6.1f}s  truth={math.degrees(b):6.1f}  {"on " if on else "off"}  behind real time {lag*1000:.0f} ms',
                  flush=True)
    print('fake heimdall: scene done', flush=True)
    conn.close()


if __name__ == '__main__':
    main()
