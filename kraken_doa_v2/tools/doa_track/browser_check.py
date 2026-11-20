"""Headless Edge over the DevTools protocol (Windows Python, `pip install --user
websockets`): the page against kraken_doa_test, the receiver set up through
the page's own send(), the Both tab with the map on the station, screenshots
through the pass + the page's track state printed per shot.

Start `wsl -e bash kraken_doa_v2/tools/doa_track/stack.sh bursty30 25` first
(in the background), then within ~10 s: `python browser_check.py`.
Screenshots: results/shots/. Read them - they are the check of the drawing."""
import asyncio, base64, json, os, subprocess, sys, time, urllib.request
import websockets

SCR = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(SCR, 'results', 'shots')   # results/ is gitignored
os.makedirs(OUT, exist_ok=True)
EDGE = r'C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe'
PORT = 9333
TRUTH = os.path.join(SCR, 'results', 'truth_browser.json')

SETUP = ['FREQ:915.000', 'DOA:1', 'TOPOLOGY:UCA', 'RADIUS:140', 'ARRAY_OFFSET:0',
         'MUSIC_COVARIANCE_ALPHA:1.00', 'SET_DECIMATOR_FREQ:0:100', 'SET_DECIMATOR_BW_HZ:0:25000',
         'DOA_TRACK_GATE:3.0', 'DOA_TRACK:normal',
         'LOCATION_SOURCE:static', 'STATIC_LOCATION:39.000000,-104.800000,0']


async def main():
    prof = os.path.join(SCR, 'results', 'edge_profile')
    edge = subprocess.Popen([EDGE, '--headless=new', f'--remote-debugging-port={PORT}', '--ignore-certificate-errors',
                             '--no-proxy-server', '--password-store=basic', '--no-first-run',
                             f'--user-data-dir={prof}', '--window-size=1700,1000', 'about:blank'])
    try:
        for _ in range(50):
            try:
                tabs = json.load(urllib.request.urlopen(f'http://127.0.0.1:{PORT}/json'))
                break
            except Exception:
                time.sleep(0.2)
        page = next(t for t in tabs if t['type'] == 'page')
        async with websockets.connect(page['webSocketDebuggerUrl'], max_size=None) as ws:
            n = [0]
            async def cdp(method, **params):
                n[0] += 1; my = n[0]
                await ws.send(json.dumps({'id': my, 'method': method, 'params': params}))
                while True:
                    m = json.loads(await ws.recv())
                    if m.get('id') == my:
                        return m.get('result', m.get('error'))
            async def js(expr):
                r = await cdp('Runtime.evaluate', expression=expr, returnByValue=True, awaitPromise=True)
                return r.get('result', {}).get('value') if isinstance(r, dict) else r
            await cdp('Page.enable'); await cdp('Runtime.enable')
            await cdp('Page.navigate', url='https://localhost:28080/')
            for _ in range(60):
                await asyncio.sleep(0.5)
                if await js('typeof ws !== "undefined" && ws && ws.readyState === 1'):
                    break
            print('page connected:', await js('ws.readyState'))
            errs = await js('JSON.stringify(window.__errs||[])')
            await js("window.__errs=[]; window.addEventListener('error', e => window.__errs.push(String(e.message)));")
            for c in SETUP:
                await js(f'send({json.dumps(c)})')
                await asyncio.sleep(0.15)
            await asyncio.sleep(2)
            await js("rpShow('both')")
            await asyncio.sleep(1)
            await js("mapCenterStation(); mapCenterOn(39.0, -104.8, 17)")
            # wait for the flight
            while not os.path.exists(TRUTH):
                await asyncio.sleep(0.5)
            truth = json.load(open(TRUTH))
            t0 = truth['flight_start_wall']
            shots = []
            for k, tf in enumerate([-3, 1.5, 3.0, 4.5, 5.2, 6.0, 7.0, 7.9, 9.5, 11.0]):
                while time.time() < t0 + tf:
                    await asyncio.sleep(0.05)
                st = await js("JSON.stringify({prof: doaTrackProfile, track: doaTracks[0] || null, hist: (doaTrackHist[0]||[]).length,"
                              " layer: mapState.rdfTrack, station: rdfStatus && rdfStatus.station, card: ($id('dtk-0')||{}).textContent,"
                              " cardShown: ($id('dtr-0')||{style:{}}).style.display !== 'none', errs: window.__errs})")
                img = await cdp('Page.captureScreenshot', format='png')
                fn = os.path.join(OUT, f'shot_{k:02d}_t{tf:+.1f}.png')
                open(fn, 'wb').write(base64.b64decode(img['data']))
                s = json.loads(st)
                tr = s['track'] or {}
                print(f"t={tf:+5.1f}s  state={tr.get('state')}  az={tr.get('az')}  rate={tr.get('rate')}  sigma={tr.get('sigma')}"
                      f"  hist={s['hist']}  card={s['card']!r} shown={s['cardShown']}  layer={s['layer']}  station={s['station'] and {k2: s['station'].get(k2) for k2 in ('lat','lon','hdg','src')}}"
                      f"  errs={s['errs']}  -> {os.path.basename(fn)}")
                shots.append(s)
            # the layer off -> nothing drawn; the row is in the Layers popover
            await js("mapSet('rdfTrack', false)")
            await asyncio.sleep(0.4)
            img = await cdp('Page.captureScreenshot', format='png')
            open(os.path.join(OUT, 'shot_layer_off.png'), 'wb').write(base64.b64decode(img['data']))
            row = await js("(() => { const e = document.querySelector('input[data-ml=\"rdfTrack\"]'); return e ? 'row ok, checked=' + e.checked : 'no row (popover not rendered yet)'; })()")
            print('layer off:', row)
            await js("mapLayOpen()")
            await asyncio.sleep(0.4)
            row = await js("(() => { const e = document.querySelector('input[data-ml=\"rdfTrack\"]'); return e ? 'row ok, checked=' + e.checked : 'no row'; })()")
            print('layers popover:', row)
            img = await cdp('Page.captureScreenshot', format='png')
            open(os.path.join(OUT, 'shot_layers_popover.png'), 'wb').write(base64.b64decode(img['data']))
            print('errors on the page:', await js('JSON.stringify(window.__errs)'))
    finally:
        edge.terminate()


asyncio.run(main())
