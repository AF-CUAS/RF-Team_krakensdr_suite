"""The tracker's Tuning panel in headless Edge (Windows Python + `pip install --user
websockets`), against kraken_doa_test fed by the fake heimdall:

  wsl -e bash kraken_doa_v2/tools/doa_track/stack.sh pass100 90     (in the background)
  python kraken_doa_v2/tools/doa_track/browser_tune_check.py        (within ~10 s)

Opens the panel with a real click, types q with real key events, applies a bad
and a good pasted line, Defaults, then reloads the page (the saved parameters
must come back from the receiver) and reads them from the receiver's
doa_settings.json. Prints PASS / FAIL per step; screenshots in results/shots/."""
import asyncio, base64, json, os, shutil, subprocess, sys, time, urllib.request
import websockets

SCR = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(SCR, 'results', 'shots')
os.makedirs(OUT, exist_ok=True)
EDGE = r'C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe'
PORT = 9334
SETUP = ['FREQ:915.000', 'DOA:1', 'TOPOLOGY:UCA', 'RADIUS:135', 'MUSIC_COVARIANCE_ALPHA:1.00',
         'SET_DECIMATOR_FREQ:0:100', 'SET_DECIMATOR_BW_HZ:0:25000', 'DOA_TRACK:normal',
         'DOA_TRACK_PARAMS:default']
fails = 0


def check(name, ok, extra=''):
    global fails
    print(('PASS ' if ok else 'FAIL ') + name + (f'  [{extra}]' if extra else ''), flush=True)
    if not ok:
        fails += 1


async def main():
    prof = os.path.join(SCR, 'results', 'edge_profile_tune')
    shutil.rmtree(prof, ignore_errors=True)   # a fresh browser: no remembered panel state
    edge = subprocess.Popen([EDGE, '--headless=new', f'--remote-debugging-port={PORT}', '--ignore-certificate-errors',
                             '--no-proxy-server', '--password-store=basic', '--no-first-run',
                             f'--user-data-dir={prof}', '--window-size=1700,1300', 'about:blank'])
    try:
        for _ in range(50):
            try:
                tabs = json.load(urllib.request.urlopen(f'http://127.0.0.1:{PORT}/json')); break
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
            async def shot(name):
                img = await cdp('Page.captureScreenshot', format='png')
                open(os.path.join(OUT, name), 'wb').write(base64.b64decode(img['data']))
            async def click(sel):
                await js(f"document.querySelector({json.dumps(sel)}).scrollIntoView({{block:'center'}})")
                await asyncio.sleep(0.2)
                r = await js(f"(() => {{ const b = document.querySelector({json.dumps(sel)}).getBoundingClientRect(); return [b.x + b.width / 2, b.y + b.height / 2]; }})()")
                for t in ('mousePressed', 'mouseReleased'):
                    await cdp('Input.dispatchMouseEvent', type=t, x=r[0], y=r[1], button='left', clickCount=1)
            async def key(k, code, vk, mods=0):
                for t in ('rawKeyDown', 'keyUp'):
                    await cdp('Input.dispatchKeyEvent', type=t, key=k, code=code, windowsVirtualKeyCode=vk, modifiers=mods)
            async def type_into(sel, text, commit='Tab'):
                await click(sel)
                await key('a', 'KeyA', 65, 2)          # Ctrl+A
                await cdp('Input.insertText', text=text)
                if commit == 'Tab':
                    await key('Tab', 'Tab', 9)
                elif commit == 'Enter':
                    await key('Enter', 'Enter', 13)
            async def connect():
                for _ in range(60):
                    await asyncio.sleep(0.5)
                    if await js('typeof ws !== "undefined" && ws && ws.readyState === 1'):
                        return True
                return False

            await cdp('Page.enable'); await cdp('Runtime.enable')
            await cdp('Page.navigate', url='https://localhost:28080/')
            check('page connected', await connect())
            await js("window.__errs=[]; window.addEventListener('error', e => window.__errs.push(String(e.message)));")
            for c in SETUP:
                await js(f'send({json.dumps(c)})'); await asyncio.sleep(0.15)
            await asyncio.sleep(3)

            # open the panel with a real click
            await click('#dtp-btn')
            await asyncio.sleep(0.5)
            check('panel opens', await js("dtpOpen() && $id('dtp-btn').textContent === 'Hide'"))
            check('13 parameter rows', await js("document.querySelectorAll('#dtp-rows .ctrl-row').length") == 13)
            check('defaults shown (q=0, gate 4)', await js("$id('dtp-q').value === '0' && $id('dtp-gate_sigma').value === '4'"),
                  await js("JSON.stringify({q: $id('dtp-q').value, g: $id('dtp-gate_sigma').value, focus: document.activeElement && document.activeElement.id, line: $id('dtp-line').value})"))

            # type q = 400 with real keys
            await type_into('#dtp-q', '400')
            await asyncio.sleep(1.0)
            check('typed q reaches the receiver', await js('dtpValues.q') == 400, str(await js('dtpValues.q')))
            check('line shows q=400', 'q=400' in (await js("$id('dtp-line').value") or ''))
            await asyncio.sleep(1.0)
            diag = await js("$id('dtp-diag').textContent")
            check('diagnostics show q 400 from the tracker', 'q 400' in (diag or ''), (diag or '').replace('\n', ' | '))
            await shot('tune_panel_q400.png')

            # a bad pasted line: refused on the page, nothing changes
            await type_into('#dtp-line', 'q=50,gate_flor=10', commit=None)
            await click('#dtp-apply')
            await asyncio.sleep(0.8)
            msg = await js("$id('dtp-msg').textContent")
            check('bad line refused with a message', 'gate_flor' in (msg or '') and await js('dtpValues.q') == 400, msg)

            # a good line
            await type_into('#dtp-line', 'q=50, gate_floor=10, shrink=0', commit=None)
            await click('#dtp-apply')
            await asyncio.sleep(1.0)
            v = await js('JSON.stringify(dtpValues)')
            check('good line applied', await js('dtpValues.q === 50 && dtpValues.gate_floor === 10 && dtpValues.shrink === 0'), v)
            check('shrink checkbox follows', await js("$id('dtp-shrink').checked === false"))

            # copy
            await click('#dtp-copy')
            await asyncio.sleep(0.4)
            check('copy reports', (await js("$id('dtp-msg').textContent") or '') in ('Copied', 'Selected - press Ctrl+C'),
                  await js("$id('dtp-msg').textContent"))

            # set a value to survive a reload, then reload: the receiver replays it
            await type_into('#dtp-meas_k', '4.5')
            await asyncio.sleep(1.0)
            await cdp('Page.reload', ignoreCache=True)
            check('reconnected after reload', await connect())
            await asyncio.sleep(2.5)
            check('panel reopens as left', await js('dtpOpen()'))
            v = await js('JSON.stringify(dtpValues)')
            check('saved values replayed to the new page', await js('dtpValues.q === 50 && dtpValues.meas_k === 4.5'), v)
            check('inputs show them', await js("$id('dtp-q').value === '50' && $id('dtp-meas_k').value === '4.5'"))
            await shot('tune_panel_after_reload.png')

            # defaults
            await click('#dtp-defaults')
            await asyncio.sleep(1.0)
            check('Defaults resets', await js('dtpValues.q === 0 && dtpValues.meas_k === 6 && dtpValues.shrink === 1'),
                  await js('JSON.stringify(dtpValues)'))
            check('no page errors', (await js('JSON.stringify(window.__errs || [])')) == '[]', await js('JSON.stringify(window.__errs)'))
    finally:
        edge.terminate()
    print('ALL PASSED' if not fails else f'{fails} FAILED')


asyncio.run(main())
