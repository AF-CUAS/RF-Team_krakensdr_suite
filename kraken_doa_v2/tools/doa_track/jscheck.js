// Parse every inline <script> of a page (syntax only):
//   ~/kdt/node-v20.18.0-linux-x64/bin/node tools/doa_track/jscheck.js kraken_doa.html
const fs = require('fs'), vm = require('vm');
const html = fs.readFileSync(process.argv[2], 'utf8');
const re = /<script([^>]*)>([\s\S]*?)<\/script>/gi;
let m, n = 0, bad = 0;
while ((m = re.exec(html))) {
  if (/\bsrc=/.test(m[1]) || /type=["'](?!text\/javascript|module)/.test(m[1])) continue;
  n++;
  try { new vm.Script(m[2], {filename: 'block' + n}); }
  catch (e) {
    bad++;
    const line = html.slice(0, m.index).split('\n').length;
    console.log('block', n, 'starting at html line', line, ':', e.message);
  }
}
console.log(n, 'blocks,', bad, 'with errors');
process.exit(bad ? 1 : 0);
