#!/usr/bin/env python3
"""Table of the tracker test runs (results/run_*.json)."""
import glob, json, os, sys

d = sys.argv[1]
rows = []
for p in sorted(glob.glob(os.path.join(d, 'run_*_a*.json'))):
    r = json.load(open(p))['result']
    tag = os.path.basename(p)[4:-5]
    f = lambda k: '' if r.get(k) is None else f'{r[k]:.2f}'
    rows.append([tag, f('raw_rms'), f('trk_all_rms'), f('trk_max'), f('rate_rms'),
                 f('trk_gap_rms'), f('hold_last_gap_rms'),
                 ' '.join(f'{k}:{v}' for k, v in sorted(r['states'].items()))])
hdr = ['case', 'raw rms', 'track rms', 'track max', 'rate rms', 'gap rms', 'hold-last gap', 'states (msgs)']
print('| ' + ' | '.join(hdr) + ' |')
print('|' + '---|' * len(hdr))
for r in rows:
    print('| ' + ' | '.join(r) + ' |')
