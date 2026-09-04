"""Aggregate trial JSONs into a report.

Deliberately reports median + IQR + min/max rather than a mean, and outcomes as
a categorical distribution. A single number hides exactly the behaviour that
matters: last night the same configuration produced longest-stall values of 5 s,
28 s and 271 s, and a mean of those three describes no run that ever happened.
"""
from __future__ import annotations

import argparse
import json
import statistics as st
from collections import Counter
from pathlib import Path

METRICS = ['time_s', 'gt_path_m', 'detour_ratio', 'mean_speed_mps',
           'final_xy_error_m', 'final_yaw_error_deg']


def load(results_dir: str | Path) -> list[dict]:
    out = []
    for f in sorted(Path(results_dir).glob('*.json')):
        try:
            out.append(json.loads(f.read_text()))
        except json.JSONDecodeError:
            pass
    return out


def summarise(rows: list[dict]) -> dict:
    """Discarded trials are reported separately, never averaged in: a degraded
    simulator yields confident numbers that describe nothing."""
    usable = [r for r in rows if r.get('outcome') not in ('SIM_DEGRADED', 'RUNNER_ERROR')]
    s: dict = {
        'n_total': len(rows),
        'n_discarded': len(rows) - len(usable),
        'outcomes': dict(Counter(r.get('outcome') for r in usable)),
    }
    for k in METRICS:
        vals = [r[k] for r in usable if k in r]
        if not vals:
            continue
        vals.sort()
        # None, not 0.0, when there are too few samples: reporting a zero IQR
        # beside a range of [78.8 .. 120.0] reads as "perfectly repeatable" when
        # it actually means "not enough data to say".
        q = st.quantiles(vals, n=4) if len(vals) > 3 else None
        s[k] = {
            'median': round(st.median(vals), 3),
            'iqr': round(q[2] - q[0], 3) if q else None,
            'min': round(min(vals), 3),
            'max': round(max(vals), 3),
            'n': len(vals),
        }
    return s


def _fmt(s: dict, label: str) -> str:
    lines = [f'  {label}  (n={s["n_total"]}, discarded={s["n_discarded"]})',
             f'    outcomes: {s["outcomes"]}']
    for k in METRICS:
        if k in s:
            v = s[k]
            iqr = 'n/a' if v['iqr'] is None else v['iqr']
            lines.append(f'    {k:20s} median {v["median"]:>8}  IQR {iqr:>7}  '
                         f'[{v["min"]} .. {v["max"]}]  n={v["n"]}')
    return '\n'.join(lines)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='Summarise benchmark trials.')
    ap.add_argument('results_dir')
    ap.add_argument('--group-by', default='sensor_noise',
                    help='field to split the report on (default: %(default)s)')
    a = ap.parse_args(argv)
    rows = load(a.results_dir)
    if not rows:
        print('no results'); return 1
    keys = sorted({str(r.get(a.group_by)) for r in rows})
    print(f'{len(rows)} trials from {a.results_dir}\n')
    for k in keys:
        grp = [r for r in rows if str(r.get(a.group_by)) == k]
        print(_fmt(summarise(grp), f'{a.group_by}={k}'))
        print()
    # the number Phase 5 exists to produce
    if a.group_by == 'sensor_noise' and len(keys) == 2:
        a0 = summarise([r for r in rows if str(r.get('sensor_noise')) == keys[0]])
        a1 = summarise([r for r in rows if str(r.get('sensor_noise')) == keys[1]])
        if ('time_s' in a0 and 'time_s' in a1
                and a0['time_s']['iqr'] is not None and a1['time_s']['iqr'] is not None):
            print('  Interpretation:')
            print(f'    scheduling-only spread (noise={keys[0]}): IQR {a0["time_s"]["iqr"]} s')
            print(f'    with sensor noise      (noise={keys[1]}): IQR {a1["time_s"]["iqr"]} s')
            extra = a1['time_s']['iqr'] - a0['time_s']['iqr']
            print(f'    attributable to sensor noise: {extra:+.3f} s of IQR')
    return 0
