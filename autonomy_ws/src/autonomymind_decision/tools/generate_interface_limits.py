#!/usr/bin/env python3
from pathlib import Path
import argparse

def parse(path):
    out={}
    for ln,line in enumerate(Path(path).read_text().splitlines(),1):
        line=line.split('#',1)[0].strip()
        if not line: continue
        if '=' not in line: raise SystemExit(f"bad config line {ln}")
        k,v=[x.strip() for x in line.split('=',1)]
        if k in out: raise SystemExit(f"duplicate key {k}")
        out[k]=int(v)
    return out

ap=argparse.ArgumentParser();ap.add_argument('config');ap.add_argument('outdir');a=ap.parse_args()
kv=parse(a.config)
keys=['interface.max_waypoints','interface.max_threats','interface.trajectory_points','interface.behavior_trace_nodes','interface.sha256_hex_capacity','interface.profile_name_capacity','interface.explanation_capacity','interface.event_summary_capacity','interface.evaluation_rows']
missing=[k for k in keys if k not in kv]
if missing: raise SystemExit('missing '+','.join(missing))
for k in keys:
    if kv[k] <= 0: raise SystemExit(f'{k} must be > 0')
out=Path(a.outdir);out.mkdir(parents=True,exist_ok=True)
text=(
    '#pragma once\n'
    f'#define FM_MAX_WAYPOINTS {kv["interface.max_waypoints"]}\n'
    f'#define FM_MAX_THREATS {kv["interface.max_threats"]}\n'
    f'#define FM_TRAJECTORY_POINTS {kv["interface.trajectory_points"]}\n'
    f'#define FM_BEHAVIOR_TRACE_NODES {kv["interface.behavior_trace_nodes"]}\n'
    f'#define FM_SHA256_HEX_CAPACITY {kv["interface.sha256_hex_capacity"]}\n'
    f'#define FM_PROFILE_NAME_CAPACITY {kv["interface.profile_name_capacity"]}\n'
    f'#define FM_EXPLANATION_CAPACITY {kv["interface.explanation_capacity"]}\n'
    f'#define FM_EVENT_SUMMARY_CAPACITY {kv["interface.event_summary_capacity"]}\n'
    f'#define FM_EVALUATION_ROWS {kv["interface.evaluation_rows"]}\n'
)
(out/'generated_limits.h').write_text(text)
print('[PASS] generated AutonomyMind interface limits')
