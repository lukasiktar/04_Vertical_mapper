#!/usr/bin/env bash
# Summarize a saved RTAB-Map database (host or container, read-only, no ROS needed).
# USE:
#   scripts/map_info.sh <name>          # maps/<name>.db
#   scripts/map_info.sh --all           # every database in maps/
set -euo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[[ $# -ge 1 ]] || { sed -n '2,5p' "$0"; exit 2; }
if [[ "$1" == "--all" ]]; then
  shopt -s nullglob
  for f in "${WS}"/maps/*.db; do "$0" "$(basename "${f%.db}")"; echo; done
  exit 0
fi
db="${WS}/maps/$1.db"
[[ -f "${db}" ]] || { echo "${db} not found" >&2; exit 1; }
python3 -I - "${db}" <<'EOF2'
import os, sqlite3, sys, datetime
db = sys.argv[1]
c = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
one = lambda q: c.execute(q).fetchone()[0]
nodes = one("select count(*) from Node")
rgbd = one("select count(*) from Data where image is not null and depth is not null")
calib = one("select count(*) from Data where calibration is not null")
links = dict(c.execute("select type, count(*) from Link group by type"))
loops, prox = links.get(1, 0) // 2, links.get(2, 0) // 2
t0, t1 = c.execute("select min(stamp), max(stamp) from Node").fetchone()
mb = os.path.getsize(db) / 1e6
print(f"map: {os.path.basename(db)}  ({mb:.0f} MB)")
if t0:
    print(f"  recorded:        {datetime.datetime.fromtimestamp(t0):%Y-%m-%d %H:%M}, {t1 - t0:.0f} s of mapping")
print(f"  keyframes:       {nodes}  (with RGB-D: {rgbd}, with calibration: {calib})")
print(f"  global loop closures: {loops}   local proximity closures: {prox}")
if nodes < 30:
    print("  NOTE: very small map; walk longer / slower before relying on it")
elif loops == 0:
    print("  NOTE: no loop closure - the walk never returned to a place it had seen; end the walk at the start")
else:
    print("  OK: loops closed; usable for localization")
EOF2
