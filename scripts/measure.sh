#!/usr/bin/env bash
# Sample GPU memory, RAM and CPU while mapping/localization runs; print peaks at the end.
#   scripts/measure.sh [duration_s=120] [label=run]      (container or host)
# Laptop: nvidia-smi (dedicated VRAM). Jetson: tegrastats (RAM is shared CPU+GPU, so "RAM"
# already includes GPU allocations; use jtop on the host for the per-process view).
# CSV is written to bags/measure_<label>_<date>.csv (container: /bags/...).
set -euo pipefail
dur="${1:-120}"; label="${2:-run}"
outdir=/bags; [[ -d "${outdir}" ]] || outdir="$(cd "$(dirname "$0")/.." && pwd)/bags"
csv="${outdir}/measure_${label}_$(date +%Y%m%d_%H%M%S).csv"
echo "t,cpu_pct,ram_used_mib,gpu_mem_mib,gpu_util_pct,top_procs" > "${csv}"

cpu_sample() { awk '/^cpu /{print $2+$3+$4+$5+$6+$7+$8, $5+$6}' /proc/stat; }
read -r t0 i0 < <(cpu_sample)
jetson=0; [[ "$(uname -m)" == "aarch64" ]] && jetson=1

echo "sampling for ${dur}s -> ${csv}"
for ((s = 0; s < dur; s++)); do
  sleep 1
  read -r t1 i1 < <(cpu_sample)
  cpu=$(awk -v a="$t0" -v b="$t1" -v c="$i0" -v d="$i1" \
    'BEGIN{dt=b-a; di=d-c; printf "%.1f", (dt>0)?100*(dt-di)/dt:0}')
  t0=$t1; i0=$i1
  ram=$(awk '/MemTotal/{t=$2}/MemAvailable/{a=$2}END{printf "%d", (t-a)/1024}' /proc/meminfo)
  if [[ ${jetson} -eq 1 ]]; then
    line="$(timeout 2 tegrastats --interval 500 | head -1 || true)"
    gpu_mem="${ram}"   # unified memory
    gpu_util="$(sed -nE 's/.*GR3D_FREQ ([0-9]+)%.*/\1/p' <<<"${line}")"
  else
    read -r gpu_mem gpu_util < <(nvidia-smi --query-gpu=memory.used,utilization.gpu \
      --format=csv,noheader,nounits | head -1 | tr -d ',')
  fi
  top="$(ps -eo comm,rss --sort=-rss | awk 'NR>1&&NR<=4{printf "%s:%dMiB ", $1, $2/1024}')"
  echo "${s},${cpu},${ram},${gpu_mem},${gpu_util:-},${top}" >> "${csv}"
done

awk -F, 'NR>1{ if($2>c)c=$2; if($3>r)r=$3; if($4>g)g=$4; if($5>u)u=$5; sc+=$2; n++ }
  END{ printf "PEAK  CPU %.1f%% (avg %.1f%%, all cores)  RAM %d MiB  GPU mem %d MiB  GPU util %d%%\n",
       c, sc/n, r, g, u }' "${csv}"
echo "nproc=$(nproc); top processes by RSS now:"; ps -eo pid,comm,%cpu,rss --sort=-rss | head -8
