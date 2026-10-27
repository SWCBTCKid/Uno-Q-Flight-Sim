#!/bin/sh
# SPIKE-002 1 Hz system/DRM sampler (unprivileged, read-only /proc and /sys).
# Usage: spike002_sampler.sh APP_PID [STREAM_HELPER_PID]
# Per line: monotonic uptime, the PID's distinct DRM clients' resident/total
# memory (fdinfo, KiB), VmRSS/VmSwap, MemFree/MemAvailable/Cached/Shmem/
# SwapFree, swap and reclaim counters, GPU devfreq and CPU clocks, the
# hottest thermal zone and loadavg.
pid=${1:?usage: spike002_sampler.sh PID}
helper=${2:-}
case $pid in *[!0-9]*|'') echo "bad pid" >&2; exit 2;; esac
case $helper in *[!0-9]*|'') [ -z "$helper" ] || { echo "bad helper pid" >&2; exit 2; };; esac
gpu=/sys/class/devfreq/5900000.gpu/cur_freq
cg=$(awk -F: '$1=="0" {print $3}' /proc/$pid/cgroup 2>/dev/null)
while [ -d "/proc/$pid" ]; do
  up=$(cut -d' ' -f1 /proc/uptime)
  drm=$(for f in /proc/$pid/fdinfo/*; do echo "FILE $f"; cat "$f" 2>/dev/null; done | awk '
    function save() { if (id != "") {res[id]=r; tot[id]=t; pur[id]=p; eng[id]=e; cyc[id]=c} r=t=p=e=c=0; id="" }
    $1=="FILE" {save(); next}
    /^drm-client-id:/ {id=$2}
    /^drm-resident-memory:/ {r=$2}
    /^drm-total-memory:/ {t=$2}
    /^drm-purgeable-memory:/ {p=$2}
    /^drm-engine-/ {e+=$2}
    /^drm-cycles-/ {c+=$2}
    END {save(); R=0; T=0; P=0; E=0; C=0; n=0; for (k in res) {n++; R+=res[k]; T+=tot[k]; P+=pur[k]; E+=eng[k]; C+=cyc[k]} printf "\"drm_clients\":%d,\"drm_resident_kib\":%d,\"drm_total_kib\":%d,\"drm_purgeable_kib\":%d,\"drm_engine_ns\":%.0f,\"drm_cycles\":%.0f", n, R, T, P, E, C}')
  st=$(awk '/^VmRSS:/ {r=$2} /^VmSwap:/ {s=$2} END {printf "\"rss_kib\":%d,\"vmswap_kib\":%d", r, s}' /proc/$pid/status 2>/dev/null)
  appcpu=$(awk '{printf "\"app_utime_ticks\":%s,\"app_stime_ticks\":%s", $14, $15}' /proc/$pid/stat 2>/dev/null)
  [ -n "$appcpu" ] || appcpu='"app_utime_ticks":0,"app_stime_ticks":0'
  if [ -n "$helper" ] && [ -d "/proc/$helper" ]; then
    helperst=$(awk '/^VmRSS:/ {r=$2} /^VmSwap:/ {s=$2} END {printf "\"helper_alive\":true,\"helper_rss_kib\":%d,\"helper_vmswap_kib\":%d", r, s}' /proc/$helper/status 2>/dev/null)
    helpercpu=$(awk '{printf "\"helper_utime_ticks\":%s,\"helper_stime_ticks\":%s", $14, $15}' /proc/$helper/stat 2>/dev/null)
    [ -n "$helperst" ] || helperst='"helper_alive":false,"helper_rss_kib":0,"helper_vmswap_kib":0'
    [ -n "$helpercpu" ] || helpercpu='"helper_utime_ticks":0,"helper_stime_ticks":0'
  else
    helperst='"helper_alive":false,"helper_rss_kib":0,"helper_vmswap_kib":0'
    helpercpu='"helper_utime_ticks":0,"helper_stime_ticks":0'
  fi
  cgcpu=$(awk '$1=="usage_usec"{u=$2} $1=="user_usec"{r=$2} $1=="system_usec"{s=$2} $1=="nr_throttled"{n=$2} $1=="throttled_usec"{t=$2} END{printf "\"cgroup_usage_usec\":%d,\"cgroup_user_usec\":%d,\"cgroup_system_usec\":%d,\"cgroup_nr_throttled\":%d,\"cgroup_throttled_usec\":%d",u,r,s,n,t}' "/sys/fs/cgroup$cg/cpu.stat" 2>/dev/null)
  [ -n "$cgcpu" ] || cgcpu='"cgroup_usage_usec":0,"cgroup_user_usec":0,"cgroup_system_usec":0,"cgroup_nr_throttled":0,"cgroup_throttled_usec":0'
  mem=$(awk '/^MemFree:/ {f=$2} /^MemAvailable:/ {a=$2} /^Cached:/ {c=$2} /^Shmem:/ {s=$2} /^SwapFree:/ {w=$2} END {printf "\"memfree_kib\":%d,\"memavail_kib\":%d,\"cached_kib\":%d,\"shmem_kib\":%d,\"swapfree_kib\":%d", f, a, c, s, w}' /proc/meminfo)
  vm=$(awk '$1=="pswpin"||$1=="pswpout"||$1=="oom_kill"||$1=="pgscan_kswapd"||$1=="pgscan_direct"||$1=="allocstall_normal"||$1=="workingset_refault_anon"||$1=="workingset_refault_file" {printf "\"%s\":%s,", $1, $2}' /proc/vmstat)
  g=$(cat $gpu 2>/dev/null || echo 0)
  cpu=$(cat /sys/devices/system/cpu/cpu[0-3]/cpufreq/scaling_cur_freq 2>/dev/null | tr '\n' ',' | sed 's/,$//')
  temp=$(cat /sys/class/thermal/thermal_zone*/temp 2>/dev/null | sort -n | tail -1)
  load=$(cut -d' ' -f1 /proc/loadavg)
  echo "{\"t\":$up,$drm,$st,$appcpu,$helperst,$helpercpu,$cgcpu,$mem,${vm}\"gpu_hz\":$g,\"cpu_khz\":[$cpu],\"temp_max_mc\":${temp:-0},\"load1\":$load}"
  sleep 1
done
