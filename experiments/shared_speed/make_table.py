"""Print one Markdown table row per run from out/<tag>.json (Task 21).

    python experiments/shared_speed/make_table.py [tag ...]      # default: every result, by drones then tag
"""
import json
import sys
from pathlib import Path

OUT = Path(__file__).resolve().parent / "out"


def span(values, digits=0):
    values = [v for v in values if v is not None]
    if not values:
        return "-"
    lo, hi = min(values), max(values)
    return f"{lo:.{digits}f}" if round(lo, digits) == round(hi, digits) else f"{lo:.{digits}f}-{hi:.{digits}f}"


def row(r):
    o = r["options"]
    d = r.get("drone_results", [])
    knobs = []
    if r.get("renode_cpus_per_group") and any(r["renode_cpus_per_group"]):
        c = r["renode_cpus_per_group"][0]
        knobs.append("Renode on " + ("P-cores" if c == list(range(12)) else "E-cores" if c == [12, 13, 14, 15]
                                     else "CPUs " + ",".join(map(str, c))))
    elif o.get("affinity", "none") != "none":
        knobs.append({"renode-p": "Renode on P-cores", "renode-p1": "Renode on one CPU per P-core",
                      "renode-e": "Renode on E-cores"}[o["affinity"]])
    if r.get("sidecar_cpus"):
        s = r["sidecar_cpus"]
        knobs.append("sidecars on " + ("E-cores" if s == [12, 13, 14, 15] else "P-cores" if s == list(range(12))
                                       else "CPUs " + ",".join(map(str, s))))
    if o.get("advance_immediately"):
        knobs.append("advance immediately")
    if o.get("quantum") != "0.01":
        knobs.append(f"machine quantum {o['quantum']}")
    if o.get("master_quantum") != "0.1":
        knobs.append(f"master quantum {o['master_quantum']}")
    if o.get("gc") != "7":
        knobs.append(f"GC {o['gc']}")
    if o.get("boot_only"):
        knobs.append("boot only")
    sizes = "+".join(str(len(g)) for g in r["groups"])
    shape = f"{r['drones']} x {r['processes']} ({sizes})" if r["processes"] > 1 else f"{r['drones']} x 1"
    if "skipped" in r:
        return f"| {r['tag']} | {shape} | {', '.join(knobs) or 'today'} | SKIPPED: {r['skipped']} (estimate {r['estimate_mb']} MB) |" + " |" * 12
    rss = lambda key: (f"{sum(r[key].values())}" + (f" ({'/'.join(str(v) for v in r[key].values())})"
                                                    if len(r.get(key, {})) > 1 else "")) if r.get(key) else "-"
    rtf_b, rtf_f = r.get("rtf_boot_mean_min") or (None, None), r.get("rtf_flight_mean_min") or (None, None)
    fmt = lambda pair: f"{pair[0]:.3f} / {pair[1]:.3f}" if pair[0] is not None else "-"
    return (f"| {r['tag']} | {shape} | {', '.join(knobs) or 'today'} | {'yes' if r.get('success') else 'NO'} | "
            f"{span([x.get('gps_fix_s') for x in d])} | {span([x.get('armable_s') for x in d])} | "
            f"{span([x.get('to_50m_s') for x in d])} | {span([x.get('flight_s') for x in d])} | "
            f"{r.get('launch_to_all_landed_s', '-')} | {rss('rss_after_boot_mb')} | {rss('rss_airborne_mb')} | "
            f"{rss('rss_end_mb')} | {r.get('renode_cores_mean', '-')} | {r.get('sidecar_cores_mean', '-')} | "
            f"{fmt(rtf_b)} | {fmt(rtf_f)} | {r.get('min_mem_available_mb', '-')} |")


HEADER = ("| Run | Drones x Renodes | Settings | OK | GPS fix (s) | Armable (s) | Arm->50 m (s) | Flight (s) | "
          "Launch to all landed (s) | RSS after boot (MB) | RSS airborne (MB) | RSS end (MB) | Renode CPU (cores) | "
          "Sidecar CPU (cores) | RTF boot mean / min | RTF flight mean / min | Min MemAvailable (MB) |\n" + "|---" * 17 + "|")

if __name__ == "__main__":
    tags = sys.argv[1:]
    results = [json.loads(p.read_text()) for p in sorted(OUT.glob("*.json"))]
    if tags:
        by_tag = {r["tag"]: r for r in results}
        results = [by_tag[t] for t in tags if t in by_tag]
    else:
        results.sort(key=lambda r: (r["drones"], r["processes"], r["tag"]))
    print(HEADER)
    for r in results:
        print(row(r))
