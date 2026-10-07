"""One line per scenario from a regression.sh run's logs.

    python tests/harness/summarize.py <prefix> [<chain file>]

Reads tests/harness/out/<prefix>_<scenario>.out for every scenario the chain
file (default tests/harness/out/<prefix>_chain.txt) lists, and prints: exit
code, processes left right after the app exited, the fleet result (or the
boot failure / single-flight outcome), EKF failsafe count, the Renode RSS sum
with every drone airborne, seconds to "Fleet ready" and to the fleet result,
and the landing spread.
"""
import re
import sys
from pathlib import Path

OUT = Path(__file__).resolve().parent / "out"


def stamp(line):
    match = re.match(r"\[\s*([\d.]+)s\]", line)
    return float(match.group(1)) if match else None


def summarize(prefix, scenario, exit_code, left):
    path = OUT / f"{prefix}_{scenario}.out"
    if not path.exists():
        return f"{scenario}: no log"
    lines = path.read_text(errors="replace").splitlines()
    timed = [line for line in lines if line.startswith("[") and "flight_log:" in line]

    def first(pattern, source=timed):
        return next((line for line in source if re.search(pattern, line)), None)

    result = first(r"Fleet result:")
    ready = first(r"Fleet ready after")
    boot_failed = first(r"FLEET BOOT_FAILED", lines)
    rss = [int(m.group(1)) for line in lines
           if (m := re.search(r"pid \d+ renode \(.*\): RSS (\d+) MB", line))]
    spread = first(r"landing points pairwise", lines)
    # the flight controller's own messages, once each (not the table's FAILSAFE state label)
    failsafes = sum(1 for line in timed if "[FC]" in line and "failsafe" in line.lower())
    parts = [f"exit={exit_code}", f"left={left}"]
    if result:
        parts.append(result.split("flight_log: ")[1].replace(" All Renode instances stopped.", ""))
        parts.append(f"finished={stamp(result):.0f}s")
    elif boot_failed:
        parts.append("BOOT FAILED: " + boot_failed.split("): ", 1)[-1][:230])
    else:
        single = first(r"Mission complete|flight .* (ended|finished)", lines) or first(r"RESULT|PASS|FAIL", lines)
        parts.append((single or "no fleet result line")[:160])
    if ready:
        parts.append(f"ready={stamp(ready):.0f}s")
    if rss:
        parts.append(f"renode_rss={sum(rss)}MB/{len(rss)}proc")
    parts.append(f"failsafes={failsafes}")
    if spread:
        parts.append(spread.split("pairwise: ")[1])
    return f"{scenario}: " + " | ".join(parts)


def main():
    prefix = sys.argv[1]
    chain = Path(sys.argv[2]) if len(sys.argv) > 2 else OUT / f"{prefix}_chain.txt"
    text = chain.read_text()
    for block in re.split(r"^##### ", text, flags=re.M)[1:]:
        scenario = block.split()[0]
        code = re.search(r"^exit=(\d+)", block, re.M)
        after = block.split("pgrep right after app exit:")[-1].strip().splitlines() if "pgrep right after" in block else ["?"]
        left = "nothing" if after and after[0] == "(nothing)" else f"{sum(1 for l in after if l[:1].isdigit())} process(es)"
        print(summarize(prefix, scenario, code.group(1) if code else "running", left))


if __name__ == "__main__":
    main()
