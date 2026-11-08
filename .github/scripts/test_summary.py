"""Test results as Markdown for a GitHub job summary: one leg's TRX files (one per job) merged into one report.

    test_summary.py TITLE PATH...

PATH: .trx files, or folders searched for them (and for KspTest's result.json files, whose per-test durations leave out
the KSP load the first test of each session waits through). Tests a job skipped because another CI job runs them
(KSPTEST_SHARD) are left out, so a split run reads as one; one that every job skipped so ran nowhere is listed, and makes
the exit status 1 (else 0). A TRX file that cannot be read is listed, the rest reported.
"""
import glob
import json
import os
import re
import sys
import xml.etree.ElementTree as ET
from datetime import datetime

NS = {"t": "http://microsoft.com/schemas/VisualStudio/TeamTest/2010"}
ELSEWHERE = re.compile(r"^runs in (another )?CI job")
SLOWEST = 10


def seconds(duration):
    """A TRX duration, hh:mm:ss.fffffff, in seconds."""
    h, m, s = duration.split(":")
    return int(h) * 3600 + int(m) * 60 + float(s)


def text(node, path):
    found = node.find(path, NS)
    return (found.text or "").strip() if found is not None else ""


def short(name):
    """Namespace.Class.Method(args) -> (Class, Method(args)): the arguments of a theory's case can hold dots."""
    base, paren, args = name.partition("(")
    parts = base.split(".")
    return (parts[-2] if len(parts) > 1 else "", parts[-1] + paren + args)


def message(raw, stdout):
    """
    A failure's message without what the summary says already (the test's name, the results folder). KspTest's report
    in the test's output has all its lines, the TRX message only the first: the lines after "==== Test: FAIL", up to
    the stack trace.
    """
    m = re.search(r"^==== \S+: FAIL\b.*$", stdout, re.M)
    if m:
        lines = []
        for line in stdout[m.end():].lstrip("\r\n").splitlines():
            if not line.strip() or line.startswith(("  at ", "  screenshots:", "in one KSP session")):
                break
            lines.append(line)
        if lines:
            return "\n".join(lines)
    raw = re.sub(r"^\S+: fail - ", "", raw)
    return re.sub(r" \(results: [^)]*\)", "", raw).strip()


def stamp(value):
    """A TRX time (ISO 8601 with up to 7 decimals and a Z or an offset) as a datetime, or None."""
    try:
        value = re.sub(r"(\.\d{6})\d+", r"\1", value.replace("Z", "+00:00"))
        return datetime.fromisoformat(value)
    except (ValueError, AttributeError):
        return None


def clock(sec):
    if sec >= 90:
        return f"{sec / 60:.1f} min"
    return f"{sec:.0f} s" if sec >= 1 else f"{sec * 1000:.0f} ms"


def step_outputs(passed, failed, skipped, nowhere):
    """The totals as step outputs, on GitHub Actions: the workflow's per-leg check run is titled with them."""
    path = os.environ.get("GITHUB_OUTPUT")
    if path:
        with open(path, "a", encoding="utf-8") as f:
            f.write(f"passed={passed}\nfailed={failed}\nskipped={skipped}\nnowhere={nowhere}\n")


def main(title, paths):
    trx, results = [], []
    for p in paths:
        if os.path.isdir(p):
            trx += sorted(glob.glob(os.path.join(p, "**", "*.trx"), recursive=True))
            results += glob.glob(os.path.join(p, "**", "result.json"), recursive=True)
        elif p.endswith(".trx"):
            trx.append(p)

    # each test's own run time from the harness, by full name
    harness = {}
    for f in results:
        try:
            for t in json.load(open(f, encoding="utf-8")).get("tests", []):
                if t.get("durationSec"):
                    harness[t.get("id") or t.get("test")] = t["durationSec"]
        except (OSError, ValueError):
            pass

    jobs, tests, elsewhere, unreadable = [], {}, set(), []
    for f in trx:
        job = os.path.splitext(os.path.basename(f))[0]
        try:
            root = ET.parse(f).getroot()
        except (ET.ParseError, OSError) as e:
            unreadable.append(f"{os.path.basename(f)}: {e}")
            continue
        counts = {"Passed": 0, "Failed": 0, "NotExecuted": 0}
        for r in root.iterfind(".//t:UnitTestResult", NS):
            outcome, msg = r.get("outcome"), text(r, "t:Output/t:ErrorInfo/t:Message")
            if outcome == "NotExecuted" and ELSEWHERE.match(msg):
                elsewhere.add(r.get("testName"))
                continue
            counts[outcome] = counts.get(outcome, 0) + 1
            name = r.get("testName")
            try:
                sec = harness.get(name, seconds(r.get("duration") or "0:0:0"))
            except ValueError:
                sec = 0.0
            if outcome == "Failed":
                msg = message(msg, text(r, "t:Output/t:StdOut"))
            tests[name] = (outcome, msg, sec, job)
        times = root.find("t:Times", NS)
        start, finish = (stamp(times.get("start")), stamp(times.get("finish"))) if times is not None else (None, None)
        wall = (finish - start).total_seconds() if start and finish else 0.0
        jobs.append((job, counts, wall))

    nowhere = sorted(elsewhere - set(tests))
    if not tests and not nowhere:
        step_outputs(0, 0, 0, 0)
        print(f"## {title}\n\nNo test results." + "".join(f"\n- unreadable: {u}" for u in unreadable) + "\n")
        return 0

    by = {"Passed": [], "Failed": [], "NotExecuted": []}
    for name, (outcome, msg, sec, job) in tests.items():
        by.setdefault(outcome, []).append((name, msg, sec, job))
    passed, failed, skipped = len(by["Passed"]), len(by["Failed"]), len(by["NotExecuted"])
    step_outputs(passed, failed, skipped, len(nowhere))
    verdict = ", ".join(v for v in (f"{failed} failed" if failed else "", f"{len(nowhere)} run in no job" if nowhere else "") if v) or "all passed"
    out = [f"## {title}: {verdict}", "",
           f"**{passed} passed, {failed} failed, {skipped} skipped** of {len(tests)}", ""]

    if nowhere:
        # every job left them to another: the jobs dealt the tests from different durations
        out += ["### Run in no job", "", "Every job skipped these as run by another, so none ran them:", "",
                ", ".join(short(n)[1] for n in nowhere), ""]
    if unreadable:
        out += ["### Unreadable results", ""] + [f"- {u}" for u in unreadable] + [""]

    if len(jobs) > 1:
        out += ["| job | passed | failed | skipped | time |", "|---|---|---|---|---|"]
        for job, c, wall in sorted(jobs):
            out.append(f"| {job} | {c['Passed']} | {c['Failed']} | {c['NotExecuted']} | {clock(wall)} |")
        out.append("")

    if failed:
        out += ["### Failed", ""]
        for name, msg, sec, job in sorted(by["Failed"]):
            cls, method = short(name)
            out += [f"- **{method}** ({cls}, {job}, {clock(sec)})", "", "  ```", *("  " + l for l in msg.splitlines()), "  ```"]
        out.append("")

    if skipped:
        reasons = {}
        for name, msg, _, _ in by["NotExecuted"]:
            reasons.setdefault(msg or "no reason given", []).append(short(name)[1])
        out += ["### Skipped", ""]
        for reason, names in sorted(reasons.items(), key=lambda kv: -len(kv[1])):
            out.append(f"<details><summary>{len(names)}: {reason}</summary>\n\n{', '.join(sorted(names))}\n\n</details>\n")

    ran = sorted(by["Passed"] + by["Failed"], key=lambda t: -t[2])[:SLOWEST]
    if len(ran) > 1:
        out += ["### Slowest", "", "| test | job | time |", "|---|---|---|"]
        out += [f"| {short(n)[1]} | {job} | {clock(sec)} |" for n, _, sec, job in ran]
        out.append("")

    print("\n".join(out))
    return 1 if nowhere else 0


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(0)
    sys.exit(main(sys.argv[1], sys.argv[2:]))
