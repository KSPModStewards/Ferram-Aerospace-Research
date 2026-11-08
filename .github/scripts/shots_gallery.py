"""One self-contained HTML page of a KspTest results folder: each run's tests with their outcome, steps and screenshots
(embedded, as JPEG when Pillow is there), and the runs' Markdown reports. For CI, uploaded unzipped as its own artifact.

    python shots_gallery.py RESULTS_DIR OUT.html [--title TITLE]
"""
import argparse
import base64
import html
import io
import json
import os
import sys

try:
    from PIL import Image
except ImportError:
    Image = None


def embed(path):
    """The image as a data: URI; JPEG when Pillow can make one (a tenth of the PNG's size)."""
    if Image is not None:
        try:
            with Image.open(path) as im:
                buf = io.BytesIO()
                im.convert("RGB").save(buf, "JPEG", quality=85, optimize=True)
                return "data:image/jpeg;base64," + base64.b64encode(buf.getvalue()).decode()
        except Exception as e:
            print(f"{path}: not converted ({e}), embedding the PNG", file=sys.stderr)
    with open(path, "rb") as f:
        return "data:image/png;base64," + base64.b64encode(f.read()).decode()


def esc(s):
    return html.escape(str(s if s is not None else ""))


def badge(status):
    s = (status or "?").lower()
    cls = "pass" if s == "pass" else "skip" if s in ("skip", "skipped", "inconclusive") else "fail"
    return f'<span class="badge {cls}">{esc(s)}</span>'


def secs(v):
    return f"{v:.0f} s" if isinstance(v, (int, float)) else ""


def tail(path, n=60):
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            return "".join(f.readlines()[-n:])
    except OSError:
        return ""


def run_section(run_dir, out):
    name = os.path.basename(run_dir.rstrip("/\\"))
    result_file = os.path.join(run_dir, "result.json")
    r, why = None, "No result.json"
    if os.path.exists(result_file):
        try:
            with open(result_file, encoding="utf-8") as f:
                r = json.load(f)
        except (OSError, ValueError) as e:
            why = f"Unreadable result.json ({e})"
    if not isinstance(r, dict):
        out.append(f'<section class="run"><h2>{badge("crash")} {esc(name)}</h2><p>{esc(why)}: the end of harness.log</p>'
                   f'<pre>{esc(tail(os.path.join(run_dir, "harness.log")))}</pre></section>')
        return 0
    shots = 0
    out.append(f'<section class="run"><h2>{badge(r.get("status"))} {esc(name)}</h2>'
               f'<p class="meta">{secs(r.get("durationSec"))} &middot; ended in {esc(r.get("scene"))}</p>')
    for t in r.get("tests", []):
        out.append(f'<article class="test"><h3>{badge(t.get("status"))} {esc(t.get("test"))} '
                   f'<span class="meta">{secs(t.get("durationSec"))}</span></h3>')
        if t.get("message"):
            out.append(f'<pre class="message">{esc(t["message"])}</pre>')
        steps = t.get("steps") or []
        if steps:
            out.append(f'<details><summary>{len(steps)} steps</summary><ol class="steps">')
            for s in steps:
                msg = f' &mdash; {esc(s["message"])}' if s.get("message") else ""
                out.append(f'<li>{badge(s.get("status"))} {esc(s.get("name"))} <span class="meta">{secs(s.get("sec"))}</span>{msg}</li>')
            out.append("</ol></details>")
        out.append('<div class="shots">')
        for rel in t.get("screenshots") or []:
            p = os.path.join(run_dir, rel)
            if not os.path.exists(p):
                out.append(f'<p class="missing">missing: {esc(rel)}</p>')
                continue
            label = os.path.splitext(os.path.basename(rel))[0]
            out.append(f'<figure><img loading="lazy" src="{embed(p)}" alt="{esc(label)}" title="click to enlarge">'
                       f'<figcaption>{esc(label)}</figcaption></figure>')
            shots += 1
        out.append("</div></article>")
    reports = sorted(f for f in os.listdir(run_dir) if f.lower().endswith(".md"))
    for rep in reports:
        with open(os.path.join(run_dir, rep), encoding="utf-8", errors="replace") as f:
            out.append(f'<details class="report"><summary>{esc(rep)}</summary><pre>{esc(f.read())}</pre></details>')
    out.append("</section>")
    return shots


CSS = """
:root { --bg: #fff; --fg: #1d1d1f; --muted: #666; --card: #f4f4f6; --line: #ddd;
        --pass: #1a7f37; --fail: #cf222e; --skip: #9a6700; }
@media (prefers-color-scheme: dark) {
  :root { --bg: #0f1115; --fg: #e6e6e6; --muted: #9aa0a6; --card: #191c22; --line: #2c313a;
          --pass: #3fb950; --fail: #f85149; --skip: #d29922; }
}
* { box-sizing: border-box; }
body { margin: 0 auto; max-width: 1400px; padding: 16px; background: var(--bg); color: var(--fg);
       font: 14px/1.45 system-ui, -apple-system, "Segoe UI", sans-serif; }
h1 { font-size: 20px; margin: 0 0 4px; }
h2 { font-size: 16px; margin: 0 0 4px; word-break: break-all; }
h3 { font-size: 15px; margin: 12px 0 6px; }
.meta { color: var(--muted); font-weight: normal; font-size: 13px; }
.run { background: var(--card); border: 1px solid var(--line); border-radius: 8px; padding: 12px 16px; margin: 16px 0; }
.test { border-top: 1px solid var(--line); padding-top: 4px; }
.badge { display: inline-block; min-width: 3.2em; text-align: center; padding: 0 6px; border-radius: 4px;
         font-size: 12px; font-weight: 600; color: #fff; text-transform: uppercase; vertical-align: 1px; }
.badge.pass { background: var(--pass); } .badge.fail { background: var(--fail); } .badge.skip { background: var(--skip); }
.shots { display: grid; grid-template-columns: repeat(auto-fill, minmax(min(100%, 420px), 1fr)); gap: 12px; margin: 8px 0; }
figure { margin: 0; }
figure img { width: 100%; height: auto; display: block; border-radius: 4px; border: 1px solid var(--line); cursor: zoom-in; }
figcaption { color: var(--muted); font-size: 12px; margin-top: 2px; word-break: break-all; }
.shots figure.big { grid-column: 1 / -1; } .shots figure.big img { cursor: zoom-out; }
pre { white-space: pre-wrap; word-break: break-word; font-size: 12px; background: var(--bg); padding: 8px;
      border-radius: 4px; border: 1px solid var(--line); max-height: 60vh; overflow: auto; }
pre.message { border-color: var(--fail); }
.steps { margin: 4px 0; padding-left: 24px; }
details { margin: 6px 0; } summary { cursor: pointer; color: var(--muted); }
"""

JS = """
document.addEventListener('click', e => {
  if (e.target.tagName === 'IMG' && e.target.closest('figure')) e.target.closest('figure').classList.toggle('big');
});
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("results")
    ap.add_argument("out")
    ap.add_argument("--title", default="KspTest screenshots")
    a = ap.parse_args()

    runs = sorted(os.path.join(a.results, d) for d in os.listdir(a.results) if os.path.isdir(os.path.join(a.results, d))) \
        if os.path.isdir(a.results) else []
    body, shots = [], 0
    for d in runs:
        shots += run_section(d, body)
    if not runs:
        body.append(f"<p>No runs in {esc(a.results)}.</p>")
    page = (f'<!doctype html><html lang="en"><head><meta charset="utf-8">'
            f'<meta name="viewport" content="width=device-width, initial-scale=1"><title>{esc(a.title)}</title>'
            f'<style>{CSS}</style></head><body><h1>{esc(a.title)}</h1>'
            f'<p class="meta">{len(runs)} run(s), {shots} screenshot(s); click a screenshot to enlarge it</p>'
            + "".join(body) + f"<script>{JS}</script></body></html>")
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, "w", encoding="utf-8") as f:
        f.write(page)
    print(f"{a.out}: {len(runs)} run(s), {shots} screenshot(s), {len(page) / 1e6:.1f} MB"
          + ("" if Image else " (no Pillow: PNGs embedded)"))


if __name__ == "__main__":
    main()
