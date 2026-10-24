"""Create a self-contained HTML report from one run manifest or batch summary."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import html
import json
from pathlib import Path


def read_rows(path: Path):
    with path.open(newline="",encoding="utf-8-sig") as handle:
        return list(csv.DictReader(handle))


def run_summary(manifest_path: Path):
    manifest=json.loads(manifest_path.read_text(encoding="utf-8"))
    csv_name=manifest.get("outputs",{}).get("telemetry_csv")
    csv_path=Path(csv_name) if csv_name else None
    if csv_path is not None and not csv_path.is_absolute():
        csv_path=manifest_path.parent/csv_path
    rows=read_rows(csv_path) if csv_path is not None and csv_path.is_file() else []
    def vals(name):
        return [float(row[name]) for row in rows if row.get(name) not in (None,"","-1")]
    speeds=vals("speed_m_s"); errors=vals("horizontal_error_m")
    xyz=[[float(row[k]) for k in ("x_m","y_m","z_m")] for row in rows]
    path_length=sum(sum((b[j]-a[j])**2 for j in range(3))**.5 for a,b in zip(xyz,xyz[1:]))
    phase=rows[-1].get("phase","unknown") if rows else "no telemetry"
    return {"kind":"run","title":manifest.get("scenario",{}).get("target_name","Mission run"),
            "manifest":manifest,"csv":csv_path,"row_count":len(rows),"final_phase":phase,
            "max_speed_m_s":max(speeds) if speeds else None,
            "final_horizontal_error_m":errors[-1] if errors else None,
            "path_length_m":path_length,"timing":manifest.get("timing",{})}


def batch_summary(path: Path):
    data=json.loads(path.read_text(encoding="utf-8"))
    return {"kind":"batch","title":"Monte Carlo batch","batch":data}


def render_html(summary, source: Path, target: Path):
    if summary["kind"]=="run":
        m=summary["manifest"]
        rows=[("Scenario",json.dumps(m.get("scenario"),ensure_ascii=False)),
              ("Seed",m.get("seed")),("Physics mode",m.get("physics",{}).get("mode")),
              ("Rows",summary["row_count"]),("Final phase",summary["final_phase"]),
              ("Final horizontal error (m)",summary["final_horizontal_error_m"]),
              ("Peak speed (m/s)",summary["max_speed_m_s"]),("Path length (m)",summary["path_length_m"]),
            ("Telemetry CSV",str(summary["csv"] or "not yet saved"))]
        timings=summary["timing"]
        timing_table="".join(f"<tr><td>{html.escape(name)}</td><td>{data.get('mean_s')}</td><td>{data.get('p95_s')}</td><td>{data.get('max_s')}</td></tr>"
                             for name,data in timings.items() if isinstance(data,dict) and "mean_s" in data)
        body=f"<h2>Run summary</h2><table>{''.join(f'<tr><th>{html.escape(str(k))}</th><td>{html.escape(str(v))}</td></tr>' for k,v in rows)}</table><h2>Timing (seconds)</h2><table><tr><th>Stage</th><th>Mean</th><th>p95</th><th>Max</th></tr>{timing_table}</table>"
    else:
        data=summary["batch"]
        tables=[]
        for name,scenario in data.get("scenarios",{}).items():
            tables.append(f"<h2>{html.escape(name)}</h2><table>"+
                          f"<tr><th>Trials</th><td>{scenario.get('trial_count')}</td></tr>"+
                          f"<tr><th>Success rate</th><td>{scenario.get('success_rate')}</td></tr>"+
                          f"<tr><th>Wilson 95% interval</th><td>{scenario.get('success_rate_wilson_95')}</td></tr>"+
                          f"<tr><th>Median final error (m)</th><td>{scenario.get('median_final_horizontal_error_m')}</td></tr>"+
                          f"<tr><th>p95 final error (m)</th><td>{scenario.get('p95_final_horizontal_error_m')}</td></tr></table>")
        body=f"<p>Seeds start at {data.get('seed_start')}; trial duration {data.get('duration_s')} s.</p>{''.join(tables)}"
    document=f"""<!doctype html><html><head><meta charset="utf-8"><title>{html.escape(summary['title'])}</title>
<style>body{{font:15px Segoe UI,Arial,sans-serif;max-width:1000px;margin:36px auto;padding:0 24px;color:#17212b}}h1{{color:#164c63}}h2{{margin-top:30px}}table{{border-collapse:collapse;width:100%;margin:12px 0 24px}}th,td{{text-align:left;border-bottom:1px solid #d5dce2;padding:9px}}th{{background:#eef3f6}}.note{{color:#596875}}</style></head>
<body><h1>{html.escape(summary['title'])}</h1><p class="note">Source: {html.escape(str(source))} · generated {datetime.now(timezone.utc).isoformat()}</p>{body}</body></html>"""
    target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(document,encoding="utf-8")


def render_pdf(summary, target: Path):
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
    lines=[f"Report: {summary['title']}",f"Generated: {datetime.now(timezone.utc).isoformat()}"]
    if summary["kind"]=="run":
        lines += [f"Rows: {summary['row_count']}",f"Final phase: {summary['final_phase']}",
                  f"Final horizontal error: {summary['final_horizontal_error_m']} m",
                  f"Peak speed: {summary['max_speed_m_s']} m/s",f"Path length: {summary['path_length_m']} m"]
    else:
        for name,scenario in summary["batch"].get("scenarios",{}).items():
            lines += [f"{name}: {scenario.get('landed_count')}/{scenario.get('trial_count')} landed; "
                      f"Wilson 95%={scenario.get('success_rate_wilson_95')}; "
                      f"median error={scenario.get('median_final_horizontal_error_m')} m"]
    target.parent.mkdir(parents=True,exist_ok=True)
    with PdfPages(target) as pdf:
        fig=plt.figure(figsize=(8.27,11.69)); fig.text(.08,.94,"\n\n".join(lines),va="top",fontsize=12,wrap=True)
        plt.axis("off"); pdf.savefig(fig); plt.close(fig)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("input",type=Path,help="run manifest JSON or batch_summary.json")
    parser.add_argument("--output",type=Path)
    parser.add_argument("--pdf",action="store_true",help="also create a one-page PDF")
    args=parser.parse_args()
    source=args.input.resolve()
    data=json.loads(source.read_text(encoding="utf-8"))
    summary=batch_summary(source) if "scenarios" in data else run_summary(source)
    output=args.output or source.with_name(source.stem+"_report.html")
    render_html(summary,source,output.resolve())
    print(f"HTML report: {output.resolve()}")
    if args.pdf:
        pdf=output.with_suffix(".pdf"); render_pdf(summary,pdf.resolve()); print(f"PDF report: {pdf.resolve()}")


if __name__=="__main__":
    main()
