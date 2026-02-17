#########################################################################
#                                                                       #
#   SECURE DRONE SWARM SYSTEM - CORE MODULE                             #
#                                                                       #
#   Developer : Md Shahanur Islam Shagor                                #
#   Role      : Project Architect & Lead Developer                      #
#   Version   : 1.0.2                                                   #
#   Status    : Production Ready                                        #
#                                                                       #
#   "Protecting the skies with decentralized intelligence."             #
#                                                                       #
#########################################################################
import argparse
import csv
import json
from datetime import datetime
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate automated plots from runtime_latency_vs_drones CSV."
    )
    parser.add_argument(
        "csv_path",
        type=Path,
        help="Path to runtime_latency_vs_drones_YYYYMMDD_HHMMSS.csv",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("performance_graphs"),
        help="Output directory for generated plots.",
    )
    return parser.parse_args()


def read_csv(csv_path: Path):
    times = []
    latencies = []
    battery_points = []

    with csv_path.open("r", encoding="utf-8", errors="ignore", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                ts = float(row["timestamp"])
                latency_ms = float(row["latency_ms"])
            except (KeyError, ValueError):
                continue
            times.append(datetime.fromtimestamp(ts))
            latencies.append(latency_ms)

            raw = row.get("drone_status_json") or "{}"
            try:
                drones = json.loads(raw)
            except json.JSONDecodeError:
                continue

            for drone_id, info in drones.items():
                battery = info.get("battery")
                ml_enabled = bool(info.get("ml_enabled", False))
                phys_samples = info.get("physical_ml_samples")
                if battery is None or phys_samples is None:
                    continue
                battery_points.append(
                    {
                        "drone_id": str(drone_id),
                        "battery": float(battery),
                        "ml_enabled": ml_enabled,
                        "physical_ml_samples": float(phys_samples),
                    }
                )

    return times, latencies, battery_points


def plot_latency_trend(times, latencies, out_path: Path):
    if not times:
        return
    import matplotlib.pyplot as plt

    plt.figure(figsize=(9, 4.5))
    plt.plot(times, latencies, linewidth=1.8, color="#1b5e20")
    plt.title("Latency Trend Over Time")
    plt.xlabel("Time")
    plt.ylabel("Round-Trip Latency (ms)")
    plt.grid(True, linestyle="--", alpha=0.4)
    plt.tight_layout()
    plt.savefig(out_path, dpi=160)
    plt.close("all")


def plot_battery_vs_ml_load(points, out_path: Path):
    if not points:
        return
    import matplotlib.pyplot as plt

    xs = [p["physical_ml_samples"] for p in points]
    ys = [p["battery"] for p in points]
    colors = ["#0d47a1" if p["ml_enabled"] else "#9e9e9e" for p in points]

    plt.figure(figsize=(8.5, 5))
    plt.scatter(xs, ys, s=22, alpha=0.6, c=colors)
    plt.title("Battery Decay vs ML Load")
    plt.xlabel("ML Load (physical_ml_samples)")
    plt.ylabel("Battery Level (%)")
    plt.grid(True, linestyle="--", alpha=0.4)
    plt.tight_layout()
    plt.savefig(out_path, dpi=160)
    plt.close("all")


def main() -> int:
    args = parse_args()
    csv_path = args.csv_path
    if not csv_path.exists():
        print(f"CSV not found: {csv_path}")
        return 1

    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    times, latencies, battery_points = read_csv(csv_path)

    run_id = csv_path.stem.replace("runtime_latency_vs_drones_", "")
    latency_out = out_dir / f"latency_trend_{run_id}.png"
    battery_out = out_dir / f"battery_vs_ml_load_{run_id}.png"

    plot_latency_trend(times, latencies, latency_out)
    plot_battery_vs_ml_load(battery_points, battery_out)

    print(f"Saved: {latency_out}")
    print(f"Saved: {battery_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
