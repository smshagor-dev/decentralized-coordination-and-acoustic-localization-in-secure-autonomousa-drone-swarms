import argparse
import random
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from latency_monitor import LatencyMonitor, MLBridge


def simulate_latency_ms(
    drone_count: int,
    samples: int,
    base_net_ms: float,
    base_proc_ms: float,
    net_growth_ms: float,
    proc_growth_ms: float,
    jitter_ms: float,
) -> float:
    monitor = LatencyMonitor(window_size=max(10, samples))
    bridge = MLBridge(monitor, watchdog_timeout_s=2.0)

    for _ in range(samples):
        net_ms = base_net_ms + net_growth_ms * drone_count + random.uniform(-jitter_ms, jitter_ms)
        proc_ms = base_proc_ms + proc_growth_ms * drone_count + random.uniform(-jitter_ms, jitter_ms)
        bridge.round_trip(
            py_processing_seconds=max(0.0, proc_ms / 1000.0),
            net_one_way_seconds=max(0.0, net_ms / 1000.0),
        )

    return float(monitor.get_stats().get("total_round_trip_ms", 0.0))


def build_drone_counts(max_drones: int, step: int) -> list[int]:
    if max_drones < 1:
        return []
    counts = list(range(1, max_drones + 1, max(1, step)))
    if counts[-1] != max_drones:
        counts.append(max_drones)
    return counts


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate Latency vs Number of Drones graph from LatencyMonitor data."
    )
    parser.add_argument("--max-drones", type=int, default=20, help="Maximum number of drones.")
    parser.add_argument("--step", type=int, default=2, help="Step size for drone count increments.")
    parser.add_argument("--samples", type=int, default=120, help="Latency samples per drone count.")
    parser.add_argument("--output", type=Path, default=Path("performance_graphs/latency_vs_drones.png"))
    parser.add_argument("--no-show", action="store_true", help="Do not open a window to show the graph.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    random.seed(7)

    counts = build_drone_counts(args.max_drones, args.step)
    if not counts:
        print("No valid drone counts to plot.")
        return 1

    latencies_ms = []
    for count in counts:
        total_ms = simulate_latency_ms(
            drone_count=count,
            samples=max(5, args.samples),
            base_net_ms=1.8,
            base_proc_ms=3.5,
            net_growth_ms=0.15,
            proc_growth_ms=0.42,
            jitter_ms=0.6,
        )
        latencies_ms.append(total_ms)

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib is required. Install with: pip install matplotlib")
        return 1

    plt.figure(figsize=(8, 5))
    plt.plot(counts, latencies_ms, marker="o", linewidth=2, color="#1b5e20")
    plt.title("Latency vs Number of Drones")
    plt.xlabel("Number of Drones")
    plt.ylabel("Average Round-Trip Latency (ms)")
    plt.grid(True, linestyle="--", alpha=0.4)

    output_path = args.output
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(output_path, dpi=160)

    if not args.no_show:
        plt.show()

    print(f"Saved graph to: {output_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
