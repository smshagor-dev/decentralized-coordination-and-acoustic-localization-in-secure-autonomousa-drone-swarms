"""
CLI for advanced personal ML training from CSV/JSON datasets.
"""

import argparse
import os
from ml_system import PhysicalMLTrainer


def parse_args():
    parser = argparse.ArgumentParser(description="Train personal drone ML model")
    parser.add_argument("--drone-id", type=int, default=1, help="Target drone id for model name")
    parser.add_argument(
        "--dataset",
        type=str,
        default="datasets/personal_training.csv",
        help="Path to CSV/JSON dataset",
    )
    parser.add_argument(
        "--generate-demo",
        action="store_true",
        help="Generate synthetic demo dataset before training",
    )
    parser.add_argument(
        "--format",
        choices=["csv", "json"],
        default="csv",
        help="Dataset format when generating demo data",
    )
    parser.add_argument("--samples", type=int, default=500, help="Demo dataset sample count")
    parser.add_argument("--min-samples", type=int, default=50, help="Minimum samples required to train")
    parser.add_argument("--poly-degree", type=int, choices=[1, 2], default=2, help="Feature polynomial degree")
    parser.add_argument("--append", action="store_true", help="Append dataset to in-memory data instead of replacing")
    parser.add_argument(
        "--export-after",
        type=str,
        default="",
        help="Optional path to export trainer in-memory dataset after training",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    os.makedirs("datasets", exist_ok=True)
    os.makedirs("models", exist_ok=True)

    if args.generate_demo:
        ok = PhysicalMLTrainer.generate_demo_dataset(
            output_path=args.dataset,
            samples=args.samples,
            file_format=args.format,
        )
        if not ok:
            print(f"Failed to generate demo dataset: {args.dataset}")
            return 1
        print(f"Demo dataset generated: {args.dataset}")

    trainer = PhysicalMLTrainer(owner_id=args.drone_id)
    trained = trainer.train_from_dataset(
        input_path=args.dataset,
        min_samples=args.min_samples,
        poly_degree=args.poly_degree,
        append=args.append,
    )
    if not trained:
        print(f"Training failed using dataset: {args.dataset}")
        return 2

    print(f"Training complete for drone {args.drone_id}")
    print(f"Model saved: {trainer.model_path()}")
    if trainer.training_metrics:
        print(f"Metrics: {trainer.training_metrics}")

    if args.export_after:
        out_fmt = "json" if args.export_after.lower().endswith(".json") else "csv"
        if trainer.export_dataset(args.export_after, file_format=out_fmt):
            print(f"Exported merged dataset: {args.export_after}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
