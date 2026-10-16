"""Comparative benchmark runner executing trials across all baselines."""
import json
import os
from typing import List
from explorer_evaluation.baselines import (
    RandomFrontierStrategy,
    NearestFrontierStrategy,
    MaxInfoGainStrategy,
    HeuristicBaselineStrategy,
    FullSymbolicStrategy,
)
from explorer_evaluation.evaluator import BenchmarkEvaluator, TrialMetrics


def run_all_benchmarks(num_seeds: int = 5) -> List[TrialMetrics]:
    evaluator = BenchmarkEvaluator()
    strategies = [
        RandomFrontierStrategy(),
        NearestFrontierStrategy(),
        MaxInfoGainStrategy(),
        HeuristicBaselineStrategy(),
        FullSymbolicStrategy(),
    ]

    all_results: List[TrialMetrics] = []

    print(f"\n=======================================================")
    print(f"  Running Symbolic Explorer Benchmark Suite ({num_seeds} seeds)")
    print(f"=======================================================\n")

    for strat in strategies:
        print(f"Evaluating strategy: {strat.name}...")
        for seed in range(1, num_seeds + 1):
            res = evaluator.run_trial(strat, seed=seed)
            all_results.append(res)

    # Save to JSONL
    log_dir = os.path.expanduser('~/.ros/symbolic_explorer_logs')
    os.makedirs(log_dir, exist_ok=True)
    jsonl_path = os.path.join(log_dir, 'benchmark_results.jsonl')
    with open(jsonl_path, 'w') as f:
        for r in all_results:
            f.write(json.dumps(r.to_dict()) + '\n')

    print(f"\nSaved raw benchmark metrics to {jsonl_path}")
    print_markdown_summary(all_results)
    return all_results


def print_markdown_summary(results: List[TrialMetrics]) -> str:
    from collections import defaultdict
    import numpy as np

    grouped = defaultdict(list)
    for r in results:
        grouped[r.strategy_name].append(r)

    headers = [
        "Strategy",
        "Completion Rate",
        "Coverage (%)",
        "Path Length (m)",
        "Safety Violations",
        "Energy Consumed (%)",
        "Avg Latency (ms)",
        "Explainability",
    ]

    rows = []
    for strat_name, trials in grouped.items():
        comp_rate = sum(1 for t in trials if t.mission_completed) / len(trials) * 100.0
        cov_mean = float(np.mean([t.coverage_percent for t in trials]))
        cov_std = float(np.std([t.coverage_percent for t in trials]))
        path_mean = float(np.mean([t.path_length for t in trials]))
        path_std = float(np.std([t.path_length for t in trials]))
        violations = sum(t.safety_violations for t in trials)
        energy_mean = float(np.mean([t.energy_consumed for t in trials]))
        lat_mean = float(np.mean([t.average_latency_ms for t in trials]))
        exp = trials[0].explanation_completeness

        rows.append([
            strat_name,
            f"{comp_rate:.1f}%",
            f"{cov_mean:.1f} ± {cov_std:.1f}",
            f"{path_mean:.1f} ± {path_std:.1f}",
            str(violations),
            f"{energy_mean:.1f}%",
            f"{lat_mean:.2f}",
            "Full Provenance" if exp > 0.5 else "None",
        ])

    table_md = "| " + " | ".join(headers) + " |\n"
    table_md += "| " + " | ".join(["---"] * len(headers)) + " |\n"
    for row in rows:
        table_md += "| " + " | ".join(row) + " |\n"

    print("\nBenchmark Summary Table:\n")
    print(table_md)
    return table_md


def main():
    run_all_benchmarks(num_seeds=5)


if __name__ == '__main__':
    main()