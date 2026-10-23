from datetime import datetime
import json
from pathlib import Path
from typing import Sequence

from assignment import calculate_balance_costs
from models import AssignmentMatrixSetup, Cell, WorldSetup
from original_stc import STCPlanningResult, plan_original_darp_stc


ARCHIVE_DIR = Path(__file__).resolve().parent / "run_archive"


def archive_run_snapshot(
    world: WorldSetup,
    assignment_setup: AssignmentMatrixSetup,
    assignment: Sequence[Sequence[Cell]],
    metric_weights: Sequence[float] | None = None,
    tuning_history: Sequence[dict[str, object]] = (),
    stc_result: STCPlanningResult | None = None,
    analysis_summary: dict[str, object] | None = None,
) -> Path:
    ARCHIVE_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    if metric_weights is None:
        metric_weights = (1.0,) * len(world.robot_positions)
    balance_j, balance_objective = calculate_balance_costs(
        [len(cells) for cells in assignment],
        world.rows * world.cols - len(world.obstacle_positions),
    )

    stc_summary = None
    if stc_result is not None:
        stc_summary = {
            "total_planning_seconds": stc_result.total_planning_seconds,
            "robots": [
                {
                    "robot_id": plan.robot_id,
                    "mode": plan.mode,
                    "planning_seconds": plan.planning_seconds,
                    "turns": plan.turns,
                    "coverage_cells": len(set(plan.fine_path)),
                    "moves": len(plan.path_edges),
                }
                for plan in stc_result.plans
            ],
        }

    snapshot = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "rows": world.rows,
        "cols": world.cols,
        "robot_positions": list(world.robot_positions),
        "obstacle_positions": list(world.obstacle_positions),
        "desireable_assign": list(assignment_setup.DesireableAssign),
        "term_thr": assignment_setup.termThr,
        "metric_matrix": assignment_setup.MetricMatrix,
        "tiles_importance": assignment_setup.TilesImportance,
        "assignment": [list(cells) for cells in assignment],
        "metric_weights": list(metric_weights),
        "tuning_history": list(tuning_history),
        "balance_j": list(balance_j),
        "balance_objective": balance_objective,
        "stc_metrics": stc_summary,
        "computational_memory_analysis": analysis_summary,
    }

    run_file = ARCHIVE_DIR / f"run_{timestamp}.json"
    run_file.write_text(json.dumps(snapshot, indent=2), encoding="utf-8")

    history_file = ARCHIVE_DIR / "history.json"
    if history_file.exists():
        try:
            history = json.loads(history_file.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            history = []
    else:
        history = []

    history.append({
        "timestamp": snapshot["timestamp"],
        "run_file": run_file.name,
        "rows": world.rows,
        "cols": world.cols,
        "robot_positions": list(world.robot_positions),
        "obstacle_positions": list(world.obstacle_positions),
        "desireable_assign": list(assignment_setup.DesireableAssign),
        "term_thr": assignment_setup.termThr,
        "metric_weights": list(metric_weights),
        "tuning_iterations": len(tuning_history),
        "balance_j": list(balance_j),
        "balance_objective": balance_objective,
        "stc_metrics": stc_summary,
        "computational_memory_analysis": analysis_summary,
    })
    history_file.write_text(json.dumps(history, indent=2), encoding="utf-8")
    return run_file


def load_archived_runs() -> list[dict[str, object]]:
    run_files = sorted(ARCHIVE_DIR.glob("run_*.json"), reverse=True)
    snapshots = []
    for run_file in run_files:
        try:
            snapshot = json.loads(run_file.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if all(key in snapshot for key in ("rows", "cols", "robot_positions", "obstacle_positions", "assignment")):
            snapshot["_file_name"] = run_file.name
            snapshots.append(snapshot)
    return snapshots


def stc_plans_from_snapshot(snapshot: dict[str, object]) -> STCPlanningResult:
    assignments = [
        [(int(cell[0]), int(cell[1])) for cell in robot_cells]
        for robot_cells in snapshot["assignment"]
    ]
    robot_positions = [
        (int(position[0]), int(position[1]))
        for position in snapshot["robot_positions"]
    ]
    return plan_original_darp_stc(
        assignments,
        robot_positions,
        int(snapshot["rows"]),
        int(snapshot["cols"]),
    )


__all__ = (
    "ARCHIVE_DIR",
    "archive_run_snapshot",
    "load_archived_runs",
    "stc_plans_from_snapshot",
)
