"""Compatibility entry point for the DARP simulation."""

from archive import (
    ARCHIVE_DIR,
    archive_run_snapshot,
    load_archived_runs,
    stc_plans_from_snapshot,
)
from analysis import estimate_numeric_matrices_bytes, summarize_tuning_analysis
from assignment import (
    calculate_balance_costs,
    construct_assignment_matrix,
    create_3d_matrix,
    divide_tiles_by_importance,
    euclidean_distance,
    tune_metric_weights,
    validate_portions,
)
from models import (
    AssignmentMatrixSetup,
    Cell,
    FREE,
    Grid,
    OBSTACLE,
    WorldSetup,
)
from original_stc import RobotSTCPlan, STCPlanningResult, plan_original_darp_stc
from simulation import main, run_simulation
from visualization import (
    initialize_display,
    present_stc_results,
    print_grid,
    print_matrix,
    show_archive_viewer,
)
from world import (
    count_reachable_passable_cells,
    create_grid_env,
    four_neighbors,
    setup_world,
    to_cell,
    validate_cell_inside_map,
    validate_free_space_connected,
    validate_grid_size,
    validate_obstacles_inside_map,
    validate_robots_inside_map,
    validate_robots_not_on_obstacles,
    validate_unique_robot_positions,
)


if __name__ == "__main__":
    main()
