# DARP Initial Area Division Reimplementation

This folder contains a small, testable reimplementation of the DARP area
division stage based on robot initial positions.

It focuses on three parts:

- Equal division of all passable cells into per-robot quotas.
- Spatial growth from each robot's initial position.
- Validation that every robot area is connected and obstacles remain blocked.

Run the tests from this folder:

```bash
python3 -m unittest -v
```

Open the visualizer in a browser:

```text
index.html
```

The visualizer supports editable grid dimensions, robots, obstacles, presets,
and JSON import/export for saving or loading different maps.
