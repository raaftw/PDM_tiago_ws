# Table Detector (Costmap Connected Components)

This node detects tables from an occupancy grid using connected component analysis and publishes their centers and corner polygons.

## Build
```bash
cd /home/raaf/PDM_tiago_ws
colcon build --packages-select pdm_test
source install/setup.bash
```

## Run
```bash
ros2 launch pdm_test table_detector.launch.py \
  occupied_threshold:=65 \
  min_table_area:=0.15 \
  max_table_area:=1.0 \
  erode_iterations:=0 \
  save_to_yaml:=True \
  yaml_output_path:="$(ros2 pkg prefix pdm_test)/share/pdm_test/maps/tables_detected.yaml"
```
Notes:
- `erode_iterations`: keep `0` for full-size obstacles; set to `1` if inflated obstacles make tables appear too large.
- Increase `occupied_threshold` (e.g., 75) if walls are slightly occupied in the map; decrease if tables are missed.
- Adjust `min_table_area`/`max_table_area` if your tables are smaller or larger than 0.5 m squares.

## Outputs
- `/detected_tables` (`PoseArray`): table centers in `map` frame.
- `/table_markers` (`MarkerArray`): line strips and labels for RViz.
- YAML file: `tables_detected.yaml` with `id`, `center`, `width`, `height`, `corners`.

## RViz Quick Setup
- Add `MarkerArray` display for topic `/table_markers`.
- Add `PoseArray` display for topic `/detected_tables`.

## Troubleshooting
- **No detections**: lower `occupied_threshold` or set `erode_iterations:=0`.
- **Tables too big**: set `erode_iterations:=1` (or higher) or increase `occupied_threshold`.
- **Wrong count**: widen `max_table_area` or lower `min_table_area` depending on the miss.
