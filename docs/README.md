# Markers Mapping

Each lower-limb joint carries one AprilTag marker.

- Family: `tagStandard41h12`
- Tag ids: `0 .. 6`
- Printable sheet: `tagStandard41h12.pdf`
- On-sheet layout: `Coordinates_tagStandard41h12.json`

> Generated from: https://shiqiliu-67.github.io/apriltag-generator/

Print the sheet, cut the tags out, and stick one on each joint per the table below. 
The black-square edge length is what the estimator needs, passed via `--tag-size` (default 0.05 m). 
Measure your printed tags and set it to match.

## Tag to joint mapping

| Tag id | Joint   | Side  |
|--------|---------|-------|
| 0      | pelvis  | root  |
| 1      | r_knee  | right |
| 2      | l_knee  | left  |
| 3      | r_ankle | right |
| 4      | l_ankle | left  |
| 5      | r_foot  | right |
| 6      | l_foot  | left  |

`pelvis` (tag 0) is the root. The rest chain off it per leg:
`pelvis -> knee -> ankle -> foot`. 
Keep the ids consistent with the mapping, otherwise the joints will be swapped.
