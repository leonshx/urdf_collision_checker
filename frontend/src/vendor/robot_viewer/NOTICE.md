The files URDFAdapter.js, UnifiedRobotModel.js, MathUtils.js and JointDragControls.js
are vendored from https://github.com/fan-ziqi/robot_viewer at revision
80047f4d0ff07bba4ebd23fd6b27d55a0864e61c, under Apache-2.0 (see LICENSE).
Only URDF viewing and drag control components are reused; the analysis UI and
Python backend are implemented in this repository.

Local modification: JointDragControls raycasts visible collision meshes as well
as visual meshes, so collision-only URDFs remain draggable. It also honors the
controller enabled flag on mouse-down.
Trailing blank lines are normalized in the vendored JavaScript files.
