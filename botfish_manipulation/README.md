# Botfish Manipulation Node

## Summary

This node acts as an interface that converts commands from ```botfish_engine``` to moveit plans that are used to control
the
arms and hands of the R2ED Humanoid robot

This node works by applying offsets to some set starting position in x, y, z space to allow for movement along a grid,
which in our case represented by a chessboard. The starting location is assumed to be the lower left corner of the grid
which in real space translates to left corner of the board closest to the arm. Currently, this node is configured to
allow
for solving the N-Queens problem and some reconfiguration will be required if a dev desires chess to be played instead.

### Config

- `cell_offset` - distance in meters that the robot arm would have to move to get to the middle of a cell. This
  should be half the width of a single cell
- `end_effector` - Name of the link that will be moved to the desired position. Defaults to `right_hand_base_link`
- `reference_link` - Name of the link that is used as the reference point for moving the `end_effector`. Determines the
  coordinate system, defaults to `right_arm_podest_link`
- `grab_height` - Height to set the `end_effector` to to be able to interact with the chess pieces. Please note that
  this will end up as vertical position of only the end_effector and that the rest of the hand will be in a different
  position. Moveit should prevent any part of the bot from colliding with the board but keep in mind this is an offset
- `move_height` - Height to set the `end_effector` to to be able to move around without interacting with chess pieces.
  Warning from above still applies
- `goal_tolerance` - Tolerance for Moveit to use when planning, generally the lower the better but this can run into
  issues with plans failing if its too small
- `max_velocity` - Determines the max velocity of the arm as a percentage, for example a value of 0.2 would allow the
  max arm velocity to be 20% of the driver determined max speed.
- `max_acceleration` - Determines the max acceleration of the arm as a percentage, for example a value of 0.2 would
  allow the arm to accelerate at 20% of its driver defined maximum velocity.
- `planning_time` - Time in seconds that the moveit planner is allowed to plan for, higher values will result in overall
  more successes but can result in a large pauses between movements.

### Subscribes

- `/engine_move` - String representation of all cells to place queens at for solving N-Queens problem. Ex: `"A1B1C1D1"`

### Publishes

- `/right_hand/target` - Vector of positions to set each of the joints to for grasping pieces
