# gazebo_marker_visuals

A Gazebo (11) visual plugin that draws a `visualization_msgs/MarkerArray` topic in Gazebo: the RViz shapes of a
node become Gazebo visuals, so you can see them in gzclient and, if you want, in chosen Gazebo cameras. They are
visuals only, with no link, no collision and no effect on physics. Simulation only; nothing here knows a particular
robot.

```
node --MarkerArray--> plugin (gzserver scene: the camera sensors)  --> camera sensors named in show_in_cameras
                      plugin (gzclient scene)                       --> gzclient window
```

Markers of type `CUBE`, `SPHERE` and `CYLINDER` are drawn with their pose, scale, colour and alpha. They are placed
in the world through TF (`world_frame` must be the TF frame that coincides with Gazebo's world frame). Other types
(lines, points, text, meshes) are skipped. `DELETE` and `DELETEALL` are honoured. A marker without an update for
`timeout` seconds of ROS time disappears, so the drawing never outlives its publisher. A marker's pose is updated
when the marker is, so a moving frame is followed at the publisher's rate.

Visibility: with `visibility` `gui` (the default) no Gazebo camera sensor draws the visuals, so a robot's cameras
(and the perception behind them) never see them. Only gzclient does, plus the camera sensors whose scoped names
contain one of the `show_in_cameras` tokens (their viewport's visibility mask is widened by Gazebo's GUI bit, e.g.
an overview camera that records videos). With `all`, every camera draws them. Gazebo 11 ignores SDF
`visibility_flags`/`visibility_mask`, which is why the plugin sets them itself.

## Use

```
roslaunch gazebo_marker_visuals marker_visuals.launch name:=my_markers topic:=/my_node/markers \
    [world_frame:=map] [visibility:=gui|all] [show_in_cameras:="overview_camera_sensor"] [namespaces:="ns1 ns2"] \
    [timeout:=2.0]
```

The launch file spawns a static model `name` with one visual-only link whose visual carries the plugin
(`sdf/marker_visuals.sdf.xacro`); Gazebo must be running. Several drawings need different names.

## Parameters (children of the `<plugin>` element)

| element | default | meaning |
| --- | --- | --- |
| `topic` | `markers` | the MarkerArray topic (relative names resolve in the root namespace) |
| `world_frame` | `map` | TF frame that coincides with Gazebo's world frame |
| `visibility` | `gui` | `gui`: gzclient and `show_in_cameras` only; `all`: every camera |
| `show_in_cameras` | (empty) | space separated tokens; a camera sensor whose scoped name contains one draws `gui` visuals |
| `namespaces` | (empty) | space separated marker namespaces to draw; empty: all |
| `timeout` | `2.0` | seconds without an update after which a marker disappears; 0: never |

## Cost

All work happens in the rendering thread before each frame: the markers that arrived since the last frame are
applied; a visual is touched only when its marker's look changed (a marker sent again unchanged is not redrawn) or
its frame moved in the world, and then its colour and scale only when they changed. Nothing runs in the physics
update. A scene that renders nothing (a headless gzserver whose camera sensors have no subscriber) does no work
beyond queuing the messages.

## Notes

- gzclient has no ROS node of its own: the plugin initialises one there (anonymous name). gzclient then needs a ROS
  master, as gzserver with gazebo_ros does.
- The test (`catkin_make run_tests_gazebo_marker_visuals`) covers the marker bookkeeping and the pose composition;
  the drawing itself is checked in a running simulation.
