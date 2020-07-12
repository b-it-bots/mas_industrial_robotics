# mir_perceive_aruco

Perceive an aruco cube on a workstation

Related to: `mir\_perceive\_aruco\_cube`

## Goal parameter description

- `location`: name of known/mapped location (e.g. `WS01`, `SH02`, `PP01`)

## Test

Make sure `planning_bringup` is already launched.
```
roslaunch mir_perceive_mock perceive_aruco_server.launch
rosrun mir_perceive_mock perceive_aruco_client_test
```
