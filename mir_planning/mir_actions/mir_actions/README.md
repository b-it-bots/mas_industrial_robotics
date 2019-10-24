# mir_actions

A bunch of action servers for performing basic robocup actions.

List of basic actions
- move base
- pick
- perceive
- place
- stage
- unstage

The action server are SMACH state machines which are wrapped with
`ActionServer`. An `ActionClient` needs to send request using
`GenericExecuteGoal`. This request message contains a single dictionary kind of
message called `parameters` of type `diagnostic_msgs/KeyValue[]`.

Each server needs different information from this request message. Please see
the following files for detailed info:
- [mir_move_base_safe](../mir_move_base_safe/README.md)
- [mir_perceive_location](../mir_perceive_location/README.md)
- [mir_perceive_cavity](../mir_perceive_cavity/README.md)
- [mir_pick_object](../mir_pick_object/README.md)
- [mir_place_object](../mir_place_object/README.md)
- [mir_stage_object](../mir_stage_object/README.md)
- [mir_unstage_object](../mir_unstage_object/README.md)
- [mir_insert_object](../mir_insert_object/README.md)
- [mir_insert_cavity](../mir_insert_cavity/README.md)
Additionally, `mir_planner_executor` also sends `next_action` as one of the
parameter. This can be used by action servers to have parallel execution of arm
while the base is in motion to save som time. At the moment, only
`mir_move_base_safe` is using this information.

This package also contains `Utils.py` which contains utility functions for
action servers.

## Dependencies

- `mir_planning_msgs`
- `mir_states`
- `mcr_states`

## Usage

```
roscore
roslaunch mir_actions run_action_servers.launch
```
