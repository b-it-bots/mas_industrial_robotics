perceive action server
======================

Exposes an actionlib server to preceive cavities from a location.

1. pipeline:

base_placement -> move arm to look_at_workspace -> triggers cavity pipeline
-> move arm to look_straight_at_workspace_left -> triggers cavity pipeline
-> move arm to look_straight_at_workspace_right -> triggers cavity pipeline

2. input/output

input: location
output: object_list

3. a client is also provided inside this package to call the action server
