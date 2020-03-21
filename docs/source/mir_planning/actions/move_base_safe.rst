.. _mir_move_base_safe:

Move base safe
==============

Move robot base from a known/mapped location to another known/mapped location while 
looking for barrier tape.


Goal parameter description
--------------------------

- ``destination_location``: name of known/mapped location (e.g. ``WS01``, ``SH02``, ``PP01``) 
- ``dont_be_safe``: bool determining if barrier tape detection should be used or
  not (e.g. ``true``, ``false``)
- ``arm_safe_position``: joint configuration name for arm while the base is in
  motion. This is used for barrier tape detection. (e.g. ``barrier_tape``, ``folded``)
