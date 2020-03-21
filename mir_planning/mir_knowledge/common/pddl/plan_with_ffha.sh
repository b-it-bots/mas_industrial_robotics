#!/bin/bash

# the plan to use (executable)
planner="rosrun ffha ffha"

# pddl inputs to the planner
domain="${1}/domain.pddl"

# Plan!
$planner -o $domain -f ${1}/problems/p${2}.pddl
