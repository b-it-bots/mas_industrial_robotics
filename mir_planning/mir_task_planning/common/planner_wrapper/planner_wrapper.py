#! /usr/bin/env python
"""
This module calls different planners depending on the request with the provided
domain and problem file.
"""
from __future__ import print_function

import os
import subprocess
import sys
import time

import yaml

import rospkg


class PlannerWrapper(object):

    """A common planner wrapper which calls other planner wrapper based on
    what is needed with params from a config file

    :param planner_commands: configurations for different planner with their shell command
    :type planner_commands: dict
    :param plan_dir: directory path where the plan files should be generated (optional)
    :type plan_dir: string
    :param plan_backup_dir: directory path where the best plan's backup copy should be placed (optional)
    :type plan_backup_dir: string
    :param plan_file_name: name of the generated plan files (optional)
    :type plan_file_name: string
    :param time_limit: time limit for the planner (in seconds) (optional)
    :type time_limit: float
    """

    def __init__(self, planner_commands, **kwargs):
        self._planner_commands = planner_commands
        self._plan_dir = kwargs.get("plan_dir", "/tmp/plan")
        self._plan_backup_dir = kwargs.get("plan_backup_dir", "~/.ros")
        self._plan_file_name = kwargs.get("plan_file_name", "task_plan")
        self._time_limit = kwargs.get("time_limit", 5)
        self._rospack_obj = rospkg.RosPack()

        # replace ~ with full path (if present)
        self._plan_dir = os.path.expanduser(self._plan_dir)
        self._plan_backup_dir = os.path.expanduser(self._plan_backup_dir)
        # create directories if they don't exist
        if not os.path.isdir(self._plan_dir):
            os.makedirs(self._plan_dir)
        if not os.path.isdir(self._plan_backup_dir):
            os.makedirs(self._plan_backup_dir)

    def plan(self, planner, domain_file, problem_file, fast_mode=False):
        """
        Perform the following tasks (returns None if any one fails)
        - Call a planner with given domain and problem file using system call.
        - Clean up any unneccessary files generated by the plan.
        - Read the generated plan file and return the plan

        :param planner: name of the planner to be used
        :type planner: string
        :param domain_file: path to domain pddl file
        :type domain_file: string
        :param problem_file: path to problem pddl file
        :type problem_file: string
        :param fast_mode: should the planner stop once a plan file is generated\
                          or should it continue to optimise it until the time runs out
        :type fast_mode: bool
        :return: plan represented as list of strings
        :rtype: list (strings) | None

        """
        # validate the planning request
        if planner not in self._planner_commands:
            return None

        print("[planner_wrapper] Trying to plan...")
        command = self._get_valid_planner_command(
            planner, domain_file, problem_file
        )
        if not command:
            return None
        print(command)

        if self._run_command_in_tmp(command.split(), fast_mode):
            plan_file = self._find_correct_plan_file()
            if plan_file:
                print("[planner_wrapper] Plan call was success")

                # read plan file
                with open(plan_file) as file_object:
                    raw_plan = file_object.read().split("\n")
                    # remove comments
                    task_plan = [
                        line
                        for line in raw_plan
                        if len(line) > 0 and line[0] != ";"
                    ]

                # delete all files in plan directory
                self._clean_plan_dir()

                # take a backup of plan file in desired directory
                with open(
                    os.path.join(self._plan_backup_dir, "task_plan.plan"), "w"
                ) as file_obj:
                    file_obj.writelines(
                        ["%s\n" % action for action in task_plan]
                    )

                return task_plan
        print("[planner_wrapper] Plan call failed")
        print("\n\nHere is the output of the command: ", command, "\n\n\n")
        with open(
            os.path.join(self._plan_dir, "command_output.txt"), "r"
        ) as output_file:
            output_text = output_file.read()
            print(output_text)
        print("\n\n\n")
        return None

    def _run_command_in_tmp(self, command, fast_mode):
        """Changes the pwd to /tmp and runs the command in subprocess module
        If fast_mode is True, stop the execution of subprocess as soon as a
        plan file is generated.

        :command: list of strings
        :fast_mode: bool
        :returns: bool (success)

        """
        success = False
        try:
            pwd = os.getcwd()
            os.chdir(self._plan_dir)
            with open("command_output.txt", "w") as output_file:
                proc_obj = subprocess.Popen(
                    command, stdout=output_file, stderr=output_file
                )
                start_time = time.time()
                while (
                    proc_obj.poll() is None
                    and time.time() < start_time + 2 * self._time_limit
                ):
                    time.sleep(0.2)
                    # terminate the process if fast_mode and found a plan file
                    if fast_mode and self._plan_file_exists():
                        proc_obj.terminate()
                        break
                if proc_obj.poll() is None:
                    print("[planner_wrapper] Terminating process manually")
                    proc_obj.kill()
            with open("command_output.txt", "r") as output_file:
                output_text = output_file.read()
                if len(output_text) > 0:
                    success = True
            os.chdir(pwd)
        except Exception as e:
            print(
                "[planner_wrapper] Encountered following error while executing command\n"
                + str(e)
            )
        return success

    def _find_correct_plan_file(self):
        """Find the latest plan file if multiple files are generated and return it.
        :returns: string (plan file path)

        """
        files_list = os.listdir(self._plan_dir)
        plan_file_list = [
            filename for filename in files_list if "task_plan" in filename
        ]
        if len(plan_file_list) == 0:
            print("[planner_wrapper] No plan files found.")
            return None
        best_plan = sorted(
            plan_file_list, key=lambda x: int(x.split(".")[-1])
        )[-1]
        return os.path.join(self._plan_dir, best_plan)

    def _clean_plan_dir(self):
        """Remove all files that are unnecessary.
        :returns: None

        """
        ls_output = os.listdir(self._plan_dir)
        for text_file in ls_output:
            try:
                os.remove(os.path.join(self._plan_dir, text_file))
            except Exception as e:
                print(
                    "[planner_wrapper] Encountered following error while cleaning plan dir\n"
                    + str(e)
                )

    def _get_valid_planner_command(self, planner, domain_file, problem_file):
        """Return a valid shell command string to invoke planner

        :planner: TODO
        :returns: TODO

        """
        command = None
        try:
            command = self._planner_commands[planner]["command"]
            # tell planner to use this file name where the plans are stored
            command = command.replace("FILENAME", self._plan_file_name)
            # give the domain file name to the planner
            command = command.replace("DOMAIN", domain_file)
            # give the problem file name to the planner
            command = command.replace("PROBLEM", problem_file)
            # give the time limit for planner
            command = command.replace("TIMELIMIT", str(self._time_limit))

            pkg_path = self._rospack_obj.get_path(
                self._planner_commands[planner]["rospkg_name"]
            )
            executable = os.path.join(
                pkg_path, self._planner_commands[planner]["executable_path"]
            )
            # use executable of planner
            command = command.replace("EXECUTABLE", executable)
        except Exception as e:
            print(
                "[planner_wrapper] Encountered following error while creating command\n"
                + str(e)
            )
        return command

    def _plan_file_exists(self):
        """Check if a plan file exists in plan dir or not.
        :returns: bool

        """
        for file_name in os.listdir(self._plan_dir):
            if self._plan_file_name in file_name:
                return True
        return False


# ==============================================================================
# Functions to use this class without ROS
# ==============================================================================


def get_planner_commands():
    """
    read the planner commands config file if possible. Returns None if fails

    :return: dict containing configuration for different planner commands
    :rtype: dict | None

    """
    code_dir = os.path.abspath(os.path.dirname(__file__))
    common_dir = os.path.dirname(code_dir)
    main_dir = os.path.dirname(common_dir)
    config_file = os.path.join(main_dir, "ros/config/planner_commands.yaml")
    planner_commands = None
    try:
        with open(config_file, "r") as file_handle:
            data = yaml.load(file_handle)
            planner_commands = data["planner_commands"]
    except Exception as e:
        pass
    return planner_commands


def get_domain_and_problem_file():
    """
    Get the file path of the domain and problem pddl file. Returns None if fails.

    :return: domain file and problem file
    :rtype: dict | None

    """
    code_dir = os.path.abspath(os.path.dirname(__file__))
    common_dir = os.path.dirname(code_dir)
    domain_file = os.path.join(common_dir, "pddl/domain.pddl")
    problem_file = os.path.join(common_dir, "pddl/problem.pddl")
    files = {"domain": None, "problem": None}
    try:
        with open(domain_file, "r") as file_handle:
            _ = file_handle.read()
        files["domain"] = domain_file
        with open(problem_file, "r") as file_handle:
            _ = file_handle.read()
        files["problem"] = problem_file
    except Exception as e:
        pass
    return files


if __name__ == "__main__":
    # try to read the planner commands config file and domain and problem file
    PLANNER_COMMANDS = get_planner_commands()
    FILES = get_domain_and_problem_file()
    if not PLANNER_COMMANDS:
        print("Could not read planner commands config file")
        sys.exit(1)
    if None in FILES.values():
        print("Could not read domain or problem file")
        sys.exit(1)

    # initialise planner wrapper object
    PLANNER_WRAPPER = PlannerWrapper(PLANNER_COMMANDS)

    # try to plan with the default problem and domain file
    task_plan = PLANNER_WRAPPER.plan("lama", FILES["domain"], FILES["problem"])
    if task_plan:
        print(len(task_plan))
