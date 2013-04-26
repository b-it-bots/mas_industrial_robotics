from bnt_task import BNTTask
from bmt_task import BMTTask
from btt_task import BTTTask
from ppt_task import PPTTask
from task import TaskSpecFormatError


def parse_task(task_type, task_spec):
    TYPE_MAP = {'BNT': BNTTask,
                'BMT': BMTTask,
                'BTT': BTTTask,
                'PPT': PPTTask}
    return TYPE_MAP[task_type](task_spec)
