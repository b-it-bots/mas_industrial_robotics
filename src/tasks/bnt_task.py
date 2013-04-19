import re
from task import Task


class BNTTask(Task):

    """
    Basic navigation test task.

    Example task specification:

    'BNT<(D1,W,1),(S1,E,3),(S2,E,3),(D2,S,3),(S3,W,3),(S2,W,3),(D2,W,3)>'
    """

    NAME = 'BNT'

    def __init__(self, spec):
        Task.__init__(self, spec)

    def _parse(self, spec):
        self.subtasks = re.findall(r'\('
                                   '(?P<place>.+?),'
                                   '(?P<orientation>[NESW]),'
                                   '(?P<break>[123])'
                                   '\)', spec)

    def __str__(self):
        r = ['Place: %s | Orientation: %s | Break: %s' % s
             for s in self.subtasks]
        return '\n'.join(r)
