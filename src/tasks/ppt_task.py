import re
from task import Task, TaskSpecFormatError


class PPTTask(Task):

    """
    Precise placement test task.

    Example task specification:

    'PPT<S1,S2>'
    """

    NAME = 'PPT'

    def __init__(self, spec):
        Task.__init__(self, spec)

    def _parse(self, spec):
        groups = re.match(r'^(?P<src>.+?),(?P<dest>.+?)$', spec)
        if groups is None:
            raise TaskSpecFormatError()
        else:
            groups = groups.groupdict()
        self.src = groups['src']
        self.dest = groups['dest']

    def __str__(self):
        return ('Source location: %s\n'
                'Destination location: %s') % (self.src, self.dest)
