import re
from task import Task, TaskSpecFormatError


class BMTTask(Task):

    """
    Basic manipulation test task.

    Example task specification:

    'BMT<D2,D2,D2,line(R20,M20_100,F20_20_B),T4>'
    """

    NAME = 'BMT'

    def __init__(self, spec):
        Task.__init__(self, spec)

    def _parse(self, spec):
        groups = re.match('(?P<init>.+?),'
                          '(?P<src>.+?),'
                          '(?P<dest>.+?),'
                          '(?P<cfg>.+?)\((?P<objects>.*?)\),'
                          '(?P<final>.+?)$', spec)
        if groups is None:
            raise TaskSpecFormatError()
        else:
            groups = groups.groupdict()
        self.init = groups['init']
        self.src = groups['src']
        self.dest = groups['dest']
        self.final = groups['final']
        self.cfg = groups['cfg']
        self.objects = list()
        for t in groups['objects'].split(','):
            if t == 'V20':
                t = 'R20'
            self.objects.append(t)

    def __str__(self):
        return ('Initial place: %s\n'
                'Source location: %s\n'
                'Destination location: %s\n'
                'Final place: %s\n'
                'Object configuration: %s\n'
                'Objects:%s') % (self.init, self.src, self.dest, self.final,
                                 self.cfg, '\n - '.join([''] + self.objects))
