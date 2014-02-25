import re
from task import Task, TaskSpecFormatError


class BTTTask(Task):

    """
    Basic transportation test task.

    Example task specification:

    'BTT<initialsituation(<S3,(F20_20_G,M20_100)><S2,(V20,R20)>);goalsituation(<S2,line(M20_100,F20_20_G)><S3,zigzag(R20,V20)>)>'

    TODO: does not respect object layout specification.
    """

    NAME = 'BTT'

    def __init__(self, spec):
        Task.__init__(self, spec)

    def _parse(self, spec):
        groups = re.match('^initialsituation\((?P<init>.+?)\);'
                          'goalsituation\((?P<goal>.+?)\)$', spec)
        if groups is None:
            raise TaskSpecFormatError()
        for k in ['init', 'goal']:
            self.__dict__[k] = dict()
            subtasks = re.findall(r'<(.+?)>', groups.groupdict()[k])
            for st in subtasks:
                g = re.match(r'(?P<platform>.+?),'
                              '(?P<cfg>.+?)?\((?P<objects>.+?)\)', st)
                if g is None:
                    raise TaskSpecFormatError()
                gd = g.groupdict()
                self.__dict__[k][gd['platform']] = gd['objects'].split(',')

    def __str__(self):
        i = ['  %s: %s' % (k, ', '.join(self.init[k])) for k in self.init]
        g = ['  %s: %s' % (k, ', '.join(self.goal[k])) for k in self.goal]
        return ('Initial situation:\n%s\n'
                'Goal situation:\n%s\n') % ('\n'.join(i), '\n'.join(g))
