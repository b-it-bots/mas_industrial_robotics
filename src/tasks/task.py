import re


class TaskSpecFormatError(Exception):
    pass


class Task(object):

    """Base class for tasks specifications for different tests."""

    def __init__(self, spec):
        self._parse(self._unpack(spec))

    def _unpack(self, spec):
        """
        Unpack task specification.

        Removes outer task name and '<', '>'.
        If task name is not correct throws a TaskSpecFormatError.
        """
        inner = re.findall(r'%s<(.*)>' % self.NAME, spec)
        if len(inner) != 1:
            raise TaskSpecFormatError()
        return inner[0]

    def _parse(self, spec):
        """
        Parse inner part of task specification.

        This should be implemented in deriving classes.
        """
        raise NotImplementedError()
