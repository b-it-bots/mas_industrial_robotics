class RearPlatformFullError(Exception):
    pass


class RearPlatform(object):

    """
    Rear platform abstraction to keep the state.
    """

    def __init__(self):
        self.locations = {'left': None, 'middle': None, 'right': None}

    def get_free_location(self):
        for k in self.locations.keys():
            if self.locations[k] is None:
                return k
        raise Exception('Rear platform is full')

    def store_object(self, location, object_name='unknown'):
        assert location in self.locations.keys()
        assert self.locations[location] == None
        self.locations[location] = object_name
