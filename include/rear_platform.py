class RearPlatformFullError(Exception):
    pass


class RearPlatformEmptyError(Exception):
    pass


class RearPlatform(object):

    """
    Rear platform abstraction to keep the state.
    """

    def __init__(self):
        self.locations = {'left': None, 'middle': None, 'right': None}

    @property
    def at(self, location):
        assert location in self.locations.keys()
        return self.locations[location]

    @property
    def empty(self):
        return not any(self.locations.values())

    def get_free_location(self):
        for k in self.locations.keys():
            if self.locations[k] is None:
                return k
        raise RearPlatformFullError('Rear platform is full')

    def get_occupied_location(self):
        for k in self.locations.keys():
            if self.locations[k] is not None:
                return k
        raise RearPlatformEmptyError('Rear platform is empty')

    def get_free_locations(self):
        return [l for l in self.locations.keys()
                if self.locations[l] is None]

    def get_occupied_locations(self):
        return [l for l in self.locations.keys()
                if self.locations[l] is not None]

    def store_object(self, location=None, object_name='unknown'):
        location = location or self.get_free_location()
        self.locations[location] = object_name

    def retrieve_object(self, location=None):
        location = location or self.get_occupied_location()
        obj, self.locations[location] = self.locations[location], None
        return obj
