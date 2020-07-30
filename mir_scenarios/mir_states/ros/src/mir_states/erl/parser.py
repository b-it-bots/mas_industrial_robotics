import glob

import pandas as pd


class ParserDataset:

    """
    Load files and store object
    set in dictionary
    """

    def load_files(self):

        # dictionary to store all object from all files
        self.objects = {}

        for file in glob.glob("./*.csv"):
            data = pd.read_csv(file, index_col=0, squeeze=True, header=0).to_dict()
            self.objects.update(data)
        # print (self.objects['DISTANCE_TUBE'])

        return self.objects


p = ParserDataset()
obj = p.load_files()
print obj["S40_40_B"]
