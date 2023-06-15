import csv
import time
import numpy as np
import csv

class CSVHandler():
    """
    Contains common CSV usage files to use across other classes.
    """

    def read_csv(self, path):
        """
        Reads a CSV specified with 'path' and returns an array
        containing the data.
        """

        # open the specified csv
        with open(path) as csv_file:

            # create a reader for the csv
            csv_reader = csv.reader(csv_file, delimiter=',')

            # skip first (title) line
            next(csv_reader)

            # initialise array to hold csv data
            data = []

            # add each row to the data array
            for row in csv_reader:
                data.append([float(value) for value in row[1:5]])

            
            return data
        



def clear_csv(file_path):
    """
    Delete all preexisting data from a file.
    """
    with open(file_path, 'w') as file:
        file.truncate(0)
        
        
        
        
        
        
        
        