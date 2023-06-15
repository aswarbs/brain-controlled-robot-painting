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
        

    def read_live_data(self, inlet, max_samples_calculated):
        """
        Read live data from the Brain Sensor, row by row.
        Return the current row of brain data from the sensor.
        """

        # Getting Data
        try:
            
            # Obtain EEG data from the LSL stream
            eeg_data, timestamp = inlet.pull_chunk(timeout = 1, max_samples = max_samples_calculated)

            # Separate the data into channels
            ch_data_tp9 = np.array(eeg_data)[:, 0]
            ch_data_af7 = np.array(eeg_data)[:, 1]
            ch_data_af8 = np.array(eeg_data)[:, 2]
            ch_data_tp10 = np.array(eeg_data)[:, 3]

            # Place this data into an array
            row_data = [np.mean(ch_data_tp9), np.mean(ch_data_af7), np.mean(ch_data_af8), np.mean(ch_data_tp10)]

            return row_data
                    
        except KeyboardInterrupt:
            print('Closing!')



def clear_csv(file_path):
    """
    Delete all preexisting data from a file.
    """
    with open(file_path, 'w') as file:
        file.truncate(0)
