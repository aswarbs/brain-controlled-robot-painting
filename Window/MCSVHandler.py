import csv
import time
import numpy as np
import csv

class CSVHandler():
    """
    Contains common CSV usage files to use across other classes.
    """

        



def clear_csv(file_path):
    """
    Delete all preexisting data from a file.
    """
    with open(file_path, 'w') as file:
        file.truncate(0)
        
        
        
        
        
        
        
        