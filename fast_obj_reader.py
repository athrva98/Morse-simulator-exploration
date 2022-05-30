# Testing the read in times of various .obj import mechanisms.
import pandas as pd
import numpy as np


path = r'/home/athrva/Desktop/morse_ros/superEnvironment_0.obj'

def read_obj(path):
    objdf = pd.read_csv(path, delimiter = ' ', header = None)
    objdf.head()
read_obj(path)