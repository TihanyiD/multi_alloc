import pandas as pd
import numpy as np
import time 

class Instrument(object):
    t_start=0
    t_end=0
    t_elapsed=0

    def __init__(self):
        self.data=dict()

    def save_measurement(self,name,value):
        self.data[name]=value

    def read_measurement(self,name):
        return self.data[name]
    
    def start(self):
        self.t_start=time.time()

    def stop(self):
        self.t_end=time.time()
        self.t_elapsed=self.t_end-self.t_start
        return self.t_elapsed