import numpy as np

from Matrix import Matrix
from DMatrix import DMatrix

class Tau_X_dmatrix(DMatrix):
    def __init__(self,path_planner):
        self.X=path_planner.X
        self.U_x=path_planner.U_x
        self.p_stay=path_planner.parameters.p_stay
        super(Tau_X_dmatrix,self).__init__()
        self.set_domains(self.U_x.U,[self.X,self.X])
        path_planner.parameters.generate_Tau_X(self)