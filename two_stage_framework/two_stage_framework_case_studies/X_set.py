import itertools
import numpy as np
from numpy import linalg

from Set import Set

class X_set(Set):
    def __init__(self,path_planner):
        parameters=path_planner.parameters
        self.size=parameters.size
        self.obsticles=parameters.obsticles
        X_f=self.generate_X_f()
        super(X_set,self).__init__(X_f)
        self.adj_matrix,self.adj_diag_matrix=self.generate_adj_matrix()

    def generate_X_f(self):
        size=self.size
        obsticles=self.obsticles
        X_all=[e for e in itertools.product(range(size[0]),range(size[1]))]
        X_f=[e for e in X_all if e not in obsticles]
        return X_f

    def generate_adj_matrix(self):
        X_f=self
        n=len(X_f)
        adj_matrix=np.eye(n,dtype=bool)
        adj_diag_matrix=np.eye(n,dtype=bool)
        for i,x_i in enumerate(X_f):
            for j,x_j in enumerate(X_f):
                if linalg.norm(np.array(x_i)-np.array(x_j),1)==1:
                        adj_matrix[i,j]=True
                if np.square(np.array(x_i)-np.array(x_j)).sum()==2:
                        adj_diag_matrix[i,j]=True
        return adj_matrix,adj_diag_matrix

    def get_N_x_(self,x):
        id_x=self.index(x)
        N_x_=[e for i,e in enumerate(self) if self.adj_matrix[id_x,i]]
        return N_x_

    def get_D_x_(self,x):
        id_x=self.index(x)
        D_x_=[e for i,e in enumerate(self) if self.adj_diag_matrix[id_x,i]]
        return D_x_

    def is_reachable(self,x,xx):
        i_x=self.index(x)
        i_xx=self.index(xx)
        return self.adj_matrix[i_x,i_xx]
