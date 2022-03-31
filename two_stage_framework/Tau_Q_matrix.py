import numpy as np

from Set import Set
from Matrix import Matrix

class Tau_Q_matrix(Matrix):
    def __init__(self,path_planner):
        super(Tau_Q_matrix,self).__init__(None,None)
        self.Q=path_planner.Q
        self.X=path_planner.X
        self.QX=path_planner.QX
        self.domain=[self.Q,self.QX]
        self.generate_Tau_Q()

    def generate_Tau_Q(self):
        self.matrix=np.zeros(tuple([len(e) for e in self.domain]))
        for i,q_i in enumerate(self.Q):
            for j,qx_j in enumerate(self.QX):
                q_j,x_j=qx_j
                if q_j==set([x_j]).intersection(set(self.Q.targets)).union(q_i):
                    self.matrix[i,j]=True

    def find_transition(self,q_k_1,x_k):
        q_k_1_ind=self.Q.index(q_k_1)
        poss_qx_k_list=[e for i,e in enumerate(self.QX) if self.matrix[q_k_1_ind,i]]
        for poss_qx_k in poss_qx_k_list:
            poss_q_k,poss_x_k=poss_qx_k
            if poss_x_k==x_k:
                q_k=poss_q_k
                break
        return q_k