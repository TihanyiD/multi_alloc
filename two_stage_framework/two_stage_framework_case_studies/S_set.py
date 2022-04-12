import numpy as np
from Set import Set

class S_set(Set):
    def __init__(self,path_planner):
        self.Q=path_planner.Q
        self.QX=path_planner.QX
        self.Tau_Q=path_planner.Tau_Q
        self.illegals=self.generate_illegals()
        self.s_H='s_H'
        self.s_G=(set(self.Q.targets),self.Q.goal)
        S=self.generate_S()
        super(S_set,self).__init__(S)

    def generate_illegals(self):
        illegals=[]
        for q in self.Q:
            for x in self.Q.targets:
                if x not in q:
                    qx=(q,x)
                    illegals=illegals+[self.QX.index(qx)]
        return illegals

    def generate_S(self):
        QX_legal=[e for i,e in enumerate(self.QX) if i not in self.illegals]
        S=[self.s_H]+QX_legal
        return S 

    def get_QX_mask(self):
        QX_inds=np.ones(len(self),dtype=bool)
        QX_inds[self.index(self.s_H)]=False
        return QX_inds

    def get_H_mask(self):
        H_inds=np.zeros(len(self),dtype=bool)
        H_inds[self.index(self.s_H)]=True
        return H_inds

    def get_G_mask(self):
        G_inds=np.zeros(len(self),dtype=bool)
        G_inds[self.index(self.s_G)]=True
        return G_inds