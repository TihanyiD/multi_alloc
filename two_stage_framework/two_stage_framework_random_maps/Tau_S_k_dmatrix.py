import numpy as np

from Matrix import Matrix
from DMatrix import DMatrix
from p_H_k_matrix import p_H_k_matrix

class Tau_S_k_dmatrix(DMatrix):
    domain=None

    def __init__(self,k):
        super(Tau_S_k_dmatrix,self).__init__()
        self.k=k
        self.generate_Tau_S_k()

    @staticmethod
    def set_up(path_planner):
        Tau_S_k_dmatrix.S=path_planner.S
        Tau_S_k_dmatrix.domain_dict=path_planner.U_x.U
        Tau_S_k_dmatrix.domain_matrix=[path_planner.S,path_planner.S]

        Tau_S_k_dmatrix.Tau_Q=path_planner.Tau_Q
        Tau_S_k_dmatrix.Tau_X=path_planner.Tau_X
        Tau_S_k_dmatrix.Tau_QX_exp=Tau_S_k_dmatrix.generate_Tau_QX_exp()

        Tau_S_k_dmatrix.generate_masks()

    @staticmethod
    def generate_Tau_QX_exp():
        n_exp_Q=int(len(Tau_S_k_dmatrix.S.QX)/len(Tau_S_k_dmatrix.Tau_Q.domain[0]))
        Tau_Q_exp=np.kron(Tau_S_k_dmatrix.Tau_Q.matrix,np.ones((n_exp_Q,1)))

        Tau_QX_exp=DMatrix()
        Tau_QX_exp.set_domains(Tau_S_k_dmatrix.domain_dict,Tau_S_k_dmatrix.domain_matrix)
        n_exp_X=int(len(Tau_S_k_dmatrix.S.QX)/len(Tau_S_k_dmatrix.Tau_X.domain_matrix[0]))
        for u in Tau_S_k_dmatrix.Tau_X.keys():
            Tau_X_exp_u=np.kron(np.ones((n_exp_X,n_exp_X)),Tau_S_k_dmatrix.Tau_X[u].matrix)
            Tau_QX_exp[u]=Tau_Q_exp*Tau_X_exp_u
        return Tau_QX_exp

    @staticmethod
    def generate_masks():
        QX_mask=Tau_S_k_dmatrix.S.get_QX_mask()
        H_mask=Tau_S_k_dmatrix.S.get_H_mask()
        G_mask=Tau_S_k_dmatrix.S.get_G_mask()

        mask_QX_QX=np.outer(QX_mask,QX_mask)
        Tau_S_k_dmatrix.mask_QX_QX=mask_QX_QX

        mask_QX_H=np.outer(QX_mask,H_mask)
        Tau_S_k_dmatrix.mask_QX_H=mask_QX_H

        mask_H_S=np.zeros((len(Tau_S_k_dmatrix.S),len(Tau_S_k_dmatrix.S)),dtype=bool)
        mask_H_S[H_mask,:]=True
        Tau_S_k_dmatrix.mask_H_S=mask_H_S
        Tau_S_k_dmatrix.values_H_S=H_mask

        mask_G_S=np.zeros((len(Tau_S_k_dmatrix.S),len(Tau_S_k_dmatrix.S)),dtype=bool)
        mask_G_S[G_mask,:]=True
        Tau_S_k_dmatrix.mask_G_S=mask_G_S
        Tau_S_k_dmatrix.values_G_S=G_mask

    def generate_Tau_S_k(self):
        self.p_H_k=p_H_k_matrix(self.k)
        p_H_k_exp=self.expand_p_H_k()
        for u in self.domain_dict:
            values_QX_QX_u=self.generate_values_QX_QX_u(p_H_k_exp,u)
            values_QX_H_u=self.generate_values_QX_H_u(p_H_k_exp,u)

            matrix_u=self.assemble_matrix_u(values_QX_QX_u,values_QX_H_u)
            self[u]=matrix_u
            
    def expand_p_H_k(self):
        n_exp=int(len(self.S.QX)/self.p_H_k.matrix.shape[0])
        p_H_k_exp=np.kron(np.ones((n_exp,n_exp)),self.p_H_k.matrix)
        return p_H_k_exp

    def generate_values_QX_QX_u(self,p_H_k_exp,u):
        values_QX_QX_u_illegals=(1-p_H_k_exp)*self.Tau_QX_exp[u]
        values_QX_QX_u=self.remove_illegals(values_QX_QX_u_illegals)
        return values_QX_QX_u

    def generate_values_QX_H_u(self,p_H_k_exp,u):
        values_QX_H_u_illegals=np.sum(p_H_k_exp*self.Tau_QX_exp[u],axis=0)
        values_QX_H_u=self.remove_illegals(values_QX_H_u_illegals)
        return values_QX_H_u

    def remove_illegals(self,matrix):
        for axis,size in enumerate(matrix.shape):
            if size>1:
                matrix=np.delete(matrix,self.S.illegals,axis)
        return matrix

    def assemble_matrix_u(self,values_QX_QX_u,values_QX_H_u):
        matrix_u=np.zeros((len(self.S),len(self.S)))

        matrix_u[self.mask_QX_QX]=values_QX_QX_u.flatten()
        matrix_u[self.mask_QX_H]=values_QX_H_u.flatten()
        matrix_u[self.mask_H_S]=self.values_H_S
        matrix_u[self.mask_G_S]=self.values_G_S
        return matrix_u