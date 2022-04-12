import numpy as np

from Matrix import Matrix

class p_H_k_matrix(Matrix):
    X=None
    y_sampler=None
    domain=None

    def __init__(self,k):
        super(p_H_k_matrix,self).__init__(None,None)
        self.k=k
        self.generate_p_H_k()

    @staticmethod
    def set_up(path_planner):
        p_H_k_matrix.X=path_planner.X
        p_H_k_matrix.y_sampler=path_planner.y_sampler
        p_H_k_matrix.domain=p_H_k_matrix.generate_domain(path_planner)

    @staticmethod
    def generate_domain(path_planner):
        domain=[path_planner.X,path_planner.X]
        return domain

    def generate_p_H_k(self):
        E_k_1=self.y_sampler.episodes[:,self.k-1-1,:]
        E_k=self.y_sampler.episodes[:,self.k-1,:]
        num_mat=(~E_k_1.T).dot(E_k.astype(int))
        den_mat=np.tile(np.sum(~E_k_1.T,axis=1).reshape((E_k.shape[1],1)),(1,E_k.shape[1]))
        self.matrix=self.X.adj_matrix*self.mat_div(num_mat,den_mat)

    @staticmethod
    def mat_div(num_mat,den_mat):
        div=np.true_divide(num_mat,den_mat)
        div[np.isnan(div)]=1
        return div