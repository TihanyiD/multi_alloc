import numpy as np

from Matrix import Matrix
from X_set import X_set
from U_x_function import U_x_function
from Tau_X_dmatrix import Tau_X_dmatrix
from y_sampler import y_sampler
from p_H_k_matrix import p_H_k_matrix
from Q_set import Q_set
from QX_set import QX_set
from S_set import S_set
from Tau_Q_matrix import Tau_Q_matrix
from Tau_S_k_dmatrix import Tau_S_k_dmatrix
from ProgressBar import ProgressBar

class Path_Planner(object):
    def __init__(self,parameters):
        self.parameters=parameters

    def set_up(self):
        print("Setting up path planner...\n")
        print("...Defining sets...")
        self.X=X_set(self)
        self.U_x=U_x_function(self)
        print("...Defining state transition dynamics...")
        self.Tau_X=Tau_X_dmatrix(self)
        self.y_sampler=y_sampler(self)
        print("...Preparing matrices for dynamic programming algorithm...")
        p_H_k_matrix.set_up(self)
        print("Finished setting up path planner!\n")

    def get_solution(self,targets,goal,x_0,print_progress=False):
        self.Q=Q_set(self,targets,goal)
        self.QX=QX_set(self)
        self.Tau_Q=Tau_Q_matrix(self)
        self.S=S_set(self)
        Tau_S_k_dmatrix.set_up(self)
        Mu,V=self.solve_problem(print_progress)
        if type(x_0) is list:
            V_ret=[V[0].get([(set([]),x_0_r)]) for x_0_r in x_0]
        else:
            V_ret=V[0].get([(set([]),x_0)])
        return V_ret,Mu,V

    def solve_problem(self,print_progress=False):
        N=self.parameters.N
        progress_bar=ProgressBar(N-1)

        V_k=self.get_V_N()
        Mu=[None]*(N-1)
        V=[None]*N
        V[N-1]=V_k
        k=N
        if print_progress:
            progress_bar.progress(N-k)
        while k>1:
            V_k_1,Mu_k_1=self.get_V_k_1(V_k,k)
            Mu[k-1-1]=Mu_k_1
            V[k-1-1]=V_k_1
            V_k=V_k_1
            k=k-1
            if print_progress:
                progress_bar.progress(N-k)
        if print_progress:
            progress_bar.progress(N-1,'Finished!\n')
        return Mu,V

    def get_V_N(self):
        V_N=Matrix([self.S],self.S.get_G_mask().astype(int))
        return V_N

    def get_V_k_1(self,V_k,k):
        V_k=V_k.matrix
        Tau_S_k=Tau_S_k_dmatrix(k)

        n_exp=len(Tau_S_k.domain_dict)
        V_k_1_exp=np.zeros(tuple(list(V_k.shape)+[n_exp]))
        for i_u,u in enumerate(Tau_S_k.domain_dict):
            V_k_1_exp[:,i_u]=Tau_S_k[u].dot(V_k)

        V_k_1=Matrix([self.S],np.amax(V_k_1_exp,axis=1))
        Mu_k_1_ids=np.argmax(V_k_1_exp,axis=1)
        Mu_k_1=Matrix([self.S],np.array([self.U_x.U[e] for e in Mu_k_1_ids.reshape((Mu_k_1_ids.shape[0],))]))
        
        return V_k_1,Mu_k_1

    def simulate_path(self,Mu,x_0,q_0):
        N=len(Mu)+1
        path=Matrix([list(range(N)),self.X],np.zeros((N,len(self.X)),dtype=bool))

        S_Mu=Mu[0].domain[0]

        q_k_1=q_0
        x_k_1=x_0
        path.set([0,x_k_1],True)
        for k in range(1,N):
            u_k_1=Mu[k-1].get([(q_k_1,x_k_1)])

            x_k=self.U_x.get_xx_u(x_k_1,u_k_1)
            q_k=S_Mu.Tau_Q.find_transition(q_k_1,x_k)

            q_k_1=q_k
            x_k_1=x_k
            path.set([k,x_k],True)
        return path

    def simulate_successful_episodes(self,path):
        episodes=self.y_sampler.episodes
        unsuccessful_episodes=path.matrix*episodes
        successful_episodes=~np.any(np.any(unsuccessful_episodes,axis=2),axis=1)
        return successful_episodes