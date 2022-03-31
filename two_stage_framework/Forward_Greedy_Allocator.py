import numpy as np
import copy

from Set import Set
from Greedy_Allocator import Greedy_Allocator

class Forward_Greedy_Allocator(Greedy_Allocator):
    def __init__(self,function_frame):
        super().__init__(function_frame)
        self.algorithm_name="Forward_Greedy"
        S_r_0=Set([])
        self.set_up(S_r_0)

    def solve_problem(self):
        N=len(self.tasks)
        forward_greedy_solution=super().solve_problem(N)
        return forward_greedy_solution

    def make_step(self,V_k_1,R_k_1):
        rho_F_vec,f_F_vec,t_F_vec=self.collect_bets(V_k_1,R_k_1)
        i_r_k=np.argmin(rho_F_vec)

        r_k=self.robots[i_r_k]
        a_k=r_k.a_r
        self.history.add((r_k.id,a_k.id))

        r_k.S_r.add(a_k)
        r_k.f_r=r_k.f_r-r_k.rho_r
        V_k=V_k_1.remove(a_k)
        R_k=Set([r for r in self.robots if r.a_r==a_k])

        """
        print("\n Step:\n")
        print("   - rho_F_vec=",rho_F_vec)
        print("   - f_F_vec=",f_F_vec)
        print("   - t_F_vec=",t_F_vec)
        print("   - (a_k,r_k)=(",str(a_k.id),",",str(r_k.id),")")
        """
        return V_k,R_k

    def place_bet(self,r,V_k_1):
        rho_r=float('inf')
        a_r=None
        bet_time=0
        for a in V_k_1:
            S_r_a=copy.copy(r.S_r).add(a)
            f_r_a,time_f_r_a=r.objective.get_value(S_r_a)
            rho_r_a=r.f_r-f_r_a
            if rho_r_a<rho_r:
                rho_r=rho_r_a
                a_r=a
            bet_time=bet_time+time_f_r_a
        r.rho_r=rho_r
        r.a_r=a_r
        return bet_time

    def get_alpha_G(self):
        K=len(self.tasks)
        return super().get_alpha_G(K)

    def get_gamma_G(self):
        K=len(self.tasks)
        return super().get_gamma_G(K)