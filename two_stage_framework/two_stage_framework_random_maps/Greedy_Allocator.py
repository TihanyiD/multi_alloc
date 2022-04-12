import pandas as pd
import numpy as np
import itertools
import copy

from Set import Set
from Robot_objective import Robot_objective
from Allocation import Allocation
from Task_Allocator import Task_Allocator
from Allocator_Solution import Allocator_Solution
from ProgressBar import ProgressBar
from Instrument import Instrument

class Greedy_Allocator(Task_Allocator):
    class Greedy_Allocator_Solution(Allocator_Solution):
        def __init__(self,greedy_allocator):
            algorithm=greedy_allocator.algorithm_name
            allocation=greedy_allocator.get_allocation()
            objective_value=allocation.get_value()
            time_data=greedy_allocator.instrument.data
            super().__init__(algorithm,allocation,objective_value,time_data)

    def __init__(self,function_frame):
        super().__init__(function_frame)
        for r in self.robots:
            r.objective=Robot_objective(r,self.function_frame)

    def set_up(self,S_r_0):
        print("--- ",self.algorithm_name," ---\n")
        print("Setting up task allocator...")
        self.instrument.start()
        setup_time=0
        for r in self.robots:
            r.S_r=copy.copy(S_r_0)
            f_r,time_f_r=r.objective.get_value(r.S_r,print_progress=True)
            r.f_r=f_r
            setup_time=setup_time+time_f_r
        self.history=Set([])
        setup_time=setup_time+self.instrument.stop()
        self.instrument.save_measurement("setup_time",setup_time)
        print("Finished setting up task allocator!\n")

    def solve_problem(self,N):
        print("Solving allocation problem...")
        self.instrument.start()
        V_k_1=copy.copy(self.tasks)
        R_k_1=copy.copy(self.robots)

        progress_bar=ProgressBar(N)
        self.instrument.save_measurement("calculation_time",0)
        for k in range(N):
            progress_bar.progress(k)
            V_k,R_k=self.make_step(V_k_1,R_k_1)
            V_k_1=V_k
            R_k_1=R_k
        progress_bar.progress(N,'Finished!\n')
        calculation_time=self.instrument.read_measurement("calculation_time")+self.instrument.stop()
        self.instrument.save_measurement("calculation_time",calculation_time)
        greedy_solution=self.Greedy_Allocator_Solution(self)
        print("Finished solving allocation problem!\n")
        return greedy_solution

    def make_step(self,V_k_1,R_k_1):
        pass

    def collect_bets(self,V_k_1,R_k_1):
        max_bet_time=-float('inf')
        for r in R_k_1:
            bet_time=self.place_bet(r,V_k_1)
            if bet_time>max_bet_time:
                max_bet_time=bet_time
        calculation_time=self.instrument.read_measurement("calculation_time")+max_bet_time
        self.instrument.save_measurement("calculation_time",calculation_time)

        n_R=len(self.robots)
        f_r_vec=np.zeros(n_R)
        rho_r_vec=np.zeros(n_R)
        for i_r,r in enumerate(self.robots):
            f_r_vec[i_r]=r.f_r
            rho_r_vec[i_r]=r.rho_r

        rho_F_vec=np.zeros(n_R)
        f_F_vec=np.zeros(n_R)
        t_F_vec=[]
        for i_r,r in enumerate(self.robots):
            rho_F_vec[i_r]=rho_r_vec[i_r]
            f_F_vec[i_r]=f_r_vec[i_r]-rho_r_vec[i_r]
            if r.a_r is not None:
                t_F_vec.append(r.a_r.id)
            else:
                t_F_vec.append(None)
            for j_r in range(n_R):
                if j_r!=i_r:
                    rho_F_vec[i_r]=rho_F_vec[i_r]*f_r_vec[j_r]
                    f_F_vec[i_r]=f_F_vec[i_r]*f_r_vec[j_r]

        """
        print(self.get_allocation().get_printable())
        print("R_k_1:",[r.id for r in R_k_1])
        if r.a_r is not None:
            print("a_k_r:",[r.a_r.id for r in self.robots])
        print("V_k_1:",[t.id for t in V_k_1])
        """
        return rho_F_vec,f_F_vec,t_F_vec
    
    def get_allocation(self):
        allocation=Allocation([])
        for r in self.robots:
            for a in r.S_r:
                allocation.add(r.id,a.id)
        return allocation

    def postprocess_solution(self,solution):
        solution.alpha_G=self.get_alpha_G()
        solution.gamma_G=self.get_gamma_G()
        return super().postprocess_solution(solution)

    def get_alpha_G(self,K):
        robot_ids=Set([e.id for e in self.robots])
        task_ids=Set([e.id for e in self.tasks])
        V=Set.descartes_product(robot_ids,task_ids)
        V_K=Set([Set(e) for e in itertools.combinations(V,K)])
        S_K=self.history
        S_K_1=Set(self.history[:-1])
        
        print("...Calculating alpha_G...")
        alpha_G=-float('inf')
        N=len(V_K)
        progress_bar=ProgressBar(N)
        instrument=Instrument()
        instrument.start()
        k=0
        for Omega in V_K:
            progress_bar.progress(k)
            k=k+1
            for j_i in S_K_1.setminus(Omega):
                i=S_K.index(j_i)
                S_i_1=Allocation(self.history[0:i])
                lhs=Allocation(S_i_1.union(Omega)).get_derivative(Set([j_i]))
                rhs=S_i_1.get_derivative(Set([j_i]))
                frac=(lhs-rhs)/lhs
                if frac>alpha_G and rhs!=0 and lhs!=0:
                    alpha_G=frac
        progress_bar.progress(N,'Finished!\n')
        calculation_time=instrument.stop()
        print("Calculation time [s]:",calculation_time)
        print("Finished calculating alpha_G!\n")
        return alpha_G

    def get_gamma_G(self,K):
        robot_ids=Set([e.id for e in self.robots])
        task_ids=Set([e.id for e in self.tasks])
        V=Set.descartes_product(robot_ids,task_ids)
        V_K=Set([Set(e) for e in itertools.combinations(V,K)])

        print("...Calculating gamma_G...")
        gamma_G=float('inf')
        N=len(self.history)*len(V_K)
        progress_bar=ProgressBar(N)
        instrument=Instrument()
        instrument.start()
        k=0
        for t in range(len(self.history)):
            S_t=Allocation(self.history[0:t])
            for Omega in V_K:
                progress_bar.progress(k)
                k=k+1
                lhs=0
                for omega in Omega.setminus(Set(S_t)):
                    lhs=lhs+S_t.get_derivative(Set([omega]))
                rhs=S_t.get_derivative(Omega)
                frac=rhs/lhs 
                if frac<gamma_G and rhs!=0 and lhs!=0:
                    gamma_G=frac
        progress_bar.progress(N,'Finished!\n')
        calculation_time=instrument.stop()
        print("Calculation time [s]:",calculation_time)
        print("Finished calculating gamma_G!\n")
        return gamma_G