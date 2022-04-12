import numpy as np
import copy

from Allocation import Allocation
from Drawer import Drawer
from Instrument import Instrument

class Task_Allocator(object):
    def __init__(self,function_frame):
        Allocation.set_up(function_frame)
        self.function_frame=function_frame
        self.path_planner=function_frame.path_planner
        self.robots=copy.deepcopy(function_frame.robots)
        self.tasks=copy.deepcopy(function_frame.tasks)
        self.instrument=Instrument()

    def solve_problem(self):
        pass

    def get_allocation(self):
        pass

    def get_objective_value(self):
        pass

    def postprocess_solution(self,solution):
        solution=self.add_optimal_policies(solution)
        solution=self.add_optimal_paths(solution)
        solution=self.add_group_objective(solution)

    def show_solution(self,solution):
        solution.print_solution()
        self.draw_solution(solution)

    def add_optimal_policies(self,solution):
        print("...Calculating optimal policies...")
        allocation=solution.allocation.get_printable()
        Mu=[]
        V=[]
        V_ret=[]
        for r in self.robots:
            allocation_r=allocation.loc["robot_"+str(r.id)].to_numpy(dtype=bool)
            S_r=self.tasks.subset(allocation_r)

            targets_r=[e.target for e in S_r]
            goal_r=r.goal
            x_0_r=r.x_0

            V_ret_r,Mu_r,V_r=self.path_planner.get_solution(targets_r,goal_r,x_0_r,print_progress=True)
            Mu=Mu+[Mu_r]
            V=V+[V_r]
            V_ret=V_ret+[V_ret_r]        
        solution.Mu=Mu
        solution.V=V
        solution.V_ret=V_ret
        print("Finished calculating optimal policies!\n")
        return solution

    def add_optimal_paths(self,solution):
        print("...Calculating paths...")
        path=[]
        for i_r,r in enumerate(self.robots):
            Mu_r=solution.Mu[i_r]
            x_0_r=r.x_0
            path_r=self.path_planner.simulate_path(Mu_r,x_0_r,set([]))
            path=path+[path_r]
        solution.path=path
        print("Finished calculating paths!\n")
        return solution

    def add_group_objective(self,solution):
        print("...Calculating group objective...")
        
        E=self.path_planner.parameters.E
        group_successful_episodes=np.ones(E,dtype=bool)
        for i_r,r in enumerate(self.robots):
            path_r=solution.path[i_r]
            successful_episodes_r=self.path_planner.simulate_successful_episodes(path_r)
            group_successful_episodes=group_successful_episodes*successful_episodes_r
        group_objective_value=sum(group_successful_episodes)/E
        solution.objective_value={"multiplicative":solution.objective_value,"group":group_objective_value}
        print("Finished calculating group objective!\n")
        return solution

    def draw_solution(self,solution):
        drawer=Drawer(self.path_planner)
        drawer.draw_path_for_all_robots(solution.path,self.robots,self.tasks)

    def draw_solution_step_by_step(self,solution,k_delta):
        drawer=Drawer(self.path_planner)
        N=self.path_planner.parameters.N
        k=0
        while k<N-1:
            drawer.draw_path_for_all_robots_step_by_step(solution.path,self.robots,self.tasks,k)
            k=k+k_delta
        drawer.draw_path_for_all_robots_step_by_step(solution.path,self.robots,self.tasks,N-1)