import numpy as np
import copy
import itertools

from Set import Set
from DMatrix import DMatrix
from Matrix import Matrix
from Instrument import Instrument
from ProgressBar import ProgressBar

class Function_Frame(DMatrix):
    def __init__(self,parameters,path_planner):
        super().__init__()
        self.robots=parameters.robots
        self.tasks=parameters.tasks
        self.path_planner=path_planner
        self.calculating_function_frame()
        
    def calculating_function_frame(self):
        print("Calculating function frame...")
        self.instrument=Instrument()
        self.instrument.start()

        task_subsets=[]
        for i in range(len(self.tasks)+1):
            task_subsets=task_subsets+[Set(list(e)) for e in itertools.combinations(self.tasks,i)]
        task_subsets=Set(task_subsets)

        domain_robots=[e.id for e in self.robots]
        domain_tasks=[set([ee.id for ee in e]) for e in task_subsets]
        self.domain_dict=["value","time"]
        self.domain_matrix=[domain_robots,domain_tasks]
        self["value"]=Matrix(self.domain_matrix,np.zeros((len(domain_robots),len(domain_tasks))))
        self["time"]=Matrix(self.domain_matrix,np.zeros((len(domain_robots),len(domain_tasks))))

        N=len(task_subsets)
        progress_bar=ProgressBar(N)
        for i_allocation,target_allocation in enumerate(task_subsets):
            progress_bar.progress(i_allocation)
            V_ret,Mu,V,t_eval=self.get_all_function_values(target_allocation)
            #save Mu and V ???
            self["value"].matrix[:,i_allocation]=V_ret
            self["time"].matrix[:,i_allocation]=t_eval*np.ones(len(V_ret))
        progress_bar.progress(N,'Finished!\n')
        calculation_time=self.instrument.stop()
        self.instrument.save_measurement("calculation_time",calculation_time)
        print("Calculation time [s]:")
        print(self.instrument.data)
        print("Finished calculating function frame!\n")

    def get_all_function_values(self,target_allocation):
        targets=[]
        for a in target_allocation:
            targets=targets+[a.target]
        goal=self.robots[0].goal
        x_0=[r.x_0 for r in self.robots]
        instrument=Instrument()
        instrument.start()
        V_ret,Mu,V=self.path_planner.get_solution(targets,goal,x_0)
        t_eval=instrument.stop()
        return V_ret,Mu,V,t_eval

    def get_value(self,r_id,S_r_id):
        return self["value"].get([r_id,S_r_id]),self["time"].get([r_id,S_r_id])