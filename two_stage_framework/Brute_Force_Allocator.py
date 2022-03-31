import numpy as np
import pandas as pd
import copy
import itertools
import pickle

from Task_Allocator import Task_Allocator
from Allocation import Allocation
from Allocator_Solution import Allocator_Solution
from Function_Frame import Function_Frame
from ProgressBar import ProgressBar

class Brute_Force_Allocator(Task_Allocator):
    class Brute_Force_Allocator_Solution(Allocator_Solution):
        def __init__(self,brute_force_allocator,opt_allocation,opt_value):
            allocation=opt_allocation
            objective_value=opt_value
            algorithm=brute_force_allocator.algorithm_name
            time_data=brute_force_allocator.instrument.data
            super().__init__(algorithm,allocation,objective_value,time_data)

    def __init__(self,function_frame):
        super().__init__(function_frame)
        self.algorithm_name="Brute_Force"
        self.set_up()

    def set_up(self):
        print("--- ",self.algorithm_name," ---\n")

    def solve_problem(self):
        print("Solving allocation problem...")
        self.instrument.start()
        N_R=len(self.robots)
        N_A=len(self.tasks)
        allocations=[self.get_allocation(e) for e in itertools.product(*([list(range(N_R))]*N_A))]

        opt_allocation=None
        opt_value=-float('inf')
        worst_allocation=None
        worst_value=float('inf')

        N=len(allocations)
        progress_bar=ProgressBar(N)
        calculation_time=0
        for k,allocation in enumerate(allocations):
            progress_bar.progress(k)
            allocation_value,allocation_time=allocation.get_value(get_time=True)
            if allocation_value>opt_value:
                opt_value=allocation_value
                opt_allocation=allocation
            if allocation_value<worst_value:
                worst_value=allocation_value
                worst_allocation=allocation
            calculation_time=calculation_time+allocation_time
        progress_bar.progress(N,'Finished!\n')
        calculation_time=calculation_time+self.instrument.stop()
        self.instrument.save_measurement("calculation_time",calculation_time)
        brute_force_solution=self.Brute_Force_Allocator_Solution(self,opt_allocation,opt_value)
        worst_solution=self.Brute_Force_Allocator_Solution(self,worst_allocation,worst_value)
        print("Finished solving allocation problem!\n")
        #worst case solution ???
        return brute_force_solution,worst_solution

    def get_allocation(self,assignment):
        allocation=Allocation([])
        for i_e,e in enumerate(assignment):
            r_id=self.robots[e].id
            a_id=self.tasks[i_e].id
            allocation.add(r_id,a_id)
        return allocation