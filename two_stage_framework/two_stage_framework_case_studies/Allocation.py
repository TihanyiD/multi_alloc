import numpy as np
import pandas as pd

class Allocation(set):
    function_frame=None
    robots=None
    tasks=None

    def __init__(self,allocation):
        super().__init__(allocation)

    @staticmethod
    def set_up(function_frame):
        Allocation.function_frame=function_frame
        Allocation.robots=function_frame.robots
        Allocation.tasks=function_frame.tasks

    def add(self,r_id,a_id):
        super().add((r_id,a_id))

    def get_value(self,get_time=False):
        value=1
        time=0
        for r in self.robots:
            S_r_id=set([])
            for ra_id in self:
                r_id,a_id=ra_id
                if r_id==r.id:
                    S_r_id.add(a_id)
            f_r,f_r_time=self.function_frame.get_value(r.id,S_r_id)
            value=value*f_r
            time=time+f_r_time
        if get_time:
            return value,time
        else:
            return value

    def get_derivative(self,omega):
        omega=set(omega)
        F_omega=Allocation(self.union(omega)).get_value()
        F=self.get_value()
        return F_omega-F

    def get_printable(self):
        columns=[]
        for a in self.tasks:
            columns.append("task_"+str(a.id))
        rows=[]
        for r in self.robots:
            rows.append("robot_"+str(r.id))
        data=np.zeros((len(rows),len(columns)),dtype=bool)
        allocation_printable=pd.DataFrame(columns=columns,index=rows,data=data)

        for ra_id in self:
            r_id,a_id=ra_id
            allocation_printable.loc["robot_"+str(r_id),"task_"+str(a_id)]=True
        return allocation_printable