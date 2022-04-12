from Set import Set
from Hazard import Hazard
from Task import Task
from Robot import Robot

class Parameters(object):
    def __init__(self,name=None):
        self.name=name

    def generate_obsticles(self):
        size=(self.map.shape[1],self.map.shape[0])
        obsticles=[]
        for i_x in range(size[0]):
            for i_y in range(size[1]):
                if self.map[i_y,i_x]:
                    obsticles=obsticles+[(i_x,i_y)]
        self.size=size
        self.obsticles=obsticles

    def generate_Hazards(self):
        hazards=Set([])
        for i in range(len(self.y_0)):
            id_h=self.hazard_ids[i]
            y_0_h=self.y_0[i]
            p_f_h=self.p_f[i]
            hazards.add(Hazard(id_h,y_0_h,p_f_h))
        self.hazards=hazards

    def generate_Robots(self):
        robots=Set([])
        for i in range(len(self.robot_positions)):
            id_r=self.robot_ids[i]
            x_0_r=self.robot_positions[i]
            linestyle_r=self.robot_linestyles[i]
            robots.add(Robot(id_r,x_0_r,self.goal,linestyle_r))
        self.robots=robots

    def generate_Tasks(self):
        tasks=Set([])
        for i in range(len(self.targets)):
            id_a=self.task_ids[i]
            target_a=self.targets[i]
            tasks.add(Task(id_a,target_a))
        self.tasks=tasks