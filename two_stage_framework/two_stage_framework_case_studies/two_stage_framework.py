import numpy as np
import warnings
import pickle
import os

from Parameters import Parameters
from Drawer import Drawer
from Matrix import Matrix
from Path_Planner import Path_Planner
from Function_Frame import Function_Frame
from Forward_Greedy_Allocator import Forward_Greedy_Allocator
from Reverse_Greedy_Allocator import Reverse_Greedy_Allocator
from Brute_Force_Allocator import Brute_Force_Allocator

# Parameter Functions
def generate_Tau_X(self):
    p_stay=self.p_stay

    for u in self.U_x.U:
        matrix_u=Matrix(self.domain_matrix,np.zeros(tuple([len(e) for e in self.domain_matrix])))
        if u=='0':
            for x in self.X:
                matrix_u.set([x,x],1)
        else:
            for x in self.X:
                if self.U_x.is_u_in_U_x_(u,x):
                    xx=self.U_x.get_xx_u(x,u)
                    matrix_u.set([x,x],p_stay)
                    matrix_u.set([x,xx],1-p_stay)
        self[u]=matrix_u
def sample_Tau_Ys(self,p_f,ys_k_1):
    ys_0=~ys_k_1
    ys_1=ys_k_1

    N_ys=self.X.adj_matrix.T.dot(ys_1.T.astype(int)).T
    D_ys=self.X.adj_diag_matrix.T.dot(ys_1.T.astype(int)).T

    p_cont=1-(((1-p_f)**N_ys)*((1-p_f/np.sqrt(2))**D_ys))
    rand=np.random.rand(*ys_1.shape)
    ys_cont=rand<=p_cont

    ys_k=ys_k_1
    ys_k[ys_0]=ys_cont[ys_0]
    return ys_k

# Parameters
example_name="case_study_1"
parameters=Parameters(name=example_name)
open_case_study=True

parameters.map=np.array([[1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1],
                         [1,0,0,0,0,0,1,1,0,1,0,0,1,0,0,0,1],
                         [1,0,1,0,1,0,1,1,0,1,0,0,0,0,1,0,1],
                         [1,0,0,0,0,0,1,1,0,1,0,1,1,0,0,0,1],
                         [1,1,1,0,1,1,1,1,0,0,0,0,0,0,1,1,1],
                         [1,1,1,0,1,1,1,1,0,0,1,1,1,1,1,1,1],
                         [0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,1],
                         [1,1,1,0,1,1,1,1,0,0,1,0,0,0,0,0,1],
                         [1,1,1,0,1,1,1,1,0,0,1,0,0,0,0,0,1],
                         [1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0],
                         [1,0,1,0,1,0,1,1,0,0,1,0,0,0,0,0,1],
                         [1,0,0,0,0,0,1,1,0,0,1,0,0,0,0,1,1],
                         [1,1,1,1,1,1,1,1,0,1,1,1,1,0,1,1,1]])

parameters.targets=[(3,9),(5,1),(8,6),(11,11),(14,1)]
parameters.task_ids=["i","ii","iii","iv","v"]

parameters.robot_positions=[(0,6),(8,12),(10,0)]
parameters.robot_ids=["1","2","3"]
parameters.robot_linestyles=[(0,()),(0,(3,3)),(0,(1,2))]

parameters.y_0=[[(13,12)],[(2,1)],[(11,2)],[(3,11)],[(13,6)]]
parameters.hazard_ids=["a","b","c","d","e"]
parameters.p_f=[0.002,0.004,0.012,0.012,0.012]

parameters.goal=(16,9)
parameters.E=5000
parameters.N=75
parameters.p_stay=0

parameters.generate_obsticles()
parameters.generate_Hazards()
parameters.generate_Tasks()
parameters.generate_Robots()

parameters.generate_Tau_X=generate_Tau_X
parameters.sample_Tau_Ys=sample_Tau_Ys

parameters_file={"Read":open_case_study,"Name":"parameters"}
samples_file={"Read":open_case_study,"Name":"samples"}
function_frame_file={"Read":open_case_study,"Name":"function_frame"}
solution_file={"Read":open_case_study,"Name":"solution"}

### Main ###
warnings.filterwarnings("ignore")

rel_path='/case_studies/'+example_name+'/'
path=os.getcwd()+rel_path
if not os.path.exists(path):
    os.makedirs(path)

# Parameters
if parameters_file["Read"]:
    infile=open(path+parameters_file["Name"],'rb')
    parameters=pickle.load(infile)
    infile.close()
else:
    outfile=open(path+parameters_file["Name"],'wb')
    pickle.dump(parameters,outfile)
    outfile.close()

parameters.parameters_file=parameters_file
parameters.samples_file=samples_file
parameters.function_frame_file=function_frame_file
parameters.solution_file=solution_file

# Setting up
path_planner=Path_Planner(parameters)
path_planner.set_up(path)

Drawer(path_planner).draw_full_example()

# Function frame
if parameters.function_frame_file["Read"]:
    print("...Reading function frame...")
    infile=open(path+parameters.function_frame_file["Name"],'rb')
    function_frame=pickle.load(infile)
    infile.close()
else:
    function_frame=Function_Frame(parameters,path_planner)
    print("...Saving function frame...")
    outfile=open(path+parameters.function_frame_file["Name"],'wb')
    pickle.dump(function_frame,outfile)
    outfile.close()

# Forward greedy
allocator_fg=Forward_Greedy_Allocator(function_frame)
if parameters.solution_file["Read"]:
    infile=open(path+parameters.solution_file["Name"]+"_fg",'rb')
    fg_solution=pickle.load(infile)
    infile.close()
else:
    fg_solution=allocator_fg.solve_problem()
    allocator_fg.postprocess_solution(fg_solution)
    fg_solution.save_solution(path+parameters.solution_file["Name"]+"_fg")
allocator_fg.show_solution(fg_solution)
#allocator_fg.draw_solution_step_by_step(fg_solution,5)

# Reverse greedy
allocator_rg=Reverse_Greedy_Allocator(function_frame)
if parameters.solution_file["Read"]:
    infile=open(path+parameters.solution_file["Name"]+"_rg",'rb')
    rg_solution=pickle.load(infile)
    infile.close()
else:
    rg_solution=allocator_rg.solve_problem()
    allocator_rg.postprocess_solution(rg_solution)
    rg_solution.save_solution(path+parameters.solution_file["Name"]+"_rg")
allocator_rg.show_solution(rg_solution)

# Brute force
allocator_bf=Brute_Force_Allocator(function_frame)
if parameters.solution_file["Read"]:
    infile=open(path+parameters.solution_file["Name"]+"_bf",'rb')
    bf_solution=pickle.load(infile)
    infile.close()
    infile=open(path+parameters.solution_file["Name"]+"_worst",'rb')
    worst_solution=pickle.load(infile)
    infile.close()
else:
    bf_solution,worst_solution=allocator_bf.solve_problem()
    allocator_bf.postprocess_solution(bf_solution)
    allocator_bf.postprocess_solution(worst_solution)
    bf_solution.save_solution(path+parameters.solution_file["Name"]+"_bf")
    worst_solution.save_solution(path+parameters.solution_file["Name"]+"_worst")

allocator_bf.show_solution(bf_solution)
allocator_bf.show_solution(worst_solution)
