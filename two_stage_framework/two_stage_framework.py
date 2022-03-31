import numpy as np
import pandas as pd
import warnings
import pickle
import random

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
def process_results_1(results):
    fg_results=[]
    rg_results=[]
    bf_results=[]
    fg_times=[]
    rg_times=[]
    bf_times=[]
    for results_i in results:
        fg_results.append(results_i["fg_solution"].objective_value['multiplicative'])
        rg_results.append(results_i["rg_solution"].objective_value['multiplicative'])
        bf_results.append(results_i["bf_solution"].objective_value['multiplicative'])
        fg_times.append(results_i["fg_solution"].time_data['calculation_time'])
        rg_times.append(results_i["rg_solution"].time_data['calculation_time'])
        bf_times.append(results_i["bf_solution"].time_data['calculation_time'])
    
    #table
    data={'fg_results':fg_results,'rg_results':rg_results,'bf_results':bf_results,'fg_times':fg_times,'rg_times':rg_times,'bf_times':bf_times}
    table=pd.DataFrame(data=data)
    table.to_csv('results_table.csv')
    #normalised fg,rg
    fg_bf_results=[fg/bf for fg,bf in zip(fg_results,bf_results)]
    rg_bf_results=[rg/bf for rg,bf in zip(rg_results,bf_results)]
    fg_or_rg_results=[]
    for fg_bf,rg_bf in zip(fg_bf_results,rg_bf_results):
        if fg_bf>=rg_bf:
            fg_or_rg_results.append('fg')
        else:
            fg_or_rg_results.append('rg')
    normalised_data={'fg/bf':fg_bf_results,'rg/bf':rg_bf_results,'better_algorithm':fg_or_rg_results}
    normalised_table=pd.DataFrame(data=normalised_data)
    normalised_table.to_csv('normalised_results_table.csv',na_rep='NaN')
def process_results_2(results):
    fg_results=[]
    rg_results=[]
    bf_results=[]
    fg_times=[]
    rg_times=[]
    bf_times=[]
    for results_i in results:
        fg_results.append(results_i["fg_solution"].objective_value['multiplicative'])
        rg_results.append(results_i["rg_solution"].objective_value['multiplicative'])
        bf_results.append(results_i["bf_solution"].objective_value['multiplicative'])
        fg_times.append(results_i["fg_solution"].time_data['calculation_time'])
        rg_times.append(results_i["rg_solution"].time_data['calculation_time'])
        bf_times.append(results_i["bf_solution"].time_data['calculation_time'])
    
    #table
    data={'fg_results':fg_results,'rg_results':rg_results,'bf_results':bf_results,'fg_times':fg_times,'rg_times':rg_times,'bf_times':bf_times}
    table=pd.DataFrame(data=data)
    table.to_csv('results_table.csv')
    #relative results r_fg,r_rg
    r_fg_results=[fg/bf for fg,bf in zip(fg_results,bf_results)]
    r_rg_results=[rg/bf for rg,bf in zip(rg_results,bf_results)]
    relative_data={'fg/bf':r_fg_results,'rg/bf':r_rg_results}
    relative_table=pd.DataFrame(data=relative_data)
    relative_table.to_csv('relative_results_table.csv')
    #times
    time_data={'fg_times':fg_times,'rg_times':rg_times,'bf_times':bf_times}
    time_table=pd.DataFrame(data=time_data)
    time_table.to_csv('time_results_table.csv')
    #box plots
    relative_boxplot=relative_table.boxplot(column=['fg/bf','rg/bf'])
    time_boxplot_fg_rg=time_table.boxplot(column=['fg_times','rg_times'])
    time_boxplot_bf=time_table.boxplot(column=['bf_times'])  
    print('Done')  
def process_results_3(results):
    fg_results=[]
    rg_results=[]
    bf_results=[]
    fg_bf_results=[]
    rg_bf_results=[]
    fg_times=[]
    rg_times=[]
    bf_times=[]
    for results_i in results:
        if results_i["bf_solution"].objective_value['multiplicative']>0:
            fg_results.append(results_i["fg_solution"].objective_value)
            rg_results.append(results_i["rg_solution"].objective_value)
            bf_results.append(results_i["bf_solution"].objective_value['multiplicative'])
            fg_bf_results.append(results_i["fg_solution"].objective_value/results_i["bf_solution"].objective_value['multiplicative'])
            rg_bf_results.append(results_i["rg_solution"].objective_value/results_i["bf_solution"].objective_value['multiplicative'])
            fg_times.append(results_i["fg_solution"].time_data['calculation_time'])
            rg_times.append(results_i["rg_solution"].time_data['calculation_time'])
            bf_times.append(results_i["bf_solution"].time_data['calculation_time'])
    
    #table
    data={'fg_results':fg_results,'rg_results':rg_results,'bf_results':bf_results,'fg_bf_results':fg_bf_results,'rg_bf_results':rg_bf_results,'fg_times':fg_times,'rg_times':rg_times,'bf_times':bf_times}
    table=pd.DataFrame(data=data)
    table.to_csv('results_table.csv')
    print('Done')  

# Parameters
def generate_random_parameters(param_ind,n_targets,n_robots,n_hazards,hazard_p_f,open_case_study):
    name_str="param_"+str(param_ind)
    parameters=Parameters(name=name_str)

    parameters.map=np.flipud(np.array([[1,1,1,0,1,0,1,1,1],
                                       [1,0,0,0,0,0,0,0,1],
                                       [1,0,1,0,1,0,1,0,1],
                                       [0,0,0,0,0,0,0,0,1],
                                       [1,0,1,0,1,0,1,0,0],
                                       [0,0,0,0,0,0,0,0,1],
                                       [1,0,1,0,1,0,1,0,1],
                                       [1,0,0,0,0,0,0,0,1],
                                       [1,1,1,0,1,0,1,1,1]]))

    target_ids=["i","ii","iii","iv","v","vi","vii","viii"]
    targets=[(1,1),(1,2),(1,3),(1,4),(1,5),(1,6),(1,7),
             (2,1),(2,3),(2,5),(2,7),
             (3,1),(3,2),(3,3),(3,4),(3,5),(3,6),(3,7),
             (4,1),(4,3),(4,5),(4,7),
             (5,1),(5,2),(5,3),(5,4),(5,5),(5,6),(5,7),
             (6,1),(6,3),(6,5),(6,7),
             (7,1),(7,2),(7,3),(7,4),(7,5),(7,6),(7,7)]
    parameters.targets=random.sample(targets,n_targets)
    parameters.task_ids=target_ids[0:n_targets]

    robot_ids=["i","ii","iii","iv","v"]
    robot_linestyles=[(0,()),(0,(3,3)),(0,(2,3)),(0,(1,3)),(0,(1,4))]
    robot_positions=[(0,3),(0,5),(3,0),(5,0),(3,8),(5,8)]
    robot_positions=[e for e in robot_positions if e not in parameters.targets]
    parameters.robot_positions=random.sample(robot_positions,n_robots)
    parameters.robot_ids=robot_ids[0:n_robots]
    parameters.robot_linestyles=robot_linestyles[0:n_robots]

    #generate this
    hazard_ids=["a","b","c","d"]
    hazards=[(1,1),(1,2),(1,6),(1,7),
             (2,1),(2,3),(2,5),(2,7),
             (3,2),(3,3),(3,4),(3,5),(3,6),
             (4,3),(4,5),
             (5,2),(5,3),(5,4),(5,5),(5,6),
             (6,1),(6,3),(6,5),(6,7),
             (7,1),(7,2),(7,6),(7,7)]
    hazards=[e for e in hazards if (e not in parameters.targets) and (e not in parameters.robot_positions)]
    parameters.y_0=[[e] for e in random.sample(hazards,n_hazards)]
    parameters.hazard_ids=hazard_ids[0:n_hazards]
    parameters.p_f=[hazard_p_f for e in hazards]

    parameters.goal=(8,4)
    parameters.E=5000
    parameters.N=20
    parameters.p_stay=0

    parameters.generate_obsticles()
    parameters.generate_Hazards()
    parameters.generate_Tasks()
    parameters.generate_Robots()

    parameters.generate_Tau_X=generate_Tau_X
    parameters.sample_Tau_Ys=sample_Tau_Ys

    parameters_file={"Read":open_case_study,"Name":"parameters"+str(param_ind)}
    samples_file={"Read":open_case_study,"Name":"samples"+str(param_ind)}
    function_frame_file={"Read":open_case_study,"Name":"function_frame"+str(param_ind)}
    solution_file={"Read":open_case_study,"Name":"solution"+str(param_ind)}

    if parameters_file["Read"]:
        infile=open(parameters_file["Name"],'rb')
        parameters=pickle.load(infile)
        infile.close()
    else:
        outfile=open(parameters_file["Name"],'wb')
        pickle.dump(parameters,outfile)
        outfile.close()

    parameters.parameters_file=parameters_file
    parameters.samples_file=samples_file
    parameters.function_frame_file=function_frame_file
    parameters.solution_file=solution_file

    return parameters



### Main ###
warnings.filterwarnings("ignore")
"""
parameters=Parameters(name="random_map_drawing")
parameters.map=np.flipud(np.array([[1,1,1,0,1,0,1,1,1],
                                       [1,0,0,0,0,0,0,0,1],
                                       [1,0,1,0,1,0,1,0,1],
                                       [0,0,0,0,0,0,0,0,1],
                                       [1,0,1,0,1,0,1,0,0],
                                       [0,0,0,0,0,0,0,0,1],
                                       [1,0,1,0,1,0,1,0,1],
                                       [1,0,0,0,0,0,0,0,1],
                                       [1,1,1,0,1,0,1,1,1]]))
parameters.generate_obsticles()
parameters.targets=[(1,1),(1,2),(1,3),(1,4),(1,5),(1,6),(1,7),
                    (2,1),(2,3),(2,5),(2,7),
                    (3,1),(3,2),(3,3),(3,4),(3,5),(3,6),(3,7),
                    (4,1),(4,3),(4,5),(4,7),
                    (5,1),(5,2),(5,3),(5,4),(5,5),(5,6),(5,7),
                    (6,1),(6,3),(6,5),(6,7),
                    (7,1),(7,2),(7,3),(7,4),(7,5),(7,6),(7,7)]
parameters.hazards=[(1,1),(1,2),(1,6),(1,7),
                    (2,1),(2,3),(2,5),(2,7),
                    (3,2),(3,3),(3,4),(3,5),(3,6),
                    (4,3),(4,5),
                    (5,2),(5,3),(5,4),(5,5),(5,6),
                    (6,1),(6,3),(6,5),(6,7),
                    (7,1),(7,2),(7,6),(7,7)]
parameters.robot_positions=[(0,3),(0,5),(3,0),(5,0),(3,8),(5,8)]
parameters.goal=(8,4)
drawer=Drawer(parameters=parameters)
drawer.draw_random_map()
"""



# Parameters
n_samples=20
n_repeat_samples=5    
n_targets=2
n_robots=2
n_hazards=3
hazard_p_f=0.02
open_case_study=False

samples=[]
for i in range(n_samples+n_repeat_samples):
    parameters_i=generate_random_parameters(i,n_targets,n_robots,n_hazards,hazard_p_f,open_case_study)
    samples.append(parameters_i)

# Solving
if False:
    infile=open("results",'rb')
    results=pickle.load(infile)
    infile.close()
else:
    results=[]
    for i,parameters in enumerate(samples):
        print("SAMPLE "+str(len(results))+"-"+str(i+1)+"/"+str(n_samples)+"-"+str(len(samples)))
        results_i={}
        results_i["name"]=parameters.name

        # Setting up
        path_planner=Path_Planner(parameters)
        path_planner.set_up()

        #Drawer(path_planner).draw_full_example()

        # Function frame
        if parameters.function_frame_file["Read"]:
            print("...Reading function frame...")
            infile=open(parameters.function_frame_file["Name"],'rb')
            function_frame=pickle.load(infile)
            infile.close()
        else:
            function_frame=Function_Frame(parameters,path_planner)
            print("...Saving function frame...")
            outfile=open(parameters.function_frame_file["Name"],'wb')
            pickle.dump(function_frame,outfile)
            outfile.close()

        # Forward greedy
        allocator_fg=Forward_Greedy_Allocator(function_frame)
        if parameters.solution_file["Read"]:
            infile=open(parameters.solution_file["Name"]+"_fg",'rb')
            fg_solution=pickle.load(infile)
            infile.close()
        else:
            fg_solution=allocator_fg.solve_problem()
            #allocator_fg.postprocess_solution(fg_solution)
            fg_solution.save_solution(parameters.solution_file["Name"]+"_fg")
        #allocator_fg.show_solution(fg_solution)
        #allocator_fg.draw_solution_step_by_step(fg_solution,5)

        # Reverse greedy
        allocator_rg=Reverse_Greedy_Allocator(function_frame)
        if parameters.solution_file["Read"]:
            infile=open(parameters.solution_file["Name"]+"_rg",'rb')
            rg_solution=pickle.load(infile)
            infile.close()
        else:
            rg_solution=allocator_rg.solve_problem()
            #allocator_rg.postprocess_solution(rg_solution)
            rg_solution.save_solution(parameters.solution_file["Name"]+"_rg")
        #allocator_rg.show_solution(rg_solution)

        # Brute force
        allocator_bf=Brute_Force_Allocator(function_frame)
        if parameters.solution_file["Read"]:
            infile=open(parameters.solution_file["Name"]+"_bf",'rb')
            bf_solution=pickle.load(infile)
            infile.close()
            infile=open(parameters.solution_file["Name"]+"_worst",'rb')
            wc_solution=pickle.load(infile)
            infile.close()
        else:
            bf_solution,wc_solution=allocator_bf.solve_problem()
            allocator_bf.postprocess_solution(bf_solution)
            allocator_bf.postprocess_solution(wc_solution)
            bf_solution.save_solution(parameters.solution_file["Name"]+"_bf")
            wc_solution.save_solution(parameters.solution_file["Name"]+"_worst")

        #allocator_bf.show_solution(bf_solution)
        #allocator_bf.show_solution(wc_solution)

        if bf_solution.objective_value['multiplicative']>0:
            results_i["fg_solution"]=fg_solution
            results_i["rg_solution"]=rg_solution
            results_i["bf_solution"]=bf_solution
            results_i["wc_solution"]=wc_solution
            results.append(results_i)
        
        if len(results)>=n_samples:
            print("Finished early! Saved "+str(n_samples)+" useable samples!")
            break

    outfile=open("results",'wb')
    pickle.dump(results,outfile)
    outfile.close()
process_results_3(results)