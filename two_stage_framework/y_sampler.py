import numpy as np
import random
import pickle

from ProgressBar import ProgressBar

class y_sampler(object):
    def __init__(self,path_planner):
        parameters=path_planner.parameters
        self.X=path_planner.X
        self.E=parameters.E
        self.N=parameters.N
        self.hazards=parameters.hazards
        self.sample_Tau_Ys=parameters.sample_Tau_Ys
        if parameters.samples_file["Read"]:
            print("...Reading samples...")
            infile=open(parameters.samples_file["Name"],'rb')
            self.episodes=pickle.load(infile)
            infile.close()
        else:
            self.progress_bar=ProgressBar(self.N)
            print("...Monte Carlo sampling...")
            self.episodes=self.generate_episodes()
            self.add_cheat_sample()
            print("...Saving samples...")
            outfile=open(parameters.samples_file["Name"],'wb')
            pickle.dump(self.episodes,outfile)
            outfile.close()

    def generate_episodes(self):
        episodes=np.zeros((self.E,self.N,len(self.X)),dtype=bool)

        for hazard in self.hazards:
            y_0_h=hazard.y_0
            p_f_h=hazard.p_f

            episodes_h=self.init_episodes(y_0_h)
            for t in range(1,self.N):
                ys_t_1=episodes_h[:,t-1,:]
                ys_t=self.sample_Tau_Ys(self,p_f_h,ys_t_1)
                episodes_h[:,t,:]=ys_t
                if t % int(self.N/self.N)==0:
                    self.progress_bar.progress(t)
            self.progress_bar.progress(self.N,'Finished!\n')

            episodes=episodes+episodes_h
        return episodes

    def init_episodes(self,y_0):
        episodes=np.zeros((self.E,self.N,len(self.X)),dtype=bool)
        for e in y_0:
            episodes[:,0,self.X.index(e)]=True
        return episodes

    def add_cheat_sample(self):
        y_0=self.episodes[0,0,:]
        self.episodes[0,:,:]=np.kron(np.ones((self.N,1),dtype=bool),y_0)