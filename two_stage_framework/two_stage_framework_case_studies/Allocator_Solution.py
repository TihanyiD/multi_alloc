import pickle

class Allocator_Solution(object):
    def __init__(self,algorithm,allocation,objective_value,time_data):
        self.algorithm=algorithm
        self.allocation=allocation
        self.objective_value=objective_value
        self.time_data=time_data

    def print_solution(self):
        print("---Solution for",self.algorithm,"---")
        print("Calculation time [s]:")
        print(self.time_data)
        print("Optimal value:")
        print(self.objective_value)
        if hasattr(self,'alpha_G'):
            print("alpha_G:",self.alpha_G)
        if hasattr(self,'gamma_G'):
            print("gamma_G:",self.gamma_G)
        print("Optimal task allocation:\n")
        print(" ",self.allocation.get_printable(),"\n")

    def save_solution(self,name):
        print("...Saving solution...")
        outfile=open(name,'wb')
        pickle.dump(self,outfile)
        outfile.close()
