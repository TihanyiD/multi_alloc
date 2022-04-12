from Set import Set
from itertools import combinations
from itertools import permutations

class Q_set(Set):
    def __init__(self,path_planner,targets,goal):
        self.X=path_planner.X
        self.targets=targets
        self.goal=goal
        Q=self.generate_Q()
        super(Q_set,self).__init__(Q)

    def generate_Q(self):
        Q=[set([])]
        for k in range(1,len(self.targets)+1):
            comb_k=list(combinations(self.targets,k))
            Q=Q+[set(e) for e in comb_k]
        return Q