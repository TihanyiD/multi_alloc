from Set import Set

class QX_set(Set):
    def __init__(self,path_planner):
        self.Q=path_planner.Q
        self.X=path_planner.X
        QX=Set.descartes_product(self.Q,self.X)
        super(QX_set,self).__init__(QX)