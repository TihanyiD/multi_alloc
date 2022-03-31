import numpy as np

class U_x_function(object):
    U=['0','E','N','W','S']
    D_U=[np.array(e) for e in [(0,0),(0,1),(1,0),(0,-1),(-1,0)]]

    def __init__(self,path_planner):
        parameters=path_planner.parameters
        self.X=path_planner.X
        self.U_x=self.generate_U_x()
    
    def generate_U_x(self):
        U_x=np.zeros((len(self.X),len(self.U)),dtype=bool)
        for i,x in enumerate(self.X):
            for j in range(len(self.U)):
                d_u=self.D_U[j]
                xx_u=tuple(np.array(x)+d_u)
                if xx_u in self.X:
                    U_x[i,j]=True
        return U_x

    def get_U_x_(self,x):
        id_x=self.X.index(x)

        U_x_logical=self.U_x[id_x,:]
        U_x_=[e for i,e in enumerate(self.U) if U_x_logical[i]]
        return U_x_

    def is_u_in_U_x_(self,u,x):
        id_x=self.X.index(x)
        id_u=self.U.index(u)
        return self.U_x[id_x,id_u]

    def get_xx_u(self,x,u):
        if self.is_u_in_U_x_(u,x):
            id_x=self.X.index(x)
            id_u=self.U.index(u)

            d_u=self.D_U[id_u]
            xx_u=tuple(np.array(x)+d_u)

            return xx_u
        else:
            return None