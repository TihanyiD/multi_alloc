class Matrix:
    def __init__(self,domain,matrix):
        if domain is not None:
            self.domain=domain
        if matrix is not None:
            self.matrix=matrix
    
    def set(self,elements,value):
        idx_list=[self.get_idx(dim+1,e) for dim,e in enumerate(elements)]
        self.matrix[tuple(idx_list)]=value

    def set_by_idx(self,idx_list,value):
        self.matrix[tuple(idx_list)]=value

    def get_idx(self,dim,element):
        idx=self.domain[dim-1].index(element)
        return idx

    def get(self,elements):
        idxs=tuple([self.get_idx(i+1,e) for i,e in enumerate(elements)])
        return self.matrix[idxs]