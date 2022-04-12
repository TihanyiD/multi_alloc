import itertools

class Set(list):
    def __init__(self,list):
        super(Set,self).__init__(list)

    @staticmethod
    def descartes_product(set_A,set_B):
        list_C=[e for e in itertools.product(set_A,set_B)]
        set_C=Set(list_C)
        return set_C

    def add(self,element):
        if element not in self:
            self.append(element)
        return self

    def remove(self,element):
        if element in self:
            i_element=self.index(element)
            self.pop(i_element)
        return self

    def intersect(set_A,set_B):
        set_C=Set([e for e in set_A if e in set_B])
        return set_C

    def setminus(set_A,set_B):
        set_C=Set([e for e in set_A if e not in set_B])
        return set_C

    def subset(set_A,mask):
        set_B=Set([e for i,e in enumerate(set_A) if mask[i]])
        return set_B