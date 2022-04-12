class Robot_objective(object):
    def __init__(self,r,function_frame):
        self.function_frame=function_frame
        self.r=r

    def get_value(self,S_r,print_progress=False):
        r_id=self.r.id
        S_r_id=set([a.id for a in S_r])
        return self.function_frame.get_value(r_id,S_r_id)

