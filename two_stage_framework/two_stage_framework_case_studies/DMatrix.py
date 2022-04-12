class DMatrix(dict):
    domain_dict=None
    domain_matrix=None
    
    def set_domains(self,domain_dict,domain_matrix):
        self.domain_dict=domain_dict
        self.domain_matrix=domain_matrix
