import sys
sys.path.append(".")

class ProgressBar:
    def __init__(self,total,bar_len=60):
        self.total=total
        self.bar_len=bar_len

    def progress(self,count,suffix=''):
        filled_len=int(round(self.bar_len*count/float(self.total)))
        percents=round(100.0*count/float(self.total),1)
        bar='='*filled_len+'-'*(self.bar_len-filled_len)

        sys.stdout.write('[%s] %s%s ...%s\r' % (bar,percents,'%',suffix))
        sys.stdout.flush()