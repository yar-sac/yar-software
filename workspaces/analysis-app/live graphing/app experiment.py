import time
import numpy as np

def FileWriter():
    f=open('txt_file_experiment.txt','a')
    i=0
    k=0
    while True:
        k+=1
        i+=1
        j= round(100*np.sin(i/20))
        data = f.write(str(i)+','+str(j)+'\n')
        print("wrote data")
        time.sleep(0.1)
        f.flush()

FileWriter()
