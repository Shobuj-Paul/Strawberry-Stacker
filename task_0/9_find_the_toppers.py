#Find the toppers within the given test case
import numpy as np

def return_max(name_list, score_list):
    max = -1
    max = np.argwhere(score_list == np.amax(score_list))
    max = max.flatten().tolist()
    name = []
    for i in max:
        name.append(name_list[i])

    return sorted(name)
    
def main():
    T = int(input())
    name = []
    score = []
    for i in range(0,T):
        N = int(input())
        
        for i in range(0,N):
            a = str(input())
            name.append(a.split()[0])
            score.append(float(a.split()[1])) 

        max = []
        max = return_max(name,score)
        
        for word in max:
            print(word)
        
if __name__=='__main__':
    try: main()
    except: pass