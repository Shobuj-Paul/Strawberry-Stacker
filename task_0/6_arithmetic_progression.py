#This task is to generate Arithmetic Progression (A.P.) and perform few operations 
#on the series. The first term of sequence a1, the common difference between 
#terms d and the number of terms n in series is given.

#Then with the use of lambda and map functions, find squares of the terms in 
#series and print it. Finally with the use of reduce function, compute the sum of the 
#squares of terms in series and print it.

def generate_AP(a, d, n):
    ap = []
    for k in range (0,n):
        ap.append(a + k*d)
    return ap

def main():
    T = int(input())
    ap = []
    ap_squares = []
    for i in range(0,T):
        a, d, n = map(int, input().split())
        ap.append(generate_AP(a,d,n))
        ap_squares.append(list(map(lambda x: x*x, generate_AP(a,d,n))))
    for i in range(0,T):
        print(*ap[i], sep=" ")
        print(*ap_squares[i], sep=" ")
        print(sum(ap_squares[i]))

if __name__=='__main__':
    try: main()
    except: pass