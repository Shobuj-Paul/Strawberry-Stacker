#Given a number N, generate a star pattern such that on the first line 
# there are N stars and on the subsequent lines the number of stars decreases by 1.
#The pattern generated should have N rows. In every row, every fifth star (*) 
# is replaced with a hash (#).
#Every row should have the required number of stars (*) and hash (#) symbols.

def operation(n):
    for i in range(n,0,-1):   
        for j in range(0,i):
            if (j+1)%5!=0:
                print("*", end="")
            elif (j+1)%5==0:
                print("#", end="")
        print("")

def main():
    T = int(input(""))
    n = []
    for i in range(0,T):
        n.append(int(input("")))
    for i in range(0,T):
        operation(n[i])

if __name__=='__main__':
    try: main()
    except: pass