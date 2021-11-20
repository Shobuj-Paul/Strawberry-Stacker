#The task is to read an integer n from STDIN.
#For all values of i<n in ascending order, do the following:
#If i is non-zero then,
#print the square of i, if i is odd.
#print double the value of i, if i is even.
#If i is zero then, add 3 to i.

def operation(n):
    a = []
    for i in range(0,n):
        if i==0:
            i+=3
        elif i!=0 and i%2==0:
            i=i*2
        elif i!=0 and i%2!=0:
            i=i*i
        a.append(i)
    return a
        
def main():
    T = int(input())
    a = []
    for i in range(0,T):
        n = int(input())
        a.append(operation(n))
    for i in range(0,T):
        print(*a[i], end=" ")
        print()

if __name__=='__main__':
    try: main()
    except: pass