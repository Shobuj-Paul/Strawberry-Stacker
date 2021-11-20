#Given 2 points (x1,y1) and (x2,y2), where x1, x2 are x-coordinates 
#and y1, y2 are y-coordinates of the points.
#Your task is to compute the Euclidean distance between them.
#The distance computed should be precise up to 2 decimal places.

from math import sqrt

def compute_distance(x1, y1, x2, y2):
    distance = sqrt((x2-x1)**2 + (y2-y1)**2)
    return distance

def main():
    T = int(input())
    d = []
    for i in range(0,T):
        (x1, y1, x2, y2) = map(int, input().split(" "))
        d.append(compute_distance(x1,y1,x2,y2))
    for i in range(0,T):
        print("Distance: %.2f" %d[i])

if __name__=='__main__':
    try: main()
    except: pass