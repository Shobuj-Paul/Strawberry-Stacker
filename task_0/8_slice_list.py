#Given a list L of numbers and its length, your task is to read these numbers into an array or a list.
#Assuming the index starts from 0, use the slice operations to do the following operations:
#   print list in reverse order
#   for every third number starting with index 0, add 3 to it and print them
#   for every fifth number starting with index 0, subtract 7 from it and print
#   add all the numbers whose index is in between 3 and 7 (inclusive) and print the result

def lst_reverse(lst):
    temp =  lst[::-1]
    return temp

def add3(lst):
    temp = []
    for i in range(1, len(lst)):
        if i%3==0:
            temp.append(lst[i]+3)
    return temp

def subtract7(lst):
    temp = []
    for i in range(1,len(lst)):
        if i%5==0:
            temp.append(lst[i]-7)
    return temp

def sum3to7(lst):
    sum = 0
    for i in range(3,8):
        sum = sum + lst[i]
    return sum
    
def main():
    T = int(input())
    lists = []
    for i in range(0,T):
        length = int(input())
        lists.append(list(map(int,input().strip().split()))[:length])
    for i in range(0,T):
        print(*lst_reverse(lists[i]))
        print(*add3(lists[i]))
        print(*subtract7(lists[i]))
        print(sum3to7(lists[i]))

if __name__=='__main__':
    try: main()
    except: pass