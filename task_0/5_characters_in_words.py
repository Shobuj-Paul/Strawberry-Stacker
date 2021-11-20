#Write a program to read a string str, which will contain words separated with space.
#Your task is to count the length of each word and print it. 
#The output should be comma-separated string.
#Each string str will begin with "@".

def count_characters(str):
    n = []
    words = str.split(" ")
    for i in words:
        n.append(len(i))
    n[0] = n[0]-1
    return n
        
def main():
    T = int(input())
    n = []
    for i in range(0,T):
        str = input()
        n.append(count_characters(str))
    for i in range(0,T):
        print(*n[i], sep=",")

if __name__=='__main__':
    try: main()
    except: pass