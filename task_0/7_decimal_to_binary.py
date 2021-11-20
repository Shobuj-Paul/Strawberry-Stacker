#Given a decimal number n, your task is to convert it 
#to its binary equivalent using a recursive function.
#The binary number output must be of length 8 bits.

def binary(n):
    if n == 0:
        return 0
    else:
        return (n % 2 + 10*binary(int(n // 2)))

def convert_8_bit(n):
    s = str(n)
    while len(s)<8:
        s = '0' + s
    return s

def dec_to_binary(n):
    return convert_8_bit(binary(n))

def main():
    T = int(input())
    n = []
    for _ in range(0,T):
        temp = int(input())
        n.append(dec_to_binary(temp))
    print(*n,sep="\n")

if __name__=='__main__':
    try: main()
    except: pass