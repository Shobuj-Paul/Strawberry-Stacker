#Given a string str, check if its a palindrome. Consider it to be case insensitive.
#A palindrome string is a sequence of characters that reads the same forwards and backwards.
#If str is found to be a palindrome, print It is a palindrome
#else print
#It is not a palindrome

def palindrome(str):
    for i in range(0, int(len(str)/2)):
        if str[i]!=str[len(str)-i-1]:
            return False
    return True

def main():
    str = []
    T = int(input(""))
    for i in range(0,T):
        s = input("").lower()
        str.append(s)
    for i in range(0,T):
        if palindrome(str[i]) == True:
            print("It is a palindrome")
        else:
            print("It is not a palindrome")

if __name__=='__main__':
    try: main()
    except: pass
