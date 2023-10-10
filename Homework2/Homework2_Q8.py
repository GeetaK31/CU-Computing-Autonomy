#Question: Write a program that accepts a comma separated sequence of words as input and prints the words in a
#comma-separated sequence after sorting them alphabetically.
try:
    User_Input = input("Enter words: ")
except ValueError:
    print("Input Not Valid.")
    exit()

X = User_Input.split(',')
Words = sorted(X)
String = ','.join(Words)

print(String)

del User_Input, X, String, Words
