# Question: Write a program that accepts a sequence of whitespace separated words as input and prints the words after
# removing all duplicate words and sorting them alphanumerically.

User_Input = input("Enter input: ")

Input_Split = User_Input.split()
Remove_Duplicate = set(Input_Split)

Answer = sorted(Remove_Duplicate)

print(' '.join(Answer))

del User_Input, Input_Split, Remove_Duplicate, Answer
