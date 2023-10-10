#Question: With a given integral number n, write a program to generate a dictionary that contains (i, i*i) such that
# is an integral number between 1 and n (both included). and then the program should print the dictionary.

number = int(input("Enter an integer (number): "))

Answer_dict = {}
for i in range(1, number + 1):
    Answer_dict[i] = i * i

print(Answer_dict)

del number, Answer_dict