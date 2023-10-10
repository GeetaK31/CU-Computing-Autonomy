#Question: Write a program which can compute the factorial of a given numbers. The results should be printed in a
# comma-separated sequence on a single line.
def factorial(number):
    if number == 0:
        return 1
    else:
        return number * factorial(number - 1)

Input_Value = int(input("Enter a number: "))

Answer = factorial(Input_Value)

print(Answer)

del Input_Value, Answer
