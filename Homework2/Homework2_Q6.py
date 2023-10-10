#Question: Write a program that calculates and prints the value according to the given formula:
#Q = Square root of [(2 * C * D)/H]
#Following are the fixed values of C and H:
#C is 50. H is 30.
#D is the variable whose values should be input to your program in a comma-separated sequence.
import math

C = 50
H = 30

try:
    Input_storage = input("Enter a values for D in a sequence separated by comma's : ")
except ValueError:
    print("Input Invalid.")
    exit()

D_Value = Input_storage.split(',')
Answer = []

for Value in D_Value:
    try:
        D = float(Value)
        Q = math.sqrt((2 * C * D) / H)
        Answer.append(round(Q))
    except ValueError:
        print(f" Not a Valid Input: {Value}")

# Print Answer
print(','.join(map(str, Answer)))

# Delete workspace variables
del Input_storage, D_Value, Answer
