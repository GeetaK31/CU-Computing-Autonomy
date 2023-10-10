#Question: Write a program which accepts a sequence of comma-separated numbers from console and generate a list
# and a tuple which contains every number

NumberSequence = input("Enter numbers in a sequence separated by commas: ")

Number_list = NumberSequence.split(',')

Number_tuple = tuple(Number_list)

print(Number_list)
print(Number_tuple)

del Number_list, Number_tuple