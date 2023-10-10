#Question: With a given list [12,24,35,24,88,120,155,88,120,155], write a program to print this list after
# removing all duplicate values with original order reserved.

Given_list = [12, 24, 35, 24, 88, 120, 155, 88, 120, 155]

Sorted_list = list(dict.fromkeys(Given_list))

print(Sorted_list)

del Given_list, Sorted_list
