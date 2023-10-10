#Question: Write a program which will find all such numbers which are divisible by 7 but are not a multiple of 5,
# between 2000 and 3200 (both included). The numbers obtained should be printed in a comma-separated sequence on
# a single line.
Ans = []

for numbers in range(2000, 3201):

    if numbers % 7 == 0 and numbers % 5 != 0:
        Ans.append(numbers)

Ans_str = Ans
print(Ans_str)

del numbers
del Ans_str
