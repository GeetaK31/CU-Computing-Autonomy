# Question 1 : Given an array of integers, find two numbers in it such that they can add up to a specific number.
#You may assume there are exactly one solution, you can’t use the same element twice
def num_sum(number_set, target):
    number_dict = {}

    for index, Curr_number in enumerate(number_set):
        complement_number = target - Curr_number
        if complement_number in number_dict:
            return [number_set[number_dict[complement_number]], Curr_number]
        number_dict[Curr_number] = index

# Example :
number_set = [2, 7, 11, 4]
#target = 13
#result = num_sum(number_set, target)
#print(result)

target = int(input("Enter value of the target : "))

result = num_sum(number_set, target)
print(result)
