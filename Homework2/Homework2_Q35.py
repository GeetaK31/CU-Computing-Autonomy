#Define a function which can generate a dictionary where the keys are numbers between 1 and 20 (both included)
# and the values are square of keys. The function should just print the values only.
def dictionary_gen_print_sq_num():
    dict_sq = {}

    for Numbers_Square in range(1, 21):
        dict_sq[Numbers_Square] = Numbers_Square ** 2

    # Print the values from the dictionary
    for Answer in dict_sq.values():
        print(Answer)

# Call the function to generate and print the square dictionary values
dictionary_gen_print_sq_num()
