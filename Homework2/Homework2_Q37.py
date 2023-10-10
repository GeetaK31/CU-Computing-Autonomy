#Question: Define a function which can generate and print a list where the values are square of numbers
#between 1 and 20 (both included).

def Square_of_numbers_list():

    Numbers_Square = []


    for num in range(1, 21):
        Numbers_Square.append(num ** 2)


    print(Numbers_Square)

Square_of_numbers_list()


