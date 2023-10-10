#Question: Define a function which can generate a dictionary where the keys are numbers between 1 and 20 (both included)
# and the values are square of keys. The function should just print the keys only.
def Key_Numbers():
    Dictionary_Square = {}


    for Numbers in range(1, 21):
        Dictionary_Square[Numbers] = Numbers ** 2


    for KeyValues in Dictionary_Square.keys():
        print(KeyValues)

Key_Numbers()

