#Question: Write a program which takes 2 digits, X,Y as input and generates a 2-dimensional array. The element value
# in the i-th row and j-th column of the array should be i*j. Note: i=0,1.., X-1; j=0,1,.., Y-1.
try:
    A, B = map(int, input("Enter (A,B): ").split(','))
except ValueError:
    print("Input Not Valid.")
    exit()


Array = [[i * j for j in range(B)] for i in range(A)]


for Row in Array:
    print(Row)

del A, B, Array
