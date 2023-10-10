#Question: Define a class named Shape and its subclass Square. The Square class has an init function which takes a
#length as argument. Both classes have a area function which can print the area of the shape where
#Shape's area is 0 by default.
class Shape:
    def __init__(self):
        pass

    def area(self):
        return 0

class Square(Shape):
    def __init__(self, length):
        self.length = length

    def area(self):
        return self.length ** 2


square = Square(4)


Area_of_Square = square.area()
print("Area of the square:", Area_of_Square)


shape = Shape()


#shape_area = shape.area()
#print("Area of the generic shape:", shape_area)

# Clear workspace variables
del square, Area_of_Square, shape#, shape_area

