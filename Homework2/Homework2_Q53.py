#Question: Define a class named Rectangle which can be constructed by a length and width. The Rectangle class has a
# method which can compute the area.
class Rectangle_Length_Width:
    def __init__(self, Length, Width):
        self.Length = Length
        self.Width = Width

Rectangle = Rectangle_Length_Width(5, 4)
Rectangle_Area = Rectangle.Length * Rectangle.Width

print("Rectangle Area:", Rectangle_Area)

del Rectangle, Rectangle_Area
