#Question: Define a class, which have a class parameter and have a same instance parameter.
class DefineClass:
    ClassParameter = "Its a Class Parameter"

    def __init__(self, InstanceParameter):
        self.InstanceParameter = InstanceParameter

Ans = DefineClass("Its an Instance Parameter")

print("Class Parameter:", DefineClass.ClassParameter)

print("Instance Parameter:", Ans.InstanceParameter)

del Ans
