#Question 9: Write a program that accepts sequence of lines as input and prints the lines after making all characters in the
# sentence capitalized.

Input_Sentences = []
while True:
    try:
        User_Input = input("Enter a sentence or keep pressing enter key : ")
        if not User_Input:
            break
        Input_Sentences.append(User_Input)
    except EOFError:
        break

UpperCase_Sentences = [Answer.upper() for Answer in Input_Sentences]

for Answer in UpperCase_Sentences:
    print(Answer)

del Input_Sentences, UpperCase_Sentences







