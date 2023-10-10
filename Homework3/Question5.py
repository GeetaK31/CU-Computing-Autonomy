#Question 5 : Given a string s containing just the characters'(', ')', '{', '}', '[' and ']', determine if the
# input string is valid. #An input string is valid if:
#Open brackets must be closed by the same type of brackets.
#Open brackets must be closed in the correct order.
#Every close bracket has a corresponding open bracket of the same type.
#Modify the “solution” function in the question5.py. (Analyze your time complexity)
class Solution:
    def isValid(self, s: str) -> bool:
        stack_list = []

        dict_bracket_map = {')': '(', '}': '{', ']': '['}

        for char in s:
            if char in dict_bracket_map:
                Top_most_element = stack_list.pop() if stack_list else '#'
                if dict_bracket_map[char] != Top_most_element:
                    return False
            else:
                stack_list.append(char)

        return not stack_list


# Example : HW3 PDF Question 5
solution = Solution()
s = "()[]{}"
print(solution.isValid(s))
