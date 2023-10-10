# Question 8 : Given two strings s and t, return true if t is an anagram of s, and false otherwise.
# An Anagram is a word or phrase formed by rearranging the letters of a different word or phrase,
# typically using all the original letters exactly once.
# Modify the “solution” function in the question8.py. (Analyze your time complexity)

class Solution:
    def isAnagram(self, s: str, t: str) -> bool:
        if len(s) != len(t):
            return False

        char_dict_s = {}
        char_dict_t = {}

        for char in s:
            char_dict_s[char] = char_dict_s.get(char, 0) + 1

        for char in t:
            char_dict_t[char] = char_dict_t.get(char, 0) + 1

        return char_dict_s == char_dict_t


# Example :
solution = Solution()

# Ex Case 1
s1 = "anagram"
t1 = "nagaram"
result1 = solution.isAnagram(s1, t1)
print(result1)

# Ex Case 2
s2 = "rat"
t2 = "car"
result2 = solution.isAnagram(s2, t2)
print(result2)
