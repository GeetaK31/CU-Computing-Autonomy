# Question 4 : Given a string s, find the length of the longest substring without repeating characters. You can
# expect the string length is less than 100, and only contains English letters.
# Modify the “solution” class in question4.py, you may design your input to test it.

class Solution:
    def lengthOfLongestSubstring(self, s: str) -> int:
        CurrChar_set = set()
        MaxLeng = 0
        left_pointer = 0

        for right_pointer in range(len(s)):
            while s[right_pointer] in CurrChar_set:
                CurrChar_set.remove(s[left_pointer])
                left_pointer += 1

            CurrChar_set.add(s[right_pointer])
            MaxLeng = max(MaxLeng, right_pointer - left_pointer + 1)

        return MaxLeng

# Example: Input: s = "abcabcbb" ; Output: 3 ; [HW3 PDF Question 4]
# Explanation: The answer is "abc", with the length of 3.
s = "abcabcbb"
solution = Solution()
result = solution.lengthOfLongestSubstring(s)
print(result)

