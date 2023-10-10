#Question 2 : Given a binary tree, find the max depth of it. Modify the “solution” function in the question2.py
#(Analyze your time complexity, and only time-complexity optimized solution gets full grade)
class TreeNode(object):
    """Definition of a binary tree node."""

    def __init__(self, x):
        self.val = x
        self.left = None
        self.right = None


def solution(root):
    if not root:
        return 0

    left_depth = solution(root.left)
    right_depth = solution(root.right)

    return max(left_depth, right_depth) + 1

# Binary Tree Example from : CU-Computing-Autonomy/Homework3/Question 2
#     3
#    / \
#   9  20
#     /  \
#    15   7

a15 = TreeNode(15)
a7 = TreeNode(7)
a20 = TreeNode(20)
a9 = TreeNode(9)
a3 = TreeNode(3)
a20.left = a15
a20.right = a7
a3.left = a9
a3.right = a20

print(solution(a3))

# Time Complexity
#For a time complexity analysis:
#Every node is visited precisely once: O(n)
#The function's other operations are all constant-time: O(1)
#As a result, the solution has an overall time complexity of O(n), which is linear in terms of the size of the binary tree.