# Question 7 : Given a binary tree and a sum, determine if the tree has a root-to-leaf path such that adding up
#all the values along the path equals the given sum. (Note: A leaf is a node with no children.)
# Definition for a binary tree node.
class TreeNode:
    def __init__(self, val=0, left=None, right=None):
        self.val = val
        self.left = left
        self.right = right


class Solution:
    def PathSum(self, root: TreeNode, sum: int) -> bool:
        if not root:
            return False
        sum -= root.val

        if not root.left and not root.right:
            return sum == 0

        return self.PathSum(root.left, sum) or self.PathSum(root.right, sum)


# Example : HW3 Question 7
# binary tree
#       5
#      / \
#     4   8
#    /   / \
#   11  13  4
#  /  \      \
# 7    2      1
root = TreeNode(5)
root.left = TreeNode(4)
root.right = TreeNode(8)
root.left.left = TreeNode(11)
root.left.left.left = TreeNode(7)
root.left.left.right = TreeNode(2)
root.right.left = TreeNode(13)
root.right.right = TreeNode(4)
root.right.right.right = TreeNode(1)

solution = Solution()
print(solution.PathSum(root, 22))
