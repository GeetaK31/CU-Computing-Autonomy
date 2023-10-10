# Question 3 : You are given two non-empty linked lists representing two non-negative integers. The digits are
# stored in reverse order and each of their nodes contain a single digit. Add the two numbers and
# return it as a linked list.
# Definition for singly-linked list.
class ListNode:
    def __init__(self, val=0, next=None):
        self.val = val
        self.next = next


class Solution:
    def addTwoNumbers(self, l1: ListNode, l2: ListNode) -> ListNode:
        DummyNode_head = ListNode()
        pointer = DummyNode_head
        Carry_Var = 0

        while l1 or l2 or Carry_Var:
            p = l1.val if l1 else 0
            q = l2.val if l2 else 0

            Sum = p + q + Carry_Var
            Carry_Var = Sum // 10
            digit = Sum % 10

            pointer.next = ListNode(digit)
            pointer = pointer.next

            if l1:
                l1 = l1.next
            if l2:
                l2 = l2.next

        return DummyNode_head.next


# Example : Two linked lists ->  numbers: 342 & 465
l1 = ListNode(2, ListNode(4, ListNode(3)))
l2 = ListNode(5, ListNode(6, ListNode(4)))

solution = Solution()
result = solution.addTwoNumbers(l1, l2)

while result:
    print(result.val, end=" -> ")
    result = result.next
