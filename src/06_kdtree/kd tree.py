from collections import namedtuple
from collections import deque
from operator import itemgetter
from pprint import pformat


class Node(namedtuple('Node', 'location left_child right_child')):
    def __repr__(self):
        return pformat(tuple(self))


def kdtree(point_list, depth=0):
    if not point_list:
        return None

    k = len(point_list[0])  # assumes all points have the same dimension
    # Select axis based on depth so that axis cycles through all valid values
    axis = depth % k

    # Sort point list by axis and choose median as pivot element
    point_list.sort(key=itemgetter(axis))
    median = len(point_list) // 2

    # Create node and construct subtrees
    return Node(
        location=point_list[median],
        left_child=kdtree(point_list[:median], depth + 1),
        right_child=kdtree(point_list[median + 1:], depth + 1)
    )

def printTree(root):
    buf = deque()
    output = []
    if not root:
        print('$')
    else:
        buf.append(root)
        count, nextCount = 1, 0
        while count:
            node = buf.popleft()
            if node:
                output.append(node.location)
                count -= 1
                for n in (node.left_child, node.right_child):
                    if n:
                        buf.append(n)
                        nextCount += 1
                    else:
                        buf.append(None)
            else:
                output.append('$')
            if not count:
                print(output)
                output = []
                count, nextCount = nextCount, 0
        # print the remaining all empty leaf node part
        output.extend(['$']*len(buf))
        print(output)

def main():
    """Example usage"""
    point_list = [(5,0,-3),(8,-1,-4),(2,-10,8),(-8,5,6),(-7,7,-6),(5,7,15),(7,3,4),(3,2,2),(-4,-1,3),(9,1,-4),(-1,5,7),(0,7,0),(0,5,2),(2,4,-1)
                  ,(4,7,4),(9,-8,5),(10,-3,-7)]
    tree = kdtree(point_list)
    print(tree)
    printTree(tree)


if __name__ == '__main__':
    main()