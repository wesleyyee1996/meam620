

class Node:

    def __init__(self, g, h, index, parent):
        self.f = g+h                # total cost
        self.g = g                  # cost to come
        self.h = h                  # heuristic
        self.index = index          # (i,j,k)
        self.parent = parent        # (i,j,k)
        self.is_closed = False      # True if node has been closed

    def __lt__(self, other):
        return self.f < other.f

    def __repr__(self):
        if self.parent:
            return f"Node g={self.g}, h={self.h}, index={self.index}, parent={self.parent.index}, is_closed={self.is_closed}"
        else:
            return f"Node g={self.g}, h={self.h}, index={self.index}, parent=None, is_closed={self.is_closed}"

