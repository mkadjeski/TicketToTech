import time
from collections import deque
import heapq


# ==========================================================
# ===============  GRAPH DEFINITIONS  ======================
# ==========================================================

# ===== SMALL GRAPH =====
SMALL_GRAPH = [
    [0, 1, 1, 0],
    [1, 0, 1, 1],
    [1, 1, 0, 1],
    [0, 1, 1, 0]
]

# ===== MEDIUM GRAPH =====
MEDIUM_GRAPH = [

    [0,	1,	1,	0,	1,	1,	1,	0],
    [1,	0,	1,	1,	1,	1,	0,	0],
    [1,	1,	0,	1,	1,	1,	0,	0],
    [0,	1,	1,	0,	0,	0,	0,	0],
    [1,	1,	1,	0,	0,	1,	0,	0],
    [1,	1,	1,	0,	1,	0,	1,	0],
    [1,	0,	0,	0,	0,	1,	0,	1],
    [0,	0,	0,	0,	0,	0,	1,	0]
]

# ===== LARGE GRAPH =====
LARGE_GRAPH = [

    [0,	1,	1,	0,	1,	1,	1,	0,	0,	0,	0,	0,	0,	0,	1,	1],
    [1,	0,	1,	1,	1,	1,	0,	0,	1,	1,	0,	0,	0,	0,	0,	0],
    [1,	1,	0,	1,	1,	1,	0,	0,	0,	1,	1,	0,	0,	1,	0,	0],
    [0,	1,	1,	0,	0,	0,	0,	0,	0,	1,	1,	1,	0,	1,	0,	0],
    [1,	1,	1,	0,	0,	1,	0,	0,	0,	0,	1,	0,	0,	0,	0,	0],
    [1,	1,	1,	0,	1,	0,	1,	0,	1,	1,	0,	0,	0,	0,	0,	0],
    [1,	0,	0,	0,	0,	1,	0,	1,	1,	1,	0,	1,	0,	1,	1,	1],
    [0,	0,	0,	0,	0,	0,	1,	0,	1,	1,	0,	1,	1,	1,	1,	1],
    [0,	1,	0,	0,	0,	1,	1,	1,	0,	1,	0,	1,	0,	1,  0,	0],
    [0,	1,	1,	1,	0,	1,	1,	1,	1,	0,	1,	1,	0,	1,	0,	0],
    [0,	0,	1,	1,	1,	0,	0,	0,	0,	1,	0,	1,	0,	0,	0,	0],
    [0,	0,	0,	1,	0,	0,	1,	1,	1,	1,	1,	0,	1,	1,	0,	0],
    [0,	0,	0,	0,	0,	0,	0,	1,	0,	0,	0,	1,	0,	1,	0,	1],
    [0,	0,	1,	1,	0,	0,	1,	1,	1,	1,	0,	1,	1,	0,	0,	0],
    [1,	0,	0,	0,	0,	0,	1,	1,	0,	0,	0,	0,	0,	0,	0,	1],
    [1,	0,	0,	0,	0,	0,	1,	1,	0,	0,	0,	0,	1,	0,	1,	0]
]

# ===== SELECT WHICH GRAPH TO USE =====
# graph = SMALL_GRAPH
# graph = MEDIUM_GRAPH
graph = SMALL_GRAPH   # <---- currently using small graph


# ==========================================================
# ===============  SEARCH ALGORITHMS  ======================
# ==========================================================
#BFS
def BFS(start, goal):
    q = deque([start])
    parent = [-1] * len(graph)
    visited = [False] * len(graph)
    visited[start] = True

    while q:
        node = q.popleft()
        if node == goal:
            break

        for neighbor in range(len(graph[node])):
            if graph[node][neighbor] != 0 and not visited[neighbor]:
                visited[neighbor] = True
                parent[neighbor] = node
                q.append(neighbor)

    if not visited[goal]:
        return []

    path = []
    cur = goal
    while cur != -1:
        path.append(cur)
        cur = parent[cur]
    return list(reversed(path))

#DFS
def DFS(start, goal):
    stack = [start]
    parent = [-1] * len(graph)
    visited = [False] * len(graph)
    visited[start] = True

    while stack:
        node = stack.pop()
        if node == goal:
            break

        for neighbor in reversed(range(len(graph[node]))):
            if graph[node][neighbor] != 0 and not visited[neighbor]:
                visited[neighbor] = True
                parent[neighbor] = node
                stack.append(neighbor)

    if not visited[goal]:
        return []

    path = []
    cur = goal
    while cur != -1:
        path.append(cur)
        cur = parent[cur]
    return list(reversed(path))


def heuristic(a, b):
    return abs(a - b)


def AStar(start, goal):
    pq = []
    heapq.heappush(pq, (heuristic(start, goal), 0.0, start))

    parent = {start: -1}
    gScore = {start: 0.0}

    while pq:
        f, g, node = heapq.heappop(pq)
        if node == goal:
            break

        for neighbor in range(len(graph[node])):
            if graph[node][neighbor] != 0:
                new_g = g + 1
                if neighbor not in gScore or new_g < gScore[neighbor]:
                    gScore[neighbor] = new_g
                    new_f = new_g + heuristic(neighbor, goal)
                    heapq.heappush(pq, (new_f, new_g, neighbor))
                    parent[neighbor] = node

    if goal not in parent:
        return []

    path = []
    cur = goal
    while cur != -1:
        path.append(cur)
        cur = parent[cur]
    return list(reversed(path))

#Bidirectional
def Bidirectional(start, goal):
    fParent = {start: -1}
    bParent = {goal: -1}

    fQ = deque([start])
    bQ = deque([goal])

    while fQ and bQ:
        f = fQ.popleft()
        if f in bParent:
            path1 = []
            cur = f
            while cur != -1:
                path1.append(cur)
                cur = fParent[cur]
            path1.reverse()

            path2 = []
            cur = bParent[f]
            while cur != -1:
                path2.append(cur)
                cur = bParent[cur]

            return path1 + path2

        for neighbor in range(len(graph[f])):
            if graph[f][neighbor] != 0 and neighbor not in fParent:
                fParent[neighbor] = f
                fQ.append(neighbor)

        b = bQ.popleft()
        if b in fParent:
            path1 = []
            cur = fParent[b]
            while cur != -1:
                path1.append(cur)
                cur = fParent[cur]
            path1.reverse()

            path2 = []
            cur = b
            while cur != -1:
                path2.append(cur)
                cur = bParent[cur]

            return path1 + path2

        for neighbor in range(len(graph[b])):
            if graph[b][neighbor] != 0 and neighbor not in bParent:
                bParent[neighbor] = b
                bQ.append(neighbor)

    return []

# Brute Force (exhaustive search of all simple paths)
def BruteForce(start, goal):
    n = len(graph)
    visited = [False] * n
    best_path = []
    current_path = []

    def dfs(node):
        nonlocal best_path

        visited[node] = True
        current_path.append(node)

        # If reached goal, check if new shortest path
        if node == goal:
            if len(best_path) == 0 or len(current_path) < len(best_path):
                best_path = current_path.copy()

        else:
            for neighbor in range(n):
                if graph[node][neighbor] != 0 and not visited[neighbor]:
                    dfs(neighbor)

        # Backtrack
        current_path.pop()
        visited[node] = False

    dfs(start)
    return best_path



# ==========================================================
# ===================== PRINTING ===========================
# ==========================================================

def printPath(path, name):
    print(f"{name} Path ({max(0, len(path)-1)} steps): ", end="")
    for p in path:
        print(p + 1, end=" ")
    print()


# ==========================================================
# ======================= MAIN LOOP ========================
# ==========================================================

def main():
    while True:
        if len(graph) == 4:
            print("\nHere is the list of locations in order.")
            print("\t1. Ashraf Islam Engineering Building" )
            print("\t2. Bruner Hall" )
            print("\t3. Library" )
            print("\t4. RUC" )
        elif len(graph) == 8:
            print("\nHere is the list of locations in order.")
            print("\t1. Ashraf Islam Engineering Building" )
            print("\t2. Bruner Hall" )
            print("\t3. Library" )
            print("\t4. RUC" )
            print("\t5. Parking Lot" )
            print("\t6. Prescott Hall" )
            print("\t7. LSC" )
            print("\t8. Stonecipher" )
        else:
            print("\nHere is the list of locations in order.")
            print("\t1. Ashraf Islam Engineering Building" )
            print("\t2. Bruner Hall" )
            print("\t3. Library" )
            print("\t4. RUC" )
            print("\t5. Parking Lot" )
            print("\t6. Prescott Hall" )
            print("\t7. LSC" )
            print("\t8. Stonecipher" )
            print("\t9. Brown Hall" )
            print("\t10. Clement Hall" )
            print("\t11. BFA" )
            print("\t12. Mem Gym" )
            print("\t13. Bell Hall" )
            print("\t14. Derryberry" )
            print("\t15. Fit" )
            print("\t16. Ray Morris" )

        max = len(graph)

        print(f"\n\nWhere would you like to start? Enter 1-{max}: ", end="")
        start = int(input()) - 1

        print(f"Where would you like to end? Enter 1–{max}: ", end="")
        goal = int(input()) - 1

        # BFS
        t0 = time.perf_counter()
        bfs_path = BFS(start, goal)
        t1 = time.perf_counter()
        printPath(bfs_path, "BFS")
        print(f"BFS Time: {int((t1 - t0)*1_000_000)} microseconds\n")

        # DFS
        t0 = time.perf_counter()
        dfs_path = DFS(start, goal)
        t1 = time.perf_counter()
        printPath(dfs_path, "DFS")
        print(f"DFS Time: {int((t1 - t0)*1_000_000)} microseconds\n")

        # A*
        t0 = time.perf_counter()
        a_path = AStar(start, goal)
        t1 = time.perf_counter()
        printPath(a_path, "A*")
        print(f"A* Time: {int((t1 - t0)*1_000_000)} microseconds\n")

        # Bidirectional
        t0 = time.perf_counter()
        bi_path = Bidirectional(start, goal)
        t1 = time.perf_counter()
        printPath(bi_path, "Bidirectional")
        print(f"Bidirectional Time: {int((t1 - t0)*1_000_000)} microseconds\n")

        # # Brute Force
        t0 = time.perf_counter()
        bf_path = BruteForce(start, goal)
        t1 = time.perf_counter()
        printPath(bf_path, "Brute Force")
        print(f"Brute Force Time: {int((t1 - t0) * 1_000_000)} microseconds\n")

        again = input("Run again? (y/n): ").lower()
        if again == "n":
            break


if __name__ == "__main__":
    main()

