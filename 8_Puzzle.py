#-----Taxi-Rank Heuristic Function-----

GOAL = [1, 2, 3, 4, 5, 6, 7, 8, 0]

def goal_positions(goal):
    #Map each tile value to its (row, col) in the goal state
    return {goal[i]: (i // 3, i % 3) for i in range(9)}

def taxi_rank_heuristic(state, goal=GOAL):
    """
    h(n) = sum of Manhattan distances
         + 0.5 * number of row/column conflict pairs
    """
    goal_pos = goal_positions(goal)
    manhattan_total = 0
    conflicts = 0

    for v in range(1, 9):  # tiles 1-8
        idx = state.index(v)
        r,  c  = idx // 3, idx % 3
        gr, gc = goal_pos[v]
        manhattan_total += abs(r - gr) + abs(c - gc)

        for u in range(v + 1, 9):  # compare every pair once
            jdx = state.index(u)
            r2,  c2  = jdx // 3, jdx % 3
            gr2, gc2 = goal_pos[u]

            # ---Row Conflict---
            # same current row, same goal row, reversed order
            if r == r2 and gr == gr2:
                if (c < c2 and gc > gc2) or (c > c2 and gc < gc2):
                    conflicts += 1

            # --- Column conflict ---
            if c == c2 and gc == gc2:
                if (r < r2 and gr > gr2) or (r > r2 and gr < gr2):
                    conflicts += 1

    return manhattan_total + 0.5 * conflicts

#-----A* Search with Taxi-Rank-----

import heapq

def get_neighbors(state):

    neighbors = []
    blank = state.index(0)
    br, bc = blank // 3, blank % 3
    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        nr, nc = br + dr, bc + dc
        if 0 <= nr < 3 and 0 <= nc < 3:
            ni = nr * 3 + nc
            ns = list(state)
            ns[blank], ns[ni] = ns[ni], ns[blank]
            neighbors.append(tuple(ns))
    return neighbors

def astar_taxi(start, goal=GOAL):

    start = tuple(start)
    goal  = tuple(goal)
    h0    = taxi_rank_heuristic(start)

    # Priority queue: (f, g, state, path)
    open_set = [(h0, 0, start, [start])]
    visited  = {}          # state -> best g seen
    nodes_explored = 0

    while open_set:
        f, g, state, path = heapq.heappop(open_set)
        nodes_explored += 1

        if state == goal:
            return path, nodes_explored

        for neighbor in get_neighbors(state):
            ng = g + 1
            if neighbor not in visited or visited[neighbor] > ng:
                visited[neighbor] = ng
                nh = taxi_rank_heuristic(neighbor)
                heapq.heappush(
                    open_set,
                    (ng + nh, ng, neighbor, path + [neighbor])
                )

    return None, nodes_explored   # unsolvable

# UsageExample
if __name__ == '__main__':
    start_state = [2, 3, 6, 4, 1, 5, 7, 8, 0]
    goal_state  = [1, 2, 3, 4, 5, 6, 7, 8, 0]

    path, explored = astar_taxi(start_state, goal_state)

    if path:
        print(f'Optimal solution: {len(path) - 1} moves')
        print(f'Nodes explored:   {explored}')
        print(f'h(start):         {taxi_rank_heuristic(start_state)}')
        for step, state in enumerate(path):
            grid = [state[i*3:(i+1)*3] for i in range(3)]
            print(f'Step {step}: {grid}')
    else:
        print('No solution found.')
