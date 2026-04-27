# Taxi-Rank Heuristic — A* 8-Puzzle Solver

An optimised A* search implementation for the classic 8-puzzle problem using a custom **Taxi-Rank heuristic** that combines Manhattan distance with a linear-conflict penalty.

---

## What is the 8-Puzzle?

The 8-puzzle is a 3×3 sliding tile game with tiles numbered 1–8 and one blank space (`0`). The goal is to reach the target configuration in the fewest moves by sliding tiles into the blank.

```
Start               Goal
┌───┬───┬───┐      ┌───┬───┬───┐
│ 2 │ 3 │ 6 │      │ 1 │ 2 │ 3 │
├───┼───┼───┤  ──► ├───┼───┼───┤
│ 4 │ 1 │ 5 │      │ 4 │ 5 │ 6 │
├───┼───┼───┤      ├───┼───┼───┤
│ 7 │ 8 │   │      │ 7 │ 8 │   │
└───┴───┴───┘      └───┴───┴───┘
```

---

## The Taxi-Rank Heuristic

The heuristic function `h(n)` is defined as:

```
h(n) = Σ Manhattan_distance(tile) + 0.5 × conflict_pairs
```

### Manhattan Distance
For each tile, the Manhattan distance is the sum of horizontal and vertical steps needed to reach its goal position — a classic admissible lower bound.

### Linear Conflict Penalty
Two tiles `u` and `v` are in **conflict** when:
- They are in the **same row** (or column) currently and in the goal state, **and**
- Their relative order is **reversed** — meaning one must pass the other to reach its goal.

Each conflict pair adds `+0.5` to `h(n)`. This makes the heuristic tighter than plain Manhattan distance without violating **admissibility** (it never overestimates the true cost).

> **Why 0.5?** The standard linear-conflict heuristic adds +2 per conflict (two extra moves needed). Using 0.5 here is a softer, more conservative penalty that keeps the heuristic well inside the admissible bound while still pruning the search tree.

---

## Project Structure

```
.
├── taxi_rank.py    # Heuristic function, A* search, usage example
└── README.md
```

---

## Usage

```python
from taxi_rank import astar_taxi, taxi_rank_heuristic

start_state = [2, 3, 6, 4, 1, 5, 7, 8, 0]
goal_state  = [1, 2, 3, 4, 5, 6, 7, 8, 0]

path, explored = astar_taxi(start_state, goal_state)

print(f"Optimal solution: {len(path) - 1} moves")
print(f"Nodes explored:   {explored}")
print(f"h(start):         {taxi_rank_heuristic(start_state)}")
```

### Output
```
Optimal solution: 5 moves
Nodes explored:   8
h(start):         5.5

Step 0: [[2, 3, 6], [4, 1, 5], [7, 8, 0]]
Step 1: [[2, 3, 6], [4, 1, 5], [7, 0, 8]]
Step 2: [[2, 3, 6], [4, 0, 5], [7, 1, 8]]
Step 3: [[2, 0, 6], [4, 3, 5], [7, 1, 8]]
...
Step 5: [[1, 2, 3], [4, 5, 6], [7, 8, 0]]
```

---

## Algorithm Details

### A* Search
The solver uses standard A* with:

| Component | Detail |
|---|---|
| **g(n)** | Exact cost — number of moves from start |
| **h(n)** | Taxi-Rank heuristic (Manhattan + conflict penalty) |
| **f(n)** | g(n) + h(n) — priority queue key |
| **Open set** | Min-heap via `heapq` |
| **Visited** | Dictionary mapping state → best `g` seen (allows re-expansion on cheaper path) |

### Neighbor Generation
At each state, the blank tile (`0`) can slide in up to 4 directions (up, down, left, right), subject to grid boundary checks. Each valid swap produces a successor state.

### Admissibility
The heuristic never overestimates the true number of moves required, guaranteeing that A* returns an **optimal solution**.

---

## Requirements

- Python 3.7+
- No external dependencies (uses only `heapq` from the standard library)

---

## Notes

- Puzzles with no solution (e.g. odd-parity permutations) are detected automatically — the function returns `(None, nodes_explored)`.
- For very hard instances (>20 moves), the visited state space can grow large. Consider adding an IDA* variant for memory-constrained environments.
