# AI-Based Crop-Safe Path Planning for an Autonomous Potato-Field Robot

**Artificial Intelligence – Group Coding Assignment | Group XX**

This project simulates a small robot that has to move through a potato field from a start point to a destination. The field is modelled as a 2D grid. The robot must never drive over potato plants, protected crop areas or obstacles, and it should prefer routes that keep a safe distance from protected crops. We used four search algorithms from class (BFS, DFS, Greedy Best-First and A*) to plan the route and compared how they perform. We also show what happens when the robot runs into obstacles it did not know about: it notices them, updates its map and replans with A*.

Group members and their contributions are listed in [CONTRIBUTIONS.md](CONTRIBUTIONS.md).

---

## Repository contents

| File / folder | What it contains |
|---|---|
| `crop_safe_planner.py` | Complete Python source code (all six tasks) |
| `README.md` | This file: project explanation, algorithms, libraries and how to run |
| `screenshots/` | Output screenshots (field map, paths of each algorithm, replanning demo, console output) |
| `RESULTS.md` | Performance comparison table and our observations |
| `CONTRIBUTIONS.md` | Names, roll numbers and contribution of each member |

## Requirements

- Python 3.8 or newer
- Matplotlib (for drawing the field and paths)

Everything else (`heapq`, `collections`, `time`, `os`) comes with Python.

```bash
pip install matplotlib
```

## How to run

```bash
python crop_safe_planner.py
```

The program prints the replanning messages and the comparison table in the terminal, opens three figure windows, and saves the figures in the `screenshots/` folder.

---

## Task 1 – Field environment

The field is written as a list of strings in `FIELD_MAP`, so the layout is easy to read and change. It is 11 rows × 15 columns.

| Symbol | Meaning | Colour in plot |
|---|---|---|
| `.` | Safe travelling cell (soil between rows) | light beige |
| `P` | Potato row (plants) | green |
| `C` | Crop-protection region (for example, young or treated plants) | orange |
| `#` | Known obstacle (stone, tool, etc.) | grey |
| `X` | Obstacle the robot does **not** know about at the start | dark red |
| `S` / `G` | Start and destination | blue circle / pink star |

Free cells next to a protected region are shaded light orange. These are the "caution" cells that get a cost penalty (Task 4).

The function `build_grid()` turns the strings into a grid. It builds two versions of the field:
- the **real field**, where `X` is an obstacle;
- the **robot's map**, where `X` looks like a normal free cell, because the robot hasn't seen it yet.

![Field map](screenshots/1_field_map.png)

## Task 2 – Problem formulation

| Component | Definition |
|---|---|
| State | Robot position `(row, col)` |
| Initial state | Start cell `S = (0, 0)` |
| Actions | Move **Up, Down, Left, Right** by one cell. Diagonal moves are not allowed, so the robot cannot cut corners across plant rows. |
| Transition model | `(row, col) + move → (row + dr, col + dc)`. The move is allowed only if the new cell is inside the grid and is not `P`, `C` or `#`. |
| Goal test | Robot position equals `G = (10, 0)` |
| Path cost | Sum of step costs. A normal step costs 1; a step into a cell touching a protected region costs 1 + PENALTY (= 5). |

In the code, `get_neighbours()` implements the actions and the transition model, `is_goal()` is the goal test, and `step_cost()` / `path_cost()` give the path cost.

## Task 3 – Search algorithms

All four algorithms use the same `get_neighbours()` function, so they all follow the same movement rules. Each one returns the path and the number of **expanded nodes**, meaning the cells taken out of the frontier and checked.

- **BFS** uses a FIFO queue (`deque`). It explores level by level, so it finds the path with the fewest moves, but it ignores step costs.
- **DFS** uses a stack (LIFO). It goes as deep as possible in one direction before backtracking. It finds *a* path, usually not a short one.
- **Greedy Best-First Search** uses a priority queue ordered by `h(n)` only. It always moves towards the cell that looks closest to the goal. It is fast, but it ignores the cost already paid, so the route is not guaranteed to be the best.
- **A\*** uses a priority queue ordered by `f(n) = g(n) + h(n)`, where `g(n)` is the real cost from the start (including penalties) and `h(n)` is the heuristic. It returns the lowest-cost path.

### Heuristic

We used the **Manhattan distance**:

```
h(n) = |row(n) − row(goal)| + |col(n) − col(goal)|
```

It suits this problem because the robot only moves in four directions, so Manhattan distance is the number of moves needed if there were nothing in the way. Every step costs at least 1, so `h(n)` never overestimates the true remaining cost (it is **admissible**). It also never drops by more than one step cost between neighbouring cells (it is **consistent**). That is why A* is guaranteed to return the cheapest path here.

## Task 4 – Crop-safe planning

There are two kinds of safety rules:

1. **Hard rule:** potato rows (`P`), protected cells (`C`) and obstacles (`#`) are listed in `BLOCKED`. `get_neighbours()` never returns them, so no algorithm can ever plan a path through them.
2. **Soft rule (penalty):** `near_protected()` checks the 8 surrounding cells. If any of them is a protected cell, entering that free cell costs `1 + PENALTY` instead of 1. We used `PENALTY = 4`.

Because A* minimises total cost, it compares a short route that brushes past the protected area with a longer route that stays away from it, and picks the safer one when it is cheaper overall. In our field, the shortest route (20 moves) passes 5 caution cells (cost 40). A* chooses a 38-move route with **no** caution cells (cost 38).

## Task 5 – Obstacle detection and replanning

The field contains two obstacles (`X`) that the robot doesn't know about. The simulation works like this:

1. The robot plans a route with A* on its own map.
2. At every cell, `sense()` looks at the four neighbouring cells in the **real** field. This is our simulated sensor.
3. If it finds an obstacle that isn't on its map yet, the robot adds it to its map.
4. If that obstacle lies on the current route, the robot runs A* again from its current position. Each time this happens, one replanning operation is counted.
5. The robot moves one step and repeats until it reaches the goal (or no safe route is left).

In our run, the first plan goes down the corridor at column 10. At (8, 10) the robot finds the hidden obstacle at (9, 10) and replans. On the new route, at (6, 11), it finds the second hidden obstacle at (6, 12) and replans again. It finally reaches the goal with **2 replans**.

![Replanning](screenshots/3_replanning.png)

## Task 6 – Performance evaluation

Each algorithm is run on the fully known field, and its time is measured with `time.perf_counter()`. The online experiment (A* with replanning) is reported in the same table.

| Algorithm | Reached | Path length (moves) | Path cost | Expanded nodes | Time (ms) | Replans |
|---|---|---|---|---|---|---|
| BFS | Yes | 20 | 40 | 87 | 0.101 | – |
| DFS | Yes | 46 | 46 | 52 | 0.048 | – |
| Greedy Best-First | Yes | 20 | 40 | 41 | 0.049 | – |
| A* | Yes | 38 | **38** | 89 | 0.297 | – |
| A* + replanning (unknown obstacles) | Yes | 48 | 48 | 157 | 0.833 | **2** |

*Times change slightly every run and from one computer to another. The other values stay the same.*

Our observations are in [RESULTS.md](RESULTS.md).

![Paths of all algorithms](screenshots/2_algorithm_paths.png)

![Console output](screenshots/4_console_output.png)

---

## References

- S. Russell and P. Norvig, *Artificial Intelligence: A Modern Approach*: the search problem formulation, BFS, DFS, Greedy Best-First and A* (concepts only).
- Lecture notes from our AI course.
- Python documentation for `heapq` and `collections.deque`, and the Matplotlib documentation for `imshow` and plotting.

The code was written by our group. No code was copied from other groups or online sources.
