# Performance Comparison and Observations

**Field:** 11 × 15 grid, start (0, 0), goal (10, 0)
**Movement:** 4 directions (up, down, left, right)
**Step cost:** 1 per move, or 1 + 4 = 5 for a cell touching a crop-protection region

- **Experiment 1:** all obstacles are known. BFS, DFS, Greedy and A* each plan once.
- **Experiment 2:** two obstacles are unknown at the start. The robot uses A*, detects them while moving and replans.

## Comparison table

| Algorithm | Reached destination | Path length (moves) | Path cost | Expanded nodes | Execution time (ms) | Replanning operations |
|---|---|---|---|---|---|---|
| BFS | Yes | 20 | 40 | 87 | 0.101 | – |
| DFS | Yes | 46 | 46 | 52 | 0.048 | – |
| Greedy Best-First | Yes | 20 | 40 | 41 | 0.049 | – |
| A* | Yes | 38 | **38** | 89 | 0.297 | – |
| A* + replanning (Exp. 2) | Yes | 48 | 48 | 157 | 0.833 | **2** |

- *Path length* is the number of moves.
- *Path cost* includes the crop-safety penalty.
- *Execution time* changes a little between runs and computers. Everything else stays the same every time.

## Observations

1. **All algorithms reached the goal and none of them entered a potato row, protected cell or obstacle.** This is because every algorithm gets its moves from the same `get_neighbours()` function, which never returns a blocked cell.

2. **BFS found the shortest path in number of moves (20).** This route goes through the narrow gap at column 5, right next to the protected region, and passes 5 caution cells. BFS counts every move as equal, so it has no way to tell that this route is risky, and its cost comes out at 40.

3. **DFS gave the longest path (46 moves).** It just follows its neighbour order (right first) as deep as it can, so the route wanders around the right side of the field. This time it happened to avoid the protected area, but that was luck and not planning. It expanded fewer nodes than BFS here, but the quality of its path cannot be relied on.

4. **Greedy Best-First expanded the fewest nodes (41) and was among the fastest.** The heuristic pulled it straight towards the goal, so it took the same risky column-5 route as BFS (cost 40). This shows that Greedy is quick but ignores the cost already paid, so it is not guaranteed to find the best route.

5. **A\* found the lowest-cost route (38), which is the crop-safe one.** It goes around the right side and doesn't touch a single caution cell. The route is 18 moves longer, but it is still cheaper than taking the 20-move route with the penalty. This is exactly the "prefer the safer alternative" behaviour that Task 4 asks for. We also checked separately with Dijkstra's algorithm that 38 is the true minimum cost for this field.

6. **A\* expanded the most nodes (89) in Experiment 1.** The Manhattan heuristic only measures distance and knows nothing about the penalty. So A* first explores the cheap-looking cells near the protected area, finds them expensive, and then has to search the longer safe route as well. A* is also a bit slower than the others because of the priority queue and cost bookkeeping. On a grid this size the difference is still well under a millisecond.

7. **Replanning worked as expected.** The first plan went down column 10, through a cell that was actually blocked.
   - At (8, 10) the robot sensed the hidden obstacle at (9, 10), updated its map and replanned (replan 1).
   - The new route went along row 6, where it sensed the second hidden obstacle at (6, 12) from (6, 11) and replanned again (replan 2).
   - The robot still reached the goal without touching protected crops.

8. **Not knowing the map has a price.** The robot travelled 48 moves (cost 48), compared with 38 if it had known about the obstacles from the start, because it had to backtrack twice, once from (8, 10) and once from (6, 11). It also expanded 157 nodes in total over the three A* runs. Even so, replanning only from the current position meant the robot never had to restart the whole journey.

## Summary

- BFS gives the fewest moves.
- Greedy is the cheapest to compute.
- DFS is fast but gives poor routes.
- A* is the only one that balances distance and crop safety, which makes it the right choice for this robot.
- With replanning, A* also copes with obstacles that only show up during the journey.
