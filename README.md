# Pathfinding Visualizer

A terminal program that generates random weighted mazes, solves them with **BFS**, **Dijkstra** and **A\***, animates each search, and compares how much work each algorithm did. Written in C++17 with no external dependencies.

## Output Demo

![Program Output](assets/output.webp)

## Features

- **Maze generator:** recursive backtracker, optional loops, and weighted "mud" terrain
- **Three solvers** (BFS, Dijkstra, A\*) behind one shared interface
- **Terminal animation** of the frontier, the explored cells and the final path, using ANSI colors
- **Comparison table** for each maze: nodes explored, path cost, path length, frontier size, runtime
- **Benchmark mode** that averages every metric over many random mazes
- **Reproducible mazes:** every run prints its seed, and `--seed` recreates the same maze

## Results

Averages over 300 random 21x35 mazes (seed 1). Mud costs 5 to enter and road costs 1.

### Weighted mazes (default settings: 10% loops, 15% mud)

| Algorithm | Avg cells explored | Avg path cost | Avg max frontier |
|-----------|-------------------:|--------------:|-----------------:|
| BFS       | 321.5              | 110.8         | 12.4             |
| Dijkstra  | 311.2              | 101.7         | 13.2             |
| A\*       | 227.6              | 101.7         | 15.7             |

Reproduce with `./pathviz --bench 300 --seed 1`.

### Unweighted mazes with loops (every step costs 1)

| Algorithm | Avg cells explored | Avg path cost | Avg max frontier |
|-----------|-------------------:|--------------:|-----------------:|
| BFS       | 371.9              | 55.5          | 15.7             |
| Dijkstra  | 373.4              | 55.5          | 15.8             |
| A\*       | 144.4              | 55.5          | 23.1             |

Reproduce with `./pathviz --bench 300 --seed 1 --mud 0 --loops 0.2`.

### Takeaways

- **BFS is fooled by mud.** It minimizes the number of steps, not the cost, so it returned a more expensive path than the optimal one in 191 of 300 mazes (about 64%). On average its path cost about 9% more.
- **Dijkstra and A\* always find the same optimal cost.** The benchmark checks this on every maze and prints a warning if they ever disagree.
- **A\* explores fewer cells:** about 27% fewer than Dijkstra on weighted mazes and about 61% fewer on unweighted ones.
- **The price is a larger frontier.** A\* keeps more cells waiting at once (15.7 vs 13.2 on weighted mazes), so it trades memory for fewer explored cells.
- **When every step costs the same, BFS and Dijkstra do the same work** (371.9 vs 373.4 cells explored), because Dijkstra's ordering by cost reduces to BFS's ordering by steps.
- **In a perfect maze** (a single route, `--loops 0 --mud 0`) all three return the same path, and A\* still explores somewhat fewer cells (192.8 vs 220.4).

Runtimes are printed too, but at this maze size they are only a few microseconds and noisy, so the explored counts are the more reliable comparison.

## Getting started

### Requirements

- A C++17 compiler (developed with g++ from MinGW-w64)
- A terminal that supports ANSI escape codes. On Windows, [Windows Terminal](https://aka.ms/terminal) is recommended.
- About `rows + 7` terminal lines for the animation (28 with the default size). Use `--rows 15` on a small window.

Developed and tested on Windows 11. The code is standard C++17 and uses only ANSI escape codes, so it should also build on Linux and macOS.

### Build

```
g++ -std=c++17 -O2 -o pathviz pathfinder.cpp
```

### Run

```
./pathviz
```

In `cmd.exe` use `.\pathviz` instead of `./pathviz`.

## Usage

```
./pathviz                  # animated run on a random maze
./pathviz --seed 7         # reproducible maze
./pathviz --no-anim        # comparison table only
./pathviz --bench 200      # average statistics over 200 random mazes
./pathviz --rows 15 --cols 31 --time 2
```

| Option        | Meaning                                                              | Default |
|---------------|----------------------------------------------------------------------|---------|
| `--rows N`    | Maze height in cells (rounded up to an odd number, minimum 5)        | 21      |
| `--cols N`    | Maze width in cells (rounded up to an odd number, minimum 5)         | 35      |
| `--seed N`    | Random seed, for reproducible mazes                                  | random  |
| `--loops F`   | Chance (0 to 1) of removing each wall between corridors; 0 gives a perfect maze with a single route | 0.10 |
| `--mud F`     | Fraction (0 to 1) of open cells turned into mud; 0 makes every step cost the same | 0.15 |
| `--time S`    | Seconds per animation                                                | 4       |
| `--no-anim`   | Skip the animation and only print the comparison table               |         |
| `--bench N`   | Solve N random mazes and print average statistics                    |         |
| `--help`      | Show all options                                                     |         |

### Reading the output

Each single-maze run ends with a table such as:

```
Algorithm   Explored  Path cost   Steps  Max frontier   Time (us)
```

| Column       | Meaning                                                                         |
|--------------|---------------------------------------------------------------------------------|
| Explored     | Number of cells the algorithm expanded ("nodes explored")                       |
| Path cost    | Total cost of entering every cell on the path (road = 1, mud = 5)               |
| Steps        | Number of moves on the path                                                     |
| Max frontier | Largest number of discovered-but-unexpanded cells at any moment (a memory proxy) |
| Time (us)    | Runtime in microseconds, averaged over 200 runs                                 |

In the animation: **yellow** cells are on the frontier, **blue** cells have been explored, the **green** line is the final path, **brown** cells are mud, **red** is the start and **magenta** is the goal.

## How it works

### Maze generation

1. **Carve a perfect maze** with a randomized depth-first search (the recursive backtracker). Every open cell is reachable, and there is exactly one route between any two cells.
2. **Add loops** by removing random walls that separate two corridors. With only one route, all three algorithms would have to return the same path. Loops create alternatives.
3. **Add mud patches.** Each patch is a plus shape (a cell and its four neighbors), and walls cut parts of it off, so patches vary in shape. Entering mud costs 5, so the path with the fewest steps is no longer always the cheapest one.

Cells sit on odd coordinates and the squares between them are walls that can be carved open, which is why the maze size is always odd. The start is the top-left cell and the goal is the bottom-right cell.

### Algorithms

| Algorithm | Frontier ordered by | Finds the cheapest path? |
|-----------|---------------------|--------------------------|
| BFS       | Order of discovery (a queue) | Only when every step costs the same |
| Dijkstra  | Cost paid so far (a min-heap) | Yes, for non-negative costs |
| A\*       | Cost so far plus an estimate of the cost remaining (a min-heap) | Yes, if the estimate never overestimates |

- A\* uses the Manhattan distance as its estimate. Every step costs at least 1, so the real remaining cost can never be smaller, which keeps the heuristic admissible. Ties are broken in favor of cells closer to the goal.
- Dijkstra and A\* use lazy deletion: when a cheaper route to a cell is found, a new heap entry is pushed and the outdated one is skipped when it surfaces.
- All three stop when the goal is expanded, so the explored counts are directly comparable.

### Rendering

The solvers never draw anything. They record what they did as a list of events (a cell was discovered, a cell was expanded), and the renderer replays that list afterwards. Each maze cell is drawn as two characters with a colored background, and the cursor jumps to a cell with an ANSI escape code so that only that cell is repainted. This keeps the algorithms independent of the display, so another front end can be added without touching them.

## Project layout

- `pathfinder.cpp`: the whole program, organized in six commented sections (grid, maze generation, solvers, renderer, reports, `main`)

## Limitations

- Movement is 4-directional (no diagonals).
- The same seed gives the same maze on the same compiler and standard library. A different standard library may generate different mazes, so benchmark numbers can differ slightly across toolchains.
- Very old terminals (such as the classic Windows console without ANSI support) will show raw escape characters instead of colors.
- On small mazes, runtimes are only microseconds, so timing differences are noisy.

## Ideas for extension

- A graphical front end that replays the same recorded events
- Diagonal movement with a matching heuristic
- Other heuristics, or additional algorithms such as greedy best-first search