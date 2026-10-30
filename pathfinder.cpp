// =============================================================================
//  pathfinder.cpp - Pathfinding visualizer: BFS vs Dijkstra vs A* in the terminal
// =============================================================================
//
//  Generates a random maze (with loops and weighted mud patches), solves it with
//  three search algorithms, animates each search with ANSI colors, and compares
//  how much work each algorithm did.
//
//  Features
//    - Maze generator: recursive backtracker, optional loops, weighted mud terrain
//    - BFS, Dijkstra and A* (Manhattan heuristic) behind one shared interface
//    - Terminal animation of the frontier, the explored cells and the final path
//    - Per-maze comparison table: nodes explored, path cost, frontier size, time
//    - Benchmark mode that averages every metric over many random mazes
//    - Seeded generation, so any maze can be reproduced exactly
//
//  Build and run
//    g++ -std=c++17 -O2 -o pathviz pathfinder.cpp
//    ./pathviz
//
//  Usage
//    ./pathviz                 animated run on a random maze
//    ./pathviz --seed 7        reproducible maze
//    ./pathviz --no-anim       comparison table only, no animation
//    ./pathviz --bench 200     average statistics over 200 random mazes
//    ./pathviz --help          list every option
//
//  Requirements
//   A C++17 compiler and a terminal that supports ANSI escape codes (Windows
//    Terminal is recommended on Windows). The animated view needs about
//    (rows + 7) terminal lines, so use --rows 15 on a small window.
//    Developed and tested on Windows 11 with g++ (MinGW-w64).
//
//  Code layout
//    Section 1  Grid             the maze data structure and neighbor lookup
//    Section 2  Maze generation  backtracker + loops + mud
//    Section 3  Solvers          BFS, Dijkstra and A* behind one shared interface
//    Section 4  Renderer         ANSI-color animation that replays solver events
//    Section 5  Reports          comparison table and benchmark
//    Section 6  main()           command-line parsing and program flow
//
//  Design note: the solvers never draw anything. They only record what they did
//  (a list of events) and the renderer replays that list afterwards. This keeps
//  the algorithms independent of the display, so another front end can be added
//  without touching them.
// =============================================================================

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <queue>
#include <random>
#include <stack>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

using namespace std;

// =============================================================================
//  SECTION 1: GRID
// =============================================================================

// Every cell stores the cost of ENTERING it. Cost 0 is reserved for walls.
const int WALL = 0;  // impassable
const int ROAD = 1;  // normal floor: costs 1 to step onto
const int MUD  = 5;  // slow terrain: costs 5 to step onto

// The four movement directions (up, down, left, right). No diagonal moves.
// Index d refers to the same direction in both arrays.
const int DR[4] = {-1, 1, 0, 0};
const int DC[4] = {0, 0, -1, 1};

struct Grid {
    int rows, cols;
    vector<int> cost;  // flat array, row-major: cell (r,c) is at index r*cols + c

    Grid(int r, int c) : rows(r), cols(c), cost(r * c, WALL) {}

    // Convert between (row, col) and a single integer cell id. The solvers use
    // ids because they make per-cell arrays such as parent[] trivial to index.
    int id(int r, int c) const { return r * cols + c; }
    int row(int id) const { return id / cols; }
    int col(int id) const { return id % cols; }

    // True if (r, c) lies on the grid.
    bool inBounds(int r, int c) const {
        return r >= 0 && r < rows && c >= 0 && c < cols;
    }
    // True if (r, c) lies on the grid and is not a wall.
    bool isOpen(int r, int c) const {
        return inBounds(r, c) && cost[id(r, c)] != WALL;
    }
};

// Writes the ids of all walkable neighbors of `cell` into out[] and returns how
// many there are (0 to 4). All three solvers use this function.
int getNeighbors(const Grid& g, int cell, int out[4]) {
    int r = g.row(cell), c = g.col(cell);
    int count = 0;
    for (int d = 0; d < 4; d++) {
        int nr = r + DR[d], nc = c + DC[d];
        if (g.isOpen(nr, nc)) out[count++] = g.id(nr, nc);
    }
    return count;
}

// =============================================================================
//  SECTION 2: MAZE GENERATION
// =============================================================================
//
//  The maze is built in three steps:
//
//  1. Carve a perfect maze with a randomized depth-first search (the recursive
//     backtracker). Every open cell is reachable and there is exactly one route
//     between any two cells.
//  2. Add loops by removing some random walls. A perfect maze has a single
//     route, so all three algorithms would have to return the same path. Loops
//     create alternative routes, which is what makes the algorithms differ.
//  3. Add mud patches. Entering mud costs more than entering road, so the path
//     with the fewest steps is no longer always the cheapest one. This is where
//     BFS (which counts steps) falls behind Dijkstra and A* (which count cost).
//
//  Layout: cells sit on odd coordinates (1, 3, 5, ...) and the squares between
//  them are walls that can be carved open. Rows and columns must therefore be
//  odd, so that the outer border is wall on every side.

// Builds a random maze with the start at (1, 1) and the goal at (rows-2, cols-2).
//   rows, cols   grid size (must be odd)
//   loopChance   chance (0..1) that each wall between two corridors is removed
//   mudFraction  fraction (0..1) of the open cells to turn into mud
//   rng          random number generator, passed by reference so it keeps advancing
Grid generateMaze(int rows, int cols, double loopChance, double mudFraction,
                  mt19937& rng) {
    Grid g(rows, cols);

    // ---- Step 1: carve a perfect maze with an iterative DFS -----------------
    stack<int> st;
    g.cost[g.id(1, 1)] = ROAD;
    st.push(g.id(1, 1));

    while (!st.empty()) {
        int cur = st.top();
        int r = g.row(cur), c = g.col(cur);

        // Look at cells two steps away that are still solid wall (= unvisited).
        vector<int> options;
        for (int d = 0; d < 4; d++) {
            int nr = r + 2 * DR[d], nc = c + 2 * DC[d];
            bool insideBorder = nr >= 1 && nr < rows - 1 && nc >= 1 && nc < cols - 1;
            if (insideBorder && g.cost[g.id(nr, nc)] == WALL) options.push_back(d);
        }

        if (options.empty()) {  // dead end: backtrack
            st.pop();
            continue;
        }

        // Pick a random direction, open the wall in between AND the new cell.
        int d = options[rng() % options.size()];
        g.cost[g.id(r + DR[d], c + DC[d])] = ROAD;
        g.cost[g.id(r + 2 * DR[d], c + 2 * DC[d])] = ROAD;
        st.push(g.id(r + 2 * DR[d], c + 2 * DC[d]));
    }

    // ---- Step 2: add loops by removing random interior walls ----------------
    uniform_real_distribution<double> chance(0.0, 1.0);
    for (int r = 1; r < rows - 1; r++) {
        for (int c = 1; c < cols - 1; c++) {
            if (g.cost[g.id(r, c)] != WALL) continue;
            // A wall is only worth removing if it separates two open cells,
            // either left/right of it or above/below it.
            bool leftRight = g.isOpen(r, c - 1) && g.isOpen(r, c + 1);
            bool upDown    = g.isOpen(r - 1, c) && g.isOpen(r + 1, c);
            if ((leftRight || upDown) && chance(rng) < loopChance) {
                g.cost[g.id(r, c)] = ROAD;
            }
        }
    }

    // ---- Step 3: sprinkle mud patches ---------------------------------------
    // Each patch is a "plus": the cell we land on plus its up/down/left/right
    // neighbors. Walls cut parts of the plus off, so patches vary in shape.
    int openCells = 0;
    for (int v : g.cost) if (v != WALL) openCells++;
    int mudTarget = static_cast<int>(openCells * mudFraction);
    int mudPlaced = 0;

    int start = g.id(1, 1), goal = g.id(rows - 2, cols - 2);

    // Turns one cell into mud if it is plain road (never a wall, the start or
    // the goal) and counts it. Cells off the grid are ignored.
    auto makeMud = [&](int r, int c) {
        if (!g.inBounds(r, c)) return;
        int cell = g.id(r, c);
        if (g.cost[cell] == ROAD && cell != start && cell != goal) {
            g.cost[cell] = MUD;
            mudPlaced++;
        }
    };

    // The attempt cap guarantees the loop ends even if the target is unreachable.
    for (int attempt = 0; attempt < 20000 && mudPlaced < mudTarget; attempt++) {
        int cr = 1 + rng() % (rows - 2);   // random landing cell inside the border
        int cc = 1 + rng() % (cols - 2);
        makeMud(cr, cc);                   // the landing cell itself
        for (int d = 0; d < 4; d++) {      // its four neighbors
            makeMud(cr + DR[d], cc + DC[d]);
        }
    }
    return g;
}

// =============================================================================
//  SECTION 3: SOLVERS
// =============================================================================

// Solvers record two kinds of events so the renderer can replay them:
//   DISCOVERED - the cell was added to the frontier (we know it exists)
//   EXPANDED   - the cell was taken off the frontier and processed
enum EventType : uint8_t { DISCOVERED = 0, EXPANDED = 1 };

struct Event {
    int cell;
    EventType type;
};

// Everything a solver reports back. All three solvers return this same struct,
// so the reports and the renderer can treat them identically.
struct SearchResult {
    string name;
    bool found = false;
    vector<int> path;     // cell ids from start to goal (inclusive)
    int pathCost = 0;     // total cost of entering every cell after the start
    int explored = 0;     // "nodes explored" = number of cells EXPANDED
    int maxFrontier = 0;  // largest the frontier ever got (a memory proxy)
    double micros = 0;    // runtime in microseconds
    vector<Event> events; // only filled when record == true
};

// All three solvers share this signature, so they can be stored in one list:
//   g       the maze
//   start   id of the start cell
//   goal    id of the goal cell
//   record  when true, also store every step in SearchResult::events
using Solver = SearchResult (*)(const Grid&, int, int, bool);

// Stands for "no route found yet". Half of the maximum int, so adding a small
// cost to it can never overflow.
const int INF = numeric_limits<int>::max() / 2;

// Rebuilds the route by following the parent[] links backwards from the goal,
// then reverses it into start-to-goal order.
vector<int> buildPath(const vector<int>& parent, int start, int goal) {
    vector<int> path;
    for (int cur = goal; cur != -1; cur = parent[cur]) {
        path.push_back(cur);
        if (cur == start) break;
    }
    reverse(path.begin(), path.end());
    return path;
}

// Sum of the cost of entering each cell on the path (the start is free).
int pathCostOf(const Grid& g, const vector<int>& path) {
    int total = 0;
    for (size_t i = 1; i < path.size(); i++) total += g.cost[path[i]];
    return total;
}

// Final step of every solver: builds the path and its cost (if the goal was
// found) and stores the elapsed time since t0.
void finishResult(SearchResult& res, const Grid& g, const vector<int>& parent,
                  int start, int goal,
                  chrono::steady_clock::time_point t0) {
    if (res.found) {
        res.path = buildPath(parent, start, goal);
        res.pathCost = pathCostOf(g, res.path);
    }
    auto t1 = chrono::steady_clock::now();
    res.micros = chrono::duration<double, micro>(t1 - t0).count();
}

// Manhattan ("taxi cab") distance between two cells. Used by A*.
int manhattan(const Grid& g, int a, int b) {
    return abs(g.row(a) - g.row(b)) + abs(g.col(a) - g.col(b));
}

// ---------------------------------------------------------------------------
//  BFS: explores in expanding rings using a plain queue.
//  It counts steps, not cost, so it ignores mud. It finds the path with the
//  fewest cells, which is the cheapest path only when every cell costs the same.
// ---------------------------------------------------------------------------
SearchResult runBFS(const Grid& g, int start, int goal, bool record) {
    auto t0 = chrono::steady_clock::now();
    SearchResult res;
    res.name = "BFS";

    int n = g.rows * g.cols;
    vector<int> parent(n, -1);
    vector<char> seen(n, 0);
    queue<int> q;

    q.push(start);
    seen[start] = 1;
    int frontier = 1;  // how many cells are currently waiting in the queue
    res.maxFrontier = 1;
    if (record) res.events.push_back({start, DISCOVERED});

    while (!q.empty()) {
        int cur = q.front();
        q.pop();
        frontier--;
        res.explored++;
        if (record) res.events.push_back({cur, EXPANDED});

        if (cur == goal) { res.found = true; break; }

        int nb[4];
        int k = getNeighbors(g, cur, nb);
        for (int i = 0; i < k; i++) {
            int next = nb[i];
            if (seen[next]) continue;
            seen[next] = 1;       // mark when discovered so it is queued only once
            parent[next] = cur;
            q.push(next);
            frontier++;
            res.maxFrontier = max(res.maxFrontier, frontier);
            if (record) res.events.push_back({next, DISCOVERED});
        }
    }
    finishResult(res, g, parent, start, goal, t0);
    return res;
}

// ---------------------------------------------------------------------------
//  Dijkstra: always expands the frontier cell with the smallest total cost from
//  the start (kept in a min-heap). Correct for any non-negative costs.
//
//  C++'s priority_queue cannot update an entry in place. When a cheaper route to
//  a cell is found, a new entry is pushed and the outdated one is skipped when
//  it surfaces ("lazy deletion").
// ---------------------------------------------------------------------------
SearchResult runDijkstra(const Grid& g, int start, int goal, bool record) {
    auto t0 = chrono::steady_clock::now();
    SearchResult res;
    res.name = "Dijkstra";

    int n = g.rows * g.cols;
    vector<int> parent(n, -1);
    vector<int> dist(n, INF);   // cheapest known cost from start to each cell
    vector<char> done(n, 0);    // true once a cell's cost is final

    using Item = pair<int, int>;  // (cost so far, cell)
    priority_queue<Item, vector<Item>, greater<Item>> pq;  // smallest cost on top

    dist[start] = 0;
    pq.push({0, start});
    int frontier = 1;  // distinct cells discovered but not yet finalized
    res.maxFrontier = 1;
    if (record) res.events.push_back({start, DISCOVERED});

    while (!pq.empty()) {
        auto [d, cur] = pq.top();
        pq.pop();
        if (done[cur]) continue;  // outdated duplicate entry: ignore it

        done[cur] = 1;
        frontier--;
        res.explored++;
        if (record) res.events.push_back({cur, EXPANDED});

        if (cur == goal) { res.found = true; break; }

        int nb[4];
        int k = getNeighbors(g, cur, nb);
        for (int i = 0; i < k; i++) {
            int next = nb[i];
            if (done[next]) continue;
            int newDist = d + g.cost[next];  // cost of stepping onto `next`
            if (newDist < dist[next]) {
                if (dist[next] == INF) {  // first time we have seen this cell
                    frontier++;
                    res.maxFrontier = max(res.maxFrontier, frontier);
                    if (record) res.events.push_back({next, DISCOVERED});
                }
                dist[next] = newDist;
                parent[next] = cur;
                pq.push({newDist, next});
            }
        }
    }
    finishResult(res, g, parent, start, goal, t0);
    return res;
}

// ---------------------------------------------------------------------------
//  A*: like Dijkstra, but ranks frontier cells by  f = g + h  where
//     g = cost paid so far,   h = optimistic estimate of the cost still to go.
//  The estimate steers the search toward the goal, so fewer cells are explored.
//
//  The heuristic is Manhattan distance. Every step costs at least 1, so the real
//  remaining cost can never be less than the Manhattan distance. A heuristic
//  that never overestimates ("admissible") keeps A* optimal. It is also
//  "consistent", so a cell's cost is final the first time it is expanded.
// ---------------------------------------------------------------------------
SearchResult runAStar(const Grid& g, int start, int goal, bool record) {
    auto t0 = chrono::steady_clock::now();
    SearchResult res;
    res.name = "A*";

    int n = g.rows * g.cols;
    vector<int> parent(n, -1);
    vector<int> gScore(n, INF);  // cheapest known cost from start
    vector<char> closed(n, 0);

    // Heap items are (f, h, cell). Ties on f are broken by smaller h, which
    // prefers cells closer to the goal and noticeably trims the search.
    using Item = tuple<int, int, int>;
    priority_queue<Item, vector<Item>, greater<Item>> pq;

    gScore[start] = 0;
    int h0 = manhattan(g, start, goal) * ROAD;
    pq.push({h0, h0, start});
    int frontier = 1;
    res.maxFrontier = 1;
    if (record) res.events.push_back({start, DISCOVERED});

    while (!pq.empty()) {
        int cur = get<2>(pq.top());
        pq.pop();
        if (closed[cur]) continue;  // outdated duplicate entry: ignore it

        closed[cur] = 1;
        frontier--;
        res.explored++;
        if (record) res.events.push_back({cur, EXPANDED});

        if (cur == goal) { res.found = true; break; }

        int nb[4];
        int k = getNeighbors(g, cur, nb);
        for (int i = 0; i < k; i++) {
            int next = nb[i];
            if (closed[next]) continue;
            int newG = gScore[cur] + g.cost[next];
            if (newG < gScore[next]) {
                if (gScore[next] == INF) {  // first time we have seen this cell
                    frontier++;
                    res.maxFrontier = max(res.maxFrontier, frontier);
                    if (record) res.events.push_back({next, DISCOVERED});
                }
                gScore[next] = newG;
                parent[next] = cur;
                int h = manhattan(g, next, goal) * ROAD;
                pq.push({newG + h, h, next});
            }
        }
    }
    finishResult(res, g, parent, start, goal, t0);
    return res;
}

// Runs a solver `reps` times without recording and averages the runtime. One
// run on a small maze takes only microseconds, so a single measurement is too
// noisy to trust. Every other statistic is identical across repetitions.
SearchResult runTimed(Solver solver, const Grid& g, int start, int goal, int reps) {
    SearchResult res = solver(g, start, goal, false);
    double total = res.micros;
    for (int i = 1; i < reps; i++) total += solver(g, start, goal, false).micros;
    res.micros = total / reps;
    return res;
}

// =============================================================================
//  SECTION 4: RENDERER (ANSI terminal animation)
// =============================================================================
//
//  Each maze cell is drawn as two spaces with a colored background, which makes
//  it roughly square. To animate, the cursor is moved to a cell with the escape
//  code ESC[row;colH and only that one cell is repainted.

// What a cell is currently showing (on top of its terrain).
enum CellState : uint8_t { NONE = 0, FRONTIER, VISITED, ON_PATH };

class Renderer {
public:
    Renderer(const Grid& grid, int s, int t)
        : g(grid), start(s), goal(t), state(grid.rows * grid.cols, NONE) {}

    // Plays back one solver's events as an animation lasting about `seconds`.
    void animate(const SearchResult& res, double seconds) {
        fill(state.begin(), state.end(), NONE);

        cout << "\033[?25l";        // hide the cursor while drawing
        cout << "\033[2J\033[H";    // clear the screen, move to top-left
        cout << "\033[1m" << res.name << "\033[0m\n";  // bold title on row 1
        for (int id = 0; id < g.rows * g.cols; id++) paint(id);
        cout << "\033[" << (TOP + g.rows + 1) << ";1H"
             << "legend:  ";
        legendSwatch(234, "road");  legendSwatch(94, "mud(x5)");
        legendSwatch(226, "frontier"); legendSwatch(33, "explored");
        legendSwatch(46, "path");
        cout << flush;

        // Pick how many events to draw per frame so the whole animation takes
        // about `seconds`, no matter how large the maze is.
        const int fps = 60;
        const int frameMs = 1000 / fps;
        int totalFrames = max(1, static_cast<int>(seconds * fps));
        int perFrame = max<int>(1, (res.events.size() + totalFrames - 1) / totalFrames);

        for (size_t i = 0; i < res.events.size(); i++) {
            const Event& e = res.events[i];
            state[e.cell] = (e.type == DISCOVERED) ? FRONTIER : VISITED;
            paint(e.cell);
            if ((i + 1) % perFrame == 0) {
                cout << flush;
                this_thread::sleep_for(chrono::milliseconds(frameMs));
            }
        }
        cout << flush;

        // Draw the final path cell by cell, so the route appears gradually.
        for (int cell : res.path) {
            state[cell] = ON_PATH;
            paint(cell);
            cout << flush;
            this_thread::sleep_for(chrono::milliseconds(12));
        }

        cout << "\033[" << (TOP + g.rows + 2) << ";1H"
             << "explored: " << res.explored
             << "   path cost: " << res.pathCost
             << "   path steps: " << (res.path.size() - 1) << "\n";
        cout << "\033[?25h" << flush;  // show the cursor again
        this_thread::sleep_for(chrono::milliseconds(1200));
    }

    // Row just below the drawing, where it is safe to print more text.
    int rowBelow() const { return TOP + g.rows + 4; }

private:
    const Grid& g;
    int start, goal;
    vector<CellState> state;
    static constexpr int TOP = 3;  // the maze's first row is on screen row 3

    // Picks a 256-color palette index for a cell. Mud gets a duller variant of
    // each color so the terrain stays visible underneath the search overlay.
    int colorFor(int id) const {
        if (id == start) return 196;  // red
        if (id == goal) return 201;   // magenta
        if (g.cost[id] == WALL) return 244;  // grey
        bool mud = (g.cost[id] == MUD);
        switch (state[id]) {
            case ON_PATH:  return mud ? 34 : 46;    // green
            case VISITED:  return mud ? 61 : 33;    // blue
            case FRONTIER: return mud ? 178 : 226;  // yellow
            default:       return mud ? 94 : 234;   // brown / near-black
        }
    }

    // Repaints a single cell at its screen position.
    void paint(int id) const {
        int screenRow = TOP + g.row(id);
        int screenCol = g.col(id) * 2 + 1;  // 2 characters per cell, 1-indexed
        cout << "\033[" << screenRow << ";" << screenCol << "H"
             << "\033[48;5;" << colorFor(id) << "m  \033[0m";
    }

    // Prints a colored box and its label at the current cursor position.
    void legendSwatch(int color, const string& label) const {
        cout << "\033[48;5;" << color << "m  \033[0m " << label << "  ";
    }
};

// =============================================================================
//  SECTION 5: REPORTS
// =============================================================================

// Comparison table for a single maze.
void printTable(const vector<SearchResult>& results) {
    cout << "\n";
    cout << left << setw(10) << "Algorithm"
         << right << setw(10) << "Explored"
         << setw(11) << "Path cost"
         << setw(8) << "Steps"
         << setw(14) << "Max frontier"
         << setw(12) << "Time (us)" << "\n";
    cout << string(65, '-') << "\n";
    for (const auto& r : results) {
        cout << left << setw(10) << r.name
             << right << setw(10) << r.explored
             << setw(11) << r.pathCost
             << setw(8) << (r.path.empty() ? 0 : r.path.size() - 1)
             << setw(14) << r.maxFrontier
             << setw(12) << fixed << setprecision(1) << r.micros << "\n";
    }
}

// Generates `count` mazes (seeds seed, seed+1, ...), solves each one with all
// three algorithms and prints the AVERAGE of every metric. Averages over many
// mazes are far more trustworthy than a single lucky or unlucky run.
void runBenchmark(int count, int rows, int cols, double loops, double mud,
                  unsigned seed) {
    vector<Solver> solvers = {runBFS, runDijkstra, runAStar};
    vector<string> names = {"BFS", "Dijkstra", "A*"};
    vector<double> sumExplored(3, 0), sumCost(3, 0), sumFrontier(3, 0), sumTime(3, 0);
    int bfsSuboptimal = 0;  // mazes where BFS found a more expensive path
    int mismatches = 0;     // sanity check: Dijkstra and A* must always agree

    for (int i = 0; i < count; i++) {
        mt19937 rng(seed + i);
        Grid g = generateMaze(rows, cols, loops, mud, rng);
        int start = g.id(1, 1), goal = g.id(rows - 2, cols - 2);

        vector<SearchResult> r;
        for (int s = 0; s < 3; s++) r.push_back(solvers[s](g, start, goal, false));

        for (int s = 0; s < 3; s++) {
            sumExplored[s] += r[s].explored;
            sumCost[s] += r[s].pathCost;
            sumFrontier[s] += r[s].maxFrontier;
            sumTime[s] += r[s].micros;
        }
        if (r[0].pathCost > r[1].pathCost) bfsSuboptimal++;
        if (r[1].pathCost != r[2].pathCost) mismatches++;
    }

    cout << "\nBenchmark: " << count << " mazes of " << rows << "x" << cols
         << "  (loops " << loops << ", mud " << mud << ", first seed " << seed << ")\n\n";
    cout << left << setw(10) << "Algorithm"
         << right << setw(12) << "Avg explored"
         << setw(11) << "Avg cost"
         << setw(14) << "Avg frontier"
         << setw(13) << "Avg time(us)" << "\n";
    cout << string(60, '-') << "\n";
    for (int s = 0; s < 3; s++) {
        cout << left << setw(10) << names[s] << right << fixed << setprecision(1)
             << setw(12) << sumExplored[s] / count
             << setw(11) << sumCost[s] / count
             << setw(14) << sumFrontier[s] / count
             << setw(13) << sumTime[s] / count << "\n";
    }
    cout << "\nBFS returned a costlier-than-optimal path in " << bfsSuboptimal
         << " of " << count << " mazes.\n";
    if (mismatches > 0) {
        cout << "WARNING: Dijkstra and A* disagreed on " << mismatches
             << " mazes (this would indicate a bug).\n";
    }
}

// =============================================================================
//  SECTION 6: MAIN
// =============================================================================

void printHelp() {
    cout << "Usage: pathviz [options]\n"
            "  --rows N     maze height in cells (default 21, forced odd, min 5)\n"
            "  --cols N     maze width in cells  (default 35, forced odd, min 5)\n"
            "  --seed N     random seed, for reproducible mazes (default: random)\n"
            "  --loops F    chance 0..1 to remove each wall between corridors\n"
            "               (default 0.10; 0 gives a perfect maze with one route)\n"
            "  --mud F      fraction 0..1 of open cells turned into mud (default 0.15;\n"
            "               0 makes every step cost the same)\n"
            "  --time S     seconds per animation (default 4)\n"
            "  --no-anim    skip the animation, only print the comparison table\n"
            "  --bench N    solve N random mazes and print average statistics\n"
            "  --help       show this message\n";
}

// Makes a size valid for the generator: at least 5 and odd.
int normalizeSize(int v) {
    v = max(v, 5);
    return (v % 2 == 0) ? v + 1 : v;
}

int main(int argc, char** argv) {
    int rows = 21, cols = 35;
    double loops = 0.10, mud = 0.15, seconds = 4.0;
    unsigned seed = random_device{}();
    bool animate = true;
    int benchCount = 0;

    // ---- Parse command-line options -----------------------------------------
    for (int i = 1; i < argc; i++) {
        string a = argv[i];
        // Returns the word that follows an option, or exits with an error.
        auto value = [&]() -> string {
            if (i + 1 >= argc) {
                cerr << "Missing value after " << a << "\n";
                exit(1);
            }
            return argv[++i];
        };
        if (a == "--rows") rows = stoi(value());
        else if (a == "--cols") cols = stoi(value());
        else if (a == "--seed") seed = static_cast<unsigned>(stoul(value()));
        else if (a == "--loops") loops = stod(value());
        else if (a == "--mud") mud = stod(value());
        else if (a == "--time") seconds = stod(value());
        else if (a == "--bench") benchCount = stoi(value());
        else if (a == "--no-anim") animate = false;
        else if (a == "--help") { printHelp(); return 0; }
        else { cerr << "Unknown option: " << a << "\n"; printHelp(); return 1; }
    }
    rows = normalizeSize(rows);
    cols = normalizeSize(cols);

    // ---- Benchmark mode: no animation, just averages ------------------------
    if (benchCount > 0) {
        runBenchmark(benchCount, rows, cols, loops, mud, seed);
        return 0;
    }

    // ---- Single-maze mode ---------------------------------------------------
    mt19937 rng(seed);
    Grid g = generateMaze(rows, cols, loops, mud, rng);
    int start = g.id(1, 1);
    int goal = g.id(rows - 2, cols - 2);

    vector<Solver> solvers = {runBFS, runDijkstra, runAStar};
    vector<SearchResult> stats;  // timing and statistics (no recording overhead)
    for (Solver s : solvers) stats.push_back(runTimed(s, g, start, goal, 200));

    Renderer renderer(g, start, goal);
    if (animate) {
        for (Solver s : solvers) {
            // Re-run with recording on to capture the events to replay.
            SearchResult recorded = s(g, start, goal, true);
            renderer.animate(recorded, seconds);
        }
        cout << "\033[" << renderer.rowBelow() << ";1H";  // move below the maze
    }

    cout << "Maze " << rows << "x" << cols << ", seed " << seed
         << " (rerun with --seed " << seed << " to see this maze again)";
    printTable(stats);
    return 0;
}
