# A* Algorithm Analysis

## Part 0: Generating Grid

Using a two-dimensional array of type Cell Class to represent the grid. The Cell
class holds a x and y value to identify its location on the grid.

Blocks are placed on the grid randomly making approximately 30 percent of
the grid NULL

## Part 1: Understanding the methods

a) According to the algorithm, the agent world is fully unblocked as it does not
know the blocks in its way. So when A* is run, it returns the shortest presumed
unblocked path, which is straight to the right. A* returns this path by using
the Manhattan distance as its heuristic.

b) The number of moves of the agent until it reaches the target or discovers
that this is impossible is bounded from above by the number of unblocked cells
squared as unblocked cells squared gives you the total number of possible moves
the the agent can make. This can be proven because the number of moves the
agent can make after each A* execution is at most the total number of cells,
therefore moves (m) must be less than or equal to total number of cells (n).
Since we are calling A* repeatedly, which can be equal to at most the total
number of cells, we get iterations (i) must be less than or equal to total number
of cells (n). Combining both inequalities, we get that Repeated A* is bounded
from above by the total number of moves an agent makes squared (im less than
or equal to n squared).

## Part 2: Effect of breaking ties

Time Analysis for Breaking Ties by Favoring Lower g-values
Total Execution Time: 6.92 seconds


On Average: 0.13 seconds
Average path length: 158

Time Analysis for Breaking Ties by Favoring Higher g-values
Total Execution Time: 4.44 seconds
On Average: 0.08 seconds
Average path length: 158

By breaking ties in favor of higher g-values instead of lower g-values, we end up
having a better run-time on average. This is the case because when exploring
cells by favoring higher g-values, the agent is able to move closer to the end
state, that is, the agent is able to explore more cells outward. When exploring
cells by favoring lower g-values, the agent explores cells that are closer to the
start state. Favoring smaller g-values, we expand a lot of unnecessary cells near
the start state versus expanding cells towards the end state.

## Part 3: Forward vs Backward

Time Analysis for Repeated Forward A*
Total Execution Time: 4.44 seconds
On Average: 0.0888 seconds
Average path length: 158

Time Analysis for Repeated Backward A*
Total Execution Time: 37.14 seconds
On Average: 0.74 seconds
Average path length: 165

In our tests, we came to the conclusion that Repeated Forward A* is consider-
ably faster than Repeated Backward A*. This is because of the way Repeated
Backward A* is implemented. Backward A* finds a path from the end to the
start, but we traverse that path from start to end. This causes backwards A*
to traverse and expand more cells closer to the start which is counterproductive
than necessary compared to Forward A*.

## Part 4: Heuristics in the Adaptive A*

This is the case because the only way the Manhattan distance is not consis-
tent in these types of gridworlds is if the agent is allowed to move diagonally.
Manhattan distance is given by the absolute value of the difference of start and
end coordinates. This is like moving the agent up or down, and left or right, to
calculate the distance from Start to End. The only way for this distance to not
be consistent is if there is a way for the agent to move in between two squares,
that is, move diagonally, which is not allowed.


The Manhattan distances are consistent in gridworlds in which the agent can
move only in the four main compass directions.

Let us assume that Manhattan distance to be inconsistent.

```
h(n)≥cost(n, a, n′) +n(m)
```
where cost(n, a, n’) is the cost of moving in 1 of 4 directions and h(n’) is the
cost from n’ to goal, and h(n) is the cost from n to goal.
Next if we move h(n’) to the left side, we get

```
h(n)−h(n′) ≥ cost(n, a, n′)
```
and since we are using Manhattan distance, h(n) - h(n’) is simply

```
|x(n)−x(n′)|+|y(n)−y(n′)|
```
and the only way that

```
|x(n)−x(n′)|+|y(n)−y(n′)| ≥ cost(n, a, n′)
```
is if we move diagonally and this is not possible by Manhattan distance. There-
fore Manhattan distance is consistent.

Adaptive A* leaves initially consistent h-values consistent even if action costs
can increase is because of how we are calculating the h-values on consecutive
iterations of A*.
Adaptive A* uses the following heuristic function

```
h(s) = g(target)−g(s)
```
for this to be consistent

```
h(n) ≤ cost(n, a, n′) +h(n′)
```
substituting the heuristic function, we get

```
g(target)−g(n) ≤ cost(n, a, n′) +g(target)−g(n′)
```
simplifying this, we get

```
g(n′)−g(n) ≤ cost(n, a, n′)
```
If g(n’) - g(n) is computed using Manhattan distance, we get the distance be-
tween n’ and n. This is the same as cost(n, a, n’). Therefore the new heuristic,
h(s) = g(target) - g(s), is consistent as well.


## Part 5: Repeated Forward A* vs Adaptive

Time Analysis for Repeated Forward A*
Total Execution Time: 4.44 seconds
On Average: 0.0888 seconds
Average path length: 158

Time Analysis for Adaptive A*
Total Execution Time: 4.38 seconds
On Average: 0.0876 seconds
Average path length: 163

Adaptive A* is considerably faster than Repeated Forward A*. This is the case
because Adaptive A* uses an improved heuristic calculation. Because Adap-
tive A* takes into account cells that have already been visited, it calculates an
updated h-cost that finds the End state faster. This is also why Adaptive A*
expands less cells than Repeated Forward A*.

## Part 6: Memory Issues

Some ways to improve memory usage would involve reducing the amount of
memory each of the cells takes up. With our implementation, a single cell takes
up 28 bytes of space which could be reduced if we use bits to represent the 5
integers. We can also reduce memory usage if we do not keep track of the parent
of the current cell and just calculate as we traverse the grid.
Our implementation of this algorithm uses a Cell class which has 5 integers to
keep track of coordinates and f, g, and h costs, and a pointer to keep track of
the parent of the cell. This totals 5*4+8 bytes which is 28 bytes per cell. For
the 1001x1001 grid, the total is 28*1001*1001, which is 28056028 bytes.

Therefore, the largest grid that can operate within 4 Megabytes is (4*1024)/
= 147. So the grid can be at most, 12x12.
