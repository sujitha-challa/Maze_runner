# Maze_runner
An Intelligent System to solve randomly generated maze using A* algorithm.

A gridworld is a discretization of terrain into square cells that are either unblocked (traversable) or blocked. Consider
the following problem: an agent in a gridworld has to move from its current cell (S) to a goal cell, the location of
a stationary target. The layout of the gridworld (what cells are blocked or unblocked) is unknown. These kinds of
challenges arise frequently in robotics, where a mobile platform equipped with sensors builds a map of the world as
it traverses an unknown environment.

Assume that the initial cell of the agent is unblocked. The agent can move from its current cell in the four main
compass directions (east, south, west, north) to any adjacent cell, as long as that cell is unblocked and still part of
the gridworld. All moves take one timestep for the agent, and thus have cost 1. (We can consider this in terms of
energy consumption for the agent, for instance.) The agent always knows which (unblocked) cell it is currently in,
and which (unblocked) cell the target is in. The agent knows that blocked cells remain blocked and unblocked cells
remain unblocked but does not know initially which cells are blocked. However, it can observe or sense the
status of some of its surrounding cells (corresponding to its field of view) and remember this information for future
use. By exploring the maze, collecting information on blockages, and integrating that into the whole, the agent must
reach the target as efficiently as possible.

We may structure a basic agent in the following way: the agent assumes that all cells are unblocked, until it observes
them to be blocked, and all path planning is done with the current state of knowledge of the gridworld under this
freespace assumption. In other words, it moves according to the following strategy:
  • Based on its current knowledge of the environment, it plans a path from its current position to the goal.
  • This path should be the shortest (presumed) unblocked path available.
  • The agent attempts to follow this path plan, observing cells in its field of view as it moves.
  • Observed blocked cells or unblocked cells are remembered, updating the agent’s knowledge of the environment.
  • If the agent discovers a block in its planned path, it re-plans, based on its current knowledge of the environment
  • The cycle repeats until the agent either a) reaches the target or b) determines that there is no unblocked path
  to the target.

Because this kind of strategy depends on being able to do a lot of searches, it is important these searches be as fast
as possible. To that end, we are going to implement Repeated Forward A*, where each forward path planning
step is done using the A* search algorithm. Recall from class that as A* runs, it maintains four values for each cell
n that it encounters.

A* maintains its fringe as a priority queue (initially containing the initial search point). The priority of the fringe
is given by the value of f for every node in the fringe. Under some assumptions on h, the first time the goal node
comes off the fringe represents the discovery of a shortest path from start to goal node. Until then, A* removes the
cell n with the smallest value of f and expands it.

Repeated Forward A* moves the agent along the identified path, until it either successfully reaches the goal or it
encounters an obstacle along its planned path. In the first case, the agent has reached the target; in the second case,
A* is re-called using the agents current position and any updated information about the location of obstacles. This
is repeated until one of the two termination conditions for this strategy is met.



