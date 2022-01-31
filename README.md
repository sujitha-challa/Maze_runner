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

Agent 1: Repeated Forward A* moves the agent along the identified path, until it either successfully reaches the goal or it
encounters an obstacle along its planned path. In the first case, the agent has reached the target; in the second case,
A* is re-called using the agents current position and any updated information about the location of obstacles. This
is repeated until one of the two termination conditions for this strategy is met.

Agent 2: The Blindfolded Agent - field of view is only in the direction of movement

Agent 3: The agent can detect obstacles by running into them, but at every location
can also sense how many of the (at most 8) immediate neighbors are blocked. However, it cannot sense which of
them are blocked without attempting to move in those directions. For each cell x, the agent should track
the following information:
• Nx: the number of neighbors cell x has.
• Whether or not cell x has been visited.
• Whether or not cell x has been confirmed as empty or blocked, or is currently unconfirmed.
• Cx: the number of neighbors of x that are sensed to be blocked.
• Bx: the number of neighbors of x that have been confirmed to be blocked.
• Ex: the number of neighbors of x that have been confirmed to be empty.
• Hx: the number of neighbors of x that are still hidden or unconfirmed either way.

Given this information, we can infer things about cells in the following way: for any cell x, let Nx be the number of
neighbors of x, Cx be the number of blocks known to neighbor x (from the partial sensing), Bx the number of blocks
confirmed to neighbor x, Ex the number of empty cells confirmed to neighbor x, and Hx the number of neighbors of
x that are still ’hidden’.
• If Cx = Bx: all remaining hidden neighbors of x are empty.
• If Nx − Cx = Ex: all remaining hidden neighbors of x are blocked.
• If Hx = 0: nothing remains to be inferred about cell x.

Agent 4: Agent 4 has same set of information as agent 3. To make it better we added additional 2 inference rules, and also
used rules from agent 3 on previous nodes visited.
• For all nodes visited, we stored all neighbors that need to be inferred in a dictionary. And used this in inference
rules.
• Inference rule 1
 This infers based on information of 2 adjacent nodes(say node 1 and node 2). When node 1 have only empty
cells other than in common region for node1 and node2, this rule works (if other conditions satisfy).
 Below is the explanation for inference rule 1.
 If 2 nodes have common nodes(say n)
number of common cells of node1 and node 2 = n
and in node 2, below are parameters
number of blocked cells to be inferred = Cnode 2 – Bnode2
Number of undetermined cells for node 2 that are not common = Hnode2 – n
 If (Cnode 2 – Bnode2 >Hnode2 – n ), then Common cells have ((Cnode 2 – Bnode2) – (Hnode2 – n)) number of blocks for sure
 So, number of blocks in common cells for sure = ((Cnode 2 – Bnode2) – (Hnode2 – n))
 In node 1, parameters are, number of blocked cells to be inferred = Cnode 1 – Bnode1
 If (Cnode 1 – Bnode1) <= ((Cnode 2 – Bnode2) – (Hnode2 – n)), then for node1, rest of undetermined nodes not in common
are empty cells.
• Inference rule 2
 This rule also infers based on information of 2 adjacent nodes(say node 1 and node 2). When node 1 have only 
blocked cells other than in common region for node1 and node2, this rule works (if other conditions satisfy)
 Below is the explanation for inference rule 2.
 If 2 nodes have common nodes(say n)
number of common cells of node1 and node 2 = n
and in node 2, below are parameters
number of empty cells to be inferred = Nnode2 - Cnode 2 – Enode2
Number of undetermined cells for node 2 that are not common = Hnode2 - n
 If (Nnode2 - Cnode 2 - Enode2 >Hnode2 - n ), then Common nodes have ((Nnode2 - Cnode 2 - Enode2) - (Hnode2 – n)) number of
empty cells for sure
 So, number of empty cells in common for sure = ((Nnode2 - Cnode 2 – Enode2) – (Hnode2 – n))
 In node 1, parameters are, number of empty cells to be inferred = Nnode2 - Cnode 2 - Enode2
 If (Nnode2 - Cnode 2 - Enode2) <= ((Nnode2 - Cnode 2 - Enode2) - (Hnode2 – n)), then for node1, rest of undetermined nodes
not in common are blocked cells.
How does Agent 4 differ from Agent 3?
• Agent-3 runs inference only on the current node and previous visited node, whereas agent-4 runs inference on of
previously visited nodes(maximum 10), so that more information can be discovered.
• Running inference again on nodes previously visited nodes, we have better discovered grid, and helps in getting
shorter path in final discovered grid as well
• Agent 3 can infer only when all of the remaining undetermined neighbor nodes are either blocked or unblocked.
• Agent 4 can infer in the above scenario and as well the other scenarios, by comparing with adjacent nodes. (These
scenarios are described above)
o When node 1 have only empty cells other than in common region for node1 and node2, this rule works (if
other conditions satisfy).
o When node 1 have only blocked cells other than in common region for node1 and node2, this rule works
(if other conditions satisfy)






