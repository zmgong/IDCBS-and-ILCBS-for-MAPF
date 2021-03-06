import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []
        mapSize = 0
        for i in self.my_map:
            for j in i:
                if j is False:
                    mapSize = mapSize + 1

        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)
            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches

            ##############################
        while True:
            trigger = False
            maxLengthOfPath = 0
            sumOfPreviousPath = 0

            for i in range(self.num_of_agents):
                if len(result[i]) > sumOfPreviousPath + mapSize:
                    raise BaseException('No solutions')
                if len(result[i]) > maxLengthOfPath:
                    maxLengthOfPath = len(result[i])
            for j in range(maxLengthOfPath):
                currentPositions = []
                currentEdges = []
                for i in range(self.num_of_agents):
                    currPosOfThisAg = None
                    currentEdge = None
                    if j >= len(result[i]):
                        currPosOfThisAg = result[i][len(result[i]) - 1]
                        if j > 0:
                            currentEdge = [currPosOfThisAg, currPosOfThisAg]
                    else:
                        currPosOfThisAg = result[i][j]
                        if j > 0:
                            currentEdge = [result[i][j-1], result[i][j]]
                    if currPosOfThisAg in currentPositions:
                        trigger = True
                        constraints.append({'agent': i, 'loc': [currPosOfThisAg], 'timestep': j, 'positive': False})
                    else:
                        currentPositions.append(currPosOfThisAg)
                    if currentEdge is not None:
                        reverseEdge = [currentEdge[1], currentEdge[0]]
                        if reverseEdge in currentEdges:
                            trigger = True
                            newEdgeConstraint = {'agent': i, 'loc': currentEdge, 'timestep': j, 'positive': False}
                            constraints.append(newEdgeConstraint)
                        elif currentEdge is not None:
                            currentEdges.append(currentEdge)

            if trigger is False:
                break
            else:
                result = []
                for i in range(self.num_of_agents):  # Find path for each agent
                    path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                                  i, constraints)
                    if path is None:
                        raise BaseException('No solutions')
                    result.append(path)


        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
