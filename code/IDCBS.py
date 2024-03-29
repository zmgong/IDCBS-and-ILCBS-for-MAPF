import copy
import operator
import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    maxLength = max(len(path1), len(path2))
    for t in range(maxLength):
        if get_location(path1, t) == get_location(path2, t):
            return [get_location(path1, t), t]
        else:
            path1Edge = [get_location(path1, t - 1), get_location(path1, t)]
            reversePath2Edge = [get_location(path2, t), get_location(path2, t - 1)]
            if path1Edge[0] == reversePath2Edge[0] and path1Edge[1] == reversePath2Edge[1]:
                return [path1Edge[0], path1Edge[1], t]
    return None


def detect_collisions(paths):
    result = []
    numberOfPaths = len(paths)
    for i in range(numberOfPaths - 1):
        for j in range(i + 1, numberOfPaths):
            collision = detect_collision(paths[i], paths[j])
            if collision is not None:
                if len(collision) == 2:
                    result.append({'a1': i, 'a2': j, 'loc': [collision[0]], 'timestep': collision[1]})
                else:
                    result.append({'a1': i, 'a2': j, 'loc': [collision[0], collision[1]], 'timestep': collision[2]})

    return result


def standard_splitting(collision):
    if len(collision['loc']) == 1:
        result = [{'agent': collision['a1'],
                   'loc': collision['loc'],
                   'timestep': collision['timestep'],
                   'positive': False},
                  {'agent': collision['a2'],
                   'loc': collision['loc'],
                   'timestep': collision['timestep'],
                   'positive': False}]
        return result
    else:
        result = [{'agent': collision['a1'],
                   'loc': [collision['loc'][0], collision['loc'][1]],
                   'timestep': collision['timestep'],
                   'positive': False},
                  {'agent': collision['a2'],
                   'loc': [collision['loc'][1], collision['loc'][0]],
                   'timestep': collision['timestep'],
                   'positive': False}]
        return result


def disjoint_splitting(collision):
    randomNum = random.randint(0, 1)
    chosenAgent = collision['a1']
    if randomNum == 1:
        chosenAgent = collision['a2']
    if len(collision['loc']) == 1:
        return [{'agent': chosenAgent,
                 'loc': collision['loc'],
                 'timestep': collision['timestep'],
                 'positive': True},
                {'agent': chosenAgent,
                 'loc': collision['loc'],
                 'timestep': collision['timestep'],
                 'positive': False}
                ]
    else:
        if chosenAgent == collision['a1']:
            return [{'agent': chosenAgent,
                     'loc': [collision['loc'][1], collision['loc'][0]],
                     'timestep': collision['timestep'],
                     'positive': True},
                    {'agent': chosenAgent,
                     'loc': [collision['loc'][0], collision['loc'][1]],
                     'timestep': collision['timestep'],
                     'positive': False}
                    ]
        else:
            return [{'agent': chosenAgent,
                     'loc': [collision['loc'][1], collision['loc'][0]],
                     'timestep': collision['timestep'],
                     'positive': False},
                    {'agent': chosenAgent,
                     'loc': [collision['loc'][0], collision['loc'][1]],
                     'timestep': collision['timestep'],
                     'positive': True}]


def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst


def conflict_graph_heuristic(node):
    V = []
    E = []
    for i in node['collisions']:
        if i['a1'] not in V:
            V.append(i['a1'])
        if i['a2'] not in V:
            V.append(i['a2'])
        # always add edge, since we just detect first collision between two agents, that means there will not be more
        # than one collision between two agents

        E.append((i['a1'], i['a2']))
    agentThatWhileIncreaseCost = []
    while len(E) != 0:
        dicForCounting = {}
        for v in V:
            dicForCounting[str(v)] = 0
        for e in E:
            if e[0] not in agentThatWhileIncreaseCost and e[1] not in agentThatWhileIncreaseCost:
                dicForCounting[str(e[0])] = dicForCounting[str(e[0])] + 1
                dicForCounting[str(e[1])] = dicForCounting[str(e[1])] + 1
        vertexToIncrease = max(dicForCounting, key=dicForCounting.get)

        for e in E:
            if e[0] == int(vertexToIncrease) or e[1] == int(vertexToIncrease):
                E.remove(e)
        agentThatWhileIncreaseCost.append(vertexToIncrease)
    return len(agentThatWhileIncreaseCost)


class IDCBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list,
                       (self.num_of_generated, len(node['collisions']), self.num_of_generated, node))
        # print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)

        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': [],
                'h_value': 0
                }
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        print()
        self.push_node(root)

        print(root['collisions'])

        for collision in root['collisions']:
            print(disjoint_splitting(collision))

        threshold = 0
        f_valueOfPrunedNodes = []
        while len(self.open_list) != 0:
            curr = self.pop_node()

            f_value = curr['h_value'] + curr['cost']
            # f_value = curr['cost']
            if f_value > threshold:
                f_valueOfPrunedNodes.append(f_value)
                if len(self.open_list) == 0:
                    self.push_node(root)
                    threshold = min(f_valueOfPrunedNodes)
                    f_valueOfPrunedNodes = []
                continue

            if len(curr['collisions']) == 0:
                self.print_results(curr)
                return curr['paths']

            self.num_of_expanded += 1
            newConstraints = disjoint_splitting(curr['collisions'][0])
            for constraint in newConstraints:
                if constraint['positive'] is False:
                    childConstraints1 = copy.deepcopy(curr['constraints'])
                    childConstraints1.append(constraint)
                    child = {'cost': 0,
                             'constraints': childConstraints1,
                             'paths': copy.deepcopy(curr['paths']),
                             'collisions': []}
                    agent = constraint['agent']
                    newPath = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent],
                                     agent, child['constraints'])
                    if newPath is not None:
                        child['paths'][agent] = copy.deepcopy(newPath)
                        child['cost'] = get_sum_of_cost(child['paths'])
                        child['collisions'] = detect_collisions(child['paths'])
                        child['h_value'] = conflict_graph_heuristic(child)
                        self.push_node(child)
                else:
                    pathIsNone = False
                    childConstraints2 = copy.deepcopy(curr['constraints'])
                    childConstraints2.append(constraint)
                    child = {'cost': 0,
                             'constraints': childConstraints2,
                             'paths': copy.deepcopy(curr['paths']),
                             'collisions': []}
                    agentList = paths_violate_constraint(constraint, child['paths'])
                    for i in agentList:
                        if i == constraint['agent']:
                            continue
                        newPath = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                                         i, child['constraints'])
                        if newPath is not None:
                            child['paths'][i] = copy.deepcopy(newPath)
                            child['cost'] = get_sum_of_cost(child['paths'])
                            child['collisions'] = detect_collisions(child['paths'])
                            child['h_value'] = conflict_graph_heuristic(child)
                        else:
                            pathIsNone = True
                            break
                    if pathIsNone is False:
                        self.push_node(child)
        return None

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
        print("SolutionL: " + str(node['paths']))
