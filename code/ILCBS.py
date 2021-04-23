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
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.


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

    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.


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

    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly
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
    a = 0
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
    # print("h: " + str(len(agentThatWhileIncreaseCost)))
    return len(agentThatWhileIncreaseCost)


def findNodeNeedToBeCollapseTo(curr, prev_curr):
    p = prev_curr
    p_list = []
    print("c: " + str(curr['constraints']))
    print("p: " + str(p['constraints']))
    while p is not None and curr['parent'] != p['parent']:
        p = p['parent']
    return p


def parentCheck(B, node):
    p = node['parent']
    while p is not None:
        if p is B:
            return True
        p = p['parent']
    return False


class ILCBSSolver(object):
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

        self.prev_curr = None

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        if node['C']:
            heapq.heappush(self.open_list, (node['F_value'], len(node['collisions']), self.num_of_generated, node))
        else:
            heapq.heappush(self.open_list, (node['f_value'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def collapse(self, B):
        print("Collapse triggered!!!!!!!!!!!")
        result = None
        ifAnyNodeIsRemoved = False
        for i in self.open_list:
            nodeInTuple = i[3]
            if parentCheck(B, nodeInTuple):
                ifAnyNodeIsRemoved = True
                if nodeInTuple['C'] is False:
                    if result is None or nodeInTuple['f_value'] < result:
                        result = nodeInTuple['f_value']
                elif nodeInTuple['C'] is True:
                    if result is None or nodeInTuple['F_value'] < result:
                        result = nodeInTuple['F_value']
                self.open_list.remove(i)
        if ifAnyNodeIsRemoved is True:
            B['C'] = True
            B['F_value'] = result
        else:
            B['C'] = False
        if B not in self.open_list:
            self.push_node(B)
        print(B['F_value'])

    def restore(self, root):
        open_restore = []
        open_restore.insert(0, root)
        threshold = root['F_value']
        target = None
        close = []
        while len(open_restore) != 0:
            curr = open_restore.pop(0)
            if curr['f_value'] == threshold:
                target = curr
            if curr['f_value'] >= threshold:
                close.append(curr)
            else:
                self.num_of_expanded += 1
                newConstraints = disjoint_splitting(curr['collisions'][0])
                close.append(curr)
                childrenList = self.generateChildren(curr, newConstraints)
                for child in childrenList:
                    open_restore.insert(0, child)

        singleParent = target['parent']
        p_list = []
        while singleParent is not None and singleParent is not root:
            p_list.append(singleParent)
            singleParent = singleParent['parent']
        for p in p_list:
            min_f = None
            for c in close:
                ifCollapse = False
                if c['parent'] is p and c is not target and c not in p_list:
                    for gc in close:
                        if gc['parent'] is c:
                            ifCollapse = True
                            if min_f is None or min_f > gc['f_value']:
                                min_f = gc['f_value']
                    if ifCollapse:
                        c['C'] = True
                        c['F_value'] = min_f
                    else:
                        c['C'] = False
                    self.push_node(c)

    def generateChildren(self, curr, newConstraints):
        childrenList = []
        for constraint in newConstraints:
            if constraint['positive'] is False:
                childConstraints1 = copy.deepcopy(curr['constraints'])
                childConstraints1.append(constraint)
                child = {'cost': 0,
                         'constraints': childConstraints1,
                         'paths': copy.deepcopy(curr['paths']),
                         'collisions': [],
                         'parent': curr,
                         'C': False}
                agent = constraint['agent']
                newPath = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent],
                                 agent, child['constraints'])
                if newPath is not None:
                    child['paths'][agent] = copy.deepcopy(newPath)
                    child['cost'] = get_sum_of_cost(child['paths'])
                    child['collisions'] = detect_collisions(child['paths'])
                    child['h_value'] = conflict_graph_heuristic(child)
                    child['f_value'] = child['cost']
                    childrenList.append(child)
            else:
                pathIsNone = False
                childConstraints2 = copy.deepcopy(curr['constraints'])
                childConstraints2.append(constraint)
                child = {'cost': 0,
                         'constraints': childConstraints2,
                         'paths': copy.deepcopy(curr['paths']),
                         'collisions': [],
                         'parent': curr,
                         'C': False}
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
                        child['f_value'] = child['cost']
                    else:
                        pathIsNone = True
                        break
                if pathIsNone is False:
                    childrenList.append(child)
        return childrenList

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
                'h_value': 0,
                'f_value': 0,
                'parent': None,
                'C': False
                }
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        print(root['collisions'])

        # Task 3.2: Testing
        for collision in root['collisions']:
            print(disjoint_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        while len(self.open_list) != 0:

            #############################
            print('Before pop')
            for i in self.open_list:
                if len(i[3]['constraints']) == 0:
                    print('root in the open????????')
            #############################

            curr = self.pop_node()

            if len(curr['collisions']) == 0:
                self.print_results(curr)
                # print(curr['paths'])
                return curr['paths']

            #############################
            print('After pop, before collapse')
            for i in self.open_list:
                if len(i[3]['constraints']) == 0:
                    print('root in the open????????')
            #############################

            if self.prev_curr is not curr['parent']:
                # Try to collapse.
                B = findNodeNeedToBeCollapseTo(curr, self.prev_curr)
                if B is None:
                    print("Fail to find the node need to collapse")
                    print("Go func: findNodeNeedToBeCollapseTo")
                    exit(1)
                self.collapse(B)

            #############################
            print('After collapse, before restore')
            for i in self.open_list:
                if len(i[3]['constraints']) == 0:
                    print('root in the open????????')
            #############################

            if curr['C'] is True:
                print("restore triggered...........")
                # restore curr
                self.restore(curr)

                #############################
                print('After restore')
                for i in self.open_list:
                    if len(i[3]['constraints']) == 0:
                        print('root in the open????????')
                #############################

            #   disjoint_splitting
            #   standard_splitting
            newConstraints = disjoint_splitting(curr['collisions'][0])
            self.prev_curr = curr
            childrenList = self.generateChildren(curr, newConstraints)
            for child in childrenList:
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
