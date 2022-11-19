# At the CT node N
# first build list of conflicts
# Now based on these conflict
# We have a list of agents a1,a2,...,ak
# for each pair ai, aj --
#   if no conflict , => no edge
#   if conflict is present -
#   Build MDDs for both
#   Check for cardinal conflict with the width of MDDs
#   if not cardinal run the joint mdd algorithm
import time as timer
import heapq
import copy
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost
from weighted_d_graph import build_wdg, build_wdg_from_partial
from minmum_vertex_cover import does_mvc_exist
from ewmvc import EwmvcSolver


def detect_collision(path1, path2):
    ##############################
    #           Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    length = max(len(path1), len(path2))

    # Vertex collision
    for time in range(length):
        loc1 = get_location(path1, time)
        loc2 = get_location(path2, time)
        if loc1 == loc2:
            return {"loc": [copy.deepcopy(loc1)], "timestep": time}
    # Edge collision
    for time in range(1, length):
        old_loc1 = get_location(path1, time - 1)
        old_loc2 = get_location(path2, time - 1)
        new_loc1 = get_location(path1, time)
        new_loc2 = get_location(path2, time)
        if (new_loc1 == old_loc2 and new_loc2 == old_loc1):
            return {"loc": [copy.deepcopy(old_loc1), copy.deepcopy(new_loc1)], "timestep": time}

    return None


def detect_collisions(paths):
    ##############################
    #           Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    collisions = []
    # detecting collisions for different combination of paths
    for a1 in range(len(paths) - 1):
        for a2 in range(a1 + 1, len(paths)):
            get_collision = detect_collision(paths[a1], paths[a2])
            if get_collision:
                collision = dict()
                collision["a1"] = a1
                collision["a2"] = a2
                collision["loc"] = get_collision["loc"]
                collision["timestep"] = get_collision["timestep"]
                collisions.append(collision)
    return collisions


def standard_splitting(collision):
    ##############################
    #           Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    constraints = []
    # Handle Edge Constraint
    if len(collision["loc"]) > 1:
        first_constraint = {
            "agent": collision["a1"],
            "loc": collision["loc"],
            "timestep": collision["timestep"]
        }
        second_constraint = {
            "agent": collision["a2"],
            "loc": [],
            "timestep": collision["timestep"]
        }
        for loc in collision["loc"]:
            second_constraint['loc'].insert(0, loc)
        constraints.append(first_constraint)
        constraints.append(second_constraint)
    # Handle Vertex Constraint
    else:
        first_constraint = {
            "agent": collision["a1"],
            "loc": collision["loc"],
            "timestep": collision["timestep"]
        }
        second_constraint = {
            "agent": collision["a2"],
            "loc": collision["loc"],
            "timestep": collision["timestep"]
        }
        constraints.append(first_constraint)
        constraints.append(second_constraint)
    return constraints


class CBSSolverWDG(object):
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
        self.closed_list = dict()
        self.cardinals_list = set()
        self.cache = dict()
        self.ewmvc_solver = EwmvcSolver()
        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'] + node['h-value'], len(node['collisions']), self.num_of_generated,
                                        node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def remove_node(self, node):
        heapq.heap(self.open_list, (node['cost'] + node['h-value'], len(node['collisions']), self.num_of_generated,
                                    node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
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
                'id': self.num_of_generated}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        # build a dependency graph based on root['collisions']
        # compute and update h-value for root based on dependency graph MVC
        # put the cardinal collision on start of collision list , if it does not exist then put one from DG
        wdg = build_wdg(self.my_map, list(range(0, self.num_of_agents)), self.starts,
                        self.goals, root['collisions'], root['constraints'], root['paths'],
                        self.cardinals_list,
                        self.cache)
        V, E, W = wdg
        root['edges'] = E
        hvaluedict = dict()
        edgedict = dict()
        weightdict = dict()
        edgedict[root['id']] = E
        weightdict[root['id']] = W
        # computing h-value
        root['h-value'] = self.ewmvc_solver.get_ewmvc_cost(V, E, W)
        root['h2applied'] = True
        hvaluedict[root['id']] = root['h-value']
        self.push_node(root)
        self.closed_list[str(root['paths'])] = root
        #max_h = 0
        for collision in root['collisions']:
            # print(standard_splitting(collision))
            pass

        ##############################
        #           High-Level Search
        while len(self.open_list) > 0:
            p = self.pop_node()

            # Lazy Approach, checking for h2 applied or not
            if p['h2applied'] is False:
                weighted_d_graph = build_wdg_from_partial(self.my_map, list(range(0, self.num_of_agents)), self.starts,
                                                          self.goals, p['collisions'], p['constraints'], p['paths'],
                                                          self.cardinals_list,
                                                          self.cache,
                                                          p['agent'],
                                                          edgedict[p['parent']],
                                                          weightdict[p['parent']]
                                                          )
                vertices, edges, weight = weighted_d_graph
                edgedict[p['id']] = edges
                weightdict[p['id']] = weight
                p['h-value'] = self.ewmvc_solver.get_ewmvc_cost(vertices, edges, weight)
                p['h2applied'] = True
                #max_h = max(max_h, p['h-value'])
                hvaluedict[p['id']] = p['h-value']
                print("node id", p['id'], "h value ", hvaluedict[p['id']])
                self.push_node(p)
                continue

            if len(p["collisions"]) == 0:
                self.print_results(p)
                #print("Max h Value ", max_h)
                return p['paths']

            collision = None
            for c in p["collisions"]:
                if str(c) in self.cardinals_list:
                    collision = c
                    break
            if collision is None:
                collision = p["collisions"].pop()  # can be any collision
            constraints = standard_splitting(collision)
            for constraint in constraints:
                q = {'cost': 0,
                     'constraints': [i for i in p['constraints']],
                     'paths': [j for j in p['paths']],
                     'collisions': []
                     }

                if constraint not in q['constraints']:
                    q['constraints'] = q['constraints'] + [constraint]
                agent = constraint['agent']
                agent_path = a_star(self.my_map, self.starts[agent], self.goals[agent],
                                    self.heuristics[agent], agent, q['constraints'])
                if agent_path is not None:
                    q['paths'][agent] = agent_path

                    if str(q['paths']) not in self.closed_list:
                        # build DG and compute MVC again, update h-value
                        q["collisions"] = detect_collisions(q['paths'])
                        q['parent'] = p['id']
                        q['cost'] = get_sum_of_cost(q['paths'])
                        # h1 is 0, we will apply h2 at expansion
                        q['h-value'] = 0
                        q['h2applied'] = False
                        q['id'] = self.num_of_generated
                        q['agent'] = agent
                        self.closed_list[str(q['paths'])] = q
                        self.push_node(q)
                    else:
                        pass

        self.print_results(root)
        return root['paths']

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.9f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
