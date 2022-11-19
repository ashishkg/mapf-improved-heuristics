import time as timer
import heapq
import copy
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
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
            return {"loc":[copy.deepcopy(loc1)],"timestep":time}
    # Edge collision
    for time in range(1, length):
        old_loc1 = get_location(path1, time-1)
        old_loc2 = get_location(path2, time-1)
        new_loc1 = get_location(path1, time)
        new_loc2 = get_location(path2, time)
        if( new_loc1 == old_loc2  and new_loc2 == old_loc1 ):
            return {"loc":[copy.deepcopy(old_loc1), copy.deepcopy(new_loc1)],"timestep":time}

    return None


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    collisions = []
    # detecting collisions for different combination of paths
    for a1 in range(len(paths)-1):
        for a2 in range( a1 + 1, len(paths)):
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
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    constraints =[]
    #Handle Edge Constraint
    if len(collision["loc"]) > 1:
        first_constraint = {
            "agent":collision["a1"],
            "loc":collision["loc"],
            "timestep":collision["timestep"]
        }
        second_constraint = {
            "agent":collision["a2"],
            "loc":[],
            "timestep":collision["timestep"]
        }
        for loc in collision["loc"]:
            second_constraint['loc'].insert(0, loc)
        constraints.append(first_constraint)
        constraints.append(second_constraint)
    # Handle Vertex Constraint
    else:
        first_constraint = {
            "agent":collision["a1"],
            "loc":collision["loc"],
            "timestep":collision["timestep"]
        }
        second_constraint = {
            "agent":collision["a2"],
            "loc":collision["loc"],
            "timestep":collision["timestep"]
        }
        constraints.append(first_constraint)
        constraints.append(second_constraint)
    return constraints





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
    pass


class CBSSolver(object):
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

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
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
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])

            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])



        self.push_node(root)
        self.closed_list[str(root['paths'])] = root
        # Task 3.1: Testing
        print("Printing Collisions at root - Task 3.1")
        print(root['collisions'])



        # Task 3.2: Testing
        print("Printing Constraints at root - Task 3.2")
        for collision in root['collisions']:
            print(standard_splitting(collision))

        ##############################
        #           High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use
        #             self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list
        #             of constraints (using your
        #                standard_splitting function). Add a new child node to your
        #                open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit
        while len(self.open_list) > 0:
            p = self.pop_node()
            if len(p["collisions"])==0:
                self.print_results(p)
                return p["paths"]
            #extract a cardinal collision or a collision from mvc
            collision = p["collisions"].pop(0)
            constraints = standard_splitting(collision)
            for constraint in constraints:
                q = {'cost': 0,
                    'constraints': [i for i in p['constraints']],
                    'paths': [j for j in p['paths']],
                    'collisions': []
                }

                if constraint not in q['constraints']:
                    q['constraints'] = q['constraints']+[constraint]
                agent = constraint['agent']
                agent_path = a_star(self.my_map, self.starts[agent], self.goals[agent],
                                    self.heuristics[agent], agent, q['constraints'])
                if agent_path is not None:
                    q['paths'][agent] = agent_path
                    if str(q['paths']) not in self.closed_list:
                        q["collisions"] = detect_collisions(q['paths'])
                        # build DG and compute MVC again
                        q['cost'] = get_sum_of_cost(q['paths'])
                        self.closed_list[str(q['paths'])] = q
                        self.push_node(q)

        self.print_results(root)
        return root['paths']


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.9f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
