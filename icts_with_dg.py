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

from mdd import build_mdd, is_joint_mdd_empty, is_k_joint_mdd_empty
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost, get_costs
from dependency_graph import build_dependency_graph, build_dependency_graph_from_partial
from minmum_vertex_cover import does_mvc_exist


class ICTSSolverDG(object):
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
        self.closed_list = dict()
        self.cache = dict()

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'] , self.num_of_generated,
                                        node))
        print(node['cost-vector'],node['cost'])
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def remove_node(self, node):
        heapq.heap(self.open_list, (node['cost'], self.num_of_generated,
                                        node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        print(node['cost-vector'], node['cost'])
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

        root = {
                'paths': [],
                'id':self.num_of_generated}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, [])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['cost-vector'] = get_costs(root['paths'])
        # build a dependency graph based on root['collisions']
        # compute and update h-value for root based on dependency graph MVC
        # computing h-value

        self.push_node(root)
        self.closed_list[str(root['cost-vector'])] = root


        ##############################
        #           High-Level Search
        while len(self.open_list) > 0:
            p = self.pop_node()
            # 1. Build MDD for all agents (c1,c2,c3,..ck)
            # 2. Perform Pairwise MDD join, if it fails add a child with h-value? HIGH LEVEL SEARCH
            #            Do pairwise MAPF, get Weight, add this weight as h-value
            # 3. if no pairwise fails then perform k-merge for mdd(Low LEVEL SEARCH), if solution found then return,
            # or go to high level
            mdds = []
            for i in range(self.num_of_agents):
                if (i, p['cost-vector'][i]) in self.cache:
                    mdd = self.cache[(i, p['cost-vector'][i])]
                    mdds.append(mdd)
                else:
                    mdd = build_mdd(self.my_map, self.starts[i], self.goals[i], i, [], p['cost-vector'][i])
                    self.cache[(i, p['cost-vector'][i])] = mdd
                    mdds.append(mdd)
                #print(mdd)
            #edges = []
            # pair wise search
            empty = False
            for i in range(self.num_of_agents - 1):
                if empty:
                    break
                for j in range(i+1, self.num_of_agents):
                    empty = is_joint_mdd_empty(self.goals[i], self.goals[j], mdds[i], mdds[j])
                    if empty:
                        print("BROKEN",i, j)
                        print(mdds[i], mdds[j])
                        break
                    else:
                        print("JOINT SUCCESSFULLY ", i, j)
            # compute the min vertex cover

            if not empty:
                #return True
                k_empty = is_k_joint_mdd_empty(mdds, self.goals)
                if not k_empty:
                    print("Found Solution", p['cost'], p['cost-vector'])
                    return k_empty
            for i in range(self.num_of_agents):
                q = {'cost': p['cost'] + 1,
                     'id': self.num_of_generated
                     }

                q['cost-vector'] = copy.deepcopy(p['cost-vector'])
                q['cost-vector'][i] = q['cost-vector'][i] + 1
                if str(q['cost-vector']) not in self.closed_list:
                    self.push_node(q)
                    self.closed_list[str(q['cost-vector'])] = q


        self.print_results(root)
        #return [root['paths'], self.num_of_expanded, self.num_of_generated, str("CPU time (s):    {:.9f}".format(
        #    timer.time() - self.start_time))]
        return None

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.9f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
