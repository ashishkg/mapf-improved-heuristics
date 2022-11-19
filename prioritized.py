import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost
from mdd import build_mdd, get_time_width_of_dag

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
        free_loc = self.get_free_loc()
        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            mdd = build_mdd(self.my_map, self.starts[i], self.goals[i],
                          i, constraints, 12)
            #print(mdd)
            if mdd is not None:
                width_time = get_time_width_of_dag(mdd)
                print(width_time)
            continue
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches

                    # Adding an Edge Constraint , the agent i made the move from loc path[n-1]
                    # to loc path[n] at timestep timesteps[n], thus it is an edge constraint for
            for n in range(len(path)):
                for j in range(i+1, self.num_of_agents):
                    # Adding an Vertex Constraint
                    constraints.append({'agent': j,
                                    'loc': [path[n]],
                                   'timestep': n})
                    # Adding an Edge Constraint , the agent i made the move from loc path[n-1]
                    # to loc path[n] at timestep timesteps[n], thus it is an edge constraint for
                    # agent j
                    if n > 0:
                        constraints.append({'agent': j,
                                            'loc': [path[n-1], path[n]],
                                            'timestep': n})
                    # Adding Future Constraints Due to Goal Reached by high priority agent
                    if n == len(path) - 1:
                        for future in range(free_loc + 1):
                            constraints.append({'agent': j,
                                                'loc': [path[n]],
                                                'timestep': n + future})
            ##############################

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result

    def get_free_loc(self):
        free_loc = 0
        for row in self.my_map:
            for col in row:
                if (col == False):
                    free_loc += 1
        return free_loc