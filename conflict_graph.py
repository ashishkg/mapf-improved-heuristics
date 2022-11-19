import copy
import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost
from mdd import build_mdd, get_cardinal_conflicts, is_joint_mdd_empty
def build_dependency_graph(my_map, agents, starts, goals, collisions, constraints, paths, cardinals_list, cache):
    # graph will be presented with a dict,
    # a set V - of all vertices = [1,2,3...N]
    # an array EDGES = [(2,3),(4,6),...] having all edges
    edges = []
    for collision in collisions:
        agent1 = collision['a1']
        agent2 = collision['a2']
        #constraints_sorted = sorted(constraints, key=lambda constraints:[constraints['timestep'],constraints['agent']])
        is_edge = is_cardinal(my_map, starts, goals, collision, constraints, paths,cardinals_list, cache,agent1,
                               agent2)
        if is_edge:
            if (agent1, agent2) not in edges:
                edges.append((agent1, agent2))
    #print(agents, edges)
    return (agents, edges)

# BUILD DEPENDENCY GRAPH FROM PARTIAL
def build_dependency_graph_from_partial(my_map, agents, starts, goals, collisions, constraints, paths, cardinals_list,
                                   cache, agent, E):
    # graph will be presented with a dict,
    # a set V - of all vertices = [1,2,3...N]
    # an array EDGES = [(2,3),(4,6),...] having all edges
    edges = copy.deepcopy(E)
    union = dict()

    for collision in collisions:
        agent1 = collision['a1']
        agent2 = collision['a2']
        if agent1 == agent or agent2 == agent:
            is_edge = is_cardinal(my_map, starts, goals, collision, constraints, paths, cardinals_list, cache, agent1,
                                   agent2)
            if (agent1, agent2) in union:
                union[(agent1, agent2)] = is_edge or union[(agent1, agent2)]
            else:
                union[(agent1, agent2)] = is_edge
    for (agent1, agent2) in union:
        if union[(agent1, agent2)]:
            if (agent1, agent2) not in edges:
                edges.append((agent1, agent2))
        else:
            if (agent1, agent2) in edges:
                edges.remove((agent1, agent2))
    return (agents, edges)


# below function computes return the h-value and the also orders the collisions

def compute_h_value_and_reorder_collisions():
    pass

# Returns true if two vertices are dependent.
# In case of cardinal conflicts , put them in the global list
def is_cardinal(my_map, starts, goals, collisions, constraints, paths,
                 cardinals_list, cache, agent1, agent2):
    constraints_sorted = sorted(constraints, key=lambda constraints: [constraints['timestep'], constraints['agent']])
    if (agent1, agent2, str(constraints_sorted)) in cache:
        # print("USING CACHE for EDGE")
        if cache[(agent1, agent2, str(constraints_sorted))]:
            return 1
        else:
            return 0
    else:
        if (str(constraints_sorted), agent1, len(paths[agent1])) in cache:
            # print("USING CACHE for MDD1")
            # print((str(constraints_sorted), agent1, len(paths[agent1])))
            mdd1 = cache[(str(constraints_sorted), agent1, len(paths[agent1]))]
        else:
            mdd1 = build_mdd(my_map, starts[agent1], goals[agent1], agent1, constraints, len(paths[agent1])-1)
            cache[(str(constraints_sorted), agent1, len(paths[agent1]))] = mdd1
        if (str(constraints_sorted), agent2, len(paths[agent2])) in cache:
            # print("USING CACHE for MDD2")
            # print((str(constraints_sorted), agent2, len(paths[agent2])))
            mdd2 = cache[(str(constraints_sorted), agent2, len(paths[agent2]))]
        else:
            mdd2 = build_mdd(my_map, starts[agent2], goals[agent2], agent2, constraints, len(paths[agent2])-1)
            cache[(str(constraints_sorted), agent2, len(paths[agent2]))] = mdd2
        # print("agent1 cost", paths[agent1])
        # print("agent2 cost", paths[agent2])
        cardinals = get_cardinal_conflicts(mdd1, mdd2)
        # print("CARDINAL")
        # print(cardinals)
        if cardinals is not None:
            cache[(agent1, agent2, str(constraints_sorted))] = True
            for collision in collisions:
                if collision['timestep'] in cardinals:
                    cardinals_list.add(str(collision))
            return 1
        else:
            # print("JOINT MDD NOT EMPTY, returned FALSE")
            cache[(agent1, agent2, str(constraints_sorted))] = False
            return 0


# FOR MVC of CT Node curr
# Fetch Parent's h-value =hp
# Now the h-value for curr can be either h-1, h or h+1??
# run the algorithm in O(n*2^(q)) where q is in [h-1,h,h+1]
# the idea is that at each CT node generation we build the DG graph but to determine the size of MVC
# we run the algorithm with this range as we know that size is in this range

