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
        is_edge = is_dependent(my_map, starts, goals, collision, constraints, paths,cardinals_list, cache,agent1,
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
            is_edge = is_dependent(my_map, starts, goals, collision, constraints, paths, cardinals_list, cache, agent1,
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
def is_dependent(my_map, starts, goals, collision, constraints, paths,
                 cardinals_list, cache, agent1, agent2):
    constraints_of_pair = filter(lambda constraint: constraint['agent'] == agent1 or constraint['agent'] == agent2,
                                 constraints)
    constraints_sorted_pairwise = str(sorted(constraints_of_pair, key=lambda constraints_of_pair: [constraints_of_pair[
                                                                                                    'timestep'],
                                                                                      constraints_of_pair['agent']]))
    constraints_of_agent1 = filter(lambda constraint: constraint['agent'] == agent1,
                                 constraints)
    constraints_sorted_agent1 = str(sorted(constraints_of_agent1, key=lambda constraints_of_agent1: [constraints_of_agent1[
                                                                                                    'timestep'],
                                                                                      constraints_of_agent1['agent']]))
    constraints_of_agent2 = filter(lambda constraint: constraint['agent'] == agent2,
                                 constraints)
    constraints_sorted_agent2 = str(sorted(constraints_of_agent2, key=lambda constraints_of_agent2: [constraints_of_agent2[
                                                                                                    'timestep'],
                                                                                      constraints_of_agent2['agent']]))

    if (agent1, agent2, constraints_sorted_pairwise) in cache:
        if cache[(agent1, agent2, constraints_sorted_pairwise)] > 0:
            return True

    else:
        if (constraints_sorted_agent1, agent1, len(paths[agent1])-1) in cache:
            mdd1 = cache[(constraints_sorted_agent1, agent1, len(paths[agent1])-1)]
        else:
            mdd1 = build_mdd(my_map, starts[agent1], goals[agent1], agent1, constraints, len(paths[agent1])-1)
            cache[(constraints_sorted_agent1, agent1, len(paths[agent1])-1)] = mdd1
        if (constraints_sorted_agent2, agent2, len(paths[agent2])-1) in cache:
            mdd2 = cache[(constraints_sorted_agent2, agent2, len(paths[agent2])-1)]
        else:
            mdd2 = build_mdd(my_map, starts[agent2], goals[agent2], agent2, constraints, len(paths[agent2])-1)
            cache[(constraints_sorted_agent2, agent2, len(paths[agent2])-1)] = mdd2

        cardinals = get_cardinal_conflicts(mdd1, mdd2)

        if cardinals is not None:
            cache[(agent1, agent2, constraints_sorted_pairwise)] = 1
            if collision['timestep'] in cardinals:
                cardinals_list.add(str(collision))
            return True
        elif is_joint_mdd_empty(goals[agent1], goals[agent2], mdd1, mdd2):
            cache[(agent1, agent2, constraints_sorted_pairwise)] = 1
            return True
        else:
            cache[(agent1, agent2, constraints_sorted_pairwise)] = 0
            return False


# FOR MVC of CT Node curr
# Fetch Parent's h-value =hp
# Now the h-value for curr can be either h-1, h or h+1??
# run the algorithm in O(n*2^(q)) where q is in [h-1,h,h+1]
# the idea is that at each CT node generation we build the DG graph but to determine the size of MVC
# we run the algorithm with this range as we know that size is in this range

