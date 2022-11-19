import copy

from single_agent_planner import build_goal_constraint, build_constraint_table, move, is_constrained
import queue
import itertools

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


# a few ideas
# (1)can be a list of lists
# (2)can be a dict indexed by time and value be a list of locations
# (3)below presents a node based ds below
#                                source
#                            /            \
#                       child1           child2
#                      /    \           /      \
#                 child3   child4   child5     child6
#                       \       \       /       /
#                               goal
def get_dag(goal_node, node_to_id, id_to_node):
    level = queue.SimpleQueue()
    goal_node['children'] = None
    closed_list = dict()
    # root generation
    level.put(goal_node)
    closed_list[(goal_node['loc'], goal_node['timestep'])] = goal_node
    output_dict = dict()
    while level.qsize() > 0:
        size = level.qsize()
        for node in range(size):
            curr = level.get()
            output_dict[curr['id']] = curr

            if len(curr['parents']) == 0:
                return (curr, output_dict)
            else:
                parent_ids = curr['parents']
                for parent_id in parent_ids:
                    parent = id_to_node[parent_id]
                    if parent_id in closed_list:
                        existing = closed_list[parent_id]
                        if curr['id'] not in existing['children']:
                            existing['children'].append(curr['id'])
                    else:
                        parent['children']= [curr['id']]
                        level.put(parent)
                        closed_list[parent_id] = parent
    return None




# Build MDD containg all paths of length = cost
def build_mdd(my_map, start_loc, goal_loc, agent, constraints, cost):
    constraints_table = build_constraint_table(constraints, agent)
    pre_goal_time = build_goal_constraint(constraints, agent, goal_loc)
    #print("Build MDD called", agent, start_loc, goal_loc, cost)
    ##############################
    count = 0
    root = {'loc': start_loc, 'timestep': 0, 'id' : count, 'parents':[]}
    open_list = queue.SimpleQueue()
    # idea is to put key in the list
    open_list.put(root)
    count += 1
    id_to_node = dict()
    id_to_node[root['id']] = root
    node_to_id = dict()
    node_to_id[(root['loc'], root['timestep'])] = root['id']
    while open_list.qsize() > 0:
        size = open_list.qsize()
        for node in range(size):
            curr = open_list.get()
            # Goal Test at Expansion
            if curr['loc'] == goal_loc and curr['timestep'] > pre_goal_time and curr['timestep'] == cost:
                return get_dag(curr, node_to_id, id_to_node)
            # generation of nodes - visit all neighbours in all directions
            for dir in range(5):
                child_loc = move(curr['loc'], dir)
                if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
                        or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
                    continue
                if my_map[child_loc[0]][child_loc[1]]:
                    continue
                if is_constrained(curr['loc'], child_loc, curr['timestep'] + 1, constraints_table):
                    continue
                child = {'loc': child_loc,
                         'timestep': curr['timestep']+1
                         }
                if child['timestep'] > cost:
                    continue
                if (child['loc'], child['timestep']) in node_to_id:
                    child_id = node_to_id[(child['loc'], child['timestep'])]
                    exisiting = id_to_node[child_id]
                    if curr['id'] not in exisiting['parents']:
                        exisiting['parents'].append(curr['id'] )
                else:
                    child['parents'] = [curr['id']]
                    child['id'] = count
                    count+= 1
                    node_to_id[(child['loc'], child['timestep'])] = child['id']
                    id_to_node[child['id']] = child
                    open_list.put(child)
    return None  # Failed to find solutions

# Attempt to join the MDDs and return True if can not join, i.e. if joint is empty
# returns False if both goals are reached
def is_joint_mdd_empty(goal1, goal2, mdd1, mdd2):
    source1, id_to_node1 = mdd1
    source2, id_to_node2 = mdd2
    valid_children1 = build_children_dict(mdd1)
    valid_children2 = build_children_dict(mdd2)
    timestep = 0
    goal1reached = False
    goal2reached = False
    joint_list = queue.SimpleQueue()
    joint_list.put((source1['loc'], source2['loc'], timestep))
    closed = set()
    while joint_list.qsize() > 0:
        size = joint_list.qsize()
        for i in range(size):
            n1, n2, t = joint_list.get()
            if n1 == goal1:
                l1 = [n1]
            else:
                l1 = valid_children1[(n1, t)]
            if n2 == goal2:
                l2 = [n2]
            else:
                l2 = valid_children2[(n2, t)]

            for t1 in l1:
                for t2 in l2:
                    if t1 == t2:
                        pass
                    else:
                        if t1 == goal1:
                            goal1reached = True
                        if t2 == goal2:
                            goal2reached = True
                        if goal1reached and goal2reached:
                            return False
                        if (t1,t2, t+1) not in closed:
                            joint_list.put((t1, t2, t+1))
                            closed.add((t1,t2, t+1))
    return True


# for cost c , returns a length c list, with width at that time step
def get_time_width_of_dag(mdd):
    source, id_to_node = mdd
    width_info = []
    open_list = queue.SimpleQueue()
    open_list.put(source)
    children_dict = dict()
    children_dict[source['id']] = source
    while open_list.qsize() > 0:
        size = open_list.qsize()
        width_info.append(size)
        for i in range(size):
            curr = open_list.get()
            # print(curr)
            if curr['children'] is not None:
                for child_id in curr['children']:
                    child = id_to_node[child_id]
                    if child_id in children_dict:
                        # do nothing , already generated
                        pass
                    else:
                        children_dict[child_id] = child
                        open_list.put(child)
    return width_info

# given two mdds, see if they have cardinal conflicts
def get_cardinal_conflicts(mdd1, mdd2):
    dag_widths1 = get_time_width_of_dag(mdd1)
    dag_widths2 = get_time_width_of_dag(mdd2)
    cardinals = []
    for i in range(1, min(len(dag_widths1), len(dag_widths2)) - 1):
        #print(i)
        if dag_widths1[i] == 1 and dag_widths2[i] == 1:
            cardinals.append(i)
    if len(cardinals) == 0:
        return None
    else:
        return cardinals


def is_k_joint_mdd_empty(mdds, goals):
    k = len(mdds)
    valid_children_list = []
    goals_reached = []
    sources = tuple()
    for i in range(k):
        valid_children = build_children_dict(mdds[i])
        valid_children_list.append(valid_children)
        goals_reached.append(False)
        source, id_to_node = mdds[i]
        s = (source['loc'])
        sources = (*sources,s)
    timestep = 0
    joint_list = queue.SimpleQueue()
    joint_list.put((sources , timestep))
    closed = set()
    while joint_list.qsize() > 0:
        if all_goals_reached(goals_reached):
            #print("GOAL REACHED NON EMPTY return False FROM MULTI MERGED")
            return False
        size = joint_list.qsize()
        for i in range(size):
            nodes, time = joint_list.get()
            next_level = []
            for j in range(k):
                if nodes[j] == goals[j]:
                    l = [nodes[j]]
                else:
                    l = valid_children_list[j][(nodes[j], time)]
                next_level.append(l)

            """for t1 in l1:
                for t2 in l2:
                    if t1 == t2:
                        pass
                    else:
                        if t1 == goal1:
                            goal1reached = True
                        if t2 == goal2:
                            goal2reached = True
                        if (t1, t2, t + 1) not in closed:
                            joint_list.put((t1, t2, t + 1))
                            closed.add((t1, t2, t + 1))"""
            #print(next_level)
            for element in itertools.product(*next_level):
                unique_test_set = set()
                has_duplicates = False
                for t in element:
                    if t in unique_test_set:
                        has_duplicates = True
                        #print("Duplicate Detected")
                    else:
                        unique_test_set.add(t)
                #print(element, " has duplicates = ", has_duplicates)
                if not has_duplicates:
                    for e in range(len(element)):
                        if element[e] == goals[e]:
                            goals_reached[e] = True
                    if (element, time + 1) not in closed:
                        #print("NEXT")
                        #print((element, time + 1))
                        if (element, time + 1) not in closed:
                            joint_list.put((element,time+1))
                            closed.add((element, time+1))

    #print("JOINT MDD EMPTY return TRUE")
    return True

    # for cost c , returns a length c list, with width at that time

    # for cost c , returns a length c list, with width at that time

"""def isQueuesNonEmpty(queues):
    for q in queues:
        if q.qsize() == 0:
            return False
    return True"""

def all_goals_reached(goalsReached):
    for g in goalsReached:
        if g is False:
            return False
    return True
def printq(q):
    l = []
    s = q.qsize()
    for i in range(s):
        p = q.get()
        l.append(p['loc'])
        q.put(p)
    print(l)


def printjointq(q):
    l = []
    s = q.qsize()
    for i in range(s):
        p = q.get()
        a,b,t = p
        l.append((a, b, t))
        q.put(p)
    print(l)

def build_children_dict(mdd):
    closed = dict()
    s, id_to_node = mdd
    open_list = queue.SimpleQueue()
    open_list.put(s)
    generated = set()
    while open_list.qsize() >0:
        #if s['loc'] in closed:
            #print(s['loc'], "--->",closed[s['loc']])
        p = open_list.get()
        #print(p)
        if p['children'] is None:
            closed[p['loc']] = []
            return closed
        else:
            children_ids = p['children']
            l=[]
            #print(children_ids)
            for child_id in children_ids:
                child = id_to_node[child_id]
                l.append(child['loc'])
                if child_id not in generated:
                    #print("putting", child_id)
                    open_list.put(child)
                    generated.add(child_id)
            closed[(p['loc'], p['timestep'])] = l
