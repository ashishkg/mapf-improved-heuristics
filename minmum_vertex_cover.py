import random


#   REFERENCE FOR ALGORITHM
#   Downey, R.G., Fellows, M.R. (1995). Parameterized Computational Feasibility.
#   In: Clote, P., Remmel, J.B. (eds) Feasible Mathematics II. Progress in Computer Science and Applied Logic, vol 13.
#   Birkhäuser Boston. https://doi.org/10.1007/978-1-4612-2566-9_7

# does a k size vertex cover exist?
# vertices = [v1,v2,v3,..]array containing all vertex
# edges = [(v1,v2),(v2,v3),...] with all vertices and set as its neighbour
# HINT: with some more complex DS(preferably linked) we can slightly optimize for space
def does_mvc_exist(vertices, edges, k):
    #print(vertices, edges, k)
    if( k < 0):
        return False
    # base case with k == 0
    if k == 0:
        if len(edges) == 0:
            return True
        else:
            return False

    if len(edges) == 0:
        return True
    edge = random.choice(edges)
    u, v = edge
    # These two blocks looks redundant, but I have separated these two calls to save space,
    # if we get True after first call, we never allocate space for second call
    edge_not_u = []
    vertices_not_u = []
    for vertex in vertices:
        if vertex != u:
            vertices_not_u.append(vertex)
    for e in edges:
        if u not in e:
            edge_not_u.append(e)

    if does_mvc_exist(vertices_not_u, edge_not_u, k - 1):
        return True

    edge_not_v = []
    vertices_not_v = []
    for e in edges:
        if v not in e:
            edge_not_v.append(e)
    for vertex in vertices:
        if vertex != v:
            vertices_not_v.append(vertex)

    return does_mvc_exist(vertices_not_v, edge_not_v, k - 1)
