from ewmvc_ILP_solver import edge_weighted_mvc
# use Depth First Search to build connected components
class EwmvcSolver:
    def __init__(self):
        pass

    # RUN DFS to obtain connected components
    def obtain_components(self,vertices, edges):
        visited = [False]*len(vertices)
        cc = [0]*len(vertices)
        counter = 0
        indexes = dict()
        neighbours = dict()
        for i in range(len(vertices)):
            indexes[vertices[i]] = i
            neighbours[i] = set()

        for edge in edges:
            x,y = edge
            u = indexes[x]
            v = indexes[y]
            existing1 = neighbours[u]
            existing1.add(v)
            existing2 = neighbours[v]
            existing2.add(u)

        for i in range(len(vertices)):
            if not visited[i]:
                counter+=1
                self.dfs(i, neighbours, counter, visited, cc)
        # makes sets of vertices
        components = []
        for c in range(1, counter+1):
            l = []
            for i in range(len(vertices)):
                if cc[i] == c:
                    l.append(vertices[i])
            components.append(l)
        return components

    # Depth First Search - Explore Routine
    def dfs(self,i, neighbours, counter, visited, cc):
        if visited[i]:
            return
        visited[i] = True
        cc[i] = counter
        for neighbour in neighbours[i]:
            self.dfs(neighbour, neighbours, counter, visited, cc )

    # Run ILP on each connected Component, and return total sum cost
    def get_ewmvc_cost(self,vertices, edges, W):
        components = self.obtain_components(vertices, edges)
        cost = 0
        for component in components:
            cost += edge_weighted_mvc(component, edges, W)
        return  cost