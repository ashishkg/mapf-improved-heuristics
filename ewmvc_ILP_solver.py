from ortools.linear_solver import pywraplp

# takes a connected graph
# returns EWMVC cost
def edge_weighted_mvc(vertices, edges, weights):
        solver = pywraplp.Solver.CreateSolver('SCIP')
        if solver:
            #print("SOLVER INITIALIZED")
            infinity = solver.infinity()
            variables = dict()
            for i in range(len(vertices)):
                x = solver.IntVar(0.0, infinity, str(i))
                variables[vertices[i]] = x
            for edge in edges:
                u,v = edge
                w = weights[(edge)]
                if u in variables and v in variables:
                    x1 = variables[u]
                    x2 = variables[v]
                    # adding constraint
                    solver.Add(x1 + x2 >= w)
                else:
                    pass
            #objective
            solver.Minimize(sum(variables.values()))
            #print("Number of Vertices", len(vertices))
            #print("Number of Variables", solver.NumVariables())
            #print("Number of Edges", len(edges))
            #print("Number of Constraints", solver.NumConstraints())
            status = solver.Solve()
            if status == pywraplp.Solver.OPTIMAL:
                #print('Solution:')
                #print('Objective value =', solver.Objective().Value())
                return solver.Objective().Value()
            else:
                print('The problem does not have an optimal solution.')
                return 0
        else:
            return None


#x = ewmvc.edge_weighted_mvc([0,1,2],[(0,1),(1,2)],{(0,1):10,(1,2):10,(0,2):20})
#print(x)