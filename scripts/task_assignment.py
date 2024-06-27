import pulp
import numpy as np
import pandas as pd

class TaskAssignmentSolver:
    def __init__(self):
        self.inf = 1000000
        self.lambda_balance = 1000  # Weight factor for balancing
        self.block_names = []

    def load_matrix(self, out_dir):
        with open(out_dir + '/cost_matrix_a.csv', 'r') as f:
            header = f.readline()
            blocks = header.split(',')[:-1]
            n = len(blocks)
            self.block_names = blocks

        # read csv file cost_matri_a.csv
        cost_matrix_a = np.genfromtxt(out_dir + '/cost_matrix_a.csv', delimiter=',', skip_header=1)[:, :n]
        # read csv file time_matrix_a.csv
        cost_matrix_b = np.genfromtxt(out_dir + '/cost_matrix_b.csv', delimiter=',', skip_header=1)[:, :n]
        # read delta_matrix
        delta_matrix = np.genfromtxt(out_dir + '/delta_matrix.csv', delimiter=',', skip_header=1)[:, :n]
        # read support matrix
        s = np.genfromtxt(out_dir + '/support_matrix.csv', delimiter=',')[:, :n]
        # read s_j
        s_j = np.genfromtxt(out_dir + '/support_req.csv', delimiter=',')[:-1]
        # read precedence matrix
        precedence_matrix = np.genfromtxt(out_dir + '/precedence.csv', delimiter=',')

        cost_matrix = np.array([cost_matrix_a, cost_matrix_b])


        return cost_matrix, delta_matrix, s, s_j, precedence_matrix

    def solve_problem(self, n, m, p, c, s, delta, s_j, precedence_constraints):
        if (n > m):
            print("Number of tasks must be greater than or equal to the number of robots")
            return None

        # Define the problem
        self.prob = pulp.LpProblem("Task_Assignment", pulp.LpMinimize)

        # Define the variables
        x = pulp.LpVariable.dicts("x", (range(n), range(m), range(p)), cat='Binary')
        y = pulp.LpVariable.dicts("y", (range(n), range(m)), cat='Binary')
        z = pulp.LpVariable.dicts("z", (range(n), range(m-n+1)), lowBound=0, cat='Integer')
        z_max = pulp.LpVariable.dicts("z_max", (range(m-n+1)), lowBound=0, cat='Integer')
        z_min = pulp.LpVariable.dicts("z_min", (range(m-n+1)), lowBound=0, cat='Integer')

        # Objective function
        self.prob += pulp.lpSum(c[i][j][t] * x[i][j][t] for i in range(n) for j in range(m) for t in range(p)) + \
                    pulp.lpSum(s[i][j] * y[i][j] for i in range(n) for j in range(m)) + \
                    self.lambda_balance * pulp.lpSum(z_max[k] - z_min[k] for k in range(m-n+1))

        # Constraints
        # Each task has one robot and one block
        for j in range(m):
            self.prob += pulp.lpSum(x[i][j][t] for i in range(n) for t in range(p)) == 1

        # A robot cannot be assigned to both pick-and-place and support tasks for the same task
        for i in range(n):
            for j in range(m):
                for t in range(p):
                    self.prob += x[i][j][t] + y[i][j] <= 1

        # If a task needs support, a robot is assigned to support
        for j in range(m):
            self.prob += pulp.lpSum(y[i][j] for i in range(n)) == s_j[j]

        # Ensures the correct block type is chosen for each task
        for j in range(m):
            self.prob += pulp.lpSum(x[i][j][t] * delta[j][t] for i in range(n) for t in range(p)) == 1

        # Each block is used at most once
        for t in range(p):
            self.prob += pulp.lpSum(x[i][j][t] for i in range(n) for j in range(m)) <= 1

        # Precedence constraints
        for (t1, t2) in precedence_constraints:
            self.prob += pulp.lpSum(x[i][j][t1] * (m - j) for i in range(n) for j in range(m)) >= \
                    pulp.lpSum(x[i][j][t2] * (m - j) for i in range(n) for j in range(m))

        # Workload balancing constraints in a window of size n
        for i in range(n):
            for k in range(m-n+1):
                self.prob += z[i][k] == pulp.lpSum(x[i][j][t] for t in range(p) for j in range(k, k+n)) + pulp.lpSum(y[i][j] for j in range(k, k+n))

        
        for i in range(n):
            for k in range(m-n+1):
                self.prob += z_min[k] <= z[i][k]
                self.prob += z_max[k] >= z[i][k]

        # Solve the problem
        self.prob.solve()
        solution = {}
        for v in self.prob.variables():
            solution[v.name] = v.varValue
        
        assignment = []
        for j in range(m):
            robot_idx = -1
            sup_idx = -1
            block_idx = -1
            for i in range(n):
                if (robot_idx == -1):
                    for t in range(p):
                        if (robot_idx == -1) and (solution[f"x_{i}_{j}_{t}"] == 1):
                            robot_idx = i
                            block_idx = t
                            
                if s_j[j] == 1 and (sup_idx == -1):
                    if solution[f"y_{i}_{j}"] == 1:
                        sup_idx = i
            assignment.append((j, robot_idx, block_idx, sup_idx))

        solution['Total Cost'] = pulp.value(self.prob.objective)
        return assignment, pulp.LpStatus[self.prob.status]


def example():
    # Example usage
    solver = TaskAssignmentSolver()
    n = 2
    m = 4
    p = 5 
    c = [[[2, 2, 2, 1000000, 1000000], 
        [3, 3, 3, 3, 3], 
        [1000000, 1000000, 4, 4, 4], 
        [5, 5, 5, 5, 5]],
        [[1000000, 1000000, 1000000, 1000000, 1000000],
        [1, 1, 1, 1, 1],
        [1000000, 1000000, 1000000, 1000000, 1000000],
        [1000000, 1000000, 1000000, 1000000, 1000000]]]
    s = [[1000000, 1000000, 1, 1],
        [1, 1, 1, 1]]
    delta = [[1, 0, 0, 1, 0],
            [0, 1, 0, 0, 1],
            [0, 1, 0, 0, 1],
            [0, 0, 0, 1, 0]]
    s_j = [0, 1, 1, 0]
    precedence_constraints = [(1, 2)]

    assignment, status = solver.solve_problem(n, m, p, c, s, delta, s_j, precedence_constraints)
    print(f"Objective Value: {assignment}, Status: {status}")

def test(task='tower'):
    output_dir = f'/home/philip/catkin_ws/src/mr_planner/outputs/lego/{task}'
    solver = TaskAssignmentSolver()
    cost_matrix, delta_matrix, s, s_j, precedence_matrix = solver.load_matrix(output_dir)
    n = cost_matrix.shape[0]
    m = cost_matrix.shape[1]
    p = cost_matrix.shape[2]
    print(n, m, p, cost_matrix, s, delta_matrix, s_j, precedence_matrix)
    assignment, status = solver.solve_problem(n, m, p, cost_matrix, s, delta_matrix, s_j, precedence_matrix)
    print(f"Status: {status}")
    
    for a in assignment:
        print(f"Task {a[0]} assigned to robot {a[1]} and lego block {a[2]} {solver.block_names[a[2]]}, support robot {a[3]}")

example()
test('bridge_top')