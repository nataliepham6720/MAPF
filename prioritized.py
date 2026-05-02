import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


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

        # vertex constraint (1.2)
        # constraints.append({'agent': 2, 'loc': [(3,4)], 'timestep': 5})
        # edge constraint (1.3)
        # constraints.append({'agent': 1, 'loc': [(1,2), (1,3)],'timestep': 1})

        #### 1.5 Constraints to obtain optimal solution without collision
        # # Force leaving corridor at t=2
        # constraints.append({'agent': 1, 'loc': [(1,3)], 'timestep': 2})
        # # Prevent going forward instead of going down
        # constraints.append({'agent': 1, 'loc': [(1,4)], 'timestep': 2})
        # # Prevent oscillation backward
        # constraints.append({'agent': 1, 'loc': [(1,3), (1,2)], 'timestep': 2})
        # # Prevent jumping back into (1,4) too early
        # constraints.append({'agent': 1, 'loc': [(1,4)], 'timestep': 3})
        # Sum of costs:    8
        # 0: [(1, 1), (1, 2), (1, 3), (1, 4), (1, 5)], 
        # 1: [(1, 2), (1, 3), (2, 3), (1, 3), (1, 4)]
        ##################################

        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None:
                # raise BaseException('No solutions')
                print("No solutions")
                return None
            result.append(path)

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches
            for j in range(i + 1, self.num_of_agents):
                # 2.1 Vertex constraints along the path
                for t in range(len(path)):
                    constraints.append({'agent': j, 'loc': [path[t]], 'timestep': t})

                # 2.2 Edge constraints
                for t in range(len(path) - 1):
                    u = path[t]
                    v = path[t + 1]
                    # Forward edge constraint
                    constraints.append({'agent': j, 'loc': [u, v], 'timestep': t + 1})
                    # Reverse edge constraint 
                    constraints.append({'agent': j, 'loc': [v, u], 'timestep': t + 1})

                
                # 2.3 Stay at goal once reached it
                goal = path[-1]
                for t in range(len(path), len(path) + 50):
                    constraints.append({'agent': j, 'loc': [goal], 'timestep': t})
            ##############################

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
