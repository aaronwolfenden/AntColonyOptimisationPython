import matplotlib.pyplot as plt
import numpy as np
import random
import time


class AntColonyOptimisation:
    """ The Main Class """

    def __init__(self, map_size=10, colony_size=20, start_point_x=0, start_point_y=0,
                 alpha=2.0, beta=1, mode=1, evaporation_rate=0.2,
                 iteration_amount=100, end_point_x=0, end_point_y=0, obstacle_amount=0,
                 dynamic_obstacles=""):
        self.start_point_x = start_point_x
        self.start_point_y = start_point_y
        self.end_point_x = end_point_x
        self.end_point_y = end_point_y
        self.map_size = map_size = map_size + 1
        self.grid_map = np.ndarray(shape=(map_size, map_size), dtype=object)
        self.colony_size = colony_size
        self.alpha = alpha
        self.beta = beta
        self.shortest_path = []
        self.evaporation_rate = evaporation_rate
        self.global_best = []
        self.mode = mode
        self.iteration_amount = iteration_amount
        self.ants = []
        self.obstacle_amount = obstacle_amount
        self.best_path_so_far = []
        self.dynamic_obstacle = dynamic_obstacles
        self.elitist_convergence_time = 0
        self.ant_system_convergence_time = 0
        self.as_average_iterations = 0
        self.eas_average_iterations = 0
        self.eas_length_list = []
        self.as_length_list = []

    def create_grid_map(self, map_size):
        """ Initialises The Grid Space Dependent On The Map Size """
        for x in range(map_size):
            for y in range(map_size):
                self.grid_map[x, y] = Node(x, y)
                # self.grid_map[x, y].print_information()
        self.set_hive()
        self.set_goal()
        if self.obstacle_amount > 0:
            self.generate_random_obstacles()

    def obstacle_add_remove(self, x=3, y=3):
        """ The Method Used To Initialise The Obstacle Attribute Of A Node"""
        if self.grid_map[x, y].obstacle_present:
            self.grid_map[x, y].obstacle_present = False
            # print("Obstacle Has Been Added At " + str(self.grid_map[x, y]) + "!")
        elif not self.grid_map[x, y].obstacle_present:
            self.grid_map[x, y].obstacle_present = True
            # print("Obstacle Has Been Added At " + "[" + str(x) + ", " + str(y) + "]" + "!")

    def display_grid(self, map_size):
        """ Initialises The Nodes Of The Grid Dependent On The Map Size """
        for x in range(map_size):
            for y in range(map_size):
                if self.grid_map[x, y].hive:
                    ax.scatter(x, y, c="#19FF30", edgecolors="#000000")
                    continue
                if self.grid_map[x, y].goal:
                    ax.scatter(x, y, c="#FF1018", edgecolors="#000000")
                    continue
                if self.grid_map[x, y].obstacle_present:
                    ax.scatter(x, y, c="#000000", edgecolors="#FF1018")
                    continue
                else:
                    cmap = plt.get_cmap('Blues')
                    ax.scatter(x, y, color=cmap(self.grid_map[x, y].pheromone_amount))

    def display_pheromone_map(self):
        """ Method For Printing Out The Pheromone Levels In Command Line"""
        for x in range(aco.map_size):
            for y in range(aco.map_size):
                print(" [" + str(aco.grid_map[x, y].x) + ", " + str(aco.grid_map[x, y].y) + "] : " +
                      str(aco.grid_map[x, y].pheromone_amount))

    def set_hive(self):
        """ Sets the hive to user defined start point"""
        self.grid_map[self.start_point_x, self.start_point_y].hive = True

    def set_goal(self):
        """ Sets the goal to user defined end point"""
        self.grid_map[self.end_point_x, self.end_point_y].goal = True

    def ant_system(self):
        """ Method For Ant Colony System"""
        start_time = time.clock()
        for i in range(aco.iteration_amount):
            ants = [(Ant(aco.grid_map[aco.start_point_x, aco.start_point_y].x,
                         aco.grid_map[aco.start_point_x, aco.start_point_y].y)) for i in range(aco.colony_size)]
            print("Iteration: " + str(i))
            if i == (aco.iteration_amount // 2) and self.dynamic_obstacle.lower() == "y":
                aco.display_grid(aco.map_size)
                plt.savefig('as_pre_obstacle')
                self.generate_random_obstacles()
            for ant in ants:
                while not ant.tour_finished:
                    if len(ant.route_taken) > 1000:
                        print("Ant Timed Out")
                        # print("[" + str(ant.current_node.x) + ", " + str(ant.current_node.y) + "]")
                        # print("Pheromone Is At: " + str(aco.grid_map[ant.current_node.x,
                        #                                         ant.current_node.y].pheromone_amount))
                        ant.tour_finished = True
                        ant.found_solution = False
                    ant.move_ant()
            for ant in ants:
                if ant.tour_finished and ant.found_solution:
                    if len(aco.best_path_so_far) == 0:
                        aco.best_path_so_far = ant.route_taken
                        new_as_convergence_time = time.clock()
                        self.as_average_iterations = i
                    elif len(aco.best_path_so_far) > len(ant.route_taken):
                        aco.best_path_so_far = ant.route_taken
                        new_as_convergence_time = time.clock()
                        self.as_average_iterations = i
                    ant.ant_system_pheromone_update()
            aco.ant_system_global_pheromone_update()
            self.as_length_list.append(len(aco.best_path_so_far))
        self.ant_system_convergence_time = (new_as_convergence_time - start_time)

    def ant_system_global_pheromone_update(self):
        """ Method For Ant System Global Pheromone Update """
        for x in range(aco.map_size):
            for y in range(aco.map_size):
                self.grid_map[x, y].pheromone_amount = (1.0 - aco.evaporation_rate) \
                                                       * self.grid_map[x, y].pheromone_amount

    def elitist_aco(self):
        """ Method For Elitist ACO"""
        start_time = time.clock()
        for i in range(aco.iteration_amount):
            ants = [(Ant(aco.grid_map[aco.start_point_x, aco.start_point_y].x, aco.grid_map[aco.start_point_x,
                                                                                            aco.start_point_y].y)) for i
                    in range(aco.colony_size)]
            print("Iteration: " + str(i))
            if i == (aco.iteration_amount // 2) and self.dynamic_obstacle.lower() == "y":
                aco.display_grid(aco.map_size)
                plt.savefig('eas_pre_obstacles')
                self.generate_random_obstacles()
            for ant in ants:
                while not ant.tour_finished:
                    if len(ant.route_taken) > (int(aco.map_size)**int(aco.map_size)):
                        print("Ant Timed Out")
                        # print("[" + str(ant.current_node.x) + ", " + str(ant.current_node.y) + "]")
                        # print("Pheromone Is At: " + str(aco.grid_map[ant.current_node.x,
                        #                                         ant.current_node.y].pheromone_amount))
                        ant.tour_finished = True
                        ant.found_solution = False
                    ant.move_ant()
            for ant in ants:
                if ant.tour_finished and ant.found_solution:
                    if not aco.global_best:
                        aco.global_best = ant.route_taken
                        new_convergence_time = time.clock()
                        self.eas_average_iterations = i
                    elif len(aco.global_best) > len(ant.route_taken):
                        aco.global_best = ant.route_taken
                        new_convergence_time = time.clock()
                        self.eas_average_iterations = i
                    ant.elitist_aco_pheromone_update()
            aco.ant_system_global_pheromone_update()
            self.eas_length_list.append(len(aco.global_best))
        self.elitist_convergence_time = (new_convergence_time - start_time)
    def ant_colony_system(self):
        """ Ant Colony System
            Adds Local Pheromone Update
            Best Only Global Update """

        for i in range(aco.iteration_amount):
            ants = [(Ant(aco.grid_map[1, 1].x, aco.grid_map[1, 1].y)) for i in range(aco.colony_size)]
            print("Iteration: " + str(i))
            for ant in ants:
                while not ant.tour_finished:
                    if len(ant.route_taken) > 1000:
                        print("Ant Timed Out")
                        # print("[" + str(ant.current_node.x) + ", " + str(ant.current_node.y) + "]")
                        # print("Pheromone Is At: " + str(aco.grid_map[ant.current_node.x,
                        #                                         ant.current_node.y].pheromone_amount))
                        ant.tour_finished = True
                        ant.found_solution = False
                    ant.move_ant()
                    ant.acs_local_pheromone_update()
                if ant.tour_finished and ant.found_solution:
                    ant.acs_offline_update()
                aco.ant_system_global_pheromone_update()

    def generate_random_obstacles(self):
        """ Generates Random Obstacles """
        amount_of_obstacles_added = 0
        while amount_of_obstacles_added < aco.obstacle_amount:
                obstacle_x = random.randint(0, aco.map_size - 1)
                obstacle_y = random.randint(0, aco.map_size - 1)
                if not aco.grid_map[obstacle_x, obstacle_y].hive and not aco.grid_map[obstacle_x, obstacle_y].goal:
                    aco.grid_map[obstacle_x, obstacle_y].obstacle_present = True
                    amount_of_obstacles_added += 1

    def display_best_path(self):
        for node in aco.global_best:
            if not node.hive and not node.goal:
                ax.scatter(self.grid_map[node.x, node.y].x, self.grid_map[node.x, node.y].y,
                           color='#e67e22', edgecolors="#000000")
class Node:
    """ Class That Describes Each Node """

    def __init__(self, x, y, pheromone_amount=0.1):
        self.x = x
        self.y = y
        self.pheromone_amount = pheromone_amount
        self.obstacle_present = False
        self.hive = False
        self.goal = False
        self.heuristic_value = 1

    def print_information(self):
        print("--- [" + str(self.x) + ", " + str(self.y) + "] ---")
        print("Node's Pheromone Strength Is: " + str(self.pheromone_amount))
        print("Obstacle Presence: " + str(self.obstacle_present))


class Ant:
    """ Class For The Ant Objects """

    def __init__(self, x_value, y_value, ):
        self.x = x_value
        self.y = y_value
        self.route_taken = [aco.grid_map[self.x, self.y]]
        self.current_node = aco.grid_map[self.x, self.y]
        self.possible_nodes = []
        self.node_choice = Node
        self.tour_finished = False
        self.found_solution = False

    def move_ant(self):
        """ Function To Deal With Moving The Ant """

        node_probability_total = 0.0
        self.get_possible_moves()

        # Code Adapted From Rochakgupta, 2018
        # Loops through all possible nodes that can be travelled to by the ant
        # node_probability_total stores the probability amount of each possible node selection and then
        # is used in conjunction with random_value to randomly decide which node to use
        if len(self.possible_nodes) > 0:
            for possible_node in self.possible_nodes:
                node_probability_total += pow(possible_node.pheromone_amount, aco.alpha) * \
                                          pow(possible_node.heuristic_value, aco.beta)
            random_value = random.uniform(0.0, node_probability_total)

            probability = 0.0
            for possible_node in self.possible_nodes:
                probability += pow(possible_node.pheromone_amount, aco.alpha) * \
                               pow(possible_node.heuristic_value, aco.beta)
                if probability >= random_value:
                    self.node_choice = possible_node
                    self.possible_nodes.clear()
                    break
                # End Of Adapted Code

            self.current_node = self.node_choice
            self.route_taken.append(self.current_node)
            self.x = self.current_node.x
            self.y = self.current_node.y
            if aco.mode == 1:
                if self.current_node.goal:
                    self.tour_finished = True
                    self.found_solution = True
                    print("Ant Has Reached Goal In: " + str(len(self.route_taken)))
            elif aco.mode == 2:
                if self.current_node.goal:
                    self.tour_finished = True
                    self.found_solution = True
                    print("Ant Has Reached Goal In: " + str(len(self.route_taken)))
            elif aco.mode == 3:
                if self.current_node.goal:
                    self.tour_finished = True
                    self.found_solution = True
                    print("Ant Has Reached Goal In: " + str(len(self.route_taken)))
        elif len(self.possible_nodes) <= 0:
            print("No Nodes Available")
            self.tour_finished = True
            self.found_solution = False

    def ant_system_pheromone_update(self):
        """ Adds Pheromone Based On Ant System Update Algorithm """
        # Local Update
        for node in self.route_taken:
            pheromone_to_add = (1 / len(self.route_taken))
            node.pheromone_amount += pheromone_to_add

    def elitist_aco_pheromone_update(self):
        """ Pheromone Update Using Elitist Ant Colony Optimisation """
        for node in self.route_taken:
            pheromone_to_add = (1 / len(self.route_taken))
            node.pheromone_amount += pheromone_to_add
        for node in aco.global_best:
            pheromone_to_add = (1 / len(aco.global_best))
            node.pheromone_amount += pheromone_to_add

    def acs_local_pheromone_update(self):
        """ Local Pheromone Update For ACS """
        pheromone_to_add = aco.evaporation_rate * aco.grid_map[self.x, self.y].pheromone_amount
        aco.grid_map[self.x, self.y].pheromone_amount = (1 / aco.evaporation_rate) * \
                                                        aco.grid_map[self.x, self.y].pheromone_amount + pheromone_to_add

    def acs_offline_update(self):
        """ Ant Colony System Offline Update """
        for node in aco.global_best:
            pheromone_to_add = (1 / len(aco.global_best))
            node.pheromone_amount = (1 - aco.evaporation_rate) * aco.grid_map[self.x, self.y].pheromone_level + \
                                    (aco.evaporation_rate * pheromone_to_add)

    def print_ant_information(self):
        """ Prints Out Ant Location On Map """
        print("Ant Is At: [" + str(self.x) + ", " + str(self.y) + "]")

    def get_possible_moves(self):
        """ Collects Possible Nodes Ant Can Move To """
        new_possible_nodes = []
        # Code Adapted From Justin Peel, 2018
        for x, y in [(self.x + i, self.y + j) for i in (-1, 0, 1) for j in (-1, 0, 1) if i != 0 or j != 0]:
            # End Of Adapted Code
            # Checks If The Move Is Outside Map Upper Boundaries
            if x >= aco.map_size or y >= aco.map_size:
                continue
            # Checks If The Node Is An Obstacle
            if not aco.grid_map[x, y].obstacle_present:
                # If The Node Is Also Not Outside Grid's Lower Boundary
                if x < 0 or y < 0:
                    continue
                # Adds The Node To A List Of Possible Nodes
                self.possible_nodes.append(aco.grid_map[x, y])
        for possible_node in self.possible_nodes:
            if possible_node not in self.route_taken:
                new_possible_nodes.append(possible_node)
        self.possible_nodes = new_possible_nodes
        # print("--- Possible Nodes For [" + str(self.x) + ", " + str(self.y) + "]")
        # for possible_node in self.possible_nodes:
        # if possible_node not in self.route_taken:
        # print("[" + str(possible_node.x) + ", " + str(possible_node.y) + "]")


fig, ax = plt.subplots()
ax.set_title('Ant Colony Optimisation')

user_map_size = int(input("Enter The Size Of The Map To Be Generated: "))
response = input("Do You Wish To Define Custom Start/Goal? (Y/N): ")
if response.lower() == 'y':
    user_start_x = int(input("Enter Start X Position: "))
    user_start_y = int(input("Enter Start Y Position: "))
    user_end_x = int(input("Enter End X Position: "))
    user_end_y = int(input("Enter End Y Position: "))
    user_generate_obstacles = int(input("How Many Obstacles Do You Want Generated: "))
    user_generate_dynamic_obstacles = str(input("Add Dynamic Obstacles? (Y/N): "))
    user_ant_colony_size = int(input("Amount Of Ants To Spawn: "))
    user_amount_of_iterations = int(input("Amount Of Iterations: "))

    aco = AntColonyOptimisation(map_size=user_map_size, start_point_x=user_start_x, start_point_y=user_start_y,
                                end_point_x=user_end_x, end_point_y=user_end_y, obstacle_amount=user_generate_obstacles,
                                colony_size=user_ant_colony_size, iteration_amount=user_amount_of_iterations,
                                dynamic_obstacles=user_generate_dynamic_obstacles)
    aco.create_grid_map(aco.map_size)
    print("\n --- Ant System ---\n")
    aco.mode = 1
    print(" --- Starting ---")
    aco.ant_system()
    aco.display_pheromone_map()
    aco.display_grid(aco.map_size)
    plt.savefig('Ant_system')
    print("Shortest Path: " + str(len(aco.best_path_so_far)))
    print("Time Taken: " + str(aco.ant_system_convergence_time))
    print("Iteration Taken To Reach: " + str(aco.as_average_iterations))
    aco = AntColonyOptimisation(map_size=user_map_size, start_point_x=user_start_x, start_point_y=user_start_y,
                                end_point_x=user_end_x, end_point_y=user_end_y, obstacle_amount=user_generate_obstacles,
                                colony_size=user_ant_colony_size, iteration_amount=user_amount_of_iterations,
                                dynamic_obstacles=user_generate_dynamic_obstacles)
    aco.create_grid_map(aco.map_size)
    print("\n --- Elite Ant System --- \n")
    aco.mode = 2
    print(" --- Starting ---")
    aco.elitist_aco()
    aco.display_pheromone_map()
    aco.display_grid(aco.map_size)
    aco.display_best_path()
    plt.savefig("Elitist_ant_system")
    print("Shortest Path: " + str(len(aco.global_best)))
    print("Time Taken: " + str(aco.elitist_convergence_time))
    print("Iteration Taken To Reach: " + str(aco.eas_average_iterations))
if response.lower() == 'n':
    user_generate_obstacles = int(input("How Many Obstacles Do You Want Generated: "))
    user_generate_dynamic_obstacles = str(input("Add Dynamic Obstacles? (Y/N): "))
    user_ant_colony_size = int(input("Amount Of Ants To Spawn: "))
    user_amount_of_iterations = int(input("Amount Of Iterations: "))
    aco = AntColonyOptimisation(map_size=user_map_size, start_point_x=0, start_point_y=0, end_point_x=user_map_size - 1,
                                end_point_y=user_map_size-1, obstacle_amount=user_generate_obstacles,
                                colony_size=user_ant_colony_size, iteration_amount=user_amount_of_iterations,
                                dynamic_obstacles=user_generate_dynamic_obstacles)
    aco.create_grid_map(aco.map_size)
    print("\n --- Ant System ---\n")
    aco.mode = 1
    print(" --- Starting ---")
    aco.ant_system()
    aco.display_pheromone_map()
    aco.display_grid(aco.map_size)
    plt.savefig('Ant_system')
    print("Shortest Path: " + str(len(aco.best_path_so_far)))
    print("Time Taken: " + str(aco.ant_system_convergence_time))
    print("Iteration Taken To Reach: " + str(aco.as_average_iterations))
    #for length in aco.as_length_list:
    #    print(aco.as_length_list)

    aco = AntColonyOptimisation(map_size=user_map_size, start_point_x=0, start_point_y=0, end_point_x=user_map_size - 1,
                                end_point_y=user_map_size-1, obstacle_amount=user_generate_obstacles,
                                colony_size=user_ant_colony_size, iteration_amount=user_amount_of_iterations,
                                dynamic_obstacles=user_generate_dynamic_obstacles)
    aco.create_grid_map(aco.map_size)
    print("\n --- Elite Ant System --- \n")
    aco.mode = 2
    print(" --- Starting ---")
    aco.elitist_aco()
    aco.display_pheromone_map()
    aco.display_grid(aco.map_size)
    aco.display_best_path()
    plt.savefig("Elitist_ant_system")
    print("Shortest Path: " + str(len(aco.global_best)))
    print("Time Taken: " + str(aco.elitist_convergence_time))
    print("Iteration Taken To Reach: " + str(aco.eas_average_iterations))
    #for length in aco.eas_length_list:
     #   print(aco.eas_length_list)


#filename = 'ant_system_details'
#with open(filename, 'w') as file_object
 #   file_object.write()


#print("\n --- Ant Colony System ---")

# aco = AntColonyOptimisation()
# aco.create_grid_map(aco.map_size)
# aco.obstacle_add_remove(5, 6)
# aco.obstacle_add_remove(3, 4)
# aco.obstacle_add_remove(3, 5)
# aco.obstacle_add_remove(3, 6)
# aco.obstacle_add_remove(4, 3)
# aco.obstacle_add_remove(5, 3)
# aco.obstacle_add_remove(4, 6)
# aco.obstacle_add_remove(5, 5)
# aco.obstacle_add_remove(5, 4)
# aco.obstacle_add_remove(3, 3)
# aco.mode = 3

# print(" --- Starting ---")
# aco.ant_colony_system()
# aco.display_pheromone_map()
# aco.display_grid(aco.map_size)
# plt.savefig("Ant Colony System")

plt.show()
