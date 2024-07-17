from controller import Robot
import math, time
import random as rnd
import numpy as np
import fnc
from fnc import calculate_velocity, prop

# Robot specs
radius = 0.02
dist_wheels = 0.052
max_speed = 6.275 * radius 

# PSO Constants
timestep = 64 
max_iterations = 1000
dim = 2
delaysec = 1
delay = int(delaysec * 1000/timestep)
frequency = 1
vel_inc = max_speed / 8


# ACO constants
resolution = 0.25
Q = 0.2
rad = 4
phe_rad = int(rad * 0.1/resolution) # Radius of pheromone detection
max_phe = 2 # Maximum value of pheromones in one section

 
evaporation_rate = 0.05

# Robot specs
radius = 0.02
dist_wheels = 0.052
max_speed = 6.275 * radius 

# Local pso constants
num_robots = 4
neighbor_size = 2
num_clusters = num_robots // neighbor_size
indices = list(range(num_robots))  
fit_obj = 0.2
n_size = num_robots//3

# Init of variables
reached_obj = [False] 
iteration = 0
vel = [0.0] * dim
desired_vel = [0.0] * dim
shuffled = False
visited_positions = []
sections = []

cat = 0
sat = 0
pat = 0
eat = 0
iat = 0
vat = 0

# Configuration
ncfg = {"random": False}
shuffle_freq = 5
def localHybrid(con, rob):
    robot = rob.robot

    map_range = int(con.size/resolution)  
    section_range = map_range//2
    pheromone_grid = np.zeros((map_range, map_range))

    class Neuron:
        def __init__(self):
            self.pos = [0.0] * dim
            self.vel = [0.0] * dim
            self.pPos = [[0.0] * dim for _ in range(len(con.obj))]  # Personal best position
            self.sPos = [[0.0] * dim for _ in range(len(con.obj))]     
            self.pFit =  [float('inf') for _ in range(len(con.obj))] 
            self.sFit =  [float('inf') for _ in range(len(con.obj))]
            self.obj = 0
            self.neigh = []
  
    # Loop to generate sections
    for i in range(-section_range, section_range):
        for j in range(-section_range, section_range):
            # Calculate the coordinates of the section
            x = i*resolution
            y = j * resolution
            section = (x, y, x + resolution, y + resolution)
            sections.append(section)

    # Calculate section of neuron for pheromone purposes
    def calculate_section(position):
        x, y = position
        section_x = int(x / resolution)
        section_y = int(y / resolution)
        return (section_x, section_y)


    def calculate_fitness(position):
        points_fitness = []
        for i in range(len(con.obj)):
            fit = math.sqrt((position[0] - con.obj[i][0]) ** 2 + (position[1]- con.obj[i][1]) ** 2)
            points_fitness.append(fit)
        return points_fitness  


    def update_pheromones(position, visited, new_phe, obj):
        fit_factor = math.sqrt((position[0] -  con.obj[obj][0]) ** 2 + (position[1]- con.obj[obj][1]) ** 2)/(con.size/2)
        x, y = visited[-1] # Retrieve the last visited position
        # Set the pheromones in the grid, from the center of the list (section range).
        new_phe[x + section_range, y + section_range] = Q/fit_factor
        # pheromone_grid[x + section_range, y + section_range] = min(pheromone_grid[x + section_range, y + section_range] + Q/fit_factor, max_phe)
        
        # send_pheromones(pheromone_grid)
        return new_phe



    def nearby_pheromones(pheromone_grid, position, direction):
        section_x, section_y = calculate_section(position)
        phe_mod = [0,0]
        # Pheromone radius is created based on section map
        while(1):    
            x0 = max(section_x - phe_rad + phe_mod[0], -section_range)
            x1 = min(section_x + phe_rad - phe_mod[0] + 1, section_range)
            y0 = max(section_y - phe_rad + phe_mod[1], -section_range)
            y1 = min(section_y + phe_rad - phe_mod[1] + 1, section_range)

            # If pheromone radius reaches out of the map, moves the radius in the opposite direction to avoid changing radius size and incentive avoiding the edge of the map
            if x1-x0 < 2 * phe_rad+1:
                if x0 == -section_range:
                    phe_mod[0] += 1
                elif x1 == section_range:
                    phe_mod[0] -= 1
            elif y1-y0 < 2 * phe_rad+1:
                if y0 == -section_range:
                    phe_mod[1] += 1
                elif y1 == section_range:
                    phe_mod[1] -= 1

            else:
                break
        
        # Extract the relevant portion of the pheromone grid
        nearby_phe = pheromone_grid[x0 + section_range: x1 + section_range, y0 + section_range: y1 + section_range] 

        # Generate meshgrid for distance and angle calculations
        x_coords, y_coords = np.meshgrid(np.arange(x0, x1), np.arange(y0, y1))
        
        # Compute squared distances instead of taking square roots
        distances_sq = (x_coords - section_x) ** 2 + (y_coords - section_y) ** 2
        
        # Compute angle differences using trigonometric identities
        angles = np.arctan2(y_coords - section_y, x_coords - section_x)
        angle_diffs = np.pi - np.abs((direction - angles + np.pi) % (2 * np.pi) - np.pi)
        # x_coords = x_coords.T
        
        # Create a mask to exclude the section where the robot is located
    
        mask =  (np.abs(x_coords - section_x) > 0) & (np.abs(y_coords - section_y) > 0)  
        # Apply the mask to the neighborhood
    
        neighborhood_masked = nearby_phe * mask
        
        # Compute weights using vectorized operations
        weights = neighborhood_masked / (np.sqrt(distances_sq)+ 1) * (1 - 0.4 * angle_diffs) / phe_rad ** 2
        
        # Compute weighted sum to get x and y components of pheromone
        x_phe = np.sum(weights * (x_coords - section_x))
        y_phe = np.sum(weights * (y_coords - section_y))
        return [x_phe, y_phe]

    def update_velocity(neuron, pPos, sPos):
        global cat, sat, pat, eat, sat, iat, vat
        w_ine = 1 # Inertia weight to balance old velocity
        w_cog = 0.5 # Weight for personal best
        w_soc = 0.5 # Weight for global best
        w_phe = 0.5 #prop(1)
        w_exp = prop(1, iteration, con.max_iterations, i=True) 

        soc_vel = [0.0] * dim
        cog_vel = [0.0] * dim
        phe_vel = [0.0] * dim
        exp_vel = [0.0] * dim
        ine_vel = [0-0] * dim
        direction = math.atan2(neuron.vel[1], neuron.vel[0])
        
        phe_vel = nearby_pheromones(pheromone_grid, neuron.pos, direction) #* w_phe * r4

        r1 = rnd.random()
        r2 = rnd.random()
        r3 = rnd.random()
        r3 = rnd.random()

        for i in range(dim):
            r4 = rnd.uniform(-1, 1) # So the exp component can be more varied
            ine_vel[i] = vel[i] * w_ine + cog_vel[i]
            soc_vel[i] = r1 * w_soc * (sPos[i]- neuron.pos[i])/(con.size/2)
            cog_vel[i] = r2 * w_cog * (pPos[i] - neuron.pos[i])/(con.size/2)
            phe_vel[i] = r3 * w_phe * phe_vel[i]
            exp_vel[i] = r4 * w_exp * max_speed
            desired_vel[i] = vel[i] * w_ine + cog_vel[i] + soc_vel[i] + exp_vel[i] + phe_vel[i]
            if desired_vel[i] > neuron.vel[i]:
                neuron.vel[i] = min(neuron.vel[i] + vel_inc, desired_vel[i])
            else:
                neuron.vel[i] = max(neuron.vel[i] - vel_inc, desired_vel[i])
        
        cat += math.sqrt(cog_vel[0]**2 + cog_vel[1]**2)
        sat += math.sqrt(soc_vel[0]**2 + soc_vel[1]**2)
        pat += math.sqrt(phe_vel[0]**2 + phe_vel[1]**2)
        eat += math.sqrt(exp_vel[0]**2 + exp_vel[1]**2)
        iat += math.sqrt(ine_vel[0]**2 + ine_vel[1]**2)
        vat += math.sqrt(neuron.vel[0]**2 + neuron.vel[1]**2)
        
        ca = cat/(iteration//frequency+1)
        sa = sat/(iteration//frequency+1)
        pa = pat/(iteration//frequency+1)
        ea = eat/(iteration//frequency+1)
        ia = iat/(iteration//frequency+1)
        ia = iat/(iteration//frequency+1)
        va = vat/(iteration//frequency+1)
        print("Averages: ca {:.3f}%, sa {:.3f}%, pa {:.3f}%, ia {:.3f}%, ea {:.3f}%".format(ca*100, sa*100, pa*100, ia*100, ea*100))
        print("Average velocity: {:.3f}%".format(va*100))
        print("CogVel: [{:.3f}, {:.3f}] /// SocVel: [{:.3f}, {:.3f}] ".format(cog_vel[0], cog_vel[1], soc_vel[0], soc_vel[1]))
        print("PheVel: [{:.3f}, {:.3f}] /// IneVel: [{:.3f}, {:.3f}]  ".format(phe_vel[0], phe_vel[1], ine_vel[0], ine_vel[1]))
        print("Neuron Velocity: [{:.3f}, {:.3f}]\n".format(neuron.vel[0], neuron.vel[1]))
    
    def update_position(neuron, gps):
        gps_value = gps.getValues()
        neuron.pos[0] = gps_value[0]
        neuron.pos[1] = gps_value[1]  

    # Initializes the algorithm
    def init_pso(neuron):
        for i in range(dim): 
            neuron.vel[i] = rnd.uniform(-max_speed, max_speed)

        for i in range(len(con.obj)):
            neuron.pPos[i]  = neuron.pos.copy()
            neuron.sPos[i]  = neuron.pos.copy()
        neuron.neigh  = [indices[(rob.ind + i) % len(indices)] for i in range(-n_size // 2, n_size // 2 + 1) if i != 0]
   
    # Function to perform the main PSO loop
    def perform_pso(neuron, gps, rob_index):
        global visited_positions, reached_obj, vel, shuffled, pheromone_grid 
        new_phe = np.zeros((map_range, map_range))
        # Randomize the indices each iteration
        if ncfg["random"]:
            if iteration % shuffle_freq == 0:
                rnd.shuffle(indices)
                shuffled = True
    
        # Functions to update position and velocity of neuron 
        if (iteration % frequency) == 0:
            update_velocity(neuron, neuron.pPos[neuron.obj], neuron.sPos[neuron.obj]) 
        section = update_position(neuron, gps)
        visited_positions.append(section)

        new_phe = update_pheromones(neuron.pos, visited_positions, new_phe, neuron.obj)
        send_pheromones(new_phe, rob.emi)

        fitnesses = calculate_fitness(neuron.pos)
        # Current objective fitness 
        fitness = fitnesses[neuron.obj]
        
        if shuffled:
            fnc.update_neigh(ncfg, rob, neuron, indices, n_size)

        for i in range(len(con.obj)):
                best = (neuron.sFit[i], neuron.pFit[i], neuron.sPos[i], neuron.pPos[i]) 
                n_best = fnc.best_position(neuron.pos, best, fitnesses[i])
                (neuron.sFit[i], neuron.pFit[i], neuron.sPos[i], neuron.pPos[i]) = n_best
                if n_best != best:
                    data = fnc.create_str(neuron.sPos, neuron.sFit)
                    fnc.lsend_position(rob.emi, neuron, data)
                    print("Send position", rob.name)

        # Check if reached objective
        if fitness <= fit_obj:
                print("Robot {} reached the objective {:.2f},{:.2f}".format(rob_index, con.obj[neuron.obj][0], con.obj[neuron.obj][1]))
                
                neuron.pFit[neuron.obj] = float('inf')  
                neuron.obj = (neuron.obj + 1) % len(con.obj)
                
                visited_positions = []

        return reached_obj

    def send_pheromones(pheromone_grid, emi):
    
        # Convert the pheromone grid to a string
        pheromone_str = ','.join(str(i) for j in pheromone_grid for i in j)
        
        # Set the receiver channel for the supervisor
        emi.setChannel(99)
        
        # Send the pheromone grid packet to the supervisor
        emi.send(pheromone_str.encode('utf-8'))

    
    def receive_pheromones(rec_str):
        # Parse the received string into a 1D array of float values
        pheromone_values = [float(value) for value in rec_str.split(',')[:-1]]
        
        # Convert the 1D array into a 2D numpy array representing the pheromone grid
        pheromone_grid = np.array(pheromone_values).reshape((map_range, map_range))
        total_pheromones = sum(sum(row) for row in pheromone_grid)
        # print("Total pheromones received:", robot_name, total_pheromones)
        return pheromone_grid 



    def run_robot():
        global reached_obj, vel
        
        motors = fnc.motor_setup(robot)
        gps, dist_sensors = fnc.set_sensors(robot, "local")
        
        # Initialize the swarm of neurons and perform PSO
        neuron = Neuron()
        init_pso(neuron)
    

        
        # Initialize the swarm of neurons and perform PSO
        neuron = Neuron()
        init_pso(neuron)
    
        
        for iteration in range(con.max_iterations):
            print("---")
            for _ in range(delay):
                robot.step(timestep)
            
            # Check if reached origin
            if reached_obj == True:
                motors[0].setVelocity(0)
                motors[1].setVelocity(0)
                print("Robot {} reached the origin!".format(rob.ind))
                break

            # Receiving best positions from supervisor
            if rob.rec.getQueueLength() > 0:
                data = fnc.receive_position(rob.rec)
                neuron.sFit, neuron.sPos = fnc.best_rec(data, neuron.sFit, neuron.sPoss)
        
            perform_pso(neuron, gps)
        
            fnc.diff_speed(neuron, motors, dist_sensors) 
            # Calculate vectorial velocity
            vel, pos = calculate_velocity(robot, gps)  

            if isinstance(neuron.sPos[0], list):
                print("Iteration:", iteration, rob.name)
                for i in (range(len(neuron.sPos))):
                    print("Obj {}. sFit: {:.3f} sPos [{:.2f},{:.2f}]".format(i, neuron.sFit[i], neuron.sPos[i][0], neuron.sPos[i][1]))
            else:
                print("Iteration: {}.  {} sFit: {:.3f}  sPos [{:.2f},{:.2f}]".format(iteration, rob.name, neuron.sFit, neuron.sPos[0], neuron.sPos[1]))
            
run_robot()