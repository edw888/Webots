from controller import Robot
import math, time,random
import numpy as np
# Peepos
# PSO Constants
timestep = 64 
max_iterations = 300
layers = 2
delaysec = 1
delay = int(delaysec * 600/timestep)

fit_obj = 0.2
size = 5  #total size of the map
dist = size/2
num_robots = 4
num_points = 4

points = [[0,2],[2,0],[0,-2],[-2,0]]
frequency = 1
Q = 0.2

resolution = 0.25
map_range = int(size/resolution) #total number of sections in the map
section_range = map_range//2
sections = [] 

visited_positions = []

# ACO constants
rad = 4
phe_rad = int(rad * 0.1/resolution) # Radius of pheromone detection
max_phe = 2 # Maximum value of pheromones in one section
pheromone_grid = np.zeros((map_range, map_range))
 
evaporation_rate = 0.05

# Robot specs
radius = 0.02
dist_wheels = 0.052
max_speed = 6.275 * radius 

size = 5
num_robots = 4
dist = size/2
#frequency = 6

# Local pso constants
neighbor_size = 2
num_clusters = num_robots // neighbor_size
indices = list(range(num_robots))  

crowding = False
random_indices = False


# obj = [[-0.5, 0], [0.5, 0]]
obj =  [0,1]  # Contains the current objective of the robot, and the loop it is on


# Init of variables
reached_objective = [False] * num_robots

velocity = [0.0] * layers
desired_vel = [0.0] * layers

cat = 0
sat = 0
pat = 0
eat = 0
iat = 0
vat = 0



class Neuron:
    def __init__(self):
        self.pos = [0.0] * layers
        self.vel = [0.0] * layers
        self.pPos = [[0.0] * layers for _ in range(len(points))]  # Personal best position
        self.nPos = [[0.0] * layers for _ in range(len(points))]     
        self.pFit =  [float('inf') for _ in range(len(points))] 
        self.nFit =  [float('inf') for _ in range(len(points))]
        self.nBestPosList = []     # Best positions of neighbors 

 
# Loop to generate sections
for i in range(-section_range, section_range):
    for j in range(-section_range, section_range):
        # Calculate the coordinates of the section
        x = i*resolution
        y = j * resolution
        section = (x, y, x + resolution, y + resolution)
        sections.append(section)

def send_data():
    data = ["hybrid", num_robots, max_iterations, size, resolution, evaporation_rate, max_phe, frequency]
    data_str = ','.join(str(i) for i in data)    
    # Set the receiver channel for the supervisor
    emi.setChannel(99)
    
    # Send the pheromone grid packet to the supervisor
    emi.send(data_str.encode('utf-8')) 


def exploration(neuron):
   # Add random exploration to the velocity with a fixed magnitude
    Ke = 0.1   
    exp_vel = [0.0] * layers
    
    for i in range(layers): 
        exp_vel[i] = random.uniform(-1, 1) * Ke 
        neuron.vel[i] += exp_vel[i]


# Calculate section of neuron for pheromone purposes
def calculate_section(position):
    x, y = position
    section_x = int(x / resolution)
    section_y = int(y / resolution)
    return (section_x, section_y)


def calculate_fitness(position):
    points_fitness = []
    for i in range(len(points)):
        fit = math.sqrt((position[0] - points[i][0]) ** 2 + (position[1]- points[i][1]) ** 2)
        points_fitness.append(fit)
   
    return points_fitness  


def update_pheromones(position, visited, new_phe, obj):
    fit_factor = math.sqrt((position[0] -  points[obj][0]) ** 2 + (position[1]- points[obj][1]) ** 2)/dist
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

 

#  Proportional to iterations
def prop(base):
   return base * (iteration/max_iterations)#**2

#  Inv. proportional to iterations
def inprop(base):
   return base * (1 - (iteration/max_iterations))#**2

def update_velocity(neuron, velocity):
    global cat, sat, pat, eat, sat, iat, vat
    w_ine = 1 # Inertia weight to balance old velocity
    w_cog = 0.5 # Weight for personal best
    w_soc = 0.5 # Weight for global best
    w_phe = 0.5 #prop(1)
    w_exp = inprop(1) 
    soc_vel = [0.0] * layers
    cog_vel = [0.0] * layers
    phe_vel = [0.0] * layers
    exp_vel = [0.0] * layers
    ine_vel = velocity.copy()
    vel_inc = 0.1
    
    direction = math.atan2(neuron.vel[1], neuron.vel[0])
    
    phe_vel = nearby_pheromones(pheromone_grid, neuron.pos, direction) #* w_phe * r4
    print("phe_vel", phe_vel)

    r1 = random.random()
    r2 = random.random()
    r3 = random.random()
    r4 = random.random()

    for i in range(layers):
        r5 = random.uniform(-1, 1) # So the exp component can be more varied
        ine_vel[i] = r1 * w_ine * ine_vel[i]
        # Social element based on global best
       
        soc_vel[i] = r2 * max(-max_speed, min(max_speed, w_soc * (neuron.nPos[obj[0]][i] - neuron.pos[i])/dist))
        # Cognitive element based on personal best
        cog_vel[i] = r3 * max(-max_speed, min(max_speed, w_cog * (neuron.pPos[obj[0]][i] - neuron.pos[i])/dist))
        phe_vel[i] = r4 * max(-max_speed, min(max_speed, w_phe * phe_vel[i]))
        exp_vel[i] = r5 * w_exp * max_speed
        desired_vel[i] = ine_vel[i] + cog_vel[i] + soc_vel[i] + exp_vel[i] + phe_vel[i]
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

   # for i in range(layers):
        #neuron.pos[i] = neuron.pos[i] +  gps_value[i]
    return calculate_section(neuron.pos)
           


def init_pso(neuron):

    # Initialize the position and velocity randomly
    # swarm[j].pos = [random.uniform(-size / 2, size / 2) for _ in range(layers)]
    # swarm[j].vel = [random.uniform(-max_speed, max_speed) for _ in range(layers)]
    for i in range(len(neuron.pPos)):
        neuron.pPos[i] = neuron.pos.copy() # Update personal best

# Function to calculate crowding distances
def crowding_distance(swarm):
    # Init of crowding distances list
    crowding_dist = [0] * num_robots
    # Sort neurons based on their fitness
    fit_sorted = sorted(range(num_robots), key=lambda i: (i != j, swarm[i].pFit))
    # Set the limits in the sorted swarm to infinite
    crowding_dist[fit_sorted[0]] = float('inf')
    crowding_dist[fit_sorted[-1]] = float('inf')

    fit_min = swarm[fit_sorted[0]].pFit
    fit_max = swarm[fit_sorted[-1]].pFit
    # Calculate different in fitness between neighbors
    for i in range(1, num_robots - 1):
        crowding_dist[fit_sorted[i]] += (swarm[fit_sorted[i + 1]].pFit - swarm[fit_sorted[i - 1]].pFit) / (fit_max - fit_min)
                                                       
    return crowding_dist


def best_positions(neuron, r_ind, fitness):
    # Neighborhood type
    if crowding:
        crowding_dist = crowding_distance(swarm, r_ind)
        neighborhood = sorted(range(num_robots), key=lambda x: crowding_dist[x])[:neighbor_size]   

    else:
        # Will create a list of J and all it's neighbors
        # If odd sized neighborhood, the odd one will be added to the right index
        neighborhood = [index for index in indices if index != j][-neighbor_size // 2:] + [index for index in indices if index != j][:neighbor_size // 2]         
    
    for i in range(len(points)):
        if fitness[i] < neuron.pFit[i]:
            neuron.pFit[i] = fitness[i]
            neuron.pPos[i] = neuron.pos.copy()
        # print("Personal best point {} : {:.2f},{:.2f}".format(i, neuron.pPos[i][0],neuron.pPos[i][1]))
        
        # Update the neighborhood best position and fitness, based on method
        if fitness[i] < neuron.nFit[i]:
            # Set neighborhood best fitness and position for j and it's neighbors
            neuron.nFit[i] = fitness[i]
            neuron.nPos[i] = neuron.pos.copy()
            send_position(robot, emi, neuron, neighborhood)
            print("Updating neighborhood best")
        # print("Neighborhood best point {} : {:.2f},{:.2f}".format(i,neuron.nPos[i][0],neuron.nPos[i][1]))

# Function to perform the main PSO loop
def perform_pso(neuron, gps, r_ind, rec):
    global visited_positions, reached_objective, velocity, swarm, pheromone_grid 
    new_phe = np.zeros((map_range, map_range))
    # Randomize the indices each iteration
    if random_indices:
        random.shuffle(indices)
        print("Indices", indices)
 
    # Functions to update position and velocity of neuron 
    section = update_position(neuron, gps)
    visited_positions.append(section)
    #update_position(neuron, gps)
 
    if (iteration % frequency) == 0:
   
        update_velocity(neuron, velocity)
        # print(iteration % frequency, iteration)
    new_phe = update_pheromones(neuron.pos, visited_positions, new_phe, obj[0])
    send_pheromones(new_phe, emi)
    total_pheromones = sum(sum(row) for row in pheromone_grid)
    print("Total pheromones:", total_pheromones)
    
    # Calculation of fitness
    # fitness = calculate_fitness(neuron.pos)
    # Calculation of all points fitnesses 
    fitnesses = calculate_fitness(neuron.pos)
    # Current objective fitness 
    fitness = fitnesses[obj[0]]
    best_positions(neuron, r_ind, fitnesses)
    
    # Check if reached objective
    if fitness <= fit_obj:
            print("Robot {} reached the objective {:.2f},{:.2f}".format(r_ind, points[obj[0]][0], points[obj[0]][1]))
            
            neuron.pFit[obj[0]] = float('inf')  
            obj[0] = (obj[0] + 1) % len(points)
            if obj[0]==0: obj[1] += 1
            
            visited_positions = []
        
    # Exploration component
    if fitness <= (neuron.nFit[obj[0]]+0.1) or fitness <= (neuron.pFit[obj[0]]+0.1):
        exploration(neuron)
        print("Exploration mode")
 
    
 
    print("Position / Velocity  Neuron {} : [{:.3f},{:.3f}] / [{:.3f},{:.3f}] ".format(r_ind, neuron.pos[0], neuron.pos[1], neuron.vel[0], neuron.vel[1]))
    print("Fitness / Neighborhood Fitness: {:.2f}  / {:.2f} ".format(fitness, neuron.nFit[obj[0]]))
    print("Personal Best / Neighborhood Best [{:.2f},{:.2f}] / [{:.2f}, {:.2f}]".format(neuron.pPos[obj[0]][0], neuron.pPos[obj[0]][1], neuron.nPos[obj[0]][0], neuron.nPos[obj[0]][1]))
    # Update neighborhood best
    



    return reached_objective
# Function sends nPos to neighborhood 
def send_position(robot, emi, neuron, neighborhood):
    for i in range(len(neighborhood)):
        # Put the position in a string to be able to send
        pos_str = ','.join(','.join(str(coord) for coord in pos) for pos in neuron.nPos)
        fit_str = ','.join(str(fit) for fit in neuron.nFit) + ',0'
        # Set the receiver channel for the supervisor
        emi_str = f"{pos_str};{fit_str}"
        #print("alo", emi_str)
        emi.setChannel(neighborhood[i])
        # Send the position packet to the supervisor
        emi.send(emi_str.encode('utf-8'))


def receive_position(rec_str, neuron):
   
    # position = [float(part) for part in rec.getString().split(',')]
    # Split the received data string into position and fitness parts
    #pos_str, fit_str = rec_str.split(';')
 
    rec_str = rec_str[:-2]
    pos_str, fit_str = rec_str.split(';')
 
   #print("papo", pos_str)
    #print("bosso", fit_str)
    # Parse position string into a list of tuples
    positions = [tuple(map(float, pos.split(','))) for pos in pos_str.split(';')]
    
    # Parse fitness string into a list of floats
    fitnesses = [float(fit) for fit in fit_str.split(',')]
    for i, (fit, fitness, pos) in enumerate(zip(neuron.nFit, fitnesses, positions)):
        if fit > fitness:
            neuron.nFit[i] = fitness
            neuron.nPos[i] = pos
      

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



   


def avoid_obstacles(ps_values, left_vel, right_vel):
    
    # Check for front obstacles
    if ps_values[0] and ps_values[7] > 100:
        if ps_values[0]> ps_values[7]:
            left_vel = -0.25 * max_speed  # Turn left 
            right_vel = 0.25 * max_speed
        else:
            left_vel = -0.25 * max_speed  # Turn right 
            right_vel = 0.25 * max_speed
        return left_vel, right_vel          
    # Check for other obstacles
    for i in range(len(ps_values)):      
        if i in [0, 1]:  # Check if the distance sensor index is 0, 1, or 2
            if ps_values[i] > 100:  # Check if the distance sensor is detecting an obstacle (threshold of 0.1)
                left_vel = -0.25 * max_speed  # Turn left (negative angular velocity)
                right_vel = 0.25 * max_speed
                break
        elif i in [6, 7]:  # Check if the distance sensor index is 5, 6, or 7
            if ps_values[i] > 100:  # Check if the distance sensor is detecting an obstacle (threshold of 0.1)
                left_vel = 0.25 * max_speed
                right_vel = -0.25 * max_speed  # Turn right (negative angular velocity)
                break
    return left_vel, right_vel  



def run_robot(robot):
    global reached_objective, velocity, robot_name, iteration, pheromone_grid
     # Get the robot's name
    
    # Create the robot swarm
    # swarm = [Neuron() for _ in range(num_robots)]
    prev_position = [0.0, 0.0]  # Previous position of the robot
    prev_time = 0.0  # Previous timestamp

    # Receiver
    rec = robot.getDevice('receiver')
    rec.enable(timestep)
    #Emitter 
    # emi = robot.getDevice('emitter')
    #Set the receiver channel based on the name of each robot (r0,r1)

    rec.setChannel(int(robot_name[1])) #Channel will be either 0,1,2 based on robot

   
    # Created motor instances
   
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
  
    
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
 
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)

    
    
    # GPS
  
    gps = robot.getDevice('gps')
    gps.enable(timestep)
    
    # Create distance sensor instances
    dist_sensors = []
    for i in range(8):
        sensor = robot.getDevice('ps' + str(i))
        sensor.enable(timestep)
        dist_sensors.append(sensor)
    

    
    
    # Initialize the swarm of neurons and perform PSO
    neuron = Neuron()
    init_pso(neuron)
 
    
    
    while robot.step(timestep) != -1:
      
        for iteration in range(max_iterations):

            print("-----------------------\nIteration start\n-----------------------") 
            print("Robot: ", robot_name) 
            while rec.getQueueLength() > 0:
                rec_str = rec.getString()
             
                if rec_str[-1] == '0':
                    receive_position(rec_str, neuron)
                    
                elif rec_str[-1] == '1':
                    pheromone_grid = receive_pheromones(rec_str)
                   # print("alabama", rec_str)
                rec.nextPacket()
                     
                
             # Read values from distance 
            ps_values = [sensor.getValue() for sensor in dist_sensors]
            for i in range(8):
                ps_values[i] = dist_sensors[i].getValue()
    
 
           
            perform_pso(neuron, gps, r_ind, rec)
           
            # pheromone_grid *= (1-evaporation_rate*max_iterations/(max_iterations+iteration*3))
            
         
            Kp = 1  # Angular scaling factor
            Kv = 1 # Linear scaling factor
            
            reset_prob = 0.1  # Chance  w_speed resets

            max_random_component = 0.1  # Experiment with different values
            
            v_speed = math.sqrt(neuron.vel[0]**2 + neuron.vel[1]**2)*Kv
            if random.random() <= reset_prob:
                w_speed = 0.0
            else:
                random_component = random.uniform(-1, 1) * max_random_component

                w_speed = math.atan2(neuron.vel[1], neuron.vel[0])*Kp

                w_speed += random_component


        
            
            print("V_speed {:.2f} W_speed {:.2f}".format(v_speed, w_speed))
             
    
        
            left_vel = v_speed - (w_speed * dist_wheels) / 2 
            right_vel = v_speed + (w_speed * dist_wheels) / 2
      
            # Obstacle avoidance check    
            left_vel, right_vel = avoid_obstacles(ps_values, left_vel, right_vel)
            #print ("Velocities: Left  {:.2f} Right {:.2f}".format(left_vel, right_vel))
            # Scales the velocity of the wheels to be proportional to eachother if any is over max_speed
            max_wheel_velocity = max(abs(left_vel), abs(right_vel))
            if max_wheel_velocity > max_speed:
                scale = max_speed / max_wheel_velocity
                left_vel *= scale
                right_vel *= scale

            #print ("Velocities: Left  {:.2f} Right {:.2f}".format(left_vel, right_vel))
    

            # Set velocities

            left_motor.setVelocity(left_vel/radius)
            right_motor.setVelocity(right_vel/radius)
             
            
            # Calculate velocity
            time = robot.getTime()
            dtime = time - prev_time
            position = [gps.getValues()[0], gps.getValues()[1]]
            dpos = [position[0] - prev_position[0], position[1] - prev_position[1]] 
            if iteration == 0:
                dpos = [0, 0] 
            velocity = [dpos[0]/dtime, dpos[1]/dtime]
            #print("prevpos[0] : {:.2f} prevpos[1] : {:.2f} ".format(prev_position[0], prev_position[1]))
            #print("pos[0] : {:.2f} pos[1] : {:.2f} ".format(neuron.pos[0], neuron.pos[1]))
            #print("posi : {:.2f} posi : {:.2f} ".format(position[0], position[1]))

            #print("time : {:.2f}  dpos[0] : {:.2f} dpos[1] : {:.2f} ".format(time, dpos[0], dpos[1]))
            #print("True velocity", velocity)
            # print("Pos swarm: {:.2f}, {:.2f}".format(neuron.pos[0], neuron.pos[1]))
               
            prev_position = position
            prev_time = time
              
            
        
            
            # Not used for now as we have Gbest
            # avg_pos = [sum(neuron.pos[i] for neuron in swarm) / num_robots for i in range(layers)] 
            #for i in range(num_robots):
        
            # Update robot pose based on wheel encoders
            
            
   
                
                
                
                 
    
             # Check if reached origin
          #  if reached_objective == True:
            #    left_motor.setVelocity(0)
            #    right_motor.setVelocity(0)
            #    print("Robot {} reached the origin!".format(r_ind))
             #   break
         
            print("Iteration: {} ".format(iteration))
            
            for _ in range(delay):
                robot.step(timestep)
        break
     

   
if __name__== "__main__":

    # create the Robot instance.
    
    robot = Robot()
    robot_name = robot.getName()
    r_ind = int(robot_name[1]) 
    emi = robot.getDevice('emitter')
    if r_ind == 0:
        send_data()
    for _ in range(delay):
        robot.step(timestep)
    run_robot(robot)
 