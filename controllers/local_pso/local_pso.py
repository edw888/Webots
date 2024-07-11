from controller import Robot
import math, time
import random

# PSO Constants
timestep = 64 
max_iterations = 1000
layers = 2
delaysec = 1
delay = int(delaysec * 1000/timestep)

# Robot specs
radius = 0.02
dist_wheels = 0.052
max_speed = 6.275 * radius 

size = 5
num_robots = 4

frequency = 6

# Local pso constants
neighbor_size = 2
num_clusters = num_robots // neighbor_size
indices = list(range(num_robots))  

obj = [[-0.5, 0], [0.5, 0]]


# Init of variables
reached_objective = [False] * num_robots

velocity = [0.0] * layers
desired_vel = [0.0] * layers


# Exclusive methods for neighborhoods
crowding = False
random_indices = False

class Neuron:
    def __init__(self):
        self.pos = [0.0] * layers
        self.vel = [0.0] * layers
        self.pPos = [0.0] * layers  # Personal best position
        self.nPos = [0.0] * layers  # Local best position
        self.nBestPosList = []  # Best positions of neighbors
        self.pFit = float('inf')
        self.nFit = float('inf')  # Local best position

def exploration(neuron):
   # Add random exploration to the velocity with a fixed magnitude
    Ke = 0.1   
    exp_vel = [0.0] * layers
    
    for i in range(layers): 
        exp_vel[i] = random.uniform(-1, 1) * Ke 
        neuron.vel[i] += exp_vel[i]

def calculate_fitness(position):
    fit = []
    for i in range(len(obj)):
        fit.append(math.sqrt((position[0] - obj[i][0]) ** 2 + (position[1] - obj[i][1]) ** 2))
    return min(fit)


def update_velocity(neuron, velocity):
    w_ine = 1
    w_cog = 1.5
    w_soc = 1
    vel_inc = 0.1
    
    soc_vel = [0.0] * layers
    cog_vel = [0.0] * layers

    r1 = random.random()
    r2 = random.random()
    for i in range(layers):
    
        # Social element based on global best
        soc_vel[i] = r1 * w_soc * (neuron.nPos[i] - neuron.pos[i]) / size
        soc_vel[i] = max(-max_speed, min(max_speed, soc_vel[i]))
        # Cognitive element based on personal best
        cog_vel[i] = r2 * w_cog * (neuron.pPos[i] - neuron.pos[i]) / size
        cog_vel[i] = max(-max_speed, min(max_speed, cog_vel[i]))

        desired_vel[i] = velocity[i] * w_ine + cog_vel[i] + soc_vel[i] 
        if desired_vel[i] > neuron.vel[i]:
            neuron.vel[i] = min(neuron.vel[i] + vel_inc, desired_vel[i])
        else:
            neuron.vel[i] = max(neuron.vel[i] - vel_inc, desired_vel[i])
    print("CogVel: [{:.3f}, {:.3f}] /// SocVel: [{:.3f}, {:.3f}] ".format(cog_vel[0], cog_vel[1], soc_vel[0], soc_vel[1]))
    print("Neuron Velocity: [{:.3f}, {:.3f}]".format(neuron.vel[0], neuron.vel[1]))
    
def update_position(neuron, gps):
    gps_value = gps.getValues()
    neuron.pos[0] = gps_value[0]
    neuron.pos[1] = gps_value[1] 
   # for i in range(layers):
        #neuron.pos[i] = neuron.pos[i] +  gps_value[i]
           


def init_pso(neuron):
    
    # Initialize the position and velocity randomly
    # swarm[j].pos = [random.uniform(-size / 2, size / 2) for _ in range(layers)]
    # swarm[j].vel = [random.uniform(-max_speed, max_speed) for _ in range(layers)]
    neuron.pPos = neuron.pos.copy() # Update personal best

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

# Function to perform the main PSO loop
def perform_pso(neuron, gps, r_ind):
    global reached_objective, velocity, swarm
    # Randomize the indices each iteration
    if random_indices:
        random.shuffle(indices)
        print("Indices", indices)
        
    # Functions to update position and velocity of neuron 
    update_position(neuron, gps)
    if (iteration % frequency) == 0:
        update_velocity(neuron, velocity)
        print(iteration % frequency, iteration)
        print("pepo")
    
    # Calculation of fitness
    fitness = calculate_fitness(neuron.pos)
    
     # End condition
    if fitness <= 0.2:
        reached_objective = True
        
    # Exploration component
    if fitness <= (neuron.nFit+0.1) or fitness <= (neuron.pFit+0.1):
        exploration(neuron)
        print("Exploration mode")
        
    # Update personal best based on fitness
    if fitness < neuron.pFit:
        neuron.pFit = fitness
        neuron.pPos = neuron.pos.copy()
        print("Updating personal best")
    
    # Update neighborhood  
    
    # Crowding based neighborhood
    if crowding:
        crowding_dist = crowding_distance(swarm, r_ind)
        neighborhood = sorted(range(num_robots), key=lambda x: crowding_dist[x])[:neighbor_size]                
    else:
        # Will create a list of J and all it's neighbors
        # If odd sized neighborhood, the odd one will be added to the right index
        index = indices[r_ind]
        n_indices = indices[index + 1:] + indices[:index]
        neighborhood = n_indices[-neighbor_size // 2:] + n_indices[:neighbor_size // 2]         
    print("neighbors of {} : {}".format(r_ind, neighborhood))
    print("Position / Velocity  Neuron {} : [{:.3f},{:.3f}] / [{:.3f},{:.3f}] ".format(r_ind, neuron.pos[0], neuron.pos[1], neuron.vel[0], neuron.vel[1]))
    print("Fitness / Neighborhood Fitness: {:.2f}  / {:.2f} ".format(fitness, neuron.nFit))
    print("Personal Best / Neighborhood Best [{:.2f},{:.2f}] / [{:.2f}, {:.2f}]".format(neuron.pPos[0], neuron.pPos[1], neuron.nPos[0], neuron.nPos[1]))
    # Update neighborhood best
    if fitness < neuron.nFit:
        
        # Set neighborhood best fitness and position for j and it's neighbors
        neuron.nFit = fitness
        neuron.nPos = neuron.pos.copy()
        send_position(robot, emi, neuron,neighborhood)

   
    



    return reached_objective, neighborhood
# Function sends nPos to neighborhood 
def send_position(robot, emi, neuron, neighborhood):
    for i in range(len(neighborhood)):
        # Put the position in a string to be able to send
        position_str = ','.join(str(coord) for coord in neuron.nPos)
        position_str += ',' + str(neuron.nFit)
        # Set the receiver channel for the supervisor
        emi.setChannel(neighborhood[i])
        # Send the position packet to the supervisor
        emi.send(position_str.encode('utf-8'))
   
 
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
    global reached_objective, velocity
     # Get the robot's name
    robot_name = robot.getName()
    r_ind = int(robot_name[1]) 
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
        global iteration
        for iteration in range(max_iterations):

            print("-----------------------\nIteration start\n-----------------------") 
            print("Robot: ", robot_name) 
            
            # Global position
            if rec.getQueueLength() > 0:
                position = [float(part) for part in rec.getString().split(',')]
                if robot_name == "r0":
                    print("Received message: {:.2f}, {:.2f}, {:.2f}".format(position[0], position[1], position[2]))
                
                rec.nextPacket()
                
                if neuron.nFit > position[2]:
                    neuron.nFit = position[2]
                    neuron.nPos[0] = float(position[0])
                    neuron.nPos[1] = float(position[1])
        
             # Read values from distance 
            ps_values = [sensor.getValue() for sensor in dist_sensors]
            for i in range(8):
                ps_values[i] = dist_sensors[i].getValue()
    
          
            
            perform_pso(neuron, gps, r_ind)
           
        
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
            if reached_objective == True:
                left_motor.setVelocity(0)
                right_motor.setVelocity(0)
                print("Robot {} reached the origin!".format(r_ind))
                break
         
            print("Iteration: {} ".format(iteration))
            
            for _ in range(delay):
                robot.step(timestep)
        break
     

   
if __name__== "__main__":

    # create the Robot instance.
    
    robot = Robot()
    emi = robot.getDevice('emitter')
    run_robot(robot)