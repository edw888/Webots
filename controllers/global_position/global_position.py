from controller import Robot
import math, time
import random



# Declaration of constants
timestep = 64 
max_iterations = 100
iteration = 0
layers = 2
delaysec = 0.25
delay = int(delaysec * 1000/timestep)

radius = 0.02
dist_wheels = 0.052

max_speed = 6.275 * radius 
size = 5   
num_robots = 3


#obj = [[-1,0], [1,0], [0,-1], [0,1] ]
obj = [1,0]

# Init of variables

reached_objective = False 
gPos = [0.0] * layers

velocity = [0.0] * layers

#reached_objective = [False] * num_robots


  
class Neuron:
    def __init__(self):
        self.pos = [0.0] * layers
        self.vel = [0.0] * layers
        self.pPos = [0.0] * layers
        self.pFit = float('inf')

def exploration(neuron):
   # Add random exploration to the velocity with a fixed magnitude
    Ke = 0.1   
    exp_vel = [0,0] * layers
    
    for i in range(layers): 
        exp_vel[i] = random.uniform(-1, 1) * Ke 
        neuron.vel[i] += exp_vel[i]
  

# Function to calculate the fitness of a robot
def calculate_fitness(position):
    fit = []
    #for i in range(len(obj)) :
     #   fit.append(math.sqrt((position[0] - obj[i][0]) ** 2 + (position[1]- obj[i][1]) ** 2))    # Fitness for 2,2
    return math.sqrt((position[0] - obj[0]) ** 2 + (position[1]- obj[1]) ** 2) # min(fit)

# Function to update the robot's velocity
def update_velocity(neuron, gPos):

    w_ine = 1  # Inertia weight to balance old velocity
    w_cog = 1 # Weight for personal best
    w_soc = 1  # Weight for global best
    
    if iteration== 0:
        soc_vel = [0, 0] * layers
        cog_vel = [0, 0] * layers
    print("IneVel: {:.2f}, {:.2f}".format(velocity[0]*w_ine, velocity[1]*w_ine))
    
    r1 = random.random()
    r2 = random.random()    
    for i in range(layers):
       
        
        soc_vel[i] = r1 * w_soc * (gPos[i] - neuron.pos[i])/size
        soc_vel[i] = max(-max_speed, min(max_speed, soc_vel[i]))
        
        cog_vel[i] = r2 * w_cog * (neuron.pPos[i] - neuron.pos[i])/size
        cog_vel[i] = max(-max_speed, min(max_speed, cog_vel[i]))
          
        
        neuron.vel[i] = neuron.vel[i] * w_ine + cog_vel[i] + soc_vel[i] 
       

    print("CogVel: {:.2f}, {:.2f}".format(cog_vel[0], cog_vel[1]))
    print("SocVel: {:.2f}, {:.2f}".format(soc_vel[0], soc_vel[1]))
    print("Neuron Velocity: {:.2f}, {:.2f}".format(neuron.vel[0], neuron.vel[1]))
 

    
def update_position(neuron):
    for i in range(layers):
        neuron.pos[i] = neuron.pos[i] + neuron.vel[i]  
        neuron.pos[i] = max(-size / 2, min(size / 2, neuron.pos[i])) 
     
# Function to perform the PSO algorithm
def init_pso(neuron, gps):
    # Initialize the global best fitness and position
    global gPos, gFit 
    gFit = float('inf')
   
    gPos = [0.0] * layers
        
    for i in range(layers):
        neuron.pos[i] = gps.getValues()[i]

    neuron.pPos = neuron.pos.copy()

def perform_pso(neuron):
    global gPos, gFit, reached_objective
   
    robot_name = robot.getName()   
     # For testing purposes, set gPos and gFit to ideal
  
    for iteration in range(max_iterations):

        for _ in range(delay):
            robot.step(timestep)
        # gFit = 0.1
        # gPos = [0.1, 0.1]  
        print("--------")
        print("Robot ", robot_name) 
        print("Iteration ", iteration)

        # Global position
        receive_position(rec)
        # Perform the main PSO loop

       
        # Update the velocity and position for each robot
        update_position(neuron)
        update_velocity(neuron, gPos) 
 
         # Update personal best position and fitness if necessary
        print("NVel: {:.2f}, {:.2f}".format(neuron.vel[0], neuron.vel[1]))

        print("NPos: {:.2f}, {:.2f}".format(neuron.pos[0], neuron.pos[1]))
        fitness = calculate_fitness(neuron.pos)
        # End condition
        if fitness <= 0.2:
            reached_objective = True
            print("Neuron reached objective")
            break

        # Exploration component
        # if fitness <= (gFit+0.1) or fitness <= (neuron.pFit+0.1):
            # exploration(neuron)
            # print("Exploration mode")
       
        # Update personal and global best
        if fitness < neuron.pFit:
            neuron.pFit = fitness
            neuron.pPos = neuron.pos.copy()
            print("Updating personal best")

            # Update the global best position and fitness if necessary
            if fitness < gFit:
                gFit = fitness
                send_position(robot, emi, neuron, gFit)

                # gPos = neuron.pos.copy()
                
                print("Updating global best")
    
        # Sending to Supervisor Gbest and Gfit
        
        print("Fitness: {:.2f}".format(fitness))
        print("Global Fitness: {:.2f}".format(gFit))
        print("Personal Best: {:.2f}, {:.2f}".format(neuron.pPos[0], neuron.pPos[1]))
        print("Global Best: {:.2f}, {:.2f}".format(gPos[0], gPos[1]))
    # for _ in range(delay/10):
            # robot.step(timestep)
    return reached_objective

            

# Function sends Gbest to supervisor // Could change to send to more
def send_position(robot, emi, neuron, gFit):
    # Put the position in a string to be able to send
    print("Mandando posicion")
    position_str = ','.join(str(coord) for coord in neuron.pPos)
    position_str += ',' + str(gFit)
    # Set the receiver channel for the supervisor
    emi.setChannel(99)
    # Send the position packet to the supervisor
    emi.send(position_str.encode('utf-8'))
    
def receive_position(rec):
    global gPos, gFit
    if rec.getQueueLength() > 0:
            position = [float(part) for part in rec.getString().split(',')]
            rec.nextPacket()
            # gPos[0] = [float(coord) for coord in position[:2]]
            gPos[0] = float(position[0])
            gPos[1] = float(position[1])
            # print(position[0], position[1], position[2])
            gFit = position[2]
    return gPos, gFit




def distance_check(pos, goal):
    # Calculate the distance between the current position and the goal position
    distance = math.sqrt((pos[0] - goal[0]) ** 2 + (pos[1] - goal[1]) ** 2)
    return distance <= 0.1   
    
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
    global reached_objective
     # Get the robot's name
    robot_name = robot.getName()
    print(robot_name)
    num = int(robot_name[1]) 
    prev_position = [0.0, 0.0]  # Previous position of the robot
    prev_time = 0.0  # Previous timestamp

    # Receiver
    
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
    
    # Create position sensor instances
    # left_ps = robot.getDevice('pos1')
    # left_ps.enable(timestep)
    
    # right_ps = robot.getDevice('pos2')
    # right_ps.enable(timestep)
    # ps_values = [0, 0, 0]

    
    
    # GPS
    for _ in range(delay):
        robot.step(timestep)
    gps = robot.getDevice('gps')
    gps.enable(timestep)
    print(gps.getValues())
    # Create distance sensor instances
    dist_sensors = []
    for i in range(8):
        sensor = robot.getDevice('ps' + str(i))
        sensor.enable(timestep)
        dist_sensors.append(sensor)
    

    # Robot pose

    position = [0, 0, 0]
   
    
    # Initialize the swarm of neurons and perform PSO
    neuron = Neuron()
    init_pso(neuron, gps)
    perform_pso(neuron)
    reached_objective = False

    velocity = [0, 0]
    while robot.step(timestep) != -1:
        global gPos, gFit
         
       
        print("-----------")
        print("Robot ", robot_name)
        for i in range(layers):
             position[i] = gps.getValues()[i]
             
        print("Position {:.2f}, {:.2f}".format(position[0], position[1]))  
        print("Goal {:.2f}, {:.2f}".format(neuron.pos[0], neuron.pos[1]))
    
    
            
        # Check if reached goal
        
        if distance_check(position, neuron.pos):
                reached_objective = True
                left_motor.setVelocity(0)
                right_motor.setVelocity(0)
                print("Robot reached goal")
                break
                
        # Read values from distance 
        ps_values = [sensor.getValue() for sensor in dist_sensors]
        for i in range(8):
            ps_values[i] = dist_sensors[i].getValue()

        # send_position(robot, emi, neuron, gFit)
        
        des_heading = math.atan2(neuron.pos[1] - position[1], neuron.pos[0] - position[0])
        angle_diff = des_heading - math.atan2(velocity[1], velocity[0])
        distance = math.sqrt((neuron.pos[0] - position[0]) ** 2 + (neuron.pos[1] - position[1]) ** 2)
  
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        # Proportional control for  angular velocity based on angle difference
        Kp = 2.0  
        w_speed = Kp * angle_diff
        # Proportional control for linear velocity, using a sigmoid function for  deceleration
        Kv = 0.5 
        v_speed = Kv * distance / (1 + math.exp(-distance))
       
    
        print(ps_values)
        print("V_speed {:.2f} W_speed {:.2f}".format(v_speed, w_speed))

        
        left_vel = v_speed - (w_speed * dist_wheels) / 2 
        right_vel = v_speed + (w_speed * dist_wheels) / 2
        
        # Obstacle avoidance check    
        left_vel, right_vel = avoid_obstacles(ps_values, left_vel, right_vel)
 
        # Scales the velocity of the wheels to be proportional to eachother if any is over max_speed
        max_wheel_velocity = max(abs(left_vel), abs(right_vel))
        if max_wheel_velocity > max_speed:
            scale = max_speed / max_wheel_velocity
            left_vel *= scale
            right_vel *= scale
       
        print ("Velocities: Left  {:.2f} Right {:.2f}".format(left_vel, right_vel))

        

        # Set the motor velocities based on the robot's position, x y forward, y is rotational
  
        left_motor.setVelocity(left_vel/radius)
        right_motor.setVelocity(right_vel/radius)
            
        
        # Calculate velocity
        time = robot.getTime()
        dtime = time - prev_time
        position = [gps.getValues()[0], gps.getValues()[1]]
        dpos = [position[0] - prev_position[0], position[1] - prev_position[1]] 

        velocity = [dpos[0]/dtime, dpos[1]/dtime]
        print("prevpos[0] : {:.2f} prevpos[1] : {:.2f} ".format(prev_position[0], prev_position[1]))
        print("posi : {:.2f} posi : {:.2f} ".format(position[0], position[1]))

        print("time : {:.2f} dtime {:.2f}  dpos[0] : {:.3f} dpos[1] : {:.3f} ".format(time, dtime, dpos[0], dpos[1]))
        print("True velocity", velocity)

            

         
                
        print("Iteration: {} gFit: {:.2}  gPos {}".format(iteration, gFit, gPos))
       

        for _ in range(delay):
            robot.step(timestep)
        prev_position = position.copy()
        prev_time = time

         
     
       
        
      
   
   
if __name__== "__main__":

    # create the Robot instance.
    
    robot = Robot()
    emi = robot.getDevice('emitter')
    rec = robot.getDevice('receiver')

    run_robot(robot)
    
   
    