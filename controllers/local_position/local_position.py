from controller import Robot
import math
import random


timestep = 64 
max_iterations = 100
layers = 2
max_speed = 6.28 # #Angular velocity
num_robots = 4
reached_origin = False 

class Neuron:
    def __init__(self):
        self.pos = [0.0] * layers
        self.vel = [0.0] * layers
        self.pPos = [0.0] * layers
        self.pFit = float('inf')

# Function to calculate the fitness of a robot
def calculate_fitness(position):
    # Fitness function calculation based on the problem domain
    # Modify this function according to your specific problem
    # Distance from origin:
    return math.sqrt(position[0] ** 2 + position[1] ** 2)
# Function to update the robot's velocity
def update_velocity(neuron, gPos):
    # PSO velocity update equation
    for i in range(layers):
        r1 = random.random()
        r2 = random.random()

        neuron.vel[i] = neuron.vel[i] + \
            r1 * (neuron.pPos[i] - neuron.pos[i]) + \
            r2 * (gPos[i] - neuron.pos[i])

        # Apply velocity limits
        neuron.vel[i] = max(-max_speed, min(max_speed, neuron.vel[i]))

# Function to update the robot's position
def update_position(neuron):
    # PSO position update equation
    for i in range(layers):
        neuron.pos[i] = neuron.pos[i] + neuron.vel[i]

# Function to perform the PSO algorithm
def perform_pso(swarm):
    # Initialize the global best fitness and position
    gFit = float('inf')
    gPos = [0.0] * layers

    # Iterate over the swarm and initialize each robot
    for i in range(num_robots):
        # Initialize the position randomly
        # swarm[i].pos = [random.uniform(-2, 2) for _ in range(layers)]

        # Initialize the velocity randomly
        # swarm[i].vel = [random.uniform(-max_speed, max_speed) for _ in range(layers)]

        # Set the initial personal best position and fitness
        swarm[i].pPos = swarm[i].pos.copy()
        swarm[i].pFit = calculate_fitness(swarm[i].pos)

        # Update the global best position and fitness if necessary
        if swarm[i].pFit < gFit:
            gFit = swarm[i].pFit
            gPos = swarm[i].pPos.copy()

    # Perform the main PSO loop
    for iteration in range(max_iterations):
        # Update the velocity and position for each robot
        for i in range(num_robots):
            update_velocity(swarm[i], gPos)
            update_position(swarm[i])

            # Update personal best position and fitness if necessary
            fitness = calculate_fitness(swarm[i].pos)
            if fitness < swarm[i].pFit:
                swarm[i].pFit = fitness
                swarm[i].pPos = swarm[i].pos.copy()

                # Update the global best position and fitness if necessary
                if fitness < gFit:
                    gFit = fitness
                    gPos = swarm[i].pos.copy()

        # Print the best fitness value at each iteration
        # print("Iteration:", iteration, "Best Fitness:", gFit)

   
def run_robot(robot):
    global reached_origin
     # Get the robot's name
    robot_name = robot.getName()
   
    # Receiver
    rec = robot.getDevice('receiver')
    rec.enable(timestep)
    #Set the receiver channel based on the name of each robot (r0,r1)
    print(int(robot_name[1]))
    rec.setChannel(int(robot_name[1])) #Channel will be either 0,1,2 based on robot
    
   
    # Created motor instances
    left_motor = robot.getDevice('motor1')
    right_motor = robot.getDevice('motor2')
    
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
 
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
    
   # Create position sensor instances
    left_ps = robot.getDevice('pos1')
    left_ps.enable(timestep)
    
    right_ps = robot.getDevice('pos2')
    right_ps.enable(timestep)
    ps_values = [0, 0, 0]
    
    
    
    # Create distance sensor instances
    # dist_sensors = []
    # for i in range(8):
        # sensor = robot.getDevice('ds' + str(i))
        # sensor.enable(timestep)
        # dist_sensors.append(sensor)
    
    # Compute encoder unit
    radius = 0.03
    wheeldist = 0.1
    circum = 2 * math.pi * radius
    encoderunit = circum / 6.28
    
    # Robot pose
    robot_pose = [0, 0, 0]  # x, y, theta
    position = [1, 1, 1]
    ps0_values = [0, 0]
    checkpoint = 0
    speed = max_speed
    
    # Initialize the swarm of neurons and perform PSO
    swarm = [Neuron() for _ in range(num_robots)]
    perform_pso(swarm)
    
    
    while robot.step(timestep) != -1:
    # Receive messages from other robots
        if rec.getQueueLength() > 0:
            # position_str = rec.getString()
            # position_parts = position_str.split(',')  # Split the string by comma
            # x = float(position_parts[0])  # Convert the first part to float (x-coordinate)
            # y = float(position_parts[1])  # Convert the second part to float (y-coordinate)
            # z = float(position_parts[2])  # Convert the third part to float (z-coordinate)
            # position = [x, y, z]
            position = [float(part) for part in rec.getString().split(',')]
            if robot_name == "r0":
                print("Received message: {:.2f}, {:.2f}, {:.2f}".format(position[0], position[1], position[2]))
            rec.nextPacket()
            for i in range(num_robots):
                swarm[i].pos = position
        
        if robot_name == "r0": 
            print("Robot {}".format(robot_name))
            print("Pos {:.2f}, {:.2f}, {:.2f}".format(position[0], position[1], position[2]))
        robot_pose = position
        # Not used for now as we have Gbest
        # avg_pos = [sum(neuron.pos[i] for neuron in swarm) / num_robots for i in range(layers)] 
        for i in range(num_robots):
            # Set the motor velocities based on the robot's position, x y forward, y is rotational
            x_vel = swarm[i].pos[0] * max_speed / 2.5  # Maximum pos
            y_vel = swarm[i].pos[1] * max_speed / 2.5  

            # Read values from distance 
            # ps_values = [sensor.getValue() for sensor in dist_sensors]
            # for i in range(8):
                # ps_values[i] = dist_sensors[i].getValue()

            # Calculate obstacle avoidance velocity
            # avoid_vel = [0, 0]
            # for j in range(8):
                # avoid_vel[0] += math.cos(robot_pose[2] + i * math.pi / 4) * ps_values[i]
                # avoid_vel[1] += math.sin(robot_pose[2] + i * math.pi / 4) * ps_values[i]

            # Update robot velocity based on PSO and obstacle avoidance
            left_vel = (2 * y_vel - x_vel) / 2 # - avoid_vel[0]
            right_vel = (2 * y_vel + x_vel) / 2 # + avoid_vel[1]
            left_vel = max(-max_speed, min(max_speed, left_vel)) # Limit vel
            right_vel = max(-max_speed, min(max_speed, right_vel))
             

            if robot_name == ("r"+ str(i)):
                left_motor.setVelocity(left_vel)
                right_motor.setVelocity(right_vel)
            
    
    
            # Update robot pose based on wheel encoders
                left_position = left_ps.getValue()
                right_position = right_ps.getValue()
    
                dist_left = (left_position - checkpoint) * encoderunit
                dist_right = (right_position - checkpoint) * encoderunit
                checkpoint = left_position
                robot_pose[0] += (dist_left + dist_right) * 0.5 * math.cos(robot_pose[2])
                robot_pose[1] += (dist_left + dist_right) * 0.5 * math.sin(robot_pose[2])
                robot_pose[2] += (dist_right - dist_left) / wheeldist
            if robot_name == "r0":
                print("pos swarm {:.2f}, {:.2f}".format(swarm[i].pos[0], swarm[i].pos[1]))    
            # Check if the robot has reached the origin
            if math.sqrt(robot_pose[0] ** 2 + robot_pose[1] ** 2) < 0.01:
                print("Robot {} reached the origin!".format(robot_name))
                reached_origin = True
                break 

        if robot_name == "r0":

            print ("Velocities lv {:.2f} rv {:.2f}".format(left_vel, right_vel))
            print("Pose {:.2f}, {:.2f}, {:.2f}".format(robot_pose[0], robot_pose[1], robot_pose[2]))

        if reached_origin == True:
            break 
        
      
   
   
if __name__== "__main__":

    # create the Robot instance.
    
    robot = Robot()
    run_robot(robot)
   
    