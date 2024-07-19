import math, time
import random as rnd
import fnc
from fnc import receive_position, calculate_velocity, prop

timestep = 64 
dim = 2
delaysec = 0.2
delay = int(delaysec * 1000/timestep)

radius = 0.02
max_speed = 6.275 * radius 
dist_wheels = 0.052


vel_inc = max_speed / 8

# Init of variables
iteration = 0
reached_objective = False 
gPos = [0.0] * dim
vel = [0.0] * dim
des_vel = [0.0] * dim

class Neuron:
    def __init__(self):
        self.pos = [0.0] * dim
        self.vel = [0.0] * dim
        self.pPos = [0.0] * dim
        self.pFit = float('inf')

def globalPSO_off(con, rob):
    robot = rob.robot

    # Function to calculate the fitness of a robot
    def calculate_fitness(pos):
        fit = []
        for i in range(len(con.obj)) :
            fit.append(math.sqrt((pos[0] - con.obj[i][0]) ** 2 + (pos[1]- con.obj[i][1]) ** 2))    # Fitness for 2,2
        return min(fit)

    # Function to update the robot's velocity
    def update_velocity(neuron, gPos):

        w_ine = 1 # Inertia weight to balance old velocity
        w_cog = 1 # Weight for personal best
        w_soc = 1 # Weight for global best
        w_exp = prop(1, iteration, con.max_iterations, i=True) 
        soc_vel = [0.0] * dim
        cog_vel = [0.0] * dim
        exp_vel = [0.0] * dim
        r1 = rnd.random()
        r2 = rnd.random()    
        for i in range(dim):
        
            r3 = rnd.uniform(-1,1)
            soc_vel[i] = r1 * w_soc * (gPos[i] - neuron.pos[i])/con.size
            soc_vel[i] = max(-max_speed, min(max_speed, soc_vel[i]))
            
            cog_vel[i] = r2 * w_cog * (neuron.pPos[i] - neuron.pos[i])/con.size
            cog_vel[i] = max(-max_speed, min(max_speed, cog_vel[i]))
            
            
            exp_vel[i] = r3 * w_exp * max_speed

            des_vel[i] = vel[i] * w_ine + cog_vel[i] + soc_vel[i] + exp_vel[i]

            if des_vel[i] > neuron.vel[i]:
                neuron.vel[i] = min(neuron.vel[i] + vel_inc, des_vel[i])
            else:
                neuron.vel[i] = max(neuron.vel[i] - vel_inc, des_vel[i])
            
        # fnc.post_vel(soc_vel, cog_vel, vel, neuron.vel)

        
    def update_position(neuron):
        for i in range(dim):
            neuron.pos[i] = neuron.pos[i] + neuron.vel[i]  
            neuron.pos[i] = max(-con.size / 2, min(con.size / 2, neuron.pos[i])) 
        
    # Function to perform the PSO algorithm
    def init_pso(neuron, gps):
        # Initialize the global best fitness and position
        global gPos, gFit 
        gFit = float('inf')
        gPos = [0.0] * dim
        for i in range(dim): 
            neuron.vel[i] = rnd.uniform(-max_speed, max_speed) 
        for i in range(dim):
            neuron.pos[i] = gps.getValues()[i]

        neuron.pPos = neuron.pos.copy()

    def perform_pso(neuron):
        global gPos, gFit, reached_objective, iteration
    
    
        for iteration in range(con.max_iterations):
            for _ in range(delay):
                robot.step(timestep)
            # Receiving best positions from supervisor
            if rob.rec.getQueueLength() > 0:
                data = receive_position(rob.rec)

                if data[2] < gFit:
                    gPos = [float(data[0]), float(data[1])]
                    gFit = data[2]
                rob.rec.nextPacket()

        
            # Update the velocity and position for each robot
            update_velocity(neuron, gPos) 
            update_position(neuron) 
            
            fitness = calculate_fitness(neuron.pos)
            # End condition
            if fitness <= 0.2:
                reached_objective = True
                print("Neuron reached objective")
                break
        
            # Update personal and global best
            if fitness < neuron.pFit:
                neuron.pFit = fitness
                neuron.pPos = neuron.pos.copy()

                # Update the global best position and fitness if necessary and send to supervisor
                if fitness < gFit:
                    gFit = fitness
                    gPos = neuron.pos.copy()
                    data = fnc.create_str(gPos, gFit)
                    fnc.send_position(rob.emi, data)
      
            # fnc.post_PSO(fitness, gFit, neuron, gPos)
  
        return reached_objective


    def distance_check(pos, goal):
        # Calculate the distance between the current position and the goal position
        distance = math.sqrt((pos[0] - goal[0]) ** 2 + (pos[1] - goal[1]) ** 2)
        return distance <= 0.05   
        

    def run_robot():
        global reached_objective, gPos, gFit
        motors = fnc.motor_setup(robot)
        gps, dist_sensors = fnc.set_sensors(robot, "global")

        # Initialize the swarm of neurons and perform PSO
        neuron = Neuron()
        init_pso(neuron, gps)
        # Completes PSO and obtains desired final positions
        perform_pso(neuron)
        reached_objective = False
        pos = [0, 0]
        vel = [0, 0]

        while robot.step(timestep) != -1:
    
            # print("-----------")
            # print("Robot ", rob.ind)
            for i in range(dim):
                pos[i] = gps.getValues()[i]
                
            # print("Position {:.2f}, {:.2f}".format(pos[0], pos[1]))  
            # print("Goal {:.2f}, {:s.2f}".format(neuron.pos[0], neuron.pos[1]))
          
            # Check if robot reached PSO goal
            if distance_check(pos, neuron.pos):
                    reached_objective = True
                    motors[0].setVelocity(0)
                    motors[1].setVelocity(0)
                    print("Robot reached goal")
                    break
            
            # Read values from distance sensors
            ps_values = [sensor.getValue() for sensor in dist_sensors]
            for i in range(8):
                ps_values[i] = dist_sensors[i].getValue()
            
            des_heading = math.atan2(neuron.pos[1] - pos[1], neuron.pos[0] - pos[0])
            adiff= des_heading - math.atan2(vel[1], vel[0])
            distance = math.sqrt((neuron.pos[0] - pos[0]) ** 2 + (neuron.pos[1] - pos[1]) ** 2)
            if adiff > math.pi:
                adiff -= 2 * math.pi
            elif adiff < -math.pi:
                adiff += 2 * math.pi
        
            Kp = 2;  Kv = 0.5 
            w_speed = Kp * adiff
            l_speed = Kv * distance / (1 + math.exp(-distance))

            left_vel, right_vel = fnc.set_velocity(l_speed, w_speed, ps_values)
            motors[0].setVelocity(left_vel/radius)
            motors[1].setVelocity(right_vel/radius)
            # Calculate vectorial velocity
            vel, pos = calculate_velocity(robot, gps)
            
            # print("Iteration: {}.  {} gFit: {:.3f}  gPos [{:.2f},{:.2f}]".format(iteration, rob.name, gFit, gPos[0], gPos[1]))
              
    run_robot()
        
    
        