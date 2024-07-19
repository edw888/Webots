from controller import Robot
import math
import random as rnd
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
fit_obj = 0.2
stuck_limit = 5 # Num. of iterations it can be stuck
exp_count = 5 # Num. of iterations it will explore
fit_stuck = fit_obj/4

# Local pso constants
num_robots = 4  
indices = list(range(num_robots))  
n_size = num_robots//3


# Init of variables
reached_obj = False
iteration = 0
vel = [0.0] * dim
des_vel = [0.0] * dim
shuffled = False

# Configurations
cfg = {"cycle" : False, "multi": True}
ncfg = {"random": False}
shuffle_freq = 5

def localPSO(con, rob):
    robot = rob.robot 

    class Neuron:
        def __init__(self):
            self.pos = [0.0] * dim
            self.vel = [0.0] * dim
            self.pPos = [[0,0] for _ in range(len(con.obj))] if any(cfg.values()) else [0.0] * dim
            self.pFit = [float('inf')] * len(con.obj) if any(cfg.values())  else float('inf')
            self.sFit = [float('inf')] * len(con.obj) if any(cfg.values()) else float('inf')
            self.sPos = [[0.0] * dim for _ in range (len(con.obj))] if any(cfg.values())  else [0.0] * dim
            self.obj = 0
            self.dynexp = 0
            self.prev_fit = [0,0]
            self.neigh = []

    def calculate_fitness(position):
        points_fitness = []
        for i in range(len(con.obj)):
            fit = math.sqrt((position[0] - con.obj[i][0]) ** 2 + (position[1] - con.obj[i][0]) ** 2)
            points_fitness.append(fit)
        return points_fitness

    # Function to update the robot's velocity
    def update_velocity(neuron, pPos, sPos):
        
        w_ine = 1
        w_cog = 1.5
        w_soc = 1
        w_exp = prop(1, iteration, con.max_iterations, i=True) 
        if neuron.dynexp: w_exp = 1; neuron.dynexp -= 1;          
        soc_vel = [0.0] * dim
        cog_vel = [0.0] * dim
        exp_vel = [0.0] * dim

        r1 = rnd.random()
        r2 = rnd.random()

        for i in range(dim):
            r3 = rnd.uniform(-1,1)
           
            soc_vel[i] = r1 * w_soc * (sPos[i] - neuron.pos[i]) / (con.size/2)
            # soc_vel[i] = max(-max_speed, min(max_speed, soc_vel[i]))
            
            cog_vel[i] = r2 * w_cog * (pPos[i] - neuron.pos[i]) / (con.size/2)
            # cog_vel[i] = max(-max_speed, min(max_speed, cog_vel[i]))
            exp_vel[i] = r3 * w_exp * max_speed
            des_vel[i] = vel[i] * w_ine + cog_vel[i] + soc_vel[i] + exp_vel[i] 
            if des_vel[i] > neuron.vel[i]:
                neuron.vel[i] = min(neuron.vel[i] + vel_inc, des_vel[i])
            else:
                neuron.vel[i] = max(neuron.vel[i] - vel_inc, des_vel[i])
        # fnc.post_vel(soc_vel, cog_vel, vel, neuron.vel)
    
    # Counts number of iterations a particle doesn't improve its position
    def dynamic_exploration(fitness, neuron):
        # Prevents being stuck too long
        fit_dif = abs(fitness - neuron.prevfit[1])
        if fit_dif < fit_stuck: 
            neuron.prevfit[0] += 1
        else: neuron.prevfit[0] = 0
        if neuron.prevfit[0] == stuck_limit: 
            neuron.dynexp = exp_count
            neuron.prevfit[0] = 0
        neuron.prevfit[1] = fitness

    def update_position(neuron, gps):
        gps_value = gps.getValues()
        neuron.pos[0] = gps_value[0]
        neuron.pos[1] = gps_value[1] 

    # Initializes the algorithm
    def init_pso(neuron):
        for i in range(dim): 
            neuron.vel[i] = rnd.uniform(-max_speed, max_speed)

        if any(cfg.values()):
            for i in range(len(con.obj)):
                neuron.pPos[i]  = neuron.pos.copy()
                neuron.sPos[i]  = neuron.pos.copy()
        else:   
            neuron.pPos = neuron.pos.copy()
            neuron.sPos = neuron.pos.copy()
        neuron.neigh  = [indices[(rob.ind + i) % len(indices)] for i in range(-n_size // 2, n_size // 2 + 1) if i != 0]

    # Function to perform the main PSO loop
    def perform_pso(neuron, gps):
        global reached_obj, vel

        shuffled = False
        active = not(iteration % frequency)
        # Randomize the indices each iteration
        if ncfg["random"]:
            if iteration % shuffle_freq == 0:
                rnd.shuffle(indices)
                shuffled = True
            
        # Functions to update position and velocity of neuron 
        if active:
            update_velocity(neuron, neuron.pPos[neuron.obj], neuron.sPos[neuron.obj]) if any(cfg.values()) else update_velocity(neuron, neuron.pPos, neuron.sPos) 
        update_position(neuron, gps)

        # Calculation of fitness
        fitnesses = calculate_fitness(neuron.pos)
        if cfg["multi"]:
            mF = min(fitnesses)
            mG = min(neuron.sFit)
            if (mF + fit_obj)/4 < (mG + fit_obj):
                neuron.obj = (fitnesses.index(mF))
            else: 
                neuron.obj = (neuron.sFit.index(mG))
        fitness = fitnesses[neuron.obj] if any(cfg.values()) else min(fitnesses) # Current objective fitness 

        if shuffled:
            fnc.update_neigh(ncfg, rob, neuron, indices, n_size)

        if not any(cfg.values()):
            best = (neuron.sFit, neuron.pFit, neuron.sPos, neuron.pPos)
            n_best = fnc.best_position(neuron.pos, best, fitness)  
            (neuron.sFit, neuron.pFit, neuron.sPos, neuron.pPos) = n_best
            if n_best[0] != best[0]:
                data = fnc.create_str(neuron.sPos, neuron.sFit)
                fnc.lsend_position(rob.emi, cfg, data)
                print("Send position", rob.name)
        else:    
            for i in range(len(con.obj)):
                best = (neuron.sFit[i], neuron.pFit[i], neuron.sPos[i], neuron.pPos[i]) 
                n_best = fnc.best_position(neuron.pos, best, fitnesses[i])
                (neuron.sFit[i], neuron.pFit[i], neuron.sPos[i], neuron.pPos[i]) = n_best
                if n_best != best:
                    data = fnc.create_str(neuron.sPos, neuron.sFit)
                    fnc.lsend_position(rob.emi, neuron, data)
                    print("Send position", rob.name)

        # Check if different modes reached an objective
        if fitness <= fit_obj:
            if cfg["cycle"]:
                neuron.pFit[neuron.obj] = float('inf')  
                neuron.obj = (neuron.obj + 1) % len(con.obj)
            else:
                print("Robot {} reached the objective".format(rob.ind))
                reached_obj = True 
        
        # fnc.post_PSO(fitness, neuron.sFit, neuron, neuron.sPos)
    
    def run_robot():
        global reached_obj, vel
        
        motors = fnc.motor_setup(robot)
        gps, dist_sensors = fnc.set_sensors(robot, "local")
        
        # Initialize the swarm of neurons and perform PSO
        neuron = Neuron()
        init_pso(neuron)
        
        for iteration in range(max_iterations):
            print("---")
            for _ in range(delay):
                robot.step(timestep)
            
            # Check if reached origin
            if reached_obj == True:
                motors[0].setVelocity(0)
                motors[1].setVelocity(0)
                print("Robot {} reached the origin!".format(rob.ind))
                break
                        
            # Receiving best positions from neighbors
            if rob.rec.getQueueLength() > 0:
                data = fnc.receive_position(rob.rec)
                neuron.sFit, neuron.sPos = fnc.best_rec(data, neuron.sFit, neuron.sPos)
        
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