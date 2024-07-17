import random as rnd
import math

radius = 0.02
max_speed = 6.275 * radius 
dist_wheels = 0.052
timestep = 64

prev_time = 0
prev_pos = [0,0]

def set_sensors(robot, mode):
    global topology

    topology = mode
    gps = robot.getDevice('gps')
    gps.enable(timestep)
    
    # Create distance sensor instances
    dist_sensors = []
    for i in range(8):
        sensor = robot.getDevice('ps' + str(i))
        sensor.enable(timestep)
        dist_sensors.append(sensor)
    return gps, dist_sensors

def motor_setup(robot):
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')

    
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)

    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
    return left_motor, right_motor
        


def create_str(sPos, sFit):
    if isinstance(sPos[0], list):
        data = []
        for i in range(len(sFit)):
            pos = ','.join(str(j) for j in sPos[i])
            data.append(f"{pos},{str(sFit[i])}")             
        data = ';'. join(data)    
    else:
        pos = ','.join(str(j) for j in sPos)
        data = f"{pos},{str(sFit)}"
    return data

def send_position(emi, data):   
    emi.setChannel(99)
    emi.send(data.encode('utf-8'))

def lsend_position(emi, neuron, data):
    for i in range(len(neuron.neigh)):
        emi.setChannel(neuron.neigh[i])
        emi.send(data.encode('utf-8'))

def best_rec(data, sFit, sPos):
    if isinstance(data[0], list):
        print(data)
        for i in range(len(data)):
            pos = [data[i][0], data[i][1]]
            fit = data[i][2]
            if fit < sFit[i]:
                sPos[i] = pos.copy()
                sFit[i] = fit
    else:
        pos = [data[0], data[1]]
        fit = data[2]
        if fit != float('inf') and fit < sFit:
            sPos = pos.copy()
            sFit = fit  
    return sFit, sPos




def rec_init(rec):
    data = rec.getString()
    rec.nextPacket()
    dat = [float(i) for i in data.split(',')]
        
    return dat

def receive_position(rec):
    data = rec.getString()
    rec.nextPacket()
    if ';' in data:
        dat = []
        for d in data.split(';'):
            dat.append([float(i) for i in d.split(',')]) 
    else: 
        dat = [float(i) for i in data.split(',')]
    
    return dat


def best_position(pos, best, fitness):
    sFit, pFit, sPos, pPos = best
    if fitness < pFit:
        pFit = fitness
        pPos = pos.copy()
    if fitness < sFit:
        sFit = fitness
        sPos = pos.copy()    
    new_best = (sFit, pFit, sPos, pPos)

    return new_best

def update_neigh(rob, neuron, indices, n_size):  
    ind = indices.copy()
    ind.remove(rob.ind)
    # If odd sized neighborhood, the odd one will be added to the left index
    neuron.neigh  = [ind[(rob.ind + i) % len(ind)] for i in range(-n_size // 2, n_size // 2 + 1) if i != 0]

def avoid_obstacles(max_speed, ps_values, left_vel, right_vel):
    
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




def diff_speed(neuron, motors, dist_sensors):
    # Read values from sensors 
    ps_values = [sensor.getValue() for sensor in dist_sensors]
    for i in range(8):
        ps_values[i] = dist_sensors[i].getValue()
    
    Kp = 2; Kv = 0.5
    # reset_prob = 0.1  # Chance  w_speed resets

    # max_random_component = 0.1  # Experiment with different values
    
    l_speed = math.sqrt(neuron.vel[0]**2 + neuron.vel[1]**2)*Kv
    # if rnd.random() <= reset_prob:
    #     w_speed = 0.0
    # else:
    #     random_component = rnd.uniform(-1, 1) * max_random_component

    w_speed = math.atan2(neuron.vel[1], neuron.vel[0])*Kp

    left_vel, right_vel = set_velocity(l_speed, w_speed, ps_values)
    motors[0].setVelocity(left_vel/radius)
    motors[1].setVelocity(right_vel/radius)

def set_velocity(l_speed, w_speed, ps_values):

    left_vel = l_speed - (w_speed * dist_wheels) / 2 
    right_vel = l_speed + (w_speed * dist_wheels) / 2

    # Obstacle avoidance check    
    left_vel, right_vel = avoid_obstacles(max_speed, ps_values, left_vel, right_vel)
    # print ("Velocities: Left  {:.2f} Right {:.2f}".format(left_vel, right_vel))
    # Scales the velocity of the wheels to be proportional to eachother if any is over max_speed
    max_wheel = max(abs(left_vel), abs(right_vel))
    if max_wheel > max_speed:
        left_vel *= (max_speed / max_wheel)
        right_vel *= (max_speed / max_wheel)

    # print ("Velocities: Left  {:.2f} Right {:.2f}".format(left_vel, right_vel))
    
    
    return left_vel, right_vel


def calculate_velocity(robot, gps):
    global prev_pos, prev_time
    
    time = robot.getTime()
    dtime = time - prev_time    
    pos = [gps.getValues()[0], gps.getValues()[1]]
    dpos = [pos[0] - prev_pos[0], pos[1] - prev_pos[1]]
    print(dpos) 
    vel = [dpos[0]/dtime, dpos[1]/dtime]
    # print(dpos, dtime, "dpos dtime")
    # print("prevpos[0] : {:.2f} prevpos[1] : {:.2f} ".format(prev_pos[0], prev_pos[1]))
    # print("pos[0] : {:.2f} pos[1] : {:.2f} ".format(neuron.pos[0], neuron.pos[1]))
    # print("posi : {:.2f} posi : {:.2f} ".format(pos[0], pos[1]))

    # print("time : {:.2f}  dpos[0] : {:.2f} dpos[1] : {:.2f} ".format(time, dpos[0], dpos[1]))
    # print("True velocity", velocity)
    # print("Pos swarm: {:.2f}, {:.2f}".format(neuron.pos[0], neuron.pos[1]))
    prev_pos = pos.copy()
    prev_time = time
    return vel, pos


def post_vel(soc_vel, cog_vel, des_vel, neuron, velocity = None):
    print("CogVel: {:.2f}, {:.2f}".format(cog_vel[0], cog_vel[1]))
    print("SocVel: {:.2f}, {:.2f}".format(soc_vel[0], soc_vel[1]))
    print("Desired Vel: {:.2f}, {:.2f}".format(des_vel[0], des_vel[1]))
    print("Neuron Velocity: {:.2f}, {:.2f}".format(neuron.vel[0], neuron.vel[1]))
    if velocity:
        print("Velocity: {:.2f}, {:.2f}".format(velocity[0], velocity[1]))          

def post_PSO(fitness, sFit, neuron, sPos):
    print("Fitness: {:.2f}".format(fitness))
    print("Global Fitness: {:.2f}".format(sFit))
    print("Personal Best: {:.2f}, {:.2f}".format(neuron.pPos[0], neuron.pPos[1]))
    print("Global Best: {:.2f}, {:.2f}".format(sPos[0], sPos[1]))


def prop(base, iteration, max_iterations, i = False):
   eq = math.sqrt(iteration/max_iterations)#**2
   if not i: return base * eq
   else:       return base * (1-eq)


