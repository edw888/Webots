import math
import random, time
from controller import Supervisor, Node
import numpy as np

supervisor = Supervisor()
timestep = 64 
dim = 2
#num_robots = 4
#size = 5
#resolution = 0.1

TIME_STEP = 64
emi = supervisor.getDevice('emitter')
# Receiver
rec = supervisor.getDevice('receiver')
rec.enable(timestep)
rec.setChannel(99) 

# Create a list of lists for each robot's position

positions = []


iteration_timer = 0
iteration = 0
ite = 0
delaysec = 1
delay = int(delaysec * 200/timestep)



def receive_config():
    while True:   
        for _ in range(delay):
            supervisor.step(timestep)
        if rec.getQueueLength() > 0:
            # print("Data Received")
            data = rec.getString().split(';')
            config = [float(i) for i in data[:4]]
            config.append(eval(data[4])) 
            if len(data) > 5:
                config.extend([float(i) for i in data[5:]])
            # Converts the last component in the string to a list
            rec.nextPacket()
            return config
           
     
config = receive_config()  

 
mode = config[0]
num_robots = int(config[1])
max_iterations = int(config[2])
multi = config[3]
obj = config[4]
size = config[5]

if len(config)> 6:
    resolution = config[6]; evaporation_rate = config[7]; max_phe = config[7]; frequency = config[9]
    # print("num_robots, max_iterations, size, resolution: ", num_robots, max_iterations, size, resolution)
    map_range = int(size/resolution)
    pheromone_grid = np.zeros((map_range, map_range))
    
class Supervisor:
    def __init__(self):
        self.gPos = [0.0] * dim
        self.gPos = [[0,0] for _ in range(len(obj))] if not mode == 0 and multi else [0.0] * dim
        self.gFit = [float('inf')] * len(obj) if not mode == 0 and multi else float('inf')

sp = Supervisor()



def random_position(supervisor):
    global positions
    for i in range(num_robots):
        robot = supervisor.getFromDef("r" + str(i))
        trans_field = robot.getField("translation")
        # Random initialization
        x = random.uniform(-size / 2.0 + 0.1, size / 2.0 -0.1)
        y = random.uniform(-size / 2.0 + 0.1, size / 2.0 -0.1)
        # Premeditates initialization
        # if i == 0:
            # x = -2
            # y = -2
        # elif i == 1:
            # x = 2
            # y = 2
        # elif i == 2:
            # x = 2
            # y = -2
        # else:
            # x = -2
            # y = 2
        trans_field.setSFVec3f([x, y, -0.1])
        # positions.append([x, y])
        data = ','.join(str(coord) for coord in [x,y])
        # data = pos +  ',' + str(float('inf'))
        # Set the receiver channel for each robot
        emi.setChannel(i)
        # Send the position packet to the current robot
        emi.send(data.encode('utf-8'))
        # data_pos.append(positions[i])
    print("Random positioning done.")

# def init_position(data_pos):   
#     pos = ','.join(str(coord) for coord in data_pos)
#     data = pos +  ',' + str(sp.gFit) 
#     for i in range(num_robots):
#         # print("Sending from sup", data_pos)
#         pos = ','.join(str(coord) for coord in data_pos)
#         data = pos +  ',' + str(sp.gFit)
    
#         # Set the receiver channel for each robot
#         emi.setChannel(i)
#         # Send the position packet to the current robot
#         emi.send(data.encode('utf-8'))

def send_position(data_pos):
    if isinstance(data_pos[0], list):
        data = []
        for i in range(len(sp.gFit)):
            pos = ','.join(str(j) for j in data_pos[i])
            data.append(pos + ',' + str(sp.gFit[i]))  
        data = ';'. join(data)
    else: 
        pos = ','.join(str(j) for j in data_pos)
        data = pos +  ',' + str(sp.gFit)  

    for i in range(num_robots):
        # Set the receiver channel for each robot
        emi.setChannel(i)
        # Send the position packet to the current robot
        emi.send(data.encode('utf-8'))
 
def send_pheromones(pheromone_grid):
    pheromone_str = ','.join(str(i) for j in pheromone_grid for i in j)+ ',1'
    # Set the receiver channel for each robot
    for i in range(num_robots):
        emi.setChannel(i)
        # Send the pheromone grid packet to the supervisor
        emi.send(pheromone_str.encode('utf-8'))
    total_pheromones = sum(sum(row) for row in pheromone_grid)

        
    
    
def receive_position():
    data = rec.getString()
    rec.nextPacket()
    if ';' in data:
        dat = []
        for d in data.split(';'):
            dat.append([float(i) for i in d.split(',')]) 
    else: 
        dat = [float(i) for i in data.split(',')]

    if isinstance(dat[0], list):
        for i in range(len(dat)):
            pos = [dat[i][0], dat[i][1]]
            fit = dat[i][2]
            if fit < sp.gFit[i]:
                sp.gPos[i] = pos.copy()
                sp.gFit[i] = fit
    else:
        pos = [dat[0], dat[1]]
        fit = dat[2]
        if fit < sp.gFit:
            sp.gPos = pos.copy()
            sp.gFit = fit  
        # print("Updated gPos supervisor {:.2f}, {:.2f}".format(sp.gPos[0], sp.gPos[1]))
        # print("Updaged gFit supervisor", sp.gFit)
        
      
  
def receive_pheromones(rec,pheromone_grid):
 
    pheromone_str = rec.getString()

    rec.nextPacket()
    # Parse the received string into a 1D array of float values
    pheromone_values = [float(value) for value in pheromone_str.split(',')]
    
    # Convert the 1D array into a 2D numpy array representing the pheromone grid
    rec_phe = np.array(pheromone_values).reshape((map_range, map_range))
 

    return rec_phe

def end_iteration(ite):
    global pheromone_grid, max_iterations, iteration, evaporation_rate
   
        
    total_pheromones = sum(sum(row) for row in pheromone_grid)

     
    pheromone_grid *= (1-evaporation_rate*max_iterations/(max_iterations+iteration*3))
    #total_pheromones = sum(sum(row) for row in pheromone_grid)
    #print("Send pheromones: ", total_pheromones)
    # Print the list of positions and pheromone quantities
    #print("Pheromone grid:")
    #for position in pheromone_list:
    #    print(f"Position {position[:2]}: {position[2]}")
    send_pheromones(pheromone_grid)
    iteration += 1
    if iteration%frequency:
        pheromone_display(pheromone_grid)
    

root_node = supervisor.getRoot()        
children_field = root_node.getField('children')  
# Set to store positions with deposited pheromones
deposited_positions = set()
def pheromone_display(pheromone_grid):
    global deposited_pheromones
# Iterate through each cell in the pheromone grid
    for i in range(map_range):
        for j in range(map_range):
        
            pheromone_level = pheromone_grid[i, j]
            if pheromone_level > 0 and (i, j) not in deposited_positions:
            # Check if the pheromone level is non-zero             
                # Create a sphere at the corresponding position on the map
                x = i * 0.25 - 2.37; y =  j * 0.25 - 2.63
                pos = f'{x} {y} -0.26'
                ball_name = f'empty_ball_{i}_{j}'  # Unique name for each ball
                children_field.importMFNodeFromString(-1, f'DEF {ball_name} empty_ball {{ translation {pos} }}')
                ball_node = supervisor.getFromDef(ball_name)
                transparency = 0.5  # Calculate transparency based on pheromone level
                radius = pheromone_level*0.1  # Calculate radius based on pheromone level
                ball_node.getField("radius").setSFFloat(radius)  # Adjust radius based on pheromone level
                ball_node.getField("transparency").setSFFloat(transparency)  # Set transparency
                deposited_positions.add((i, j))
            elif (i, j) in deposited_positions:

                ball_name = f'empty_ball_{i}_{j}'
                ball_node = supervisor.getFromDef(ball_name)
                if pheromone_level > 0.1:
                    radius = pheromone_level*0.1  # Calculate radius based on pheromone level
                    ball_node.getField("radius").setSFFloat(radius)  
                else:
                    # supervisor.removeNode(ball_name)
                    ball_node.remove()
                    deposited_positions.remove((i, j)) 
               # if pheromone_level > 0:
                #    radius = pheromone_level*0.1
                 #   ball_node.getField("radius").setSFFloat(radius)  # Adjust radius based on pheromone level
               # else:
                 #   ball_node.remove()
                
                    
                

    
def test(pheromone_grid):
# Iterate through each cell in the pheromone grid
    root_node = supervisor.getRoot()
    children_field = root_node.getField('children')
    children_field.importMFNodeFromString(-1, 'DEF empty_ball empty_ball { translation 0 0 -0.26 }')
    ball_node = supervisor.getFromDef('empty_ball')
    #color_field = ball_node.getField('color')
 
    # Create a sphere at the corresponding position on the map
    #sphere = supervisor.getFromDef('ball')  # Replace "Sphere" with the name of your object
    #translation = [i * 0.25 - 2.5, 0.1, j * 0.25 - 2.5]  # Adjust scale and translation based on your map
    #sphere.getField("translation").setSFVec3f(translation)
    # Adjust size or color of the sphere based on pheromone level
    
    #sphere.getField('radius').setSFVec3f(0.1)
    #sphere.getField("radius").setSFFloat(1 * 0.1)  # Adjust radius based on pheromone level
    # You can also adjust color using sphere.getField("appearance").getField("material").getField("diffuseColor").setSFColor(...)    
            # You can also adjust color using sphere.getField("appearance").getField("material").getField("diffuseColor").setSFColor(...)    


random_position(supervisor)
# init_position()
print("Sending initial position")

while supervisor.step(TIME_STEP) != -1:
 
    #queue_length = rec.getQueueLength()
    
    # Now you can print or use the queue length as needed
    #print("Queue length:", queue_length)
    while rec.getQueueLength() == 0:
        #print("Waiting for packet...")
        
        supervisor.step(TIME_STEP)  # Step forward in the simulation
    
 
    
    if rec.getQueueLength() > 0:
        # print("Packet received!", rec.getQueueLength())
        if mode <= 2: # PSO modes
            rec_pos = receive_position()
            send_position(sp.gPos)
            
        else: # Hybrid modes
            rec_phe = receive_pheromones(rec, pheromone_grid)
            pheromone_grid = np.minimum(np.add(pheromone_grid, rec_phe), max_phe)

            ite += 1
        rec.nextPacket()
    if ite == num_robots:
        ite = 0
        if mode > 2:
            end_iteration(ite)
           
    
   
    
       
   
              
      
