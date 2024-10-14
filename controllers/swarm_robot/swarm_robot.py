from controller import Robot
import math, time
import random
from fnc import rec_init


mode = 1

if mode == 0:
    from gpso_offline import globalPSO_off as algo
elif mode == 1:
    from global_pso import globalPSO as algo 
elif mode == 2:
    from local_pso import localPSO as algo
elif mode == 3:
    from local_hybrid import localHybrid as algo


timestep = 64 
dim = 2
delaysec = 1
delay = int(delaysec * 1000/timestep)

class rob:
    robot = Robot()
    name = robot.getName()
    ind = int(name[1]) 
    emi = robot.getDevice('emitter')
    rec = robot.getDevice('receiver')
    rec.setChannel(ind)
 
class con:
    max_iterations = 1000
    num_robots = 4
    size = 5
    # obj = [[-2.5,0], [2.5,0], [0,-2.5], [0,2.5] ]
    obj = [[-1,0], [1,0], [0,-1], [0,1] ]
    # obj = [[0,0]]
    multi = 1 if len(obj)>1 else 0
  

def send_config():
    data = [mode, con.num_robots, con.max_iterations, con.multi, con.obj, con.size]
    data_str = ';'.join(str(i) for i in data)    
    # Set the receiver channel for the supervisor
    rob.emi.setChannel(99)
    
    # Send the pheromone grid packet to the supervisor
    rob.emi.send(data_str.encode('utf-8')) 


# offline_gPos()
if __name__ == "__main__":
    
    rob.rec.enable(timestep)
    rob.rec.setChannel(rob.ind)

    # Sends the data to supervisor once
    if rob.ind == 0:
        send_config()

    while True:
        for _ in range(delay):
            rob.robot.step(timestep)
        if rob.rec.getQueueLength() > 0:
            rec_init(rob.rec)
            rob.rec.nextPacket()
            break

    # while True:
    # print(rob.rec.getQueueLength())
    # for _ in range(delay):
    #     robot.step(timestep)
    # if rob.rec.getQueueLength() > 0:
    #     receive_position()
    #     break

    algo(con, rob)