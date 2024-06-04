"""drive_robot controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math
import sys
from my_controller import pan

print ("hello" + p1)

def run_robot(robot):
 
    # get the time step of the current world.
    timestep = 64 
    max_speed = 6.28 # #Angular velocity
    name = robot.getName()
   
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
    ps_values = [0, 0]
    dist_values = [0, 0]
    
    # GPS
  
    gps = robot.getDevice('gps')

    gps.enable(timestep)
    
    if name == "r1":
        p1 = gps
        print("Initializing P1")
    else: p1 = 0
    
    if name == "r2":
        p2 = gps
        print("Initializing P2")
    else: p2 = 0
    
    if name == "r3":
        p3 = gps
        print("Initializing P3")
    else: p3 = 0
    
   
    for i in range(0, len(sys.argv)):
           print("argv[%i]=%s" % (i, sys.argv[i]))   
    
    #Compute encoder unit
    radius = 0.03
    wheeldist = 0.1
    circum = 2 * 3.14 * radius
    encoderunit = circum/6.28
    
    #Robot pose
    robot_pose = [0, 0 ,0]  #x,y, theta
    ps0_values = [0, 0]
    checkpoint = 0

    while robot.step(timestep) != -1:
        
        #GPS sensor
        gps_value = gps.getValues()
        msg = "GPS values:"
        for each_val in gps_value:
            msg += "{0:0.2f}  ".format(each_val)
        print(msg)
        
       
        # print(gps_value[0], gps_value[1], gps_value[2])
     
        
        
        
        
        
        # Read value from position sensor
        ps_values[0] = left_ps.getValue()
        ps_values[1] = right_ps.getValue()
      
        left_motor.setVelocity(max_speed)
        right_motor.setVelocity(max_speed)   
        
        print("---------")
        # print("position sensor values: {} {}".format(ps_values[0],ps_values[1]))
       
        # for ind in range(2): 
            # diff = ps_values[ind] - ps0_values[ind]
            # if diff < 0.001:
                # diff = 0
                # ps_values[ind] = ps0_values[ind]
            # dist_values[ind] = diff * encoderunit
       
       #Computer linear and angular velocity
        # v = (dist_values[0] + dist_values[1])/2.0
        # w = (dist_values[0] - dist_values[1])/wheeldist
       
        # dt = 1
        # robot_pose[2] += (w * dt)
       
        # vx = v * math.cos(robot_pose[2])
        # vy = v * math.sin(robot_pose[2])
        # robot_pose[0] += (vx * dt)
        # robot_pose[1] += (vy * dt)
        
     
    
        # print("robot_pose: {}".format(robot_pose))
        
       
        # if ps_values[0] < 5:
            # checkpoint = 0
        # if ps_values[0] >= 10 and ps_values[0] <= 20:
            # checkpoint = 1
        # if ps_values[0] >= 20 and ps_values[0] <= 30:
            # checkpoint = 2
        # if ps_values[0] >= 30 and ps_values[0] <= 40:
            # checkpoint = 3
        # if ps_values[0] >= 40 and ps_values[0] >= 5:
            # checkpoint = 4
           
        # print( "Checkpoint = {}".format(checkpoint))
        
      
      
      
        # for ind in range(2):
            # ps0_values[ind] = ps_values[ind]
        
        # if (checkpoint == 0):
            # left_motor.setVelocity(max_speed)
            # right_motor.setVelocity(max_speed)
        # elif (checkpoint == 1):
            # left_motor.setVelocity(0.5 * max_speed)
            # right_motor.setVelocity(0.5 * max_speed)
        # elif (checkpoint == 2):
            # left_motor.setVelocity(1.5* max_speed)
            # right_motor.setVelocity(1.5 * max_speed)
        # elif (checkpoint == 3):
            # left_motor.setVelocity(0.5 * max_speed)
            # right_motor.setVelocity(0.5 * max_speed)
        # elif (checkpoint == 4):
            # left_motor.setVelocity(-2*max_speed)
            # right_motor.setVelocity(-2*max_speed)
       
        
   
   
   
   
   
if __name__== "__main__":

    # create the Robot instance.
    robot = Robot()
    run_robot(robot)
   
    