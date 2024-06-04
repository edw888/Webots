import math
from controller import Robot, DistanceSensor, LED, Motor

# Device constants
DISTANCE_SENSORS_NUMBER = 8
GROUND_SENSORS_NUMBER = 3
LEDS_NUMBER = 10

DISTANCE_SENSORS_NAMES = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]
GROUND_SENSORS_NAMES = ["gs0", "gs1", "gs2"]
LEDS_NAMES = ["led0", "led1", "led2", "led3", "led4", "led5", "led6", "led7", "led8", "led9"]

# Breitenberg constants
weights = [[-1.3, -1.0], [-1.3, -1.0], [-0.5, 0.5], [0.0, 0.0],
           [0.0, 0.0], [0.05, -0.5], [-0.75, 0], [-0.75, 0]]
offsets = [0.5 * math.pi, 0.5 * math.pi]

# Initialize robot
robot = Robot()

# Initialize devices
distance_sensors = []
ground_sensors = []
leds = []

for i in range(DISTANCE_SENSORS_NUMBER):
    sensor = robot.getDevice(DISTANCE_SENSORS_NAMES[i])
    sensor.enable(robot.getBasicTimeStep())
    distance_sensors.append(sensor)

for i in range(GROUND_SENSORS_NUMBER):
    sensor = robot.getDevice(GROUND_SENSORS_NAMES[i])
    sensor.enable(robot.getBasicTimeStep())
    ground_sensors.append(sensor)

for i in range(LEDS_NUMBER):
    led = robot.getDevice(LEDS_NAMES[i])
    leds.append(led)

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Helper functions
def get_sensor_input():
    distance_sensor_values = []
    for sensor in distance_sensors:
        distance_sensor_values.append(sensor.getValue() / 4096)
    
    ground_sensor_values = []
    for sensor in ground_sensors:
        ground_sensor_values.append(sensor.getValue())
    
    return distance_sensor_values, ground_sensor_values

def set_actuators(speeds, leds_values):
    for i, led in enumerate(leds):
        led.set(leds_values[i])
    
    left_motor.setVelocity(speeds[0])
    right_motor.setVelocity(speeds[1])

def blink_leds():
    counter = int(robot.getTime() * 1000) // 100
    for i, led in enumerate(leds):
        led.set(counter % LEDS_NUMBER == i)

def run_braitenberg(distance_sensor_values):
    speeds = [0.0, 0.0]
    for i in range(DISTANCE_SENSORS_NUMBER):
        for j in range(2):
            speeds[j] += distance_sensor_values[i] * weights[i][j]

    speeds = [offsets[j] + speeds[j] * math.pi for j in range(2)]
    speeds = [max(-math.pi, min(speed, math.pi)) for speed in speeds]
    
    return speeds

def go_backwards():
    left_motor.setVelocity(-math.pi)
    right_motor.setVelocity(-math.pi)
    robot.step(200)

def turn_left():
    left_motor.setVelocity(-math.pi)
    right_motor.setVelocity(math.pi)
    robot.step(200)

# Main loop
while robot.step() != -1:
    distance_sensor_values, ground_sensor_values = get_sensor_input()
    blink_leds()
    
    if any(value < 500 for value in ground_sensor_values):
        go_backwards()
        turn_left()
    else:
        speeds = run_braitenberg(distance_sensor_values)
    
    set_actuators(speeds, [True] * LEDS_NUMBER)
