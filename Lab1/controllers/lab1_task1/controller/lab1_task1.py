"""lab1_task1 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

#getting the position sensors

leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)

imu = robot.getDevice('inertial unit')
imu.enable(timestep)
robot.step(timestep)

meterstoinch=.394
wheelrad= .8
wheel_circum = wheelrad


def getDistance():
    return [leftposition_sensor.getValue()*wheelrad, rightposition_sensor.getValue()*wheelrad]
# Main loop:
# - perform simulation steps until Webots is stopping the controller

distStart = getDistance()
print(distStart)
# Main loop:
# - perform simulation steps until Webots is stopping the controller

while robot.step(timestep) != -1 and getDistance()[0] - distStart[0] <10:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    leftMotor.setVelocity(6.28)
    rightMotor.setVelocity(6.28)
    print("dist traveled" + str(getDistance()[0] - distStart[0]))
    
    #print( "Left position sensor: " +str(leftposition_sensor.getValue()))
    #print("Right position sensor: " +str(rightposition_sensor.getValue()))
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
print(getDistance())
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

# Enter here exit cleanup code.
