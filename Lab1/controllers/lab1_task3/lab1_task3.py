from controller import Robot, Motor, DistanceSensor, PositionSensor, DistanceSensor, InertialUnit
import math
robot = Robot()
X=10 #amount of time for rotation
H=0.25 # height
W=0.5 #width
a=1.91986
b=1.22173# angles 1 
timestep = 64
max_speed=2
leftMotor=robot.getDevice('left wheel motor')
rightMotor=robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)

rightMotor.setPosition(float('inf'))
rightMotor.setVelocity(0)
leftposition_sensor=robot.getDevice('left wheel sensor')
rightposition_sensor=robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)
imu=robot.getDevice('inertial unit')
imu.enable(timestep)
wheel_radius=0.0205 # this is in m since we know the dimmesions of our robot
wheel_circum=2*wheel_radius*3.14
encodervalue=wheel_circum/6.28
distance_wheels=0.053
angle1=1.57
angle2=b
angle3=a
angle4=a
angle5=b
start_time=robot.getTime()
linear_velocity=max_speed*wheel_radius
duration_motion1=(0.5*H)/linear_velocity
duration_motion2=(0.5*W)/linear_velocity
duration_motion3=math.sin(b)*H/linear_velocity
duration_motion4=W/linear_velocity
duration_motion5=(math.sin(b)*H)/linear_velocity
duration_motion6=W*0.5/linear_velocity
rate_rotation=(2*linear_velocity)/distance_wheels
duration_turn1=angle1/rate_rotation
duration_turn2=angle2/rate_rotation
duration_turn3=angle3/rate_rotation
duration_turn4=angle4/rate_rotation
duration_turn5=angle5/rate_rotation

start_time=robot.getTime()
rot_startTime1=start_time+duration_motion1
rot_endTime1=rot_startTime1+duration_turn1


rot_startTime2=rot_endTime1+duration_motion2
rot_endTime2=rot_startTime2+duration_turn2

rot_startTime3=rot_endTime2+duration_motion3
rot_endTime3=rot_startTime3+duration_turn3

rot_startTime4=rot_endTime3+duration_motion4
rot_endTime4=rot_startTime4+duration_turn4


rot_startTime5=rot_endTime4+duration_motion5
rot_endTime5=rot_startTime5+duration_turn5

rot_startTime6=rot_startTime5+duration_motion5

ps_value=[0,0]

last_psvalue=[0,0]
dist_value=[0,0]


robot_pose=[0,0,0]


while robot.step(timestep) != -1:
    print(str(imu.getRollPitchYaw()[2]))
    currentTime=robot.getTime()
    rightMotor.setVelocity(max_speed)
    leftMotor.setVelocity(max_speed)
    
    if rot_startTime1<currentTime<rot_endTime1:
        rightMotor.setVelocity(max_speed)
        leftMotor.setVelocity(-max_speed) 
        
    if rot_startTime2<currentTime<rot_endTime2:
        rightMotor.setVelocity(max_speed)
        leftMotor.setVelocity(-max_speed) 
        
    if rot_startTime3<currentTime<rot_endTime3:
        rightMotor.setVelocity(max_speed)
        leftMotor.setVelocity(-max_speed) 
    
    if rot_startTime4<currentTime<rot_endTime4:
        rightMotor.setVelocity(max_speed)
        leftMotor.setVelocity(-max_speed)    
        
    if rot_startTime5<currentTime<rot_endTime5:
        rightMotor.setVelocity(max_speed)
        leftMotor.setVelocity(-max_speed)
        
    if  currentTime>rot_startTime6:
        rightMotor.setVelocity(0)
        leftMotor.setVelocity(0)
        
    if a+b>3.15:
        print("impossible amigo")
        break
        
        
            
        
    
    
    
       
        
     
    
    pass