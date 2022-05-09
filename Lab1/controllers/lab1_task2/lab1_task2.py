from controller import Robot, Motor, DistanceSensor, PositionSensor, DistanceSensor, InertialUnit
import math
robot = Robot()
X=10 #amount of time for rotation
Y=6 # rotation angle in radians
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
ps_value=[0,0]
last_psvalue=[0,0]
dist_value=[0,0]
file = open("data_task2.txt", "w")

robot_pose=[0,0,0]
distance_wheels=0.053
Vl=max_speed*wheel_radius
VR=-max_speed*wheel_radius
angular_velocity=(Vl-VR)/distance_wheels

start_time=robot.getTime()

while robot.step(timestep) != -1:
    currentTime=robot.getTime()
    if Y<=angular_velocity*X:
        
    
        
    
        ps_value[0]=leftposition_sensor.getValue()
        ps_value[1]=rightposition_sensor.getValue()
        
        for ind in range(2):
            diff=ps_value[ind]-last_psvalue[ind]
            dist_value[ind]=diff*encodervalue
        
        
        v=(dist_value[0]+dist_value[1])/2.0
        w=(dist_value[0]-dist_value[1])/distance_wheels # angular velocity
        dt=1 #change in time
        robot_pose[2]+= (w * dt)
        vx=v*math.cos(robot_pose[2])
        vy=v*math.sin(robot_pose[2])
        robot_pose[0]+=vx*dt
        robot_pose[1]+=(vy*dt)
        print(" robot_pose: {}".format(robot_pose))
        rightMotor.setVelocity(-max_speed)
        leftMotor.setVelocity(max_speed)
        print(str(imu.getRollPitchYaw()[2]))
        print("time elapsed " + str(currentTime))
        print(w)
        print("------")
        file.write(str(robot_pose[2]) +"\n")
        
        for ind in range(2):
           last_psvalue[ind]=ps_value[ind]
           
    if Y>angular_velocity*X:
         print("Operation impossible, Laws of physics disagree")
         break
    if  currentTime<=X+start_time and robot_pose[2]>Y :
        rightMotor.setVelocity(0)
        leftMotor.setVelocity(0) 
        file.close()
        print("operation success")
        break  
           
    
    
       
        
     
    
    pass
