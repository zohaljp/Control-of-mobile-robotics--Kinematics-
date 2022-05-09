from controller import Robot, Motor, DistanceSensor, PositionSensor, DistanceSensor, InertialUnit
import math
robot = Robot()
X1=20 #amount of time for rotation1
X2=30#amount of time for rotation 2
R1=0.3
R2=0.5
timestep = 64
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
C1=6.28*R1
C2=6.28*R2
angular_velocity1=6.28/X1
Vl1=angular_velocity1*(R1+0.5*distance_wheels)
VR1=angular_velocity1*(R1-0.5*distance_wheels)
V1=(Vl1+VR1)/2

angular_velocity2=6.28/X2
Vl2=angular_velocity2*(R2+0.5*distance_wheels)
VR2=angular_velocity2*(R2-0.5*distance_wheels)
V2=(Vl2+VR2)/2

start_time=robot.getTime()
while robot.step(timestep) != -1:
    currentTime=robot.getTime()+start_time
    if Vl1/wheel_radius>6.28 or VR1/wheel_radius>6.28 or  Vl2/wheel_radius>6.28 or VR2/wheel_radius>6.28:
        print("not possible")
        break
    
    if Vl1<=6.28 and VR1 <=6.28 and currentTime<= (6.28*R1)/V1:
        
        
        rightMotor.setVelocity(Vl1/wheel_radius)
        leftMotor.setVelocity(VR1/wheel_radius)
        postion_sensorleft=leftposition_sensor.getValue()
        position_sensorright=rightposition_sensor.getValue()
        dist_leftsensor=postion_sensorleft * encodervalue
        dist_rightsensor=position_sensorright* encodervalue
        print("time elapsed " + str(currentTime))
        print("left distance"+str(dist_leftsensor))
        print("right distance"+str(dist_rightsensor))
        print("------")
        
        
        
    if currentTime> (6.28*R1)/V1 + start_time:
    
        rightMotor.setVelocity(VR2/wheel_radius)
        leftMotor.setVelocity(Vl2/wheel_radius)
        postion_sensorleft=leftposition_sensor.getValue()
        position_sensorright=rightposition_sensor.getValue()
        dist_leftsensor=postion_sensorleft * encodervalue
        dist_rightsensor=position_sensorright* encodervalue
        print("time elapsed " + str(currentTime))
        print("left distance"+str(dist_leftsensor))
        print("right distance"+str(dist_rightsensor))
        print("------")
    
    if currentTime>(6.28*R1)/V1+ start_time+(6.28*R2)/V2:
        rightMotor.setVelocity(0)
        leftMotor.setVelocity(0)
        break
    
        
     
        
        
        
    
        
        
            
        
            
         
        
        
            
       
          
            
            
                
            
        
        
        
           
            
         
    
    pass