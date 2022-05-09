from controller import Robot, Motor, DistanceSensor, InertialUnit


robot = Robot()
timestep = int(robot.getBasicTimeStep())
max_speed=6.28
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

Y=0.3#distance to be travelled
X=3 #time for travel 
wheel_radius=0.0205 # this is in m since we know the dimmesions of our robot
linear_velocity=max_speed*wheel_radius
duration_motion=Y/linear_velocity
wheel_circum=2*wheel_radius*3.14
encodervalue=wheel_circum/6.28
file = open("data_task1.txt", "w")



start_time=robot.getTime()

while robot.step(timestep) != -1:
    
    
    if  duration_motion>X:
        print("They laws of Physics disagree")
        break
    
    
    currentTime=robot.getTime()
    if currentTime< start_time + duration_motion:
        postion_sensorleft=leftposition_sensor.getValue()
        position_sensorright=rightposition_sensor.getValue()
        dist_leftsensor=postion_sensorleft * encodervalue
        dist_rightsensor=position_sensorright* encodervalue
        print("left postion sensor "+str(postion_sensorleft))
        print("right postion sensor "+ str(position_sensorright))
        print("--------")
        print("left distance "+ str(dist_leftsensor))
        print("right distance "+ str(dist_rightsensor))
        print("time elapsed " + str(currentTime))
        rightMotor.setVelocity(max_speed)
        leftMotor.setVelocity(max_speed)
        
        file.write(str(dist_rightsensor) +"\n")

        
     
    
    
    if currentTime>start_time + duration_motion:
        rightMotor.setVelocity(0)
        leftMotor.setVelocity(0)
        file.close()
        break
    
     
    
    pass