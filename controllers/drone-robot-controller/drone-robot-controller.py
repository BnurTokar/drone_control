"""drone-robot-controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot


def run_robot(robot):  
    # create the Robot instance.
    
    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    
    
    front_left_motor = robot.getDevice("front left propeller")
    front_right_motor = robot.getDevice("front right propeller")
    rear_left_motor = robot.getDevice("rear left propeller")
    rear_right_motor = robot.getDevice("rear right propeller")
    
    
    motors = [front_left_motor, front_right_motor, rear_left_motor, rear_right_motor]
    for motor in motors:
        motor.setPosition(float('inf'))
        motor.setVelocity(1)
    
    
    
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        # Read the sensors:
        # Enter here functions to read sensor data, like:
        #  val = ds.getValue()
    
        # Process sensor data here.
    
        # Enter here functions to send actuator commands, like:
        #  motor.setPosition(10.0)
        pass




if __name__ == "__main__":

    # create the Robot instance.
    my_robot = Robot()
    run_robot(my_robot)
  