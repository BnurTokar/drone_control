"""drone-robot-controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import numpy as np

def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)

def run_robot(robot):  
    # create the Robot instance.
    
    # get the time step of the current world.
    time_step = int(robot.getBasicTimeStep())
    
    # Constants, empirically found.
    K_VERTICAL_THRUST = 68.5  # with this thrust, the drone lifts.
    # Vertical offset where the robot actually targets to stabilize itself.
    K_VERTICAL_OFFSET = 0.6
    K_VERTICAL_P = 3.0        # P constant of the vertical PID.
    K_ROLL_P = 50.0           # P constant of the roll PID.
    K_PITCH_P = 30.0          # P constant of the pitch PID.
    
    
    # Mavic 2 Pro kamerasını etkinleştir
    camera = robot.getDevice("camera")
    camera.enable(time_step)
    imu = robot.getDevice("inertial unit")
    imu.enable(time_step)
    gps = robot.getDevice("gps")
    gps.enable(time_step)
    gyro = robot.getDevice("gyro")
    gyro.enable(time_step)
    compass = robot.getDevice("compass")
    compass.enable(time_step)

    camera_pitch_motor = robot.getDevice("camera pitch")
    camera_pitch_motor.setPosition(0.7)
    
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
    while robot.step(time_step) != -1:
        
        roll, pitch, yaw = imu.getRollPitchYaw()
        x_pos, y_pos, altitude = gps.getValues()
        roll_acceleration, pitch_acceleration, _ = gyro.getValues()
        
        print("x: {}, y: {}".format(roll, pitch))

        roll_disturbance = 0
        pitch_disturbance = 0
        yaw_disturbance = 0
        target_altitude = 0.5

        roll_input = K_ROLL_P * clamp(roll, -1, 1) + roll_acceleration + roll_disturbance
        pitch_input = K_PITCH_P * clamp(pitch, -1, 1) + pitch_acceleration + pitch_disturbance
        yaw_input = yaw_disturbance 
        
        clamped_difference_altitude = clamp(target_altitude - altitude + K_VERTICAL_OFFSET, -1, 1)
        vertical_input = K_VERTICAL_P * pow(clamped_difference_altitude, 3.0)

        front_left_motor_input = K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
        front_right_motor_input = K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
        rear_left_motor_input = K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input
        rear_right_motor_input = K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input

        front_left_motor.setVelocity(front_left_motor_input)
        front_right_motor.setVelocity(-front_right_motor_input)
        rear_left_motor.setVelocity(-rear_left_motor_input)
        rear_right_motor.setVelocity(rear_right_motor_input)



if __name__ == "__main__":

    # create the Robot instance.
    my_robot = Robot()
    run_robot(my_robot)
  