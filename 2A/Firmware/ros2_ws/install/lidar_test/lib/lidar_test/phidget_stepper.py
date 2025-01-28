# Includes
from Phidget22.Devices.Stepper import *
from Phidget22.Phidget import *
from lidar_vl53l1x import *
from robot_config import *
import numpy as np
import time

wheel_perimeter = 2*np.pi*WHEEL_RADIUS

def stepper_init(serial_number: int, hub_port: int, rescale_factor=1/3200) -> Stepper:
	"""
	Initialization of a Phidgets Stepper

	@param serial_number: The serial number of the Phidget Hub
	@param hub_port: The port hub of the stepper on the Phidget Hub
	"""
	stepper = Stepper()
	stepper.setDeviceSerialNumber(serial_number)
	stepper.setHubPort(hub_port)
	stepper.openWaitForAttachment(5000)
	stepper.setRescaleFactor(rescale_factor)
	stepper.setControlMode(StepperControlMode.CONTROL_MODE_STEP)
	stepper.setCurrentLimit(3)
	stepper.setEngaged(True)

	return stepper

def move_forward(stepper_left: Stepper, stepper_right: Stepper, lidar_data: LidarVL53L1X, serial: Serial, speed: float, distance: float):
    """
    A function to make the robot move forward by a distance goal. 
    Providing a negative distance will make it run backward.

    @param stepper_left: The stepper left
    @param stepper_right: The stepper right
    @param lidar_data: The structure containing the lidar distance data
    @param serial: The serial communication used for the lidar
    @param speed: The speed of the robot
    @param distance: The distance goal given to the robot
    """
    speed_factor = 1.0
    step = distance/wheel_perimeter # Distance to step conversion 
    stepper_left.setTargetPosition(step)
    stepper_right.setTargetPosition(-step)
    while abs(distance/wheel_perimeter - stepper_left.getPosition()) >= 1e-2:
        # Obstacle detection 
        lidar_data.rx_storage = serial.read(LIDAR_FRAME_SIZE * 2)
        processed_data = get_lidar_data(lidar_data)
        distances = [distance[1] for distance in processed_data.measure]
        min_distance = min(distances)
        min_distance_index = distances.index(min_distance) 
        if min_distance_index in [0, 1, 7, 8, 9, 10, 14, 15]: # Lower the detection threshold sideways
            threshold = 250
        else:
            threshold = 650
        if 1 <= min_distance <= threshold:
            speed_factor = 0.0
        elif min_distance == 0 or min_distance >= 1000: # Not optimal
            speed_factor = speed_factor
        elif threshold <= min_distance <=  1000:
            speed_factor = 1-np.exp(-(min_distance-threshold)/40) # First order gradient speed

        stepper_left.setVelocityLimit(speed * speed_factor)
        stepper_right.setVelocityLimit(speed * speed_factor)

    # Reinitializing position to 0
    stepper_left.addPositionOffset(-stepper_left.getPosition())
    stepper_right.addPositionOffset(-stepper_right.getPosition())
    stepper_left.setVelocityLimit(0)
    stepper_right.setVelocityLimit(0)
    return


def rotate_left(stepper_left: Stepper, stepper_right: Stepper, speed: float, theta: float):
	"""
    A function to make the robot rotate left by a angle goal. 
    Providing a negative angle will make it turn right.

    @param stepper_left: The stepper left
    @param stepper_right: The stepper right
    @param speed: The speed of the robot
    @param distance: The angle goal given to the robot
    """
	speed_factor = 1.0
	stepper_left.setVelocityLimit(speed*speed_factor)
	stepper_right.setVelocityLimit(speed*speed_factor)
	distance = np.pi*WHEEL_ENTRAXE*theta/360 # Angle to distance conversion 
	step = distance/wheel_perimeter
	stepper_left.setTargetPosition(-step)
	stepper_right.setTargetPosition(-step)
	while abs(distance/wheel_perimeter - stepper_left.getPosition()) >= 5e-1:
        # Debug
		print(f"Left {stepper_left.getPosition()}, Right: {stepper_right.getPosition()}")
		print(abs(distance/wheel_perimeter - stepper_left.getPosition()))
	time.sleep(1)
	stepper_left.addPositionOffset(-stepper_left.getPosition())
	stepper_right.addPositionOffset(-stepper_right.getPosition())
	return
