Project Description: This project controls a two-wheeled robot with an ultrasonic sensor mounted on a servo motor for obstacle detection and avoidance. The ultrasonic sensor rotates on the servo to scan for obstacles in different directions (left, center, right), and the robot adjusts its movement accordingly.

Components:
Arduino: Main controller for the robot.
Ultrasonic Sensor (HC-SR04): Measures distance to detect obstacles.
Servo Motor: Rotates the ultrasonic sensor to scan the environment.
DC Motors: Controls the movement of the robot.
Motor Driver: Drives the DC motors.
Wheels: Provide movement based on motor inputs.
Code Overview:
Servo Motor Control:

The ultrasonic sensor is attached to a servo motor, which allows it to rotate and scan for obstacles in three directions: left, center, and right. The servo angles are set at:
0°: Left
90°: Center (default forward-facing position)
180°: Right
Ultrasonic Sensor:

The ultrasonic sensor measures the distance from an object by sending out sound waves and calculating the time it takes for the echo to return. This distance helps determine whether an obstacle is too close to the robot.
If an object is detected within a certain threshold (25 cm in this case), the robot will stop and scan the surroundings.
Movement Control:

Forward Movement: The robot moves forward if no obstacle is detected within the threshold.
Left/Right Turns: If an obstacle is detected in the forward direction, the servo rotates the ultrasonic sensor to check left and right. If either side is clear, the robot turns in that direction.
Backward Movement: If no path is clear in any direction, the robot will move backward and then turn to try another path.
Servo Scanning Logic:

The function scan_with_servo() handles the scanning process. It rotates the servo to three positions (left, center, right), collects distance readings, and determines which direction is clear for the robot to proceed.
The robot prefers to move forward but will turn left or right if the center is blocked.
Motor Speed Control:

Motor speeds are controlled using the analogWrite() function, with a speed value of 180 to ensure smooth operation. During turns, only one motor moves to allow the robot to rotate.
Functions:
move_forward(): Moves the robot forward by activating both motors.
move_backward(): Reverses the robot’s movement.
turn_left(): Turns the robot left by stopping the right motor.
turn_right(): Turns the robot right by stopping the left motor.
stop(): Stops all movement of the robot.
get_distance(): Measures the distance from an obstacle using the ultrasonic sensor.
scan_with_servo(): Rotates the ultrasonic sensor to scan the environment and makes decisions based on detected distances.
Workflow:
Startup:

The ultrasonic sensor starts in the center position (90°).
The robot moves forward by default until an obstacle is detected.
Obstacle Detection:

When an obstacle is detected within 25 cm, the robot stops and calls the scan_with_servo() function to scan the environment.
Servo Scanning:

The servo motor rotates the ultrasonic sensor left (0°), center (90°), and right (180°) to check for obstacles in all directions.
Decision Making:

Based on the distances measured during the scan:
If the path ahead is clear, the robot moves forward.
If there is an obstacle ahead, but the left or right side is clear, the robot turns in that direction.
If all directions are blocked, the robot moves backward and then turns to find a new path.
Continuous Movement:

The robot continuously monitors its surroundings and adjusts its movements to avoid collisions, ensuring smooth navigation in complex environments.
Example Behavior:
Case 1: The robot moves forward. If an obstacle appears within 25 cm, it stops and scans.
Case 2: The robot scans left and right. If the left side is clear, it turns left and moves forward. If the right is clear, it turns right.
Case 3: If obstacles are detected in all directions, the robot moves backward and reorients itself to find a new path.
Notes:
Servo Initialization: The servo is initialized at 90°, which points the ultrasonic sensor forward.
Adjustable Parameters:
The threshold distance (25 cm) can be changed depending on the environment.
The motor speeds can be adjusted to control the robot’s movement speed and turning radius.
