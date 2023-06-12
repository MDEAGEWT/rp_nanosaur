#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

import numpy as np
import cv2
import cv_bridge

# Create a bridge between ROS and OpenCV
bridge = cv_bridge.CvBridge()

# Robot's speed when following the line
LINEAR_SPEED = 0.21

# Proportional constant to be applied on speed when turning 
# (Multiplied by the error value)
KP = 8/1000

# If the line is completely lost, the error value shall be compensated by:
LOSS_FACTOR = 1.2 #1.5

# Send messages every $TIMER_PERIOD seconds
TIMER_PERIOD = 0.01

# When about to end the track, move for ~$FINALIZATION_PERIOD more seconds
FINALIZATION_PERIOD = 4

# BGR values to filter only the selected color range
lower_bgr_values = np.array([31,  42,  53])
upper_bgr_values = np.array([255, 255, 255])

def crop_size(height, width):
    """
    Get the measures to crop the image
    Output:
    (Height_upper_boundary, Height_lower_boundary,
     Width_left_boundary, Width_right_boundary)
    """
    ## Update these values to your liking.
    return (3*height//4, height, 0, width)


# Global vars. initial values
image_input = 0
should_move = False

def start_follower_callback(request, response):
    """
    Start the robot.
    In other words, allow it to move (again)
    """
    global should_move
    should_move = True
    return response

def stop_follower_callback(request, response):
    """
    Stop the robot
    """
    global should_move
    should_move = False
    return response

def image_callback(msg):
    """
    Function to be called whenever a new Image message arrives.
    Update the global variable 'image_input'
    """
    global image_input
    image_input = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

def get_contour_data(mask, out):
    line = {}
    sum_x = 0
    sum_y = 0
            
    # get a list of contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    # plot the area in light blue
    cv2.drawContours(out, contours, -1, (255,255,0), 1)   
    
    for contour in contours:    
        M = cv2.moments(contour)    
        
        # Contour is part of the track
        # if M["m00"] == 0:
        #     M["m00"] = 0.0000001
        line['x'] = int(M["m10"]/M["m00"])
        line['y'] = int(M["m01"]/M["m00"])
        
        sum_x += line['x']
        sum_y += line['y']
        
    line['x'] = int(sum_x / len(contours))
    line['y'] = int(sum_y / len(contours))    
    return line

def timer_callback():
    """
    Function to be called when the timer ticks.
    According to an image 'image_input', determine the speed of the robot
    so it can follow the contour
    """
    global image_input
    global should_move

    # Wait for the first image to be received
    if type(image_input) != np.ndarray:
        return

    height, width, _ = image_input.shape

    image = image_input.copy()

    # global crop_w_start
    crop_h_start, crop_h_stop, crop_w_start, crop_w_stop = crop_size(height, width)

    # get the bottom part of the image (matrix slicing)
    crop = image[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop]
    
    # get a binary picture, where non-zero values represent the line.
    # (filter the color values so only the contour is seen)
    mask = cv2.inRange(crop, lower_bgr_values, upper_bgr_values)

    # get the centroid of the biggest contour in the picture,
    # and plot its detail on the cropped part of the output image
    output = image
    line = get_contour_data(mask, output[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop])   
    # also get the side in which the track mark "is"
    
    message = Twist()
    
    # if there even is a line in the image:
    # (as the camera could not be reading any lines)       

    x = crop_w_start + line['x']

    # error:= The difference between the center of the image
    # and the center of the line
    error = x - width//2

    message.linear.x = LINEAR_SPEED

    # plot the line centroid on the image
    crop_center = (int((crop_w_start+crop_w_stop)/2), int((crop_h_start+crop_h_stop)/2))
    cv2.circle(output, (crop_w_start + line['x'], crop_h_start + line['y']), 3, (0,255,0), 3)
    cv2.circle(output, crop_center, 3, (255,0,0), 3)

    
    # Determine the speed to turn and get the line in the center of the camera.
    message.angular.z = float(error) * -KP
    
    print("Error: {} | Angular Z: {}, ".format(error, message.angular.z))
    cv2.putText(output, f'{error}:{message.angular.z}', 
                (crop_center[0], crop_center[1]-20), 
                cv2.FONT_HERSHEY_PLAIN, 2, (255,255,0), 2)
    

    # Plot the boundaries where the image was cropped
    cv2.rectangle(output, (crop_w_start, crop_h_start), (crop_w_stop, crop_h_stop), (0,0,255), 2)

    # Show the output image to the user
    cv2.imshow("output", output)
    # Print the image for 5milis, then resume execution
    cv2.waitKey(1)

    # Publish the message to 'cmd_vel' 
    # ros2 service call /start_follower std_srvs/srv/Empty
    # ros2 service call /stop_follower std_srvs/srv/Empty   
    if should_move:
        publisher.publish(message)
    else:
        empty_message = Twist()
        publisher.publish(empty_message)


def main():
    rclpy.init()
    
    # global node
    node = Node('follower')

    global publisher
    publisher = node.create_publisher(Twist, '/nanosaur/cmd_vel', rclpy.qos.qos_profile_system_default)
    subscription = node.create_subscription(Image, '/nanosaur/camera/image_raw',
                                            image_callback,
                                            rclpy.qos.qos_profile_sensor_data)

    timer = node.create_timer(TIMER_PERIOD, timer_callback)

    start_service = node.create_service(Empty, 'start_follower', start_follower_callback)
    stop_service = node.create_service(Empty, 'stop_follower', stop_follower_callback)

    rclpy.spin(node)

if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, rclpy.exceptions.ROSInterruptException):
        empty_message = Twist()
        publisher.publish(empty_message)

        node.destroy_node()
        rclpy.shutdown()
        exit()
