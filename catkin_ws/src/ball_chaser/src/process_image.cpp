#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    
    if(!client.call(srv))
        ROS_ERROR("Failed to call service drive_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    float white_skewedness = 0; 
    float number_of_white_pixels = 0; 
    bool do_you_see_white_ball = false; 
    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    for(int row = 0; row < img.height; row++) {
        int row_offset = row * img.step;
        for(int col = 0; col < img.step; col++) {
            int index = row_offset + col;
            if(img.data[index] == white_pixel){
                do_you_see_white_ball = true;
                number_of_white_pixels++;
                white_skewedness += (col-(int)img.step/2)/100;
            }
        }
    }
    //ROS_INFO_STREAM(std::to_string(number_of_white_pixels) + ", " + std::to_string(white_skewedness) + ", " + std::to_string(white_skewedness / number_of_white_pixels));
    if(number_of_white_pixels == 0){
	    drive_robot(0.0,0.0); 
    }
    else {
        drive_robot(1, -1*(white_skewedness / number_of_white_pixels));
        std::string msg_feedback = "wheel velocities set - linear_x: " + std::to_string(5000/number_of_white_pixels) + " , angular_z: " + std::to_string(-0.3*(white_skewedness / number_of_white_pixels));
        ROS_INFO_STREAM(msg_feedback);
    }
    

    return;
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
