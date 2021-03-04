#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ROS_INFO_STREAM("Driving robot to the specified direction");

    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;

    srv.request.linear_x  = lin_x;
    srv.request.angular_z = ang_z;

    // Call command_robot service and pass the requested velocities
    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");

}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    ROS_INFO("Call process image with the following params: image height: %d, image width: %d, image step: %d", 
                                                            img.height, img.width, img.step);
    
    int white_pixel = 255;
    int posPixel;
    bool whitePixelFound = false;

    // Loop through each pixel in the image and check if there's a bright white one
    for ( int it = 0; it < img.data.size(); it +=3 )
    {
       // Get the values of red, green and blue numbers of a pixel
       int redNum   = img.data[it];
       int greenNum = img.data[it + 1];
       int blueNum  = img.data[it + 2];

       // Check if the pixel is white and exit when the first white pixel was found
       if ( redNum == white_pixel && greenNum == white_pixel && blueNum == white_pixel )
       {
	   posPixel = it % ( ( 3 * img.width ) ) / 3;
	   whitePixelFound = true;
           break;
       }
    }

    // Request a stop when there's no white ball seen by the camera and exit the function
    if ( !whitePixelFound  )
    {
       drive_robot( 0.0, 0.0 );
       return;
    }
	
    if ( posPixel <  img.width / 3 )
    {
       // Drive left 
       drive_robot( 0.05, 0.1 );
    }
    else if ( posPixel >= 2 * ( img.width / 3 ) )
    {
       // Drive right 
       drive_robot( 0.05, -0.1 );
    }
    else
    {
       // Go straight
       drive_robot( 0.05, 0.0 );
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
