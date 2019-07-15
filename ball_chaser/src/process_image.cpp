#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

ros::ServiceClient client;

void drive_robot(float lin_x, float ang_z)
{
	ball_chaser::DriveToTarget srv;

	srv.request.linear_x = lin_x;
	srv.request.angular_z = ang_z;

	if(!client.call(srv))
		ROS_ERROR("Failed to call service command_robot");
}

void process_image_callback(const sensor_msgs::Image img)
{
	int white_pixel = 255;

	int left = 0, forward = 0, right = 0; 
	int leftRange = img.width/3;
	int rightRange = img.width*2/3;

	    for(int i=0;i<img.height*img.width*3;i += 3)
	    {
		// If image pixel is white
		if(img.data[i]==white_pixel && img.data[i+1]==white_pixel && img.data[i+2]==white_pixel)
		{
		    int position = i % (img.width*3) / 3;
		    if(position<leftRange)
		        left++;
		    else if(position>=rightRange)
		        right++;
		    else if(position>=leftRange && position<rightRange)
		        forward++;
		}
	    }

	if(left + forward + right == 0) drive_robot(0.0, 0.0);
	else if(left > forward)
	{
		if(left > right)
			drive_robot(0, 1.0);
		else
			drive_robot(0, -1.0);
	}else if(left < forward)
	{
		if(forward > right)
			drive_robot(1.0, 0.0);
		else
			drive_robot(0, -1.0);
	}


}

int main(int argc, char** argv)
{

	ros::init(argc, argv, "process_image");
	ros::NodeHandle n;

	client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

	ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

	ros::spin();

	return 0;
}







