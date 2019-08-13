#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <algorithm>
#include <vector>

using namespace std;

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

	int leftRange = img.width/3;
	int rightRange = (img.width/3)*2;
	int step;

	int lcount = 0, rcount = 0, fcount = 0;

	for(int pixel=0; pixel < img.height * img.step; pixel+=3)
	{

		if(img.data[pixel] == white_pixel && img.data[pixel+1] == white_pixel && img.data[pixel+2] == white_pixel)
		{
			int position = pixel % (img.width * 3) / 3;
			if(position < leftRange)
				lcount++;
			else if(position >= leftRange && position < rightRange)
				fcount++;
			else if(position >= rightRange)
				rcount++;
		}

	}

	vector <int> counts{lcount, rcount, fcount};
	int move = *max_element(counts.begin(), counts.end());

	if(move == 0)
		drive_robot(0.0, 0.0);
	else if (move == lcount)
		drive_robot(0.0, 0.2);
	else if (move == rcount)
		drive_robot(0.0, -0.2);
	else if (move == fcount)
		drive_robot(1.0, 0.0);
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







