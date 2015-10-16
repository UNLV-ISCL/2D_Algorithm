#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

class Control{
	public:
		Control()
		{
		 detSub = nh.subscribe("det_out", 1, &Control::ReceiveDet, this); 
		 pub = nh.advertise<std_msgs::Float32MultiArray>("move_zumo", 1);
		}

		void ReceiveDet(std_msgs::Float32MultiArray det);

	private:
		ros::NodeHandle nh;
		ros::Subscriber detSub;
		ros::Publisher pub;

};
	

void Control::ReceiveDet(std_msgs::Float32MultiArray det){
	std_msgs::Float32MultiArray out;
	out.data.push_back( det.data[0]);
	out.data.push_back( det.data[1]);
        pub.publish(out);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "Zumo");
	ROS_INFO_STREAM("Zumo Controller Active!");


	Control control;
	
	ros::Rate rate(20);
	while(ros::ok()){
		ros::spinOnce();		
		rate.sleep();
	}
}
