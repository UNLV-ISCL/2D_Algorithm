#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <algorithm>
#include <math.h>
#include <iostream>
#include <fstream>

#define PI 3.1415926
using namespace std;

class Planner{
	public:
		Planner();
		double userinput;
		//int interrupt;
	private:
		ros::NodeHandle nh_;
		ros::Publisher pub_;
		ros::Publisher testpub;
    ros::Publisher subdirections_pub;
    ros::Publisher boundrypub;                          
		ros::Subscriber sub_laser;
		ros::Subscriber sub_rc;
		void scanVal(const sensor_msgs::LaserScan::ConstPtr& laser);
		void receiveDirect(const std_msgs::Float32  rc);
};
	
  Planner::Planner(){
	sub_laser = nh_.subscribe<sensor_msgs::LaserScan>("scan", 1, &Planner::scanVal, this);
	sub_rc = nh_.subscribe<std_msgs::Float32>("direction", 1, &Planner::receiveDirect, this);
	pub_ = nh_.advertise<std_msgs::Float32>("det_out", 1);
	testpub = nh_.advertise<std_msgs::Float32>("debug_test", 1);
  subdirections_pub=nh_.advertise<std_msgs::Float64MultiArray>("debug_subdirections", 1);
  boundrypub=nh_.advertise<std_msgs::Int32MultiArray>("debug_valleybou", 1);

}

void Planner::receiveDirect(std_msgs::Float32 rc){

	userinput=rc.data;

}

void Planner::scanVal(const sensor_msgs::LaserScan::ConstPtr& laser){

      std_msgs::Float32 det;

//test messgas define
      std_msgs::Float32 r;
      std_msgs::Float64MultiArray sub_directions;
      std_msgs::Int32MultiArray valleybou;

	      int n = laser -> ranges.size();
        float unit=laser -> angle_increment*(180/PI);

        int count=0;  
        float value;
        float threshold =2.0f;//the obstacle distance that car still can avoid
        double vehicle_size=0.6;
        vector<int> det_safe;
         
        double theta =acos(1-pow(vehicle_size,2.0)/(2*pow(threshold,2.0)))*180.0/PI;
        float limit=round(4*theta/unit);//number of vectors in a minimum safe valley

//take care of "nan" data and normalize range 
	      bool stuck=0;
        for (int i=0; i<n; i++){
            value= laser -> ranges[i];

            if (!isnan(value)){

               if (value >= threshold){
                  det_safe.push_back(1);//safe==1
               }
               else{
                  det_safe.push_back(0);//unsafe==0
               }

	       if (value <=0.8){
		         stuck=1;
	       }

             }
             else{   
                 det_safe.push_back(1);
             }
                 
        }  
       
        //count beams in safe valleys
		    vector<double> valley;
        valleybou.data.clear();
        for (int j=0; j< n; j++){
             
             if ((det_safe[j] == 0) || (j== n-1)   ){

                  if (det_safe[j] ==1){
                      count++;     
                  }         
                  if (!(count ==0) && count >= limit ){
                      
					           double right_bound = (j - count)*unit+limit*unit/2-30;
                      if (right_bound <0)
                      {
                        right_bound=right_bound+360;
                      }
                      
					            double left_bound = j*unit-limit*unit/2-30;
                      if (left_bound <0)
                      {
                        left_bound=left_bound+360;
                      }   
                      
                      if (right_bound>left_bound)
                      {
                        valley.push_back(right_bound);
                        valley.push_back(360);
                        valley.push_back(0);
                        valley.push_back(left_bound); 
                      }
                       
                      else 
                      {
                        valley.push_back(right_bound);
					              valley.push_back(left_bound);
                      }
					           
					            valleybou.data.push_back(j - count);
					            valleybou.data.push_back(j);                     

                  }
                  count=0; 
                  continue;

              }
              else{
                  count++;
              }
                  
        }

		bool inside = 0;
		for (int m = 0; m < valley.size(); m+=2){

      sub_directions.data.push_back(valley[m]);
      sub_directions.data.push_back(valley[m+1]);

			if (userinput >= valley[m] && userinput <= valley[m+1]){

			    inside = 1;
			}
		}

		if (inside){
			det.data=userinput;
		}
		else
		{
      if (userinput >=210.0 && userinput<=330.0)
      {
          det.data=userinput;
      }
      else
      {
			        double delta1, delta2;
			        vector<double> delta_vector;
			        for (int k= 0; k < valley.size(); k++){
				          delta1 = abs(valley[k] - userinput);
                  delta2= valley[k]+360.0-userinput;
                  if (delta1 >delta2){
				            delta_vector.push_back(delta2);
                  }
                  else{
				            delta_vector.push_back(delta1);                     
                  }

			        }

			        //choose the subfirection wihch closest to direction
			        int min_index = min_element(delta_vector.begin(), delta_vector.end()) - delta_vector.begin();

			        if (valley.size() == 0 ){
				         det.data=270.0;//no safe valley to go, car rotate to find another direction 

			        }

			        else{
			         	 det.data=valley[min_index];

			        }
      }
		}

        r.data=valley.size();
        boundrypub.publish(valleybou);
	      testpub.publish(r);
        subdirections_pub.publish(sub_directions);
	      pub_.publish(det);

}

int main(int argc, char **argv){
	ros::init(argc, argv, "algorithm_node");
	ROS_INFO_STREAM("Algorithm node active!");
	Planner planner;

	ros::spin();
	return 0;
}
