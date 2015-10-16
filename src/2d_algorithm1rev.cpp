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
    std_msgs::Float32 det;                            //initialize det
    std_msgs::Float32 r;                              //initialize r
    std_msgs::Float64MultiArray sub_directions;       //initialize sub_directions
    std_msgs::Int32MultiArray valleybou;              //initialize valleybou
	  vector<int> det_safe;                             //safe direction vector
    vector<double> valley;
    float value;                                      //init value
    //bool stuck=0;                                   //init stuck

    int count=0;                                      //init count
    float scale_safe_valley = 4;                      //scale the safe vector
    float threshold =2.0f;                            //the obstacle distance that car still can avoid
    double vehicle_size=0.6;                          //width of vehicle in (m)
    

    int n = laser -> ranges.size();                   //take size of /scan
    float unit=laser -> angle_increment*(180/PI);     //convert angle increment to degrees
  
    double theta =acos(1-pow(vehicle_size,2.0)/(2*pow(threshold,2.0)))*180.0/PI;      //min width of safe direction
    float limit=round(scale_safe_valley*theta/unit);                                  //number of vectors in a minimum safe valley

    //take care of "nan" data and normalize range 

    
                                
    for (int i=0; i<n; i++){
        value= laser -> ranges[i];                    //take ith range value in for loop
        if (!isnan(value)){                           //if ith value is not nan ->
            if (value >= threshold){                  //if ith value is not nan and if the value is >= greater than threshold
                det_safe.push_back(1);                //index ith det_safe  to "1"
            }
            else{
                det_safe.push_back(0);                //if ith value is  notnan and if the value is < threshold index ith det_safe to "0"
            }
	          //if (value <= 0.8){                      //if value is less than 
		            //stuck=1;
	          //}
        }//(!isnan(value)){
        else{                                         //if ith value is real and  nan
            det_safe.push_back(1);                    //if ith value is real and  nan set ith det_safe to "1"
        }            
    }//(int i=0; i<n; i++){
       
    //count beams in safe valleys
		    
    valleybou.data.clear();                              
        for (int j=0; j< n; j++){                     //for loop for each value in /scan
             
             if ((det_safe[j] == 0) || (j== n-1)   ){                         //if det_safe=0 or it is the last itteration<--- this doesnt make sense .... not sure i uderstand it

                  if (det_safe[j] ==1){                                       //if jth value is clear increase count
                      count++;     
                  }         
                  if (!(count ==0) && count >= limit ){                       //(FIND RIGHT AND LEFT BOUND) if the count is not zero and >= safe limit 
                      
					           double right_bound = (j - count)*unit+limit*unit/2-30;   //  Finds Right bound of safe direction subtract 30 because lidar the starts at -30 degrees (cartesian) not 0 degrees add half the limit because you cant send the quad immediately in the direction of a safe vally 
                      if (right_bound <0)                                     //rather than have -10 degrees, right boundary would read 350 degrees
                      {
                        right_bound=right_bound+360;
                      }
                      
					            double left_bound = j*unit-limit*unit/2-30;             //left bound reduced by half the limit (in degrees)
                      if (left_bound <0)
                      {
                        left_bound=left_bound+360;
                      }   
                      
                      if (right_bound>left_bound)                             // this splits the right and left bound to two seperate clear vectors if it goes over the 0 degree mark
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
		for (int m = 0; m < valley.size(); m+=2){                       //this says if the user input is between the safe values that were defined in push back then input=1

      sub_directions.data.push_back(valley[m]);
      sub_directions.data.push_back(valley[m+1]);

			if (userinput >= valley[m] && userinput <= valley[m+1]){

			    inside = 1;
			}
		}

		if (inside){                                                    //if input=1 then the safe output direction will be the user input
			det.data=userinput;
		}
		else
		{
      if (userinput >=210.0 && userinput<=330.0)                    // if the userinput is in the direction where the lidar cant see (bakward) then go in the user direction
      {
          det.data=userinput;
      }
      else
      {                                                             //i know what the section is supposed to do but i havent understood how yet, if the userinput isnt in a safe vally that was previously defined then it will find which limit of the safe vallys that the userinput is cloes then use that as the input direction. 
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
