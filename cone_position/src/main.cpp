#include "ConeMap.h"

int main(int argc,char* argv[]){
     ros::init(argc, argv, "cone_positon");
     ros::NodeHandle nh;
     ConeMap coneMap(nh);
    ros::Rate rate(10);
     while (ros::ok()){  
        ros::spinOnce();   
        rate.sleep();
     }
    return 0; 
}