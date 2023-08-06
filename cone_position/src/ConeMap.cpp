#include "ConeMap.h"

ConeMap::ConeMap(ros::NodeHandle &nh_){
    nh = nh_;
    init();
    coneSub = nh.subscribe<common_msgs::Cone>("/cone_position",10,&ConeMap::doConeMsg,this);
    carPositionSub = nh.subscribe<common_msgs::HUAT_Carstate>("/Carstate",10,&ConeMap::doCarMsg,this);
    globalMapPub = nh.advertise<sensor_msgs::PointCloud2>("/globalMapOnly",10);
    conepub = nh.advertise<common_msgs::HUAT_map>("/NEWconeposition", 10);
    Yemap = nh.advertise<common_msgs::HUAT_map>("/coneMap",10);
    coneMarker = nh.advertise<visualization_msgs::Marker>("/coneMarker",10);
}


void ConeMap::init(){
    Carstate.car_state.x = 0;
    Carstate.car_state.y = 0;
    Carstate.car_state.theta = 0;
    cloud->clear();
    clearFile();
}


void ConeMap::doCarMsg(const common_msgs::HUAT_Carstate::ConstPtr &msgs){
    hadRecived = true;
    Carstate.car_state.x = msgs->car_state.x;
    Carstate.car_state.y = msgs->car_state.y;
    Carstate.car_state.theta = msgs->car_state.theta;
}


void ConeMap::doConeMsg(const common_msgs::Cone::ConstPtr &msgs){
  // 检查INS数据是否已经更新
  if (!hadRecived)
  {
    ROS_WARN("INS not update!");
    return;
  }

  if (msgs->points.empty()) {
    ROS_WARN("cone is empty!");
    return;
  }
  // 清空地图中的锥桶信息
  Ymap.cone.clear();

//准备旋转
tf2::Transform transform;

transform.setOrigin(tf2::Vector3( 0,0,0));
tf2::Quaternion q;
q.setRPY(0, 0, (Carstate.car_state.theta));

transform.setRotation(q);

tf2::Vector3 lidarPosition(lidarToIMUDist,0,0);
lidarPosition = transform * lidarPosition;
tf2::Vector3 pos(Carstate.car_state.x , Carstate.car_state.y, 0);

    //用于发布一个pointcloud2消息便于观察点的样子
pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud(new pcl::PointCloud<pcl::PointXYZ>);

if(!firstConeMsg){//如果是第一次接受到锥桶信息就进入，这里面省去了对比，直接进行赋值，节省时间，缺点是篇幅会比较长？
   firstConeMsg = true;
  for (int i = 0; i < msgs->points.size(); i++) {
    tf2::Vector3 pointToTf(msgs->points[i].x , msgs->points[i].y , 0);
    tf2::Vector3 pointToTf_b = transform * pointToTf;
    Ycone.position_baseLink.x = pointToTf.x() + lidarToIMUDist;
    Ycone.position_baseLink.y = pointToTf.y() ;
    Ycone.position_baseLink.z = msgs->points[i].z;

    Ycone.position_global.x = pointToTf_b.x() + pos.x() + lidarPosition.x(); 
    Ycone.position_global.y = pointToTf_b.y() + pos.y() + lidarPosition.y();
    Ycone.position_global.z = msgs->points[i].z;
    /******************************************出图用的，这个可以让杂点消失***********************************************/
    //      if(Ycone.position_global.x>45||Ycone.position_global.y>25||Ycone.position_global.x<-35||Ycone.position_global.y<-5){
    //   continue;
    // }
      /******************************************出图用的，这个可以让杂点消失***********************************************/

    //第一个到的消息不需要检测，直接给新id
    Ycone.id = getNewId();
    Ymap.cone.push_back(Ycone);
    pcl::PointXYZ point;
    point.x = Ycone.position_global.x;
    point.y = Ycone.position_global.y;
    point.z = Ycone.position_global.z;
    
    cloud->push_back(point);
    cloud->width = cloud->points.size();
    cloud->height = 1;
    point_ids.push_back(Ycone.id);
    conepub.publish(Ycone);
    global_cloud->push_back(point);
    saveCone(Ycone);
  //  savePoint(Ycone.position_global.x,Ycone.position_global.y,Ycone.position_global.z,Ycone.id);
    visCone(Ycone.position_global.x,Ycone.position_global.y,Ycone.position_global.z,Ycone.id);
    }
  }else{
    //如果不是第一次的锥桶信息，那么就需要知道是否与已有的锥桶重合，设置一个阈值，为err（在头文件中可以更改，他是个宏定义），在此阈值内的视为统一锥桶
    for (int i = 0; i < msgs->points.size(); i++) {
    tf2::Vector3 pointToTf(msgs->points[i].x , msgs->points[i].y , 0);
    tf2::Vector3 pointToTf_b = transform * pointToTf;
    Ycone.position_baseLink.x = pointToTf.x() + lidarToIMUDist;
    Ycone.position_baseLink.y = pointToTf.y() ;
    Ycone.position_baseLink.z = msgs->points[i].z;

    Ycone.position_global.x = pointToTf_b.x() + pos.x() + lidarPosition.x(); 
    Ycone.position_global.y = pointToTf_b.y() + pos.y() + lidarPosition.y();
    Ycone.position_global.z = msgs->points[i].z;
    /******************************************出图用的，这个可以让杂点消失***********************************************/
    //      if(Ycone.position_global.x>45||Ycone.position_global.y>25||Ycone.position_global.x<-35||Ycone.position_global.y<-5){
    //   continue;
    // }
      /******************************************出图用的，这个可以让杂点消失***********************************************/

    pcl::PointXYZ point;
    point.x =  Ycone.position_global.x;
    point.y =  Ycone.position_global.y;
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
   // bool isFound = false;

     if (kdtree.nearestKSearch(point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
      /**********************修改这里V改变认为是同一锥筒的阈值*****************************/
      if(pointNKNSquaredDistance[0]<=err&&!(pointIdxNKNSearch[0] >= cloud->size())){//修改距离，在这个距离内认为是同一点
      /**********************修改这里^改变认为是同一锥筒的阈值*****************************/
              //找到点就修正其值
        cloud->points[pointIdxNKNSearch[0]].x = (cloud->points[pointIdxNKNSearch[0]].x+point.x)/2.0;
        cloud->points[pointIdxNKNSearch[0]].y = (cloud->points[pointIdxNKNSearch[0]].y+point.y)/2.0;
        Ycone.position_global.x =  cloud->points[pointIdxNKNSearch[0]].x;
        Ycone.position_global.y =  cloud->points[pointIdxNKNSearch[0]].y;
        Ycone.id = point_ids[pointIdxNKNSearch[0]];
         Ymap.cone.push_back(Ycone);

        point.x =  Ycone.position_global.x;
    point.y =  Ycone.position_global.y;
        global_cloud->push_back(point);
        saveCone(Ycone);
        visCone(Ycone.position_global.x,Ycone.position_global.y,Ycone.position_global.z,Ycone.id);
     //   savePoint(Ycone.position_global.x,Ycone.position_global.y,Ycone.position_global.z,Ycone.id);
        // std::cout<<"这是老锥筒"<<std::endl;
      }else{
           // 没有找到距离在 0.05以内的最近邻点，生成新的 ID
            int id = getNewId();
            Ycone.id = id;
            Ymap.cone.push_back(Ycone);
            cloud->push_back(point);
            cloud->width = cloud->points.size();
            cloud->height = 1;
            point_ids.push_back(Ycone.id);
            global_cloud->push_back(point);
            saveCone(Ycone);
            visCone(Ycone.position_global.x,Ycone.position_global.y,Ycone.position_global.z,Ycone.id);
          //  savePoint(Ycone.position_global.x,Ycone.position_global.y,Ycone.position_global.z,Ycone.id);//给-号是因为坐标系y轴副方向为正
            // std::cout<<"这是新锥筒"<<std::endl;
      }
     }else{
                 // 没有找到距离在 0.05以内的最近邻点，生成新的 ID
            int id = getNewId();
            Ycone.id = id;
            Ymap.cone.push_back(Ycone);
            cloud->push_back(point);
            cloud->width = cloud->points.size();
            cloud->height = 1;
            point_ids.push_back(Ycone.id);
            global_cloud->push_back(point);
            saveCone(Ycone);
            visCone(Ycone.position_global.x,Ycone.position_global.y,Ycone.position_global.z,Ycone.id);
          //  savePoint(Ycone.position_global.x,Ycone.position_global.y,Ycone.position_global.z,Ycone.id);//给-号是因为坐标系y轴副方向为正
            // std::cout<<"这是新锥筒"<<std::endl;
     }
  }
}
kdtree.setInputCloud(cloud);
/*******************************排序，并不需要****************************************/
// std::sort(Ymap.cone.begin(), Ymap.cone.end(), 
//           [](const common_msgs::HUAT_cone& a, const common_msgs::HUAT_cone& b){
//               return a.id < b.id; 
//           });
/*******************************排序，并不需要****************************************/
Yemap.publish(Ymap);
saveMap(Ymap);
  sensor_msgs::PointCloud2 global_cloud_msg;
  pcl::toROSMsg(*global_cloud, global_cloud_msg);
  global_cloud_msg.header.frame_id = "velodyne";
  global_cloud_msg.header.stamp = ros::Time::now();

  // 发布PointCloud2消息
  globalMapPub.publish(global_cloud_msg);
}


int ConeMap::getNewId(){
  static int id = -1;
  std::lock_guard<std::mutex> lock(mtx); // 获取互斥锁
  id++;
  return id;
}


void ConeMap::saveMap(common_msgs::HUAT_map& map){
   std::stringstream ss;
       for (int i = 0; i < map.cone.size(); i++) {
       ss<<"ID:"<<map.cone[i].id<<std::endl;  
       ss<<"position_global:"<<map.cone[i].position_global.x<<"\t"<<map.cone[i].position_global.y<<std::endl;
       ss<<"position_baseLink:"<<map.cone[i].position_baseLink.x<<"\t"<<map.cone[i].position_baseLink.y<<std::endl;
    }
    std::string str = ss.str();

    std::ofstream f;
    f.open("/home/ros/LIDAR_ye/src/testData/Map.txt",std::ios_base::app); 
    if (f.fail()) { 
        std::cerr << "Failed to open file: " << std::strerror(errno) << std::endl; 
        return; 
    }
    f << str;
    f << "\n\n\n";
    f.close();
}


void ConeMap::saveCone(common_msgs::HUAT_cone& cone){
   std::stringstream ss;
       ss<<cone.id<<"\n"<<cone.position_baseLink.x<<"\t"<<cone.position_global.y<<"\n"<<cone.position_global.x<<"\t"<<cone.position_global.y<<std::endl;
    std::string str = ss.str();
    std::ofstream f;
    f.open("/home/ros/LIDAR_ye/src/testData/Cone.txt",std::ios_base::app); 
    if (f.fail()) { 
        std::cerr << "Failed to open file: " << std::strerror(errno) << std::endl; 
 
        return; 
    }
    f << str;
    f.close();
}


void ConeMap::savePoint(double x,double y,double z,int id){
   std::stringstream ss;
       ss<<id<<"\t"<<x<<"\t"<<y<<"\t"<<z<<std::endl;
    std::string str = ss.str();
    std::ofstream f;
    f.open("/home/ros/LIDAR_ye/src/testData/Point.txt",std::ios_base::app); 
    if (f.fail()) { 
        std::cerr << "Failed to open file: " << std::strerror(errno) << std::endl; 
 
        return; 
    }
    f << str;
    f.close();
}


void ConeMap::visCone(double x,double y,double z,int id){
  visualization_msgs::Marker marker; // 创建一个名为marker的visualization_msgs/Marker消息对象
  marker.header.frame_id = "velodyne"; // 设置marker消息的坐标系为map
  marker.lifetime = ros::Duration(); // 设置marker消息的持续时间为永久
  marker.header.stamp = ros::Time::now(); // 设置marker消息的时间戳为当前时间
  marker.ns = "my_namespace"; // 设置marker消息的namespace为my_namespace，用于在RViz中区分不同的marker
  marker.id = id; // 设置marker消息的id为0，用于在RViz中区分不同的marker
  marker.type = visualization_msgs::Marker::SPHERE; // 设置marker消息的类型为球体
  marker.action = visualization_msgs::Marker::ADD; // 设置marker消息的操作类型为ADD，表示添加一个新的marker
  marker.pose.position.x = x; // 设置marker消息的位置x坐标为0
  marker.pose.position.y = y; // 设置marker消息的位置y坐标为0
marker.pose.position.z = z; // 设置marker消息的位置z坐标为0
marker.pose.orientation.x = 0.0; // 设置marker消息的方向x分量为0
marker.pose.orientation.y = 0.0; // 设置marker消息的方向y分量为0
marker.pose.orientation.z = 0.0; // 设置marker消息的方向z分量为0
marker.pose.orientation.w = 1.0; // 设置marker消息的方向w分量为1，表示不进行旋转
marker.scale.x = 0.5; // 设置marker消息的大小x分量为0.1
marker.scale.y = 0.5; // 设置marker消息的大小y分量为0.1
marker.scale.z = 0.5; // 设置marker消息的大小z分量为0.1
marker.color.a = 1.0; // 设置marker消息的颜色透明度为1.0，表示完全不透明
marker.color.r = 1.0; // 设置marker消息的颜色红分量为1.0，表示红色
marker.color.g = 1.0; // 设置marker消息的颜色绿分量为0.0，表示不带绿色
marker.color.b = 1.0; // 设置marker消息的颜色蓝分量为0.0，表示不带蓝色
coneMarker.publish(marker); // 发布marker消息到ROS系统中，用于在RViz中显示
}


void ConeMap::clearFile() {
    std::ofstream f("/home/ros/LIDAR_ye/src/testData/Cone.txt", std::ios::out | std::ios::trunc);
    if (f.fail()) { 
        std::cerr << "Failed to open file: " << std::strerror(errno) << std::endl; 
        return; 
    }
    f.close();
    std::ofstream ff("/home/ros/LIDAR_ye/src/testData/Map.txt", std::ios::out | std::ios::trunc);
    if (ff.fail()) { 
        std::cerr << "Failed to open file: " << std::strerror(errno) << std::endl; 
        return; 
    }
    ff.close();
}