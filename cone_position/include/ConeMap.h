#pragma one


#include <ros/ros.h>
#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include <utility>  
#include <map>
#include "common_msgs/HUAT_ASENSING.h"
#include "common_msgs/HUAT_cone.h"     
#include "common_msgs/Cone.h"
#include "common_msgs/HUAT_Carstate.h"
#include "common_msgs/HUAT_map.h"
#include <sstream>
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <flann/flann.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mutex> //锁


#define PII 3.14159265358979   
#define lidarToIMUDist 1.87 //雷达到惯导的距离
#define err 2.5   //认为是同一锥桶的半径


class ConeMap{
private:

    /**
     * @brief ros节点
    */
    ros::NodeHandle nh;

    /**
     * @brief 这个是雷达聚类结果订阅
    */
    ros::Subscriber coneSub;

    /**
     * @brief 这个是车身位姿订阅，用于旋转锥桶
    */
    ros::Subscriber carPositionSub;

    /**
     * @brief 这个是锥桶可视化发布，是所有的锥桶
    */
    ros::Publisher coneMarker;

    /**
     * @brief 这个是每一帧信息发布，包括局部坐标，全局坐标，唯一id
    */
    ros::Publisher Yemap;

    /**
     * @brief 这是每一个锥桶的发布，包括局部坐标，全局坐标，唯一id，
    */
    ros::Publisher conepub;

    /**
     * @brief 全局坐标可视化，这个可以可视化每一帧的，也可以可视化全部的，默认可视化每一帧的，与可是化全局的coneMarker配合使用
    */
    ros::Publisher globalMapPub;

    /**
     * @brief 存储车身位姿
    */
    common_msgs::HUAT_Carstate Carstate;

    /**
     * @brief 存储锥桶信息
    */
    common_msgs::HUAT_cone Ycone;

    /**
     * @brief 存储车身每一帧的锥桶信息
    */
    common_msgs::HUAT_map  Ymap;

    /**
     * @brief 搜索最近点用的kdtree对象
    */
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    
    /**
     * @brief 全局点云，用于存放所有锥桶的位置，与下面的ids作对应，所以该点云不可以排序，排序就乱了
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    /**
     * @brief 存放所有点云的id，和点云用的索引一样，所以上面的点云和这里的id都不可以排序
    */
    std::vector<int> point_ids;

    /**
     * @brief 判断是否受到消息
    */
    bool hadRecived = false;
    bool firstConeMsg = false;

    /**
     * @brief 定义互斥锁对象，保证id的绝对唯一
    */
    std::mutex mtx;


public:

    /**
     * @brief 构造函数，初始化ros节点，订阅，发布对象
     * @param nh_ 这个是ros节点，通过这里传入
    */
    ConeMap(ros::NodeHandle &nh_);

    /**
     * @brief 锥桶信息回调函数，用于全局坐标，唯一id的计算
     * @param msgs 雷达聚类结果，也就是雷达坐标系下的锥桶位置信息
    */
    void doConeMsg(const common_msgs::Cone::ConstPtr &msgs);

    /**
     * @brief 车身位姿回调函数，接受车身位姿，用于旋转锥桶位置
     * @brief 车身位姿
    */
    void doCarMsg(const common_msgs::HUAT_Carstate::ConstPtr &msgs);

    /**
     * @brief 没什么卵用，之前用的，留着吧，可以用他清空点云
    */
    void init();

    /**
     * @brief 用于获取唯一id，绝对可靠
    */
    int getNewId();

    /**
     * @brief 锥桶可视化
     * @param x 锥桶全局的x坐标
     * @param y 锥桶全局的y坐标
     * @param z 锥桶全局的z坐标
     * @param id 锥桶id,这个id是用来区分每一个锥桶的，正好也可以验证锥桶id的唯一性
    */
    void visCone(double x,double y,double z,int id);

    /**
     * @brief 用于存储信息的，存在一个txt中
    */
    void saveMap(common_msgs::HUAT_map& map);
    void saveCone(common_msgs::HUAT_cone& cone);
    void savePoint(double x,double y,double z,int id);
    void clearFile();
};


