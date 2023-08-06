# cone_posision_new
转换雷达坐标系的锥桶至全局坐标
## 订阅
●锥桶局部坐标（激光雷达坐标系）:/cone_position
●车全局位姿:/Carstate
## 发布
●单个锥桶信息（局部，全局，id）:/NEWconeposition
●当前帧中的全部锥桶信息（局部，全局，id）:/coneMap
●可视化锥桶全局坐标（所有的锥桶都会留下，不会消失）:/coneMarker
●可视化锥桶全局坐标（只会显示当前帧的锥桶）:/NEWconeposition
## 存储锥桶使用的方式
●所有的点信息使用
``` 
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
std::vector<int> point_ids;
``` 
进行存储，其中cloud是锥桶全局坐标信息，point_ids存放对应索引，所以此点云不可以通过kdtree进行排序，并且也没必要，全部点云数据小于200个， 消耗时间不大
对锥桶进行处理
### 当车位姿信息未更新时，或者锥桶信息为空时
●return
### 如果是首次接收的锥桶信息
●因为是首次接收的锥桶信息，所以无需判断是否重复，处理后直接放入即可
### 如果不是首次接收的最锥桶信息
●再处理为全局坐标后使用kdTree的方法进行搜索最近点
``` 
    pcl::PointXYZ point;
    point.x =  Ycone.position_global.x;
    point.y =  Ycone.position_global.y;
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);


     if (kdtree.nearestKSearch(point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
``` 
●可以修改此处信息更改认为是同一锥桶的范围
``` 
if(pointNKNSquaredDistance[0]<=2.5&&!(pointIdxNKNSearch[0] >= cloud->size()))
``` 
●如果认为是同一锥桶，那么就会使用先前的点与新点的平均值取代该锥桶坐标，相当于对锥桶坐标进行修正。
        ``` 
        cloud->points[pointIdxNKNSearch[0]].x = (cloud->points[pointIdxNKNSearch[0]].x+point.x)/2.0;
        cloud->points[pointIdxNKNSearch[0]].y = (cloud->points[pointIdxNKNSearch[0]].y+point.y)/2.0;
        Ycone.position_global.x =  cloud->points[pointIdxNKNSearch[0]].x;
        Ycone.position_global.y =  cloud->points[pointIdxNKNSearch[0]].y;
        Ycone.id = point_ids[pointIdxNKNSearch[0]];
``` 
## 旋转逻辑
●首先创建旋转矩阵，所需的车身位置向量和雷达到惯导距离向量，使用此旋转矩阵旋转雷达到惯导的距离，以便后面直接使用
``` 
tf2::Transform transform;

transform.setOrigin(tf2::Vector3( 0,0,0));
tf2::Quaternion q;
q.setRPY(0, 0, (Carstate.car_state.theta));

transform.setRotation(q);

tf2::Vector3 lidarPosition(lidarToIMUDist,0,0);
lidarPosition = transform * lidarPosition;
tf2::Vector3 pos(Carstate.car_state.x , Carstate.car_state.y, 0);
``` 
●对于局部坐标来说，坐标轴只是从雷达坐标系转到了以惯导为原点的坐标系，所以对局部坐标的处理只需要再x坐标上加上雷达到惯导的距离即可
``` 
  Ycone.position_baseLink.x = pointToTf.x() + lidarToIMUDist;
    Ycone.position_baseLink.y = pointToTf.y() ;
    Ycone.position_baseLink.z = msgs->points[i].z;
```

●对于全局坐标来说，就需要进行旋转与平移两步了，先进行旋转，再加上车身位置，最后加上雷达到惯导的距离（旋转后）即可得到
``` 
 tf2::Vector3 pointToTf_b = transform * pointToTf;
 Ycone.position_global.x = pointToTf_b.x() + pos.x() +           lidarPosition.x(); 
 Ycone.position_global.y = pointToTf_b.y() + pos.y() + lidarPosition.y();
 Ycone.position_global.z = msgs->points[i].z;
``` 

