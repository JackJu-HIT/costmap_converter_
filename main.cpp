/*
*Function: convert costmap obstacle to Polygon
*Create by:juchunyu@qq.com
*Date:2024-12-18 13:52:00
*Last modified:juchunyu@qq.com
*/
#include "costmap_converter/costmap_converter_manager.h"
#include <iostream>
using namespace std;

int main()
{
  
  
  costmap_converter::costmap_converter_manager clusterObj;
  
  clusterObj.init(costmap_converter::COSTMAP_TO_POLYGONS_DBSMCCH);

  std::vector<double> x_arr = {10,9.8,9.8,10,10.1,10.1,9.7,9.6,9.5,1,1.1,1.2,1.3,1.4, 5,5,5,5.2,4.8,4.7,4.7,4.6};
  std::vector<double> y_arr = {10,9.8,10.0,9.8,10.1,9.8,9.8,9.6,9.5,1.0,1.0,1.0,1.0,1.0,5,5.2,5.1,5.0,4.9,4.9,5.8,5.6};
   
  std::vector<costmap_converter::obstaclePointsClouds> obstacleClouds;
  for(int i = 0;i < x_arr.size();i++)
  {
       costmap_converter::obstaclePointsClouds tempPointClouds;
       tempPointClouds.x = x_arr[i];
       tempPointClouds.y = y_arr[i];
       obstacleClouds.push_back(tempPointClouds);
  }

  std::cout << " x_arr size =" <<  x_arr.size() << std::endl;
  
  clusterObj.updateObstacleCloud(obstacleClouds);
  
  costmap_converter::ObstacleArrayConstPtr obstacles = clusterObj.getComputeClusterResults();
  
  clusterObj.show();

}
