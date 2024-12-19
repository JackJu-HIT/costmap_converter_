/*
*@Function: convert costmap obstacle to Polygon manager
*@Create by:juchunyu@qq.com
*@Date:2024-12-18 9:52:00
*@Last modified:juchunyu@qq.com
*/
#ifndef COSTMAP_CONVERTER_MANAGER_H_
#define COSTMAP_CONVERTER_MANAGER_H_

#include <costmap_converter/costmap_to_polygons.h>
#include <costmap_converter/costmap_to_polygons_concave.h>
#include <costmap_converter/costmap_to_lines_ransac.h>
#include <costmap_converter/costmap_to_lines_convex_hull.h>

namespace costmap_converter
{
    enum METHOD 
    {
        COSTMAP_TO_POLYGONS_DBSMCCH,               //单元格转换成凸多边形表示
        COSTMAP_TO_POLYGONS_DBSCONCAVEHULL,        //单元格转换成凹多边形表示
        COSTMAP_TO_LINES_DBSRANSAC,                //单元格转换成点，线
        COSTMAP_TO_LINES_DBSMCCH                   //单元格转换成点，线
    };

    struct obstaclePointsClouds
    {
        float x;
        float y;
    };
    

    class costmap_converter_manager
    {
    private:
       boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> costmap_converter_;
       costmap_converter::ObstacleArrayConstPtr obstacles_;
       std::vector<double> x_arr_,y_arr_;
       double resolution_ = 0.05;
       costmap_converter::costmapinfo map_;
      
    public:
        costmap_converter_manager();
        ~costmap_converter_manager();
        void init(METHOD method);
        void updateObstacleCloud(const std::vector<obstaclePointsClouds> &obstacleClouds);
        costmap_converter::ObstacleArrayConstPtr getComputeClusterResults();
        void show();
        void createCostmap();

    };




}


#endif // end COSTMAP_CONVERTER_MANAGER_H_







