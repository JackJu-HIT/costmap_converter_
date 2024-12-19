#include <costmap_converter/costmap_converter_manager.h>
#include <iostream>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;//可视化
using namespace costmap_converter;

costmap_converter_manager::costmap_converter_manager()
{

}
    
costmap_converter_manager::~costmap_converter_manager()
{

}


void costmap_converter_manager::init(METHOD method)
{
    switch(method)
    {
        
        case METHOD::COSTMAP_TO_POLYGONS_DBSMCCH:
            printf("聚类方法使用 CostmapToPolygonsDBSMCCH:单元格转换成凸多边形表示!\n");
            costmap_converter_  = boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> (new costmap_converter::CostmapToPolygonsDBSMCCH());//单元格转换成凸多边形表示。
            break;

        case METHOD::COSTMAP_TO_POLYGONS_DBSCONCAVEHULL:
            printf("聚类方法使用 CostmapToPolygonsDBSConcaveHull:单元格转换成凹多边形表示!\n");
            costmap_converter_  = boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> (new costmap_converter::CostmapToPolygonsDBSConcaveHull());//单元格转换成凹多边形表示。
            break;

        case METHOD::COSTMAP_TO_LINES_DBSRANSAC:
            printf("聚类方法使用 CostmapToLinesDBSRANSAC:单元格转换成点，线!\n");
            costmap_converter_  = boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> (new costmap_converter::CostmapToLinesDBSRANSAC());//单元格转换成点，线。
            break;

        case METHOD::COSTMAP_TO_LINES_DBSMCCH:
            printf("聚类方法使用 CostmapToLinesDBSMCCH:单元格转换成点，线!\n");
            costmap_converter_  =  boost::shared_ptr<costmap_converter::BaseCostmapToPolygons>(new costmap_converter::CostmapToLinesDBSMCCH());//单元格转换成点，线
            break;

        default:
			printf("选择的聚类方法无效，将使用默认聚类方法: CostmapToPolygonsDBSMCCH:单元格转换成凸多边形表示!, \n");
            costmap_converter_  = boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> (new costmap_converter::CostmapToPolygonsDBSMCCH());//单元格转换成凸多边形表示。
			break;

    }

    costmap_converter_->initialize();


}

void costmap_converter_manager::createCostmap()
{
    double x_min = std::numeric_limits<double>::max();
    double x_max = std::numeric_limits<double>::lowest();
    double y_min = std::numeric_limits<double>::max();
    double y_max = std::numeric_limits<double>::lowest();

   for(int i = 0; i < x_arr_.size();i++)
   {
      x_min = std::min(x_min, static_cast<double>(x_arr_[i]));
      x_max = std::max(x_max, static_cast<double>(x_arr_[i]));
      y_min = std::min(y_min, static_cast<double>(y_arr_[i]));
      y_max = std::max(y_max, static_cast<double>(y_arr_[i]));
    }
    map_.resolution   = resolution_;
    map_.xcellSize    = static_cast<int>((x_max - x_min) / resolution_);
    map_.yCellSize    = static_cast<int>((y_max - y_min) / resolution_);
    map_.xMetersSize  = (map_.xcellSize - 1 + 0.5) * resolution_;
    map_.yMetersSize  = (map_.yCellSize - 1 + 0.5) * resolution_;
    map_.orgin_x      = x_min;
    map_.orgin_y      = y_min;
}

void costmap_converter_manager::updateObstacleCloud(const std::vector<obstaclePointsClouds> &obstacleClouds)
{
   x_arr_.clear();
   y_arr_.clear();
   for(int i = 0; i < obstacleClouds.size();i++)
   {
       x_arr_.push_back(obstacleClouds[i].x);
       y_arr_.push_back(obstacleClouds[i].y);
   }
   createCostmap();

  costmap_converter_->setCostmap2D(x_arr_,y_arr_,map_);
    
}

costmap_converter::ObstacleArrayConstPtr costmap_converter_manager::getComputeClusterResults()
{
    costmap_converter_->startWorker();
    obstacles_ = costmap_converter_->getObstacles();
    return obstacles_;
}

void costmap_converter_manager::show()
{

    //plot orginal point cloud
    plt::scatter(x_arr_,y_arr_,10);

    //plot cluster 
    std::vector<double> x_res;
    std::vector<double> y_res;
    for (std::size_t i=0; i<obstacles_->obstacles.size(); ++i)
    {
        const costmap_converter::ObstacleMsg* obstacle = &obstacles_->obstacles.at(i);
        const geometry_msgs::Polygon* polygon = &obstacle->polygon;

        if (polygon->points.size()==1 && obstacle->radius > 0) // Circle
        {
            std::cout << "Obstacle Type:Circle " << "(" <<polygon->points[0].x  << ", " <<  polygon->points[0].y << ")  r=" <<obstacle->radius << std::endl;
        }
        else if (polygon->points.size()==1) // Point
        {
            std::cout << "Obstacle Type:Point "  <<  "(" <<polygon->points[0].x  << ", " <<  polygon->points[0].y<< ")" << std::endl;
            //obstacles_.push_back(ObstaclePtr(new PointObstacle(polygon->points[0].x, polygon->points[0].y)));
        }
        else if (polygon->points.size()==2) // Line
        {
            std::cout << "Obstacle Type:Line " << "Line start = (" <<polygon->points[0].x  << "," <<  polygon->points[0].y << 
                ") Line end = ( " <<polygon->points[1].x  << "," <<  polygon->points[1].y << ")" << std::endl;
            //obstacles_.push_back(ObstaclePtr(new LineObstacle(polygon->points[0].x, polygon->points[0].y,
                                                        //    polygon->points[1].x, polygon->points[1].y )));
        }
        else if (polygon->points.size()>2) // Real polygon
        {
            std::cout << "Obstacle Type:Polygon ";

            for(int j = 0; j < polygon->points.size();j++)
            {   if(j > 0)
                std::cout << "->";
                std::cout << "(" << polygon->points[j].x << "," <<  polygon->points[j].y << ")";
                x_res.push_back( polygon->points[j].x);
                y_res.push_back( polygon->points[j].y);
            }
            std::cout << std::endl;
            std::map<std::string, std::string> keywords2;
            keywords2.insert(std::pair<std::string, std::string>("label", "PolyObstacle") );
            keywords2.insert(std::pair<std::string, std::string>("linewidth", "2.5") );
            plt::plot(x_res,y_res,keywords2);
            x_res.clear();
            y_res.clear();
        }

    }
    plt::show();

}
