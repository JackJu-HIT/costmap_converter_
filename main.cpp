/*
*Function: convert costmap obstacle to Polygon
*Create by:juchunyu
*Date:2024-6-7 9:52:00
*Last modified:juchunyu
*/
#include <costmap_converter/costmap_to_polygons.h>
#include <iostream>

int main()
{

  boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> costmap_converter_(new costmap_converter::CostmapToPolygonsDBSMCCH());
   
  
  costmap_converter_->initialize();

  costmap_converter_->setCostmap2D();

  costmap_converter_->startWorker();

  costmap_converter::ObstacleArrayConstPtr obstacles = costmap_converter_->getObstacles();


  for (std::size_t i=0; i<obstacles->obstacles.size(); ++i)
  {
    const costmap_converter::ObstacleMsg* obstacle = &obstacles->obstacles.at(i);
    const geometry_msgs::Polygon* polygon = &obstacle->polygon;

    if (polygon->points.size()==1 && obstacle->radius > 0) // Circle
    {
      std::cout << "Obstacle Type:Circle " << "(" <<polygon->points[0].x  << ", " <<  polygon->points[0].y << ")  r=" <<obstacle->radius << std::endl;
    }
    else if (polygon->points.size()==1) // Point
    {
      std::cout << "Obstacle Type:Point "  <<  "(" <<polygon->points[0].x  << ", " <<  polygon->points[0].y<< ")" << std::endl;
    }
    else if (polygon->points.size()==2) // Line
    {
        std::cout << "Obstacle Type:Line " << "Line start = (" <<polygon->points[0].x  << "," <<  polygon->points[0].y << 
            ") Line end = ( " <<polygon->points[1].x  << "," <<  polygon->points[1].y << ")" << std::endl;
     
    }
    else if (polygon->points.size()>2) // Real polygon
    {
        std::cout << "Obstacle Type:Polygon ";

        for(int j = 0; j < polygon->points.size();j++)
        {   if(j > 0)
              std::cout << "->";
            std::cout << "(" << polygon->points[j].x << "," <<  polygon->points[j].x << ")";
        }
        std::cout << std::endl;
    }

   
  }

  
  


}
