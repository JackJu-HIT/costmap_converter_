/*
*Function: convert costmap obstacle to Polygon
*Create by:juchunyu@qq.com
*Date:2024-6-7 9:52:00
*Last modified:juchunyu@qq.com
*/
#include <costmap_converter/costmap_to_polygons.h>
#include <costmap_converter/costmap_to_polygons_concave.h>
#include <costmap_converter/costmap_to_lines_ransac.h>
#include <costmap_converter/costmap_to_lines_convex_hull.h>
#include <iostream>
#include "matplotlib-cpp/matplotlibcpp.h"
#include <cmath>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace plt = matplotlibcpp;//可视化

using namespace std;

// 2D点的结构体
struct Point {
    double x, y;
    Point(double x, double y) : x(x), y(y) {}
};

// 计算两点之间的距离
double distance(const Point& p1, const Point& p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

// 计算点到直线 y = kx + b 的距离
double perpendicularDistance(const Point& p, double k, double b) {
    return fabs(p.y - k * p.x - b) / sqrt(k * k + 1); // 直线 y = kx + b 上的垂直距离
}

// 投影多边形到直线 y = kx + b 上
vector<Point> projectPolygon(const vector<Point>& polygon, double k, double b) {
    vector<Point> projectedPolygon;
    for (const auto& point : polygon) {
        // 计算投影点坐标
        double projX = (k * (point.y - b) + point.x) / (k * k + 1);
        double projY = (k * k * point.y + k * point.x + b) / (k * k + 1);
        projectedPolygon.push_back(Point(projX, projY));
    }
    return projectedPolygon;
}


int main()
{

   //boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> costmap_converter_(new costmap_converter::CostmapToPolygonsDBSMCCH());//单元格转换成凸多边形表示。
   //boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> costmap_converter_(new costmap_converter::CostmapToPolygonsDBSConcaveHull());//单元格转换成凹多边形表示。
   // boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> costmap_converter_(new costmap_converter::CostmapToLinesDBSRANSAC());//单元格转换成点，线。
  //boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> costmap_converter_(new costmap_converter::CostmapToLinesDBSMCCH);//单元格转换成点，线。
   costmap_converter_->initialize();


  //std::vector<double> x_arr = {10,9.8,9.8,10,10.1,10.1,9.7,9.6,9.5,1,1.1,1.2,1.3,1.4};
  //std::vector<double> y_arr = {10,9.8,10.0,9.8,10.1,9.8,9.8,9.6,9.5,1.0,1.0,1.0,1.0,1.0};

  std::vector<double> x_arr = {10,9.8,9.8,10,10.1,10.1,9.7,9.6,9.5,1,1.1,1.2,1.3,1.4, 5,5,5,5.2,4.8,4.7,4.7,4.6};
  std::vector<double> y_arr = {10,9.8,10.0,9.8,10.1,9.8,9.8,9.6,9.5,1.0,1.0,1.0,1.0,1.0,5,5.2,5.1,5.0,4.9,4.9,5.8,5.6};

  std::cout << " x_arr size =" <<  x_arr.size() << std::endl;


  costmap_converter_->setCostmap2D(x_arr,y_arr);

  costmap_converter_->startWorker();

  /*
  for(int i = 0; i < x_arr.size();i++ )
  {
    plt::scatter(x_arr[i],y_arr[i],50);
  }
  */
  plt::scatter(x_arr,y_arr,10);


  
  
 // costmap_converter_->getObstacles();
 costmap_converter::ObstacleArrayConstPtr obstacles = costmap_converter_->getObstacles();

  //if (!obstacles)
    //return;
  std::vector<double> x_res;
  std::vector<double> y_res;
  for (std::size_t i=0; i<obstacles->obstacles.size(); ++i)
  {
    const costmap_converter::ObstacleMsg* obstacle = &obstacles->obstacles.at(i);
    const geometry_msgs::Polygon* polygon = &obstacle->polygon;

    if (polygon->points.size()==1 && obstacle->radius > 0) // Circle
    {
      std::cout << "Obstacle Type:Circle " << "(" <<polygon->points[0].x  << ", " <<  polygon->points[0].y << ")  r=" <<obstacle->radius << std::endl;
      //obstacles_.push_back(ObstaclePtr(new CircularObstacle(polygon->points[0].x, polygon->points[0].y, obstacle->radius)));
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
        //plt::fill(x_res,y_res,keywords2);

        x_res.clear();

        y_res.clear();
  
    }

  }

    plt::xlabel("x");
    plt::ylabel("y");
    plt::title("DBSCAN Clustering Algorithm");
    plt::legend();
     
    plt::show();


#if 0
    // 定义多边形顶点
    vector<Point> polygon = {
        Point(1, 1),
        Point(2, 3),
        Point(4, 5),
        Point(6, 2),
        Point(3, 1)
    };

    // 指定直线的斜率和截距
    double k = 1; // 斜率
    double b = 0; // 截距

    // 投影多边形到 y = kx + b 直线上
    vector<Point> projectedPolygon = projectPolygon(polygon, k, b);

    // 输出投影后的多边形顶点
    cout << "Projected Polygon:" << endl;
    for (const auto& point : projectedPolygon) {
        cout << "(" << point.x << ", " << point.y << ")" << endl;
    }



        plt::xlabel("x");
        plt::ylabel("y");

       // plt::xlim(-10, 18);
        //plt::ylim(-10, 16);
        plt::title("DBSCAN Clustering Algorithm");
        plt::legend();
            //plt::pause(0.0001);
        plt::show();
      

    /*
    costmap_converter_->setOdomTopic(cfg_.odom_topic);
        costmap_converter_->initialize(ros::NodeHandle(nh, "costmap_converter/" + converter_name));
        costmap_converter_->setCostmap2D(costmap_);
        
        costmap_converter_->startWorker(ros::Rate(cfg_.obstacles.costmap_converter_rate), costmap_, cfg_.obstacles.costmap_converter_spin_thread);
    */



   //delete costmap_converter_;
#endif




}
