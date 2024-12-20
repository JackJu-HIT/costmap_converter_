/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann, Otniel Rinaldo
 *********************************************************************/

#ifndef COSTMAP_CONVERTER_INTERFACE_H_
#define COSTMAP_CONVERTER_INTERFACE_H_

//#include <costmap_2d/costmap_2d_ros.h>
//#include <ros/ros.h>
//#include <ros/callback_queue.h>
#include <boost/thread.hpp>
//#include <costmap_2d/costmap_2d.h>
//#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Polygon.h>
#include <costmap_converter/ObstacleArrayMsg.h>
#include <memory>
#include <boost/make_shared.hpp>
#include "tool.h"


namespace costmap_converter
{
//! Typedef for a shared dynamic obstacle container
typedef boost::shared_ptr<ObstacleArrayMsg> ObstacleArrayPtr;
//! Typedef for a shared dynamic obstacle container (read-only access)
typedef boost::shared_ptr< const ObstacleArrayMsg > ObstacleArrayConstPtr;

//! Typedef for a shared polygon container 
typedef boost::shared_ptr< std::vector<geometry_msgs::Polygon> > PolygonContainerPtr;
//! Typedef for a shared polygon container (read-only access)
typedef boost::shared_ptr< const std::vector<geometry_msgs::Polygon> > PolygonContainerConstPtr;
  

/**
 * @class BaseCostmapToPolygons
 * @brief This abstract class defines the interface for plugins that convert the costmap into polygon types
 * 
 * Plugins must accept a costmap_2d::Costmap2D datatype as information source.
 * The interface provides two different use cases:
 * 1. Manual call to conversion routines: setCostmap2D(), compute() and getPolygons() 
 *    (in subsequent calls setCostmap2D() can be substituted by updateCostmap2D())
 * 2. Repeatedly process costmap with a specific rate (startWorker() and stopWorker()):
 *    Make sure that the costmap is valid while the worker is active (you can specify your own spinner or activate a threaded spinner).
 *    Costmaps can be obtained by invoking getPolygons().
 */
class BaseCostmapToPolygons
{
public: 
  
    /**
     * @brief Initialize the plugin
     * @param nh Nodehandle that defines the namespace for parameters
     */
    virtual void initialize( ) = 0;
    
    /**
     * @brief Destructor
     */
    virtual ~BaseCostmapToPolygons() 
    {
      //stopWorker();
    }

    
    /**
     * @brief Pass a pointer to the costap to the plugin.
     * @warning The plugin should handle the costmap's mutex locking.
     * @sa updateCostmap2D
     * @param costmap Pointer to the costmap2d source
     */
    virtual void setCostmap2D(std::vector<double> x,std::vector<double> y,const costmapinfo &map) = 0;
    
    /**
     * @brief Get updated data from the previously set Costmap2D
     * @warning The plugin should handle the costmap's mutex locking.
     * @sa setCostmap2D
     */
    virtual void updateCostmap2D(std::vector<double> x,std::vector<double> y,const costmapinfo &map)= 0; 
    
     /**
     * @brief This method performs the actual work (conversion of the costmap to polygons)
     */
    virtual void compute() = 0;
    
    /**
     * @brief Get a shared instance of the current polygon container
     *
     * If this method is not implemented by any subclass (plugin) the returned shared
     * pointer is empty.
     * @remarks If compute() or startWorker() has not been called before, this method returns an empty instance!
     * @warning The underlying plugin must ensure that this method is thread safe.
     * @return Shared instance of the current polygon container
     */
    virtual PolygonContainerConstPtr getPolygons(){return PolygonContainerConstPtr();}

  /**
   * @brief Get a shared instance of the current obstacle container
   * If this method is not overwritten by the underlying plugin, the obstacle container just imports getPolygons().
   * @remarks If compute() or startWorker() has not been called before, this method returns an empty instance!
   * @warning The underlying plugin must ensure that this method is thread safe.
   * @return Shared instance of the current obstacle container
   * @sa getPolygons
   */
    //virtual void getObstacles()
    virtual ObstacleArrayConstPtr getObstacles()
    {
      ObstacleArrayPtr obstacles = boost::make_shared<ObstacleArrayMsg >();
      PolygonContainerConstPtr polygons = getPolygons();
      if (polygons)
      {
        for (const geometry_msgs::Polygon& polygon : *polygons)
        {
          obstacles->obstacles.emplace_back();
          obstacles->obstacles.back().polygon = polygon;
        }
      }
      return obstacles;
      #if   0
      //ObstacleArrayPtr obstacles = boost::make_shared<ObstacleArrayMsg>();
      PolygonContainerConstPtr polygons = getPolygons();
      if (polygons)
      {
        for (const geometry_msgs::Polygon& polygon : *polygons)
        {
          std::cout << "follw edge coordiates:" << std::endl;
          for (const geometry_msgs::Point32& point : polygon.points) {
            std::cout << "  Point: (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;

          }
          std::cout << "follw edge coordiates end:" << std::endl;
          //obstacles->obstacles.emplace_back();
          //obstacles->obstacles.back().polygon = polygon;
        }
      }

      #endif
      //return obstacles;
    }

    /**
     * @brief Set name of robot's odometry topic
     *
     * Some plugins might require odometry information
     * to compensate the robot's ego motion
     * @param odom_topic topic name
     */
    virtual void setOdomTopic(const std::string& odom_topic) {}

    /**
     * @brief Determines whether an additional plugin for subsequent costmap conversion is specified
     *
     * @return false, since all plugins for static costmap conversion are independent
     */
    virtual bool stackedCostmapConversion() {return false;}

     /**
      * @brief Instantiate a worker that repeatedly coverts the most recent costmap to polygons.
      * The worker is implemented as a timer event that is invoked at a specific \c rate.
      * The passed \c costmap pointer must be valid at the complete time and must be lockable.
      * By specifying the argument \c spin_thread the timer event is invoked in a separate
      * thread and callback queue or otherwise it is called from the global callback queue (of the
      * node in which the plugin is used).
      * @param rate The rate that specifies how often the costmap should be updated
      * @param costmap Pointer to the underlying costmap (must be valid and lockable as long as the worker is active
      * @param spin_thread if \c true,the timer is invoked in a separate thread, otherwise in the default callback queue)
     */
    void startWorker()
    {
 
      BaseCostmapToPolygons::workerCallback();

    }
    
    /**
     * @brief Stop the worker that repeatedly converts the costmap to polygons
     */
    void stopWorker()
    {
    }

protected:
  
    /**
     * @brief Protected constructor that should be called by subclasses
     */
    BaseCostmapToPolygons() {}
    
    /**
     * @brief Blocking method that checks for new timer events (active if startWorker() is called with spin_thread enabled) 
     */
    void spinThread()
    {
    }
    
    /**
     * @brief The callback of the worker that performs the actual work (updating the costmap and converting it to polygons)
     */
    void workerCallback()
    {
        compute();
    }
    
private:


  bool need_to_terminate_;
};    


}



#endif // end COSTMAP_CONVERTER_INTERFACE_H_
