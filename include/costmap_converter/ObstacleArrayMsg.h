// Generated by gencpp from file costmap_converter/ObstacleArrayMsg.msg
// DO NOT EDIT!


#ifndef COSTMAP_CONVERTER_MESSAGE_OBSTACLEARRAYMSG_H
#define COSTMAP_CONVERTER_MESSAGE_OBSTACLEARRAYMSG_H


#include <string>
#include <vector>
#include <map>

//#include <ros/types.h>
//#include <ros/serialization.h>
//#include <ros/builtin_message_traits.h>
//#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <costmap_converter/ObstacleMsg.h>

namespace costmap_converter
{
template <class ContainerAllocator>
struct ObstacleArrayMsg_
{
  typedef ObstacleArrayMsg_<ContainerAllocator> Type;

  ObstacleArrayMsg_()
    : header()
    , obstacles()  {
    }
  ObstacleArrayMsg_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , obstacles(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::costmap_converter::ObstacleMsg_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::costmap_converter::ObstacleMsg_<ContainerAllocator> >::other >  _obstacles_type;
  _obstacles_type obstacles;





  typedef boost::shared_ptr< ::costmap_converter::ObstacleArrayMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::costmap_converter::ObstacleArrayMsg_<ContainerAllocator> const> ConstPtr;

}; // struct ObstacleArrayMsg_

typedef ::costmap_converter::ObstacleArrayMsg_<std::allocator<void> > ObstacleArrayMsg;

typedef boost::shared_ptr< ::costmap_converter::ObstacleArrayMsg > ObstacleArrayMsgPtr;
typedef boost::shared_ptr< ::costmap_converter::ObstacleArrayMsg const> ObstacleArrayMsgConstPtr;

// constants requiring out of line definition




//template<typename ContainerAllocator>
//std::ostream& operator<<(std::ostream& s, const ::costmap_converter::ObstacleArrayMsg_<ContainerAllocator> & v)
//{
//ros::message_operations::Printer< ::costmap_converter::ObstacleArrayMsg_<ContainerAllocator> >::stream(s, "", v);
//return s;
//}

} // namespace costmap_converter

#if 0
namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'costmap_converter': ['/tmp/binarydeb/ros-kinetic-costmap-converter-0.0.12/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::costmap_converter::ObstacleArrayMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::costmap_converter::ObstacleArrayMsg_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::costmap_converter::ObstacleArrayMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::costmap_converter::ObstacleArrayMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::costmap_converter::ObstacleArrayMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::costmap_converter::ObstacleArrayMsg_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::costmap_converter::ObstacleArrayMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8a1bdcde72c65ca7d3ce8ebf52d43516";
  }

  static const char* value(const ::costmap_converter::ObstacleArrayMsg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8a1bdcde72c65ca7ULL;
  static const uint64_t static_value2 = 0xd3ce8ebf52d43516ULL;
};

template<class ContainerAllocator>
struct DataType< ::costmap_converter::ObstacleArrayMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "costmap_converter/ObstacleArrayMsg";
  }

  static const char* value(const ::costmap_converter::ObstacleArrayMsg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::costmap_converter::ObstacleArrayMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Message that contains a list of polygon shaped obstacles.\n\
# Special types:\n\
# Polygon with 1 vertex: Point obstacle\n\
# Polygon with 2 vertices: Line obstacle\n\
# Polygon with more than 2 vertices: First and last points are assumed to be connected\n\
\n\
std_msgs/Header header\n\
\n\
costmap_converter/ObstacleMsg[] obstacles\n\
\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: costmap_converter/ObstacleMsg\n\
# Special types:\n\
# Polygon with 1 vertex: Point obstacle (you might also specify a non-zero value for radius)\n\
# Polygon with 2 vertices: Line obstacle\n\
# Polygon with more than 2 vertices: First and last points are assumed to be connected\n\
\n\
std_msgs/Header header\n\
\n\
# Obstacle footprint (polygon descriptions)\n\
geometry_msgs/Polygon polygon\n\
\n\
# Specify the radius for circular/point obstacles\n\
float64 radius\n\
\n\
# Obstacle ID\n\
# Specify IDs in order to provide (temporal) relationships\n\
# between obstacles among multiple messages.\n\
int64 id\n\
\n\
# Individual orientation (centroid)\n\
geometry_msgs/Quaternion orientation\n\
\n\
# Individual velocities (centroid)\n\
geometry_msgs/TwistWithCovariance velocities\n\
\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Polygon\n\
#A specification of a polygon where the first and last points are assumed to be connected\n\
Point32[] points\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point32\n\
# This contains the position of a point in free space(with 32 bits of precision).\n\
# It is recommeded to use Point wherever possible instead of Point32.  \n\
# \n\
# This recommendation is to promote interoperability.  \n\
#\n\
# This message is designed to take up less space when sending\n\
# lots of points at once, as in the case of a PointCloud.  \n\
\n\
float32 x\n\
float32 y\n\
float32 z\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: geometry_msgs/TwistWithCovariance\n\
# This expresses velocity in free space with uncertainty.\n\
\n\
Twist twist\n\
\n\
# Row-major representation of the 6x6 covariance matrix\n\
# The orientation parameters use a fixed-axis representation.\n\
# In order, the parameters are:\n\
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n\
float64[36] covariance\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Twist\n\
# This expresses velocity in free space broken into its linear and angular parts.\n\
Vector3  linear\n\
Vector3  angular\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
# It is only meant to represent a direction. Therefore, it does not\n\
# make sense to apply a translation to it (e.g., when applying a \n\
# generic rigid transformation to a Vector3, tf2 will only apply the\n\
# rotation). If you want your data to be translatable too, use the\n\
# geometry_msgs/Point message instead.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const ::costmap_converter::ObstacleArrayMsg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::costmap_converter::ObstacleArrayMsg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.obstacles);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ObstacleArrayMsg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::costmap_converter::ObstacleArrayMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::costmap_converter::ObstacleArrayMsg_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "obstacles[]" << std::endl;
    for (size_t i = 0; i < v.obstacles.size(); ++i)
    {
      s << indent << "  obstacles[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::costmap_converter::ObstacleMsg_<ContainerAllocator> >::stream(s, indent + "    ", v.obstacles[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif

#endif // COSTMAP_CONVERTER_MESSAGE_OBSTACLEARRAYMSG_H
