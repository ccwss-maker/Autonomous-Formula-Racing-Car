// Generated by gencpp from file box_ros_msgs/Odom.msg
// DO NOT EDIT!


#ifndef BOX_ROS_MSGS_MESSAGE_ODOM_H
#define BOX_ROS_MSGS_MESSAGE_ODOM_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>

namespace box_ros_msgs
{
template <class ContainerAllocator>
struct Odom_
{
  typedef Odom_<ContainerAllocator> Type;

  Odom_()
    : pose()
    , twist()  {
    }
  Odom_(const ContainerAllocator& _alloc)
    : pose(_alloc)
    , twist(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::PoseWithCovariance_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef  ::geometry_msgs::TwistWithCovariance_<ContainerAllocator>  _twist_type;
  _twist_type twist;





  typedef boost::shared_ptr< ::box_ros_msgs::Odom_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::box_ros_msgs::Odom_<ContainerAllocator> const> ConstPtr;

}; // struct Odom_

typedef ::box_ros_msgs::Odom_<std::allocator<void> > Odom;

typedef boost::shared_ptr< ::box_ros_msgs::Odom > OdomPtr;
typedef boost::shared_ptr< ::box_ros_msgs::Odom const> OdomConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::box_ros_msgs::Odom_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::box_ros_msgs::Odom_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::box_ros_msgs::Odom_<ContainerAllocator1> & lhs, const ::box_ros_msgs::Odom_<ContainerAllocator2> & rhs)
{
  return lhs.pose == rhs.pose &&
    lhs.twist == rhs.twist;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::box_ros_msgs::Odom_<ContainerAllocator1> & lhs, const ::box_ros_msgs::Odom_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace box_ros_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::box_ros_msgs::Odom_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::box_ros_msgs::Odom_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::box_ros_msgs::Odom_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::box_ros_msgs::Odom_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::box_ros_msgs::Odom_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::box_ros_msgs::Odom_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::box_ros_msgs::Odom_<ContainerAllocator> >
{
  static const char* value()
  {
    return "66583e0eea98e483a46bb9d1453778fa";
  }

  static const char* value(const ::box_ros_msgs::Odom_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x66583e0eea98e483ULL;
  static const uint64_t static_value2 = 0xa46bb9d1453778faULL;
};

template<class ContainerAllocator>
struct DataType< ::box_ros_msgs::Odom_<ContainerAllocator> >
{
  static const char* value()
  {
    return "box_ros_msgs/Odom";
  }

  static const char* value(const ::box_ros_msgs::Odom_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::box_ros_msgs::Odom_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/PoseWithCovariance pose\n"
"geometry_msgs/TwistWithCovariance twist\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/PoseWithCovariance\n"
"# This represents a pose in free space with uncertainty.\n"
"\n"
"Pose pose\n"
"\n"
"# Row-major representation of the 6x6 covariance matrix\n"
"# The orientation parameters use a fixed-axis representation.\n"
"# In order, the parameters are:\n"
"# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n"
"float64[36] covariance\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/TwistWithCovariance\n"
"# This expresses velocity in free space with uncertainty.\n"
"\n"
"Twist twist\n"
"\n"
"# Row-major representation of the 6x6 covariance matrix\n"
"# The orientation parameters use a fixed-axis representation.\n"
"# In order, the parameters are:\n"
"# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n"
"float64[36] covariance\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Twist\n"
"# This expresses velocity in free space broken into its linear and angular parts.\n"
"Vector3  linear\n"
"Vector3  angular\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::box_ros_msgs::Odom_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::box_ros_msgs::Odom_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pose);
      stream.next(m.twist);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Odom_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::box_ros_msgs::Odom_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::box_ros_msgs::Odom_<ContainerAllocator>& v)
  {
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::PoseWithCovariance_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "twist: ";
    s << std::endl;
    Printer< ::geometry_msgs::TwistWithCovariance_<ContainerAllocator> >::stream(s, indent + "  ", v.twist);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BOX_ROS_MSGS_MESSAGE_ODOM_H
