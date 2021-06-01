// Generated by gencpp from file geometry_msgs/gnss_bestpose.msg
// DO NOT EDIT!


#ifndef GEOMETRY_MSGS_MESSAGE_GNSS_BESTPOSE_H
#define GEOMETRY_MSGS_MESSAGE_GNSS_BESTPOSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace geometry_msgs
{
template <class ContainerAllocator>
struct gnss_bestpose_
{
  typedef gnss_bestpose_<ContainerAllocator> Type;

  gnss_bestpose_()
    : header()
    , measurement_time(0.0)
    , sol_status(0.0)
    , sol_type(0.0)
    , latitude(0.0)
    , longitude(0.0)
    , height_msl(0.0)
    , undulation(0.0)
    , datum_id(0.0)
    , latitude_std_dev(0.0)
    , longitude_std_dev(0.0)
    , height_std_dev(0.0)
    , base_station_id(0.0)
    , differential_age(0.0)
    , solution_age(0.0)
    , num_sats_tracked(0)
    , num_sats_in_solution(0)
    , num_sats_l1(0)
    , num_sats_multi(0)
    , reserved(0)
    , extended_solution_status(0)
    , galileo_beidou_used_mask(0)
    , gps_glonass_used_mask(0)  {
    }
  gnss_bestpose_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , measurement_time(0.0)
    , sol_status(0.0)
    , sol_type(0.0)
    , latitude(0.0)
    , longitude(0.0)
    , height_msl(0.0)
    , undulation(0.0)
    , datum_id(0.0)
    , latitude_std_dev(0.0)
    , longitude_std_dev(0.0)
    , height_std_dev(0.0)
    , base_station_id(0.0)
    , differential_age(0.0)
    , solution_age(0.0)
    , num_sats_tracked(0)
    , num_sats_in_solution(0)
    , num_sats_l1(0)
    , num_sats_multi(0)
    , reserved(0)
    , extended_solution_status(0)
    , galileo_beidou_used_mask(0)
    , gps_glonass_used_mask(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _measurement_time_type;
  _measurement_time_type measurement_time;

   typedef float _sol_status_type;
  _sol_status_type sol_status;

   typedef float _sol_type_type;
  _sol_type_type sol_type;

   typedef double _latitude_type;
  _latitude_type latitude;

   typedef double _longitude_type;
  _longitude_type longitude;

   typedef double _height_msl_type;
  _height_msl_type height_msl;

   typedef float _undulation_type;
  _undulation_type undulation;

   typedef float _datum_id_type;
  _datum_id_type datum_id;

   typedef float _latitude_std_dev_type;
  _latitude_std_dev_type latitude_std_dev;

   typedef float _longitude_std_dev_type;
  _longitude_std_dev_type longitude_std_dev;

   typedef float _height_std_dev_type;
  _height_std_dev_type height_std_dev;

   typedef float _base_station_id_type;
  _base_station_id_type base_station_id;

   typedef float _differential_age_type;
  _differential_age_type differential_age;

   typedef float _solution_age_type;
  _solution_age_type solution_age;

   typedef uint32_t _num_sats_tracked_type;
  _num_sats_tracked_type num_sats_tracked;

   typedef uint32_t _num_sats_in_solution_type;
  _num_sats_in_solution_type num_sats_in_solution;

   typedef uint32_t _num_sats_l1_type;
  _num_sats_l1_type num_sats_l1;

   typedef uint32_t _num_sats_multi_type;
  _num_sats_multi_type num_sats_multi;

   typedef uint32_t _reserved_type;
  _reserved_type reserved;

   typedef uint32_t _extended_solution_status_type;
  _extended_solution_status_type extended_solution_status;

   typedef uint32_t _galileo_beidou_used_mask_type;
  _galileo_beidou_used_mask_type galileo_beidou_used_mask;

   typedef uint32_t _gps_glonass_used_mask_type;
  _gps_glonass_used_mask_type gps_glonass_used_mask;




  typedef boost::shared_ptr< ::geometry_msgs::gnss_bestpose_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::geometry_msgs::gnss_bestpose_<ContainerAllocator> const> ConstPtr;

}; // struct gnss_bestpose_

typedef ::geometry_msgs::gnss_bestpose_<std::allocator<void> > gnss_bestpose;

typedef boost::shared_ptr< ::geometry_msgs::gnss_bestpose > gnss_bestposePtr;
typedef boost::shared_ptr< ::geometry_msgs::gnss_bestpose const> gnss_bestposeConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::geometry_msgs::gnss_bestpose_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::geometry_msgs::gnss_bestpose_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace geometry_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/home/apollo_ros_bridge/ros_pkgs/src/common_msgs-indigo-devel/geometry_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::geometry_msgs::gnss_bestpose_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::geometry_msgs::gnss_bestpose_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::geometry_msgs::gnss_bestpose_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::geometry_msgs::gnss_bestpose_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::geometry_msgs::gnss_bestpose_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::geometry_msgs::gnss_bestpose_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::geometry_msgs::gnss_bestpose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a6ee131ebe2b0e83ebc279abc37a5e2c";
  }

  static const char* value(const ::geometry_msgs::gnss_bestpose_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa6ee131ebe2b0e83ULL;
  static const uint64_t static_value2 = 0xebc279abc37a5e2cULL;
};

template<class ContainerAllocator>
struct DataType< ::geometry_msgs::gnss_bestpose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/gnss_bestpose";
  }

  static const char* value(const ::geometry_msgs::gnss_bestpose_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::geometry_msgs::gnss_bestpose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
float64 measurement_time\n\
float32 sol_status\n\
float32 sol_type\n\
float64 latitude\n\
float64 longitude\n\
float64 height_msl\n\
float32 undulation\n\
float32 datum_id\n\
float32 latitude_std_dev\n\
float32 longitude_std_dev\n\
float32 height_std_dev \n\
float32 base_station_id\n\
float32 differential_age\n\
float32 solution_age\n\
uint32 num_sats_tracked\n\
uint32 num_sats_in_solution\n\
uint32 num_sats_l1\n\
uint32 num_sats_multi\n\
uint32 reserved\n\
uint32 extended_solution_status\n\
uint32 galileo_beidou_used_mask\n\
uint32 gps_glonass_used_mask\n\
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
";
  }

  static const char* value(const ::geometry_msgs::gnss_bestpose_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::geometry_msgs::gnss_bestpose_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.measurement_time);
      stream.next(m.sol_status);
      stream.next(m.sol_type);
      stream.next(m.latitude);
      stream.next(m.longitude);
      stream.next(m.height_msl);
      stream.next(m.undulation);
      stream.next(m.datum_id);
      stream.next(m.latitude_std_dev);
      stream.next(m.longitude_std_dev);
      stream.next(m.height_std_dev);
      stream.next(m.base_station_id);
      stream.next(m.differential_age);
      stream.next(m.solution_age);
      stream.next(m.num_sats_tracked);
      stream.next(m.num_sats_in_solution);
      stream.next(m.num_sats_l1);
      stream.next(m.num_sats_multi);
      stream.next(m.reserved);
      stream.next(m.extended_solution_status);
      stream.next(m.galileo_beidou_used_mask);
      stream.next(m.gps_glonass_used_mask);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct gnss_bestpose_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::geometry_msgs::gnss_bestpose_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::geometry_msgs::gnss_bestpose_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "measurement_time: ";
    Printer<double>::stream(s, indent + "  ", v.measurement_time);
    s << indent << "sol_status: ";
    Printer<float>::stream(s, indent + "  ", v.sol_status);
    s << indent << "sol_type: ";
    Printer<float>::stream(s, indent + "  ", v.sol_type);
    s << indent << "latitude: ";
    Printer<double>::stream(s, indent + "  ", v.latitude);
    s << indent << "longitude: ";
    Printer<double>::stream(s, indent + "  ", v.longitude);
    s << indent << "height_msl: ";
    Printer<double>::stream(s, indent + "  ", v.height_msl);
    s << indent << "undulation: ";
    Printer<float>::stream(s, indent + "  ", v.undulation);
    s << indent << "datum_id: ";
    Printer<float>::stream(s, indent + "  ", v.datum_id);
    s << indent << "latitude_std_dev: ";
    Printer<float>::stream(s, indent + "  ", v.latitude_std_dev);
    s << indent << "longitude_std_dev: ";
    Printer<float>::stream(s, indent + "  ", v.longitude_std_dev);
    s << indent << "height_std_dev: ";
    Printer<float>::stream(s, indent + "  ", v.height_std_dev);
    s << indent << "base_station_id: ";
    Printer<float>::stream(s, indent + "  ", v.base_station_id);
    s << indent << "differential_age: ";
    Printer<float>::stream(s, indent + "  ", v.differential_age);
    s << indent << "solution_age: ";
    Printer<float>::stream(s, indent + "  ", v.solution_age);
    s << indent << "num_sats_tracked: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.num_sats_tracked);
    s << indent << "num_sats_in_solution: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.num_sats_in_solution);
    s << indent << "num_sats_l1: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.num_sats_l1);
    s << indent << "num_sats_multi: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.num_sats_multi);
    s << indent << "reserved: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.reserved);
    s << indent << "extended_solution_status: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.extended_solution_status);
    s << indent << "galileo_beidou_used_mask: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.galileo_beidou_used_mask);
    s << indent << "gps_glonass_used_mask: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.gps_glonass_used_mask);
  }
};

} // namespace message_operations
} // namespace ros

#endif // GEOMETRY_MSGS_MESSAGE_GNSS_BESTPOSE_H