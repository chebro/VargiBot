// Generated by gencpp from file pkg_vb_sim/conveyorBeltPowerMsgRequest.msg
// DO NOT EDIT!


#ifndef PKG_VB_SIM_MESSAGE_CONVEYORBELTPOWERMSGREQUEST_H
#define PKG_VB_SIM_MESSAGE_CONVEYORBELTPOWERMSGREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace pkg_vb_sim
{
template <class ContainerAllocator>
struct conveyorBeltPowerMsgRequest_
{
  typedef conveyorBeltPowerMsgRequest_<ContainerAllocator> Type;

  conveyorBeltPowerMsgRequest_()
    : power(0)  {
    }
  conveyorBeltPowerMsgRequest_(const ContainerAllocator& _alloc)
    : power(0)  {
  (void)_alloc;
    }



   typedef int8_t _power_type;
  _power_type power;





  typedef boost::shared_ptr< ::pkg_vb_sim::conveyorBeltPowerMsgRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pkg_vb_sim::conveyorBeltPowerMsgRequest_<ContainerAllocator> const> ConstPtr;

}; // struct conveyorBeltPowerMsgRequest_

typedef ::pkg_vb_sim::conveyorBeltPowerMsgRequest_<std::allocator<void> > conveyorBeltPowerMsgRequest;

typedef boost::shared_ptr< ::pkg_vb_sim::conveyorBeltPowerMsgRequest > conveyorBeltPowerMsgRequestPtr;
typedef boost::shared_ptr< ::pkg_vb_sim::conveyorBeltPowerMsgRequest const> conveyorBeltPowerMsgRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pkg_vb_sim::conveyorBeltPowerMsgRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pkg_vb_sim::conveyorBeltPowerMsgRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::pkg_vb_sim::conveyorBeltPowerMsgRequest_<ContainerAllocator1> & lhs, const ::pkg_vb_sim::conveyorBeltPowerMsgRequest_<ContainerAllocator2> & rhs)
{
  return lhs.power == rhs.power;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::pkg_vb_sim::conveyorBeltPowerMsgRequest_<ContainerAllocator1> & lhs, const ::pkg_vb_sim::conveyorBeltPowerMsgRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace pkg_vb_sim

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::pkg_vb_sim::conveyorBeltPowerMsgRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pkg_vb_sim::conveyorBeltPowerMsgRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pkg_vb_sim::conveyorBeltPowerMsgRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pkg_vb_sim::conveyorBeltPowerMsgRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pkg_vb_sim::conveyorBeltPowerMsgRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pkg_vb_sim::conveyorBeltPowerMsgRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pkg_vb_sim::conveyorBeltPowerMsgRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cb865fe69e245bdfa52dafc5c79743f2";
  }

  static const char* value(const ::pkg_vb_sim::conveyorBeltPowerMsgRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcb865fe69e245bdfULL;
  static const uint64_t static_value2 = 0xa52dafc5c79743f2ULL;
};

template<class ContainerAllocator>
struct DataType< ::pkg_vb_sim::conveyorBeltPowerMsgRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pkg_vb_sim/conveyorBeltPowerMsgRequest";
  }

  static const char* value(const ::pkg_vb_sim::conveyorBeltPowerMsgRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pkg_vb_sim::conveyorBeltPowerMsgRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int8 power\n"
;
  }

  static const char* value(const ::pkg_vb_sim::conveyorBeltPowerMsgRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pkg_vb_sim::conveyorBeltPowerMsgRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.power);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct conveyorBeltPowerMsgRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pkg_vb_sim::conveyorBeltPowerMsgRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pkg_vb_sim::conveyorBeltPowerMsgRequest_<ContainerAllocator>& v)
  {
    s << indent << "power: ";
    Printer<int8_t>::stream(s, indent + "  ", v.power);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PKG_VB_SIM_MESSAGE_CONVEYORBELTPOWERMSGREQUEST_H