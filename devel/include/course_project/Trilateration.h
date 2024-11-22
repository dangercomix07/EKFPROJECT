// Generated by gencpp from file course_project/Trilateration.msg
// DO NOT EDIT!


#ifndef COURSE_PROJECT_MESSAGE_TRILATERATION_H
#define COURSE_PROJECT_MESSAGE_TRILATERATION_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <course_project/Landmark.h>
#include <course_project/Landmark.h>
#include <course_project/Landmark.h>

namespace course_project
{
template <class ContainerAllocator>
struct Trilateration_
{
  typedef Trilateration_<ContainerAllocator> Type;

  Trilateration_()
    : landmarkA()
    , landmarkB()
    , landmarkC()  {
    }
  Trilateration_(const ContainerAllocator& _alloc)
    : landmarkA(_alloc)
    , landmarkB(_alloc)
    , landmarkC(_alloc)  {
  (void)_alloc;
    }



   typedef  ::course_project::Landmark_<ContainerAllocator>  _landmarkA_type;
  _landmarkA_type landmarkA;

   typedef  ::course_project::Landmark_<ContainerAllocator>  _landmarkB_type;
  _landmarkB_type landmarkB;

   typedef  ::course_project::Landmark_<ContainerAllocator>  _landmarkC_type;
  _landmarkC_type landmarkC;





  typedef boost::shared_ptr< ::course_project::Trilateration_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::course_project::Trilateration_<ContainerAllocator> const> ConstPtr;

}; // struct Trilateration_

typedef ::course_project::Trilateration_<std::allocator<void> > Trilateration;

typedef boost::shared_ptr< ::course_project::Trilateration > TrilaterationPtr;
typedef boost::shared_ptr< ::course_project::Trilateration const> TrilaterationConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::course_project::Trilateration_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::course_project::Trilateration_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::course_project::Trilateration_<ContainerAllocator1> & lhs, const ::course_project::Trilateration_<ContainerAllocator2> & rhs)
{
  return lhs.landmarkA == rhs.landmarkA &&
    lhs.landmarkB == rhs.landmarkB &&
    lhs.landmarkC == rhs.landmarkC;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::course_project::Trilateration_<ContainerAllocator1> & lhs, const ::course_project::Trilateration_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace course_project

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::course_project::Trilateration_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::course_project::Trilateration_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::course_project::Trilateration_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::course_project::Trilateration_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::course_project::Trilateration_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::course_project::Trilateration_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::course_project::Trilateration_<ContainerAllocator> >
{
  static const char* value()
  {
    return "45e1ed04607c6f7e36ae2697ced8826f";
  }

  static const char* value(const ::course_project::Trilateration_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x45e1ed04607c6f7eULL;
  static const uint64_t static_value2 = 0x36ae2697ced8826fULL;
};

template<class ContainerAllocator>
struct DataType< ::course_project::Trilateration_<ContainerAllocator> >
{
  static const char* value()
  {
    return "course_project/Trilateration";
  }

  static const char* value(const ::course_project::Trilateration_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::course_project::Trilateration_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Landmark landmarkA\n"
"Landmark landmarkB\n"
"Landmark landmarkC\n"
"\n"
"================================================================================\n"
"MSG: course_project/Landmark\n"
"float32 x\n"
"float32 y\n"
"float32 distance\n"
"float32 variance\n"
;
  }

  static const char* value(const ::course_project::Trilateration_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::course_project::Trilateration_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.landmarkA);
      stream.next(m.landmarkB);
      stream.next(m.landmarkC);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Trilateration_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::course_project::Trilateration_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::course_project::Trilateration_<ContainerAllocator>& v)
  {
    s << indent << "landmarkA: ";
    s << std::endl;
    Printer< ::course_project::Landmark_<ContainerAllocator> >::stream(s, indent + "  ", v.landmarkA);
    s << indent << "landmarkB: ";
    s << std::endl;
    Printer< ::course_project::Landmark_<ContainerAllocator> >::stream(s, indent + "  ", v.landmarkB);
    s << indent << "landmarkC: ";
    s << std::endl;
    Printer< ::course_project::Landmark_<ContainerAllocator> >::stream(s, indent + "  ", v.landmarkC);
  }
};

} // namespace message_operations
} // namespace ros

#endif // COURSE_PROJECT_MESSAGE_TRILATERATION_H
