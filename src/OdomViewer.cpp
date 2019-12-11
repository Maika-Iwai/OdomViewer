// -*- C++ -*-
/*!
 * @file  OdomViewer.cpp
 * @brief Odometry Viewer RTC
 * @date $Date$
 *
 * $Id$
 */

#include "OdomViewer.h"

pcl::visualization::CloudViewer viewer("PointCloudViewer");

// Module specification
// <rtc-template block="module_spec">
static const char* odomviewer_spec[] =
  {
    "implementation_id", "OdomViewer",
    "type_name",         "OdomViewer",
    "description",       "Odometry Viewer RTC",
    "version",           "1.0.0",
    "vendor",            "Maika Iwai, Robot System Design Laboratory, Meijo University",
    "category",          "ImageProcessing",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.TrajectoryBuffer", "10000",

    // Widget
    "conf.__widget__.TrajectoryBuffer", "text",

    // Constraints

    "conf.__type__.TrajectoryBuffer", "int",

    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
OdomViewer::OdomViewer(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_pose3DQuaternionIn("Pose3DQuaternion", m_pose3DQuaternion)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
OdomViewer::~OdomViewer()
{
}



RTC::ReturnCode_t OdomViewer::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("Pose3DQuaternion", m_pose3DQuaternionIn);
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("TrajectoryBuffer", m_trajectorybuffer, "10000");
  // </rtc-template>
  
  return RTC::RTC_OK;
}


RTC::ReturnCode_t OdomViewer::onActivated(RTC::UniqueId ec_id)
{

  p_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
  p_cloud->resize(1000);
  p_cloud->width = m_trajectorybuffer;
  p_cloud->height = 1;
  p_cloud->points.resize(p_cloud->width * p_cloud->height);
  count = 0;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t OdomViewer::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t OdomViewer::onExecute(RTC::UniqueId ec_id)
{
  
  if(m_pose3DQuaternionIn.isNew()){
    m_pose3DQuaternionIn.read();

    printf("\r(px, py, pz, tx, ty, tz, tw) = (%5.2f, %5.2f, %5.2f, %5.2f, %5.2f, %5.2f)", m_pose3DQuaternion.point3D.x, m_pose3DQuaternion.point3D.y, m_pose3DQuaternion.point3D.z, m_pose3DQuaternion.quaternion.x, m_pose3DQuaternion.quaternion.y, m_pose3DQuaternion.quaternion.z, m_pose3DQuaternion.quaternion.w);
    
    pcl::PointXYZRGBA &point = p_cloud->points[count];
    point.x = m_pose3DQuaternion.point3D.x;
    point.y = m_pose3DQuaternion.point3D.y;
    point.z = m_pose3DQuaternion.point3D.z;
    point.r = 255;
    point.g = 255;
    point.b = 255;
    point.a = 0;
    
    
    viewer.showCloud(p_cloud);
    if(count<p_cloud->width){
      count++;
    }
    else{
      count = 0;
    }
  }
  return RTC::RTC_OK;
}

extern "C"
{
 
  void OdomViewerInit(RTC::Manager* manager)
  {
    coil::Properties profile(odomviewer_spec);
    manager->registerFactory(profile,
                             RTC::Create<OdomViewer>,
                             RTC::Delete<OdomViewer>);
  }
  
};


