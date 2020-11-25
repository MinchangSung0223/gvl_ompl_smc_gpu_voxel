// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the GPU Voxels Software Library.
//
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE.txt in the top
// directory of the source code.
//
// © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Andreas Hermann
 * \date    2018-01-07
 *
 */
//----------------------------------------------------------------------/*
#include "gvl_ompl_planner_helper.h"
#include <signal.h>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <icl_core_config/Config.h>
#include <icl_core_performance_monitor/PerformanceMonitor.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <gpu_voxels/GpuVoxels.h>
#include <gpu_voxels/helpers/MetaPointCloud.h>
#include <gpu_voxels/robot/urdf_robot/urdf_robot.h>
#include <gpu_voxels/logging/logging_gpu_voxels.h>
#include <thread>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

#define IC_PERFORMANCE_MONITOR
#include <icl_core_performance_monitor/PerformanceMonitor.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace gpu_voxels;
namespace bfs = boost::filesystem;



GvlOmplPlannerHelper::GvlOmplPlannerHelper(const ob::SpaceInformationPtr &si)
    : ob::StateValidityChecker(si)
    , ob::MotionValidator(si)
{

    si_ = si;
    stateSpace_ = si_->getStateSpace().get();
    assert(stateSpace_ != nullptr);

    gvl = gpu_voxels::GpuVoxels::getInstance();
    gvl->initialize(150, 150, 100, 0.02);

    // We add maps with objects, to collide them
    gvl->addMap(MT_BITVECTOR_VOXELLIST,"myRobotMap");
    gvl->addMap(MT_PROBAB_VOXELMAP,"myEnvironmentMap");
    gvl->addMap(MT_BITVECTOR_VOXELLIST,"mySolutionMap");
    //gvl->addMap(MT_PROBAB_VOXELMAP,"myQueryMap");

    gvl->addRobot("myUrdfRobot", "indy7_coarse/indy7_ros.urdf", true);

    //gvl->visualizeMap("myEnvironmentMap");

    PERF_MON_ENABLE("pose_check");
    PERF_MON_ENABLE("motion_check");
    PERF_MON_ENABLE("motion_check_lv");
}



GvlOmplPlannerHelper::~GvlOmplPlannerHelper()
{
    gvl.reset(); // Not even required, as we use smart pointers.
}

void GvlOmplPlannerHelper::moveObstacle()
{
    //gvl->clearMap("myEnvironmentMap");
    static float x(0.0);

    // use this to animate a single moving box obstacle
    //gvl->insertBoxIntoMap(Vector3f(2.0, x ,0.0), Vector3f(2.2, x + 0.2 ,1.2), "myEnvironmentMap", eBVM_OCCUPIED, 2);
    x -= 0.1;

      // gvl->insertBoxIntoMap(Vector3f(1.0,1.0,0.0), Vector3f(1.2,1.2,1.2), "myEnvironmentMap", eBVM_OCCUPIED, 2);
   //gvl->insertBoxIntoMap(Vector3f(1.8,x+1.8,0.0), Vector3f(2.0,x+2.0,1.2), "myEnvironmentMap", eBVM_OCCUPIED, 2);
    //gvl->insertBoxIntoMap(Vector3f(0.0,0.0,0.0), Vector3f(3.0,3.0,0.01), "myEnvironmentMap", eBVM_OCCUPIED, 2);
    gvl->visualizeMap("myEnvironmentMap");
}

void GvlOmplPlannerHelper::doVis()
{

    // tell the visualier that the map has changed:
    gvl->visualizeMap("myRobotMap");
    gvl->visualizeMap("myEnvironmentMap");
    gvl->visualizeMap("mySolutionMap");
    //gvl->visualizeMap("myQueryMap");
}

void GvlOmplPlannerHelper::visualizeSolution(ob::PathPtr path)
{
    gvl->clearMap("mySolutionMap");

    PERF_MON_SUMMARY_PREFIX_INFO("pose_check");
    PERF_MON_SUMMARY_PREFIX_INFO("motion_check");
    PERF_MON_SUMMARY_PREFIX_INFO("motion_check_lv");

    std::cout << "Robot consists of " << gvl->getRobot("myUrdfRobot")->getTransformedClouds()->getAccumulatedPointcloudSize() << " points" << std::endl;

    og::PathGeometric* solution = path->as<og::PathGeometric>();
    solution->interpolate();


    for(size_t step = 0; step < solution->getStateCount(); ++step)
    {

        const double *values = solution->getState(step)->as<ob::RealVectorStateSpace::StateType>()->values;

        robot::JointValueMap state_joint_values;
        state_joint_values["joint0"] = values[0];
        state_joint_values["joint1"] = values[1];
        state_joint_values["joint2"] = values[2];
        state_joint_values["joint3"] = values[3];
        state_joint_values["joint4"] = values[4];
        state_joint_values["joint5"] = values[5];

        // update the robot joints:
        gvl->setRobotConfiguration("myUrdfRobot", state_joint_values);
        // insert the robot into the map:
        gvl->insertRobotIntoMap("myUrdfRobot", "mySolutionMap", BitVoxelMeaning(eBVM_SWEPT_VOLUME_START + (step % 249) ));
    }

    gvl->visualizeMap("mySolutionMap");

}

void roscallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
{
  static float x(0.0);
  gvl->clearMap("myEnvironmentMap");
  
  std::vector<Vector3f> point_data;
  point_data.resize(msg->points.size());

  for (uint32_t i = 0; i < msg->points.size(); i++)
  {
    point_data[i].x = msg->points[i].x;
    point_data[i].y = msg->points[i].y;
    point_data[i].z = msg->points[i].z;
  }

  //my_point_cloud.add(point_data);
  my_point_cloud.update(point_data);

  // transform new pointcloud to world coordinates
  my_point_cloud.transformSelf(&tf);
  
  new_data_received = true;

  //LOGGING_INFO(Gpu_voxels, "DistanceROSDemo camera callback. PointCloud size: " << msg->points.size() << endl);
  
}


void rosjointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  //std::cout << "Got JointStateMessage" << std::endl;
  gvl->clearMap("myRobotMap");

  for(size_t i = 0; i < msg->name.size(); i++)
  {
    myRobotJointValues[msg->name[i]] = msg->position[i];
  }
  // update the robot joints:
  gvl->setRobotConfiguration("myUrdfRobot", myRobotJointValues);
  // insert the robot into the map:
  gvl->insertRobotIntoMap("myUrdfRobot", "myRobotMap",(eBVM_OCCUPIED));
}


void GvlOmplPlannerHelper::rosIter(){

  signal(SIGINT, ctrlchandler);
  signal(SIGTERM, killhandler);
    icl_core::config::GetoptParameter points_parameter("points-topic:", "t",
                                                    "Identifer of the pointcloud topic");
    icl_core::config::GetoptParameter roll_parameter  ("roll:", "r",
                                                    "Camera roll in degrees");
    icl_core::config::GetoptParameter pitch_parameter ("pitch:", "p",
                                                    "Camera pitch in degrees");
    icl_core::config::GetoptParameter yaw_parameter   ("yaw:", "y",
                                                    "Camera yaw in degrees");
    icl_core::config::GetoptParameter voxel_side_length_parameter("voxel_side_length:", "s",
                                                                "Side length of a voxel, default 0.01");
    icl_core::config::GetoptParameter filter_threshold_parameter ("filter_threshold:", "f",
                                                                "Density filter threshold per voxel, default 1");
    icl_core::config::GetoptParameter erode_threshold_parameter  ("erode_threshold:", "e",
                                                                "Erode voxels with fewer occupied neighbors (percentage)");
    icl_core::config::addParameter(points_parameter);
    icl_core::config::addParameter(roll_parameter);
    icl_core::config::addParameter(pitch_parameter);
    icl_core::config::addParameter(yaw_parameter);
    icl_core::config::addParameter(voxel_side_length_parameter);
    icl_core::config::addParameter(filter_threshold_parameter);
    icl_core::config::addParameter(erode_threshold_parameter);
    voxel_side_length = icl_core::config::paramOptDefault<float>("voxel_side_length", 0.01f);

    const Vector3f camera_offsets(1.5f+0.89f, 1.5f-0.08f, 1.30f); 
    float roll = icl_core::config::paramOptDefault<float>("roll", -135.0f) * 3.141592f / 180.0f;
    float pitch = icl_core::config::paramOptDefault<float>("pitch", 0.0f) * 3.141592f / 180.0f;
    float yaw = icl_core::config::paramOptDefault<float>("yaw", 90.0f) * 3.141592f / 180.0f;
    tf = Matrix4f::createFromRotationAndTranslation(Matrix3f::createFromRPY(0+roll, 0 + pitch, 0 + yaw), camera_offsets);
    


    std::string point_cloud_topic = icl_core::config::paramOptDefault<std::string>("points-topic", "/camera/depth/color/points");
    int argc;
    char **argv;
    ros::init(argc, argv, "distance_ros_demo");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> >(point_cloud_topic, 1,roscallback);
    ros::Subscriber sub1 = nh.subscribe("joint_states", 1, rosjointStateCallback); 
    gvl->addMap(MT_BITVECTOR_OCTREE, "myEnvironmentMap");
    gvl->addPrimitives(primitive_array::ePRIM_SPHERE, "measurementPoints");
    //PBA
  gvl->addMap(MT_DISTANCE_VOXELMAP, "pbaDistanceVoxmap");
  shared_ptr<DistanceVoxelMap> pbaDistanceVoxmap = dynamic_pointer_cast<DistanceVoxelMap>(gvl->getMap("pbaDistanceVoxmap"));

  gvl->addMap(MT_PROBAB_VOXELMAP, "erodeTempVoxmap1");
  shared_ptr<ProbVoxelMap> erodeTempVoxmap1 = dynamic_pointer_cast<ProbVoxelMap>(gvl->getMap("erodeTempVoxmap1"));
  gvl->addMap(MT_PROBAB_VOXELMAP, "erodeTempVoxmap2");
  shared_ptr<ProbVoxelMap> erodeTempVoxmap2 = dynamic_pointer_cast<ProbVoxelMap>(gvl->getMap("erodeTempVoxmap2"));

  gvl->addMap(MT_COUNTING_VOXELLIST, "countingVoxelList");
  shared_ptr<CountingVoxelList> countingVoxelList = dynamic_pointer_cast<CountingVoxelList>(gvl->getMap("countingVoxelList"));

  gvl->addMap(MT_COUNTING_VOXELLIST, "countingVoxelListFiltered");
  shared_ptr<CountingVoxelList> countingVoxelListFiltered = dynamic_pointer_cast<CountingVoxelList>(gvl->getMap("countingVoxelListFiltered"));

  //PBA map clone for visualization without artifacts
  gvl->addMap(MT_DISTANCE_VOXELMAP, "pbaDistanceVoxmapVisual");
  shared_ptr<DistanceVoxelMap> pbaDistanceVoxmapVisual = dynamic_pointer_cast<DistanceVoxelMap>(gvl->getMap("pbaDistanceVoxmapVisual"));
  shared_ptr<NTree::GvlNTreeDet> myEnvironmentMap = dynamic_pointer_cast<NTree::GvlNTreeDet>(gvl->getMap("myEnvironmentMap"));
  pbaDistanceVoxmapVisual->clearMap();








    ros::Rate r(30);
    size_t iteration = 0;
    size_t num_colls = 0;
    size_t num_colls2 = 0;
    BitVectorVoxel collision_types;
    while(ros::ok()){
      ros::spinOnce();
      
      if (new_data_received) 
      {
           new_data_received = false;
           iteration++;

           pbaDistanceVoxmap->clearMap();
      
           countingVoxelList->clearMap();
           countingVoxelListFiltered->clearMap();
           erodeTempVoxmap1->clearMap();
           erodeTempVoxmap2->clearMap();
           countingVoxelList->insertPointCloud(my_point_cloud, eBVM_OCCUPIED);
           gvl->visualizeMap("countingVoxelList");
           num_colls = gvl->getMap("countingVoxelList")->as<gpu_voxels::voxellist::CountingVoxelList>()->collideWith(gvl->getMap("mySolutionMap")->as<gpu_voxels::voxellist::BitVectorVoxelList>(), 1.0f);
           num_colls2 = gvl->getMap("countingVoxelList")->as<gpu_voxels::voxellist::CountingVoxelList>()->collideWith(gvl->getMap("myRobotMap")->as<gpu_voxels::voxellist::BitVectorVoxelList>(), 1.0f); 
           std::cout << "Detected " << num_colls-num_colls2 << " collisions " << std::endl;
           
      }
      r.sleep();
    }
    exit(EXIT_SUCCESS);
}

void GvlOmplPlannerHelper::insertStartAndGoal(const ompl::base::ScopedState<> &start, const ompl::base::ScopedState<> &goal) const
{

    gvl->clearMap("myQueryMap");

    robot::JointValueMap state_joint_values;
    state_joint_values["joint0"] = start[0];
    state_joint_values["joint1"] = start[1];
    state_joint_values["joint2"] = start[2];
    state_joint_values["joint3"] = start[3];
    state_joint_values["joint4"] = start[4];
    state_joint_values["joint5"] = start[5];

    // update the robot joints:
    gvl->setRobotConfiguration("myUrdfRobot", state_joint_values);
    gvl->insertRobotIntoMap("myUrdfRobot", "myQueryMap", BitVoxelMeaning(eBVM_SWEPT_VOLUME_START));

    state_joint_values["joint0"] = goal[0];
    state_joint_values["joint1"] = goal[1];
    state_joint_values["joint2"] = goal[2];
    state_joint_values["joint3"] = goal[3];
    state_joint_values["joint4"] = goal[4];
    state_joint_values["joint5"] = goal[5];

    // update the robot joints:
    gvl->setRobotConfiguration("myUrdfRobot", state_joint_values);
    gvl->insertRobotIntoMap("myUrdfRobot", "myQueryMap", BitVoxelMeaning(eBVM_SWEPT_VOLUME_START+1));

}

bool GvlOmplPlannerHelper::isValid(const ompl::base::State *state) const
{

    PERF_MON_START("inserting");

    std::lock_guard<std::mutex> lock(g_i_mutex);

    gvl->clearMap("myRobotMap");

    const double *values = state->as<ob::RealVectorStateSpace::StateType>()->values;

    robot::JointValueMap state_joint_values;
    state_joint_values["joint0"] = values[0];
    state_joint_values["joint1"] = values[1];
    state_joint_values["joint2"] = values[2];
    state_joint_values["joint3"] = values[3];
    state_joint_values["joint4"] = values[4];
    state_joint_values["joint5"] = values[5];

    // update the robot joints:
    gvl->setRobotConfiguration("myUrdfRobot", state_joint_values);
    // insert the robot into the map:
    gvl->insertRobotIntoMap("myUrdfRobot", "myRobotMap", eBVM_OCCUPIED);

    PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("insert", "Pose Insertion", "pose_check");

    PERF_MON_START("coll_test");
    size_t num_colls_pc = gvl->getMap("myRobotMap")->as<voxelmap::ProbVoxelMap>()->collideWith(gvl->getMap("myEnvironmentMap")->as<voxelmap::ProbVoxelMap>(), 0.7f);
    PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("coll_test", "Pose Collsion", "pose_check");

    //std::cout << "Validity check on state ["  << values[0] << ", " << values[1] << ", " << values[2] << ", " << values[3] << ", " << values[4] << ", " << values[5] << "] resulting in " <<  num_colls_pc << " colls." << std::endl;

    return num_colls_pc == 0;
}

bool GvlOmplPlannerHelper::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                                       std::pair< ompl::base::State*, double > & lastValid) const
{

    //    std::cout << "LongestValidSegmentFraction = " << stateSpace_->getLongestValidSegmentFraction() << std::endl;
    //    std::cout << "getLongestValidSegmentLength = " << stateSpace_->getLongestValidSegmentLength() << std::endl;
    //    std::cout << "getMaximumExtent = " << stateSpace_->getMaximumExtent() << std::endl;


    std::lock_guard<std::mutex> lock(g_j_mutex);
    gvl->clearMap("myRobotMap");

    /* assume motion starts in a valid configuration so s1 is valid */

    bool result = true;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    //std::cout << "Called interpolating motion_check_lv to evaluate " << nd << " segments" << std::endl;

    PERF_MON_ADD_DATA_NONTIME_P("Num poses in motion", float(nd), "motion_check_lv");
    if (nd > 1)
    {
        /* temporary storage for the checked state */
        ob::State *test = si_->allocState();

        for (int j = 1; j < nd; ++j)
        {
            stateSpace_->interpolate(s1, s2, (double)j / (double)nd, test);
            if (!si_->isValid(test))

            {
                lastValid.second = (double)(j - 1) / (double)nd;
                if (lastValid.first != nullptr)
                    stateSpace_->interpolate(s1, s2, lastValid.second, lastValid.first);
                result = false;
                break;

            }
        }

        si_->freeState(test);

    }

    if (result)
        if (!si_->isValid(s2))
        {
            lastValid.second = (double)(nd - 1) / (double)nd;
            if (lastValid.first != nullptr)
                stateSpace_->interpolate(s1, s2, lastValid.second, lastValid.first);
            result = false;
        }


    if (result)
        valid_++;
    else
        invalid_++;


    return result;
}

bool GvlOmplPlannerHelper::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const
{
    std::lock_guard<std::mutex> lock(g_i_mutex);
    gvl->clearMap("myRobotMap");


    //        std::cout << "LongestValidSegmentFraction = " << stateSpace_->getLongestValidSegmentFraction() << std::endl;
    //        std::cout << "getLongestValidSegmentLength = " << stateSpace_->getLongestValidSegmentLength() << std::endl;
    //        std::cout << "getMaximumExtent = " << stateSpace_->getMaximumExtent() << std::endl;



    /* assume motion starts in a valid configuration so s1 is valid */

    bool result = true;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    // not required with ProbabVoxels:
    //    if(nd > 249)
    //    {
    //        std::cout << "Too many intermediate states for BitVoxels" << std::endl;
    //        exit(1);
    //    }

    if (nd > 1)
    {
        PERF_MON_START("inserting");

        /* temporary storage for the checked state */
        ob::State *test = si_->allocState();

        for (int j = 1; j < nd; ++j)
        {
            stateSpace_->interpolate(s1, s2, (double)j / (double)nd, test);


            const double *values = test->as<ob::RealVectorStateSpace::StateType>()->values;

            robot::JointValueMap state_joint_values;
            state_joint_values["joint0"] = values[0];
            state_joint_values["joint1"] = values[1];
            state_joint_values["joint2"] = values[2];
            state_joint_values["joint3"] = values[3];
            state_joint_values["joint4"] = values[4];
            state_joint_values["joint5"] = values[5];

            // update the robot joints:
            gvl->setRobotConfiguration("myUrdfRobot", state_joint_values);
            // insert the robot into the map:
            gvl->insertRobotIntoMap("myUrdfRobot", "myRobotMap", eBVM_OCCUPIED);

        }
        PERF_MON_ADD_DATA_NONTIME_P("Num poses in motion", float(nd), "motion_check");

        si_->freeState(test);

        PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("insert", "Motion Insertion", "motion_check");

        //gvl->visualizeMap("myRobotMap");
        PERF_MON_START("coll_test");
        size_t num_colls_pc = gvl->getMap("myRobotMap")->as<voxelmap::ProbVoxelMap>()->collideWith(gvl->getMap("myEnvironmentMap")->as<voxelmap::ProbVoxelMap>(), 0.7f);
        //std::cout << "CheckMotion1 for " << nd << " segments. Resulting in " << num_colls_pc << " colls." << std::endl;
        PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("coll_test", "Pose Collsion", "motion_check");

        result = (num_colls_pc == 0);

    }


    if (result)
        valid_++;
    else
        invalid_++;


    return result;


}
