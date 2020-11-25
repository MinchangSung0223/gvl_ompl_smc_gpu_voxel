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
#ifndef GVL_LINKAGE_TEST_LIB_H_INCLUDED
#define GVL_LINKAGE_TEST_LIB_H_INCLUDED

#include <gpu_voxels/GpuVoxels.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <ompl/config.h>
#include <iostream>
#include <tuple>

#include <mutex>

robot::JointValueMap myRobotJointValues;
gpu_voxels::GpuVoxelsSharedPtr gvl;
PointCloud my_point_cloud;
Matrix4f tf;
using boost::dynamic_pointer_cast;
using boost::shared_ptr;
using gpu_voxels::voxelmap::ProbVoxelMap;
using gpu_voxels::voxelmap::DistanceVoxelMap;
using gpu_voxels::voxellist::CountingVoxelList;
float voxel_side_length = 0.01f; // 1 cm voxel size
bool new_data_received;



void ctrlchandler(int)
{
  exit(EXIT_SUCCESS);
}

void killhandler(int)
{
  exit(EXIT_SUCCESS);
}
class GvlOmplPlannerHelper : public ompl::base::StateValidityChecker, public ompl::base::MotionValidator, public std::enable_shared_from_this<GvlOmplPlannerHelper>
{
public:
    GvlOmplPlannerHelper(const ompl::base::SpaceInformationPtr &si);
    ~GvlOmplPlannerHelper();

    virtual bool isValid(const ompl::base::State *state) const;
    virtual bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                             std::pair< ompl::base::State*, double > & lastValid) const;
    virtual bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const;

    void plan();
    void rosIter();
    
    std::shared_ptr<GvlOmplPlannerHelper> getptr() {
        return shared_from_this();
    }
   
    void insertStartAndGoal(const ompl::base::ScopedState<> &start, const ompl::base::ScopedState<> &goal) const;
    void visualizeSolution(ompl::base::PathPtr path);
    void doVis();
    void moveObstacle();

private:

    

    ompl::base::StateSpace *stateSpace_;
    ompl::base::SpaceInformationPtr si_;

    mutable std::mutex g_i_mutex;
    mutable std::mutex g_j_mutex;
};



#endif
