#include "drake/manipulation/util/move_ik_demo_base.h"

#include <utility>

#include "drake/common/drake_throw.h"
#include "drake/manipulation/util/robot_plan_utils.h"
#include "drake/multibody/parsing/parser.h"

namespace drake {
namespace manipulation {
namespace util {

using planner::ConstraintRelaxingIk;

// class definition is within the hpp file

MoveIkDemoBase::MoveIkDemoBase(std::string robot_description,
                               std::string base_link,
                               std::string ik_link,
                               int print_interval)
    : robot_description_(std::move(robot_description)),
      ik_link_(std::move(ik_link)),
      print_interval_(print_interval),
      plant_(0.0),
      constraint_relaxing_ik_(robot_description_, ik_link_) {  // model path; end-effector name 
  multibody::Parser(&plant_).AddModelFromFile(robot_description_);
  plant_.WeldFrames(plant_.world_frame(),
                    plant_.GetBodyByName(base_link).body_frame());
  plant_.Finalize();
  context_ = plant_.CreateDefaultContext();
  joint_names_ = GetJointNames(plant_);
  joint_velocity_limits_ = plant_.GetVelocityUpperLimits();
}

MoveIkDemoBase::~MoveIkDemoBase() {}

void MoveIkDemoBase::set_joint_velocity_limits(
    const Eigen::Ref<const Eigen::VectorXd>& velocity_limits) {
  DRAKE_THROW_UNLESS(velocity_limits.size() ==
                     joint_velocity_limits_.size());
  joint_velocity_limits_ = velocity_limits;
}

void MoveIkDemoBase::HandleStatus(
    const Eigen::Ref<const Eigen::VectorXd>& q) {
  status_count_++;
  plant_.SetPositions(context_.get(), q);  // q -> current joint position; set to context_
  // q is obtained in the main function
  if (status_count_ % print_interval_ == 1) {
    const math::RigidTransform<double> current_link_pose =
        plant_.EvalBodyPoseInWorld(
            *context_, plant_.GetBodyByName(ik_link_));
    const math::RollPitchYaw<double> rpy(current_link_pose.rotation());
    // which link's pose?  end-effector (output its pose in real-time)
    //drake::log()->info("{} at: {} {}",
    //                   ik_link_,  // link name (printed on the terminal screen)
    //                   current_link_pose.translation().transpose(), // link pose
    //                   rpy.vector().transpose());  // what is this -> it continues to output the end-effector pose in real-time
    // can be recorded locally, so that we may have a better start & goal position pair
  }
}

// an optional 
std::optional<lcmt_robot_plan> MoveIkDemoBase::Plan(
    const math::RigidTransformd& goal_pose, std::vector<double>& times) {  // Input from user side, get the joint angle
    // using the inverse kinematic 

  DRAKE_THROW_UNLESS(status_count_ > 0);

  // Create a single waypoint for our plan (the destination).
  // This results in a trajectory with two knot points (the
  // current pose (read from the status message currently being
  // processes and passed directly to PlanSequentialTrajectory as
  // iiwa_q) and the calculated final pose).
  ConstraintRelaxingIk::IkCartesianWaypoint wp;   // arm pose (user input) IK -> inverse kine~
  wp.pose = goal_pose; // goal pose
  wp.constrain_orientation = true;  // ik solver will not consider the goal orientation if it is set to be false
  std::vector<ConstraintRelaxingIk::IkCartesianWaypoint> waypoints;
  waypoints.push_back(wp);
  std::vector<Eigen::VectorXd> q_sol;
  std::cout << "check point!" << std::endl;
  const bool result =  //there should be a loop in this function ubtil the solver find a solution
    // this one is for the inverse kinematics.
      constraint_relaxing_ik_.PlanSequentialTrajectory(
          waypoints, plant_.GetPositions(*context_), &q_sol);
  std::cout << "The length of the vector:" << q_sol.size() << std::endl;
  std::cout << "The length of the vector:" << q_sol.size() << std::endl;
  for(unsigned int i = 0; i != q_sol.size(); i++){
    if(i==0)
      std::cout << "The current joint angles are:" << std::endl;
    else
      std::cout << "The goal joint angles are:" << std::endl;
    std::cout << q_sol[i] << std::endl;  
  }
  // seems to have joint angle, computed from PlanSequentialTrajectory.
  // Yes, we can directly provide with the joint angle values to q_sol!
  // waypoints only includes the goal pose (not joint angles, need to be computed)
  // plant_.GetPositions(*context_) -> get joint angles for current position
  // q_sol include the two sets of joint angles, for the current position & goal position
  drake::log()->info("IK result: {}", result);
  //result 
  if (result) {
    drake::log()->info("IK sol size {}", q_sol.size());

    // Run the resulting plan over 2 seconds (which is a fairly
    // arbitrary choice).  This may be slowed down if executing
    // the plan in that time would exceed any joint velocity
    // limits.
    // std::vector<double> times{0, 5};  // This parameter can determine the motion velosity
    // I think we can set it as an input parameter
    DRAKE_DEMAND(q_sol.size() == times.size());  // q_sol has two sets

    ApplyJointVelocityLimits(  // apply the maximum velocity constraint
        q_sol, joint_velocity_limits_, &times);
    lcmt_robot_plan plan = EncodeKeyFrames(
        joint_names_, times, q_sol);
    return plan;
  }

  return std::nullopt;
}

}  // namespace util
}  // namespace manipulation
}  // namespace drake
