/// @file
///
/// kuka_plan_runner is designed to wait for LCM messages constraining
/// a lcmt_robot_plan message, and then execute the plan on an iiwa arm
/// (also communicating via LCM using the
/// lcmt_iiwa_command/lcmt_iiwa_status messages).
///
/// When a plan is received, it will immediately begin executing that
/// plan on the arm (replacing any plan in progress).
///
/// If a stop message is received, it will immediately discard the
/// current plan and wait until a new plan is received.

#include <iostream>
#include <fstream>
#include <memory>

#include "lcm/lcm-cpp.hpp"

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_robot_plan.hpp"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;
using drake::Vector1d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

const char* const kLcmStatusChannel = "IIWA_STATUS";
const char* const kLcmCommandChannel = "IIWA_COMMAND";
const char* const kLcmPlanChannel = "COMMITTED_ROBOT_PLAN";
const char* const kLcmStopChannel = "STOP";
const int kNumJoints = 7;

using trajectories::PiecewisePolynomial;
typedef PiecewisePolynomial<double> PPType;
typedef Polynomial<double> PPPoly;
typedef PPType::PolynomialMatrix PPMatrix;

const char* const Main_channel = "Hand_Position";  // The lcm channel for python3& 

class example_t
{
    public:
        double    pnt[3];
        double    motion_time;
    public:
        /**
         * Encode a message into binary form.
         *
         * @param buf The output buffer.
         * @param offset Encoding starts at thie byte offset into @p buf.
         * @param maxlen Maximum number of bytes to write.  This should generally be
         *  equal to getEncodedSize().
         * @return The number of bytes encoded, or <0 on error.
         */
        inline int encode(void *buf, int offset, int maxlen) const;
        /**
         * Check how many bytes are required to encode this message.
         */
        inline int getEncodedSize() const;
        /**
         * Decode a message from binary form into this instance.
         *
         * @param buf The buffer containing the encoded message.
         * @param offset The byte offset into @p buf where the encoded message starts.
         * @param maxlen The maximum number of bytes to read while decoding.
         * @return The number of bytes decoded, or <0 if an error occured.
         */
        inline int decode(const void *buf, int offset, int maxlen);
        /**
         * Retrieve the 64-bit fingerprint identifying the structure of the message.
         * Note that the fingerprint is the same for all instances of the same
         * message type, and is a fingerprint on the message type definition, not on
         * the message contents.
         */
        inline static int64_t getHash();
        /**
         * Returns "example_t"
         */
        inline static const char* getTypeName();
        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static uint64_t _computeHash(const __lcm_hash_ptr *p);
};
int example_t::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();
    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;
    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;
    return pos;
}
int example_t::decode(const void *buf, int offset, int maxlen)
{
    int pos = 0, thislen;
    int64_t msg_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (msg_hash != getHash()) return -1;
    thislen = this->_decodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;
    return pos;
}

int example_t::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t example_t::getHash()
{
    static int64_t hash = static_cast<int64_t>(_computeHash(NULL));
    return hash;
}

const char* example_t::getTypeName()
{
    return "example_t";
}

int example_t::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->pnt[0], 3);
    if(tlen < 0) return tlen; else pos += tlen;
    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->motion_time, 1);
    if(tlen < 0) return tlen; else pos += tlen;
    return pos;
}

int example_t::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;
    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->pnt[0], 3);
    if(tlen < 0) return tlen; else pos += tlen;
    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->motion_time, 1);
    if(tlen < 0) return tlen; else pos += tlen;
    return pos;
}

int example_t::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __double_encoded_array_size(NULL, 3);
    enc_size += __double_encoded_array_size(NULL, 1);
    return enc_size;
}

uint64_t example_t::_computeHash(const __lcm_hash_ptr *)
{
    uint64_t hash = 0x1baa9e29b0fbaa8bLL;
    return (hash<<1) + ((hash>>63)&1);
}


class Handler {
  public:
    ~Handler() {}
    void handleMessage(const ::lcm::ReceiveBuffer *, const std::string &chan,
                      const example_t *msg)
    {
        std::ofstream file;
        file.open("./test.txt", std::ios_base::out);
        printf("Received message on channel \"%s\":\n", chan.c_str());
        // printf("  timestamp   = %ld\n", msg->timestamp);
        // printf("  position    = (%f, %f, %f)\n", msg->position[0], msg->position[1],
        //        msg->position[2]);
        // printf("  orientation = (%f, %f, %f)\n", msg->orientation[0], msg->orientation[1],
        //        msg->orientation[2]);
        // printf("  Motion_time = %f\n", msg->motion_time);
        // printf("  source        = '%s'\n", msg->source.c_str());
        std::cout << msg->pnt[0] << " " << msg->pnt[1] << " " <<msg->pnt[2] << std::endl;
        file << msg->pnt[0] << "\n"; 
        file << msg->pnt[1] << "\n"; 
        file << msg->pnt[2] << "\n"; 
        file << msg->motion_time << "\n";
        file.close();
    }
};

class RobotPlanRunner {
 public:
  /// plant is aliased
  explicit RobotPlanRunner(const multibody::MultibodyPlant<double>& plant)
      : plant_(plant), plan_number_(0) {
    lcm_.subscribe(kLcmStatusChannel,
                    &RobotPlanRunner::HandleStatus, this);
    lcm_.subscribe(kLcmPlanChannel,
                    &RobotPlanRunner::HandlePlan, this);
    lcm_.subscribe(kLcmStopChannel,
                    &RobotPlanRunner::HandleStop, this);
  }

  void Run() {
    int cur_plan_number = plan_number_;
    int64_t cur_time_us = -1;
    int64_t start_time_us = -1;

    // Initialize the timestamp to an invalid number so we can detect
    // the first message.
    iiwa_status_.utime = cur_time_us;

    lcmt_iiwa_command iiwa_command;
    iiwa_command.num_joints = kNumJoints;
    iiwa_command.joint_position.resize(kNumJoints, 0.);
    iiwa_command.num_torques = 0;
    iiwa_command.joint_torque.resize(kNumJoints, 0.);

    double last_x = -1;
    double last_y = -1;
    double last_z = -1;
    // double c_yaw = 0.;  // define the constant pose angles
    // double c_pitch = 0.852009;
    // double c_roll = 0.;
    int flag = 0;

    // ConstraintRelaxingIk::IkCartesianWaypoint wp;
    // wp.pose = goal_pose;
    // wp.constrain_orientation = true;
    // std::vector<ConstraintRelaxingIk::IkCartesianWaypoint> waypoints;
    // waypoints.push_back(wp);
    // std::vector<Eigen::VectorXd> q_sol;
    // const bool result =
    //     constraint_relaxing_ik_.PlanSequentialTrajectory(
    //         waypoints, plant_.GetPositions(*context_), &q_sol);
    // drake::log()->info("IK result: {}", result);

    ::lcm::LCM lc_0;
    Handler handlerObject;
    lc_0.subscribe(Main_channel, &Handler::handleMessage, &handlerObject);
    while (lc_0.handle() >= 0) { 
      break;
    } 
    std::cout << "11" << std::endl;

    std::vector<double> times;  // Control the motion velocity (motion time)
    times.push_back(0);
    std::ifstream file;
    file.open("./test.txt");
    std::string str;
    std::vector<double> pose_info;
    while (std::getline(file, str))
    {
      pose_info.push_back(std::stod(str));
    }
    file.close();
    if (last_x == -1 && last_y == -1 && last_z == -1)
      flag = 1; //first time to receive the command from python
    else if (last_x != pose_info[0] || last_y != pose_info[1] || last_z != pose_info[2])
        flag = 1; // make sure that the command received from python changed
    last_x = pose_info[0]; last_y = pose_info[1]; last_z = pose_info[2];
    // math::RigidTransformd pose(
    //     math::RollPitchYawd(c_roll, c_pitch, c_yaw),
    //     Eigen::Vector3d(pose_info[0], pose_info[1], pose_info[2]));   // This corresponds to the input parameters
    // // I found that ik will not provide with an exact solution as I defined in the pose variable
    // // Instead, it will only give a close solution -> some orientation angle may be different
    // // but surprisingly, the xyz values are almost the same    
    // times.push_back(pose_info[3]);
    if (flag == 1) {  // related to the motion stability  (if *new* commands from python received:
      std::cout << last_x << " " << last_y << " " <<last_z << std::endl;
      // pose -> goal position
      // times -> related to velosity
      flag = 0;
    }

    while (true) {
      // Call lcm handle until at least one status message is
      // processed.
      while (0 == lcm_.handleTimeout(10) || iiwa_status_.utime == -1) { }

      cur_time_us = iiwa_status_.utime;

      if(lc_0.handle() >= 0){
        times.push_back(0);
        file.open("./test.txt");
        while (std::getline(file, str))
        {
          pose_info.push_back(std::stod(str));
        }
        file.close();
        if (last_x == -1 && last_y == -1 && last_z == -1)
          flag = 1; //first time to receive the command from python
        else if (last_x != pose_info[0] || last_y != pose_info[1] || last_z != pose_info[2])
            flag = 1; // make sure that the command received from python changed
        last_x = pose_info[0]; last_y = pose_info[1]; last_z = pose_info[2];
        // math::RigidTransformd pose(
        //     math::RollPitchYawd(c_roll, c_pitch, c_yaw),
        //     Eigen::Vector3d(pose_info[0], pose_info[1], pose_info[2]));   // This corresponds to the input parameters
        // // I found that ik will not provide with an exact solution as I defined in the pose variable
        // // Instead, it will only give a close solution -> some orientation angle may be different
        // // but surprisingly, the xyz values are almost the same    
        // times.push_back(pose_info[3]);
        if (flag == 1) {  // related to the motion stability  (if *new* commands from python received:
          std::cout << last_x << " " << last_y << " " <<last_z << std::endl;
          // pose -> goal position
          // times -> related to velosity
          flag = 1;
        }

      }

      if (plan_) {
        if (plan_number_ != cur_plan_number) {
          std::cout << "Starting new plan." << std::endl;
          start_time_us = cur_time_us;
          cur_plan_number = plan_number_;
        }

        const double cur_traj_time_s =
            static_cast<double>(cur_time_us - start_time_us) / 1e6;
        const auto desired_next = plan_->value(cur_traj_time_s);

        iiwa_command.utime = iiwa_status_.utime;

        for (int joint = 0; joint < kNumJoints; joint++) {
          iiwa_command.joint_position[joint] = desired_next(joint);
        }

        lcm_.publish(kLcmCommandChannel, &iiwa_command);
      }
    }
  }

 private:
  void HandleStatus(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_iiwa_status* status) {
    iiwa_status_ = *status;
  }

  void HandlePlan(const ::lcm::ReceiveBuffer*, const std::string&,
                  const lcmt_robot_plan* plan) {
    std::cout << "New plan received." << std::endl;
    if (iiwa_status_.utime == -1) {
      std::cout << "Discarding plan, no status message received yet"
                << std::endl;
      return;
    } else if (plan->num_states < 2) {
      std::cout << "Discarding plan, Not enough knot points." << std::endl;
      return;
    }

    std::vector<Eigen::MatrixXd> knots(plan->num_states,
                                       Eigen::MatrixXd::Zero(kNumJoints, 1));
    for (int i = 0; i < plan->num_states; ++i) {
      const auto& state = plan->plan[i];
      for (int j = 0; j < state.num_joints; ++j) {
        if (!plant_.HasJointNamed(state.joint_name[j])) {
          continue;
        }
        const multibody::Joint<double>& joint =
            plant_.GetJointByName(state.joint_name[j]);
        DRAKE_DEMAND(joint.num_positions() == 1);
        const int idx = joint.position_start();
        DRAKE_DEMAND(idx < kNumJoints);

        // Treat the matrix at knots[i] as a column vector.
        if (i == 0) {
          // Always start moving from the position which we're
          // currently commanding.
          DRAKE_DEMAND(iiwa_status_.utime != -1);
          knots[0](idx, 0) = iiwa_status_.joint_position_commanded[j];

        } else {
          knots[i](idx, 0) = state.joint_position[j];
        }
      }
    }

    for (int i = 0; i < plan->num_states; ++i) {
      std::cout << knots[i] << std::endl;
    }

    std::vector<double> input_time;
    for (int k = 0; k < static_cast<int>(plan->plan.size()); ++k) {
      input_time.push_back(plan->plan[k].utime / 1e6);
    }
    const Eigen::MatrixXd knot_dot = Eigen::MatrixXd::Zero(kNumJoints, 1);
    plan_.reset(new PiecewisePolynomial<double>(
        PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
            input_time, knots, knot_dot, knot_dot)));
    ++plan_number_;
  }

  void HandleStop(const ::lcm::ReceiveBuffer*, const std::string&,
                  const lcmt_robot_plan*) {
    std::cout << "Received stop command. Discarding plan." << std::endl;
    plan_.reset();
  }

  ::lcm::LCM lcm_;
  const multibody::MultibodyPlant<double>& plant_;
  int plan_number_{};
  std::unique_ptr<PiecewisePolynomial<double>> plan_;
  lcmt_iiwa_status iiwa_status_;
};

int do_main() {
  multibody::MultibodyPlant<double> plant(0.0);
  multibody::Parser(&plant).AddModelFromFile(
      FindResourceOrThrow("drake/manipulation/models/iiwa_description/urdf/"
                          "iiwa14_no_collision.urdf"));
  plant.WeldFrames(plant.world_frame(),
                   plant.GetBodyByName("base").body_frame());
  plant.Finalize();

  RobotPlanRunner runner(plant);
  runner.Run();
  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake


int main() {
  return drake::examples::kuka_iiwa_arm::do_main();
}
