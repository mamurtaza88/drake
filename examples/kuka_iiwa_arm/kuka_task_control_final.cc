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
#include <memory>

//Manually adding the libraries
#include <cmath>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <ctime> 
#include <qpmad/solver.h>
#include <chrono>


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


std::chrono::high_resolution_clock::time_point tic1, toc1;

const char* const Main_channel = "Hand_Position";  // The lcm channel for python3& 
double x=0;
double y=0;
double z=0;
double barrier_active = 1;
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
        // std::ofstream file;
        // file.open("./test.txt", std::ios_base::out);
        // printf("Received message on channel \"%s\":\n", chan.c_str());
        // // printf("  timestamp   = %ld\n", msg->timestamp);
        // // printf("  position    = (%f, %f, %f)\n", msg->position[0], msg->position[1],
        // //        msg->position[2]);
        // // printf("  orientation = (%f, %f, %f)\n", msg->orientation[0], msg->orientation[1],
        // //        msg->orientation[2]);
        // // printf("  Motion_time = %f\n", msg->motion_time);
        // // printf("  source        = '%s'\n", msg->source.c_str());
        // std::cout << msg->pnt[0] << " " << msg->pnt[1] << " " <<msg->pnt[2] << std::endl;
        // file << msg->pnt[0] << "\n"; 
        // file << msg->pnt[1] << "\n"; 
        // file << msg->pnt[2] << "\n"; 
        // file << msg->motion_time << "\n";
        // file.close();
        x = msg->pnt[0];
        y = msg->pnt[1];
        z = msg->pnt[2];
        if(msg->motion_time == 1){
          barrier_active = 1;
        }
        else if(msg->motion_time == 2){
          barrier_active = 2;
        }
        else if(msg->motion_time == 3){
          barrier_active = 3;
        }
        else if(msg->motion_time == 4){
          barrier_active = 4;
        }
        else{
          barrier_active=0;
        }
    }
};


class RobotPlanRunner {
 public:
  /// plant is aliased
  explicit RobotPlanRunner(const multibody::MultibodyPlant<double>& plant)
      : mplant(plant){
    lcm_.subscribe(kLcmStatusChannel,
                    &RobotPlanRunner::HandleStatus, this);

     //Define the initial parameters for kp and kd matrices to be used in pd controller
    double kp = 210, kv = 21, kpOr = 210*1.2, kvOr = 0;
    
    //Define the scaling factor that you want your torque to be initialized. Defining here for easy access
    mScale = 0.98;

    // mplant_diff(mplant);

    //Initializing the parameters by calling the function
    InitDynamicParam(kp, kv, kpOr, kvOr);
   
    InitAutoDiffXd();
  }

  ~RobotPlanRunner()
  {
    ee_position.close();
    ee_velocity.close();
    joint_position.close();
    joint_velocity.close();
    torque_value.close();
    target_position.close();
    barrier_position.close();
    out_barrier_position.close();
    ee_rpy.close();
    target_rpy.close();
    mu_singular.close();
    std::cout << "fail counter = " << fail_check << std::endl;

  }

  // *************************** Read Matrix to read from text file. Use to read F & G Matrices **************************

  Eigen::MatrixXd readMatrix(const char *filename)
  {
    int cols = 0, rows = 0;
    double buff[10000];

      // Read numbers from file into buffer.
    std::ifstream infile;
    infile.open(filename);
    while (! infile.eof())
    {
      std::string line;
      getline(infile, line);

      int temp_cols = 0;
      std::stringstream stream(line);
      while(! stream.eof())
        stream >> buff[cols*rows+temp_cols++];

      if (temp_cols == 0)
        continue;

      if (cols == 0)
        cols = temp_cols;

      rows++;
    }

    infile.close();

    rows--;

      // Populate matrix with numbers.
    Eigen::MatrixXd result(rows,cols);
      for (int i = 0; i < rows; i++)
       for (int j = 0; j < cols; j++)
          result(i,j) = buff[ cols*i+j ];

    return result;
  }

  void Run() {
    mCur_time_us = -1;
    int64_t start_time_us = -1;

    // Initialize the timestamp to an invalid number so we can detect
    // the first message.
    iiwa_status_.utime = mCur_time_us;

    lcmt_iiwa_command iiwa_command;
    iiwa_command.num_joints = kNumJoints;
    iiwa_command.joint_position.resize(kNumJoints, 0.);
    iiwa_command.num_torques = kNumJoints;
    iiwa_command.joint_torque.resize(kNumJoints, 0.);

    double last_x = -1;
    double last_y = -1;
    double last_z = -1;
    // double c_yaw = 0.;  // define the constant pose angles
    // double c_pitch = 0.852009;
    // double c_roll = 0.;
    int flag = 0;

    // spdlog::info("x = {}, y = {}, z = {} ",x, y, z);
        ::lcm::LCM lc_0;
    Handler handlerObject;
    lc_0.subscribe(Main_channel, &Handler::handleMessage, &handlerObject);
    while (lc_0.handle() >= 0) { 
      break;
    } 
    // spdlog::info("x = {}, y = {}, z = {} ",x, y, z);


    // std::vector<double> times;  // Control the motion velocity (motion time)
    // times.push_back(0);
    // std::ifstream file;
    // file.open("./test.txt");
    // std::string str;
    // std::vector<double> pose_info;
    // while (std::getline(file, str))
    // {
    //   pose_info.push_back(std::stod(str));
    // }
    // file.close();
    // if (last_x == -1 && last_y == -1 && last_z == -1)
    //   flag = 1; //first time to receive the command from python
    // else if (last_x != pose_info[0] || last_y != pose_info[1] || last_z != pose_info[2])
    //     flag = 1; // make sure that the command received from python changed
    // last_x = pose_info[0]; last_y = pose_info[1]; last_z = pose_info[2];
    // // math::RigidTransformd pose(
    // //     math::RollPitchYawd(c_roll, c_pitch, c_yaw),
    // //     Eigen::Vector3d(pose_info[0], pose_info[1], pose_info[2]));   // This corresponds to the input parameters
    // // // I found that ik will not provide with an exact solution as I defined in the pose variable
    // // // Instead, it will only give a close solution -> some orientation angle may be different
    // // // but surprisingly, the xyz values are almost the same    
    // // times.push_back(pose_info[3]);
    // if (flag == 1) {  // related to the motion stability  (if *new* commands from python received:
    //   // std::cout << last_x << " " << last_y << " " <<last_z << std::endl;
    //   spdlog::info("last_x = {}, last_y = {}, last_z = {} ",last_x, last_y, last_z);
    //   // pose -> goal position
    //   // times -> related to velosity
    //   flag = 0;
    // }



    while (true) {
      // Call lcm handle until at least one status message is
      // processed.
      while (0 == lcm_.handleTimeout(15) || iiwa_status_.utime == -1) {
        bias_ref = 0;//iiwa_status_.utime;
       }

      mCur_time_us = iiwa_status_.utime;// - bias_ref;
      // spdlog::info("x = {}, y = {}, z = {} ",x, y, z);
      lc_0.handleTimeout(1);
        
      tic1 = std::chrono::high_resolution_clock::now();
      UpdateDynamicParam();

      //Update Autodiff Context and plant
      UpdateAutoDiff();

      //Update to calculate J_mu
      UpdateJacobianDiff();

      //Declare the joint torque vector
      Eigen::VectorXd joint_torque_cmd = Eigen::VectorXd::Zero(7);

      //Compute the pseudo inverse of the jacobian
      mPinv_Jv_robot1 = mJv_robot1.transpose() * (mJv_robot1 * mJv_robot1.transpose() + 0.00025 * Eigen::Matrix3d::Identity()).llt().solve(Eigen::Matrix3d::Identity());
      
      //Compute the pseudo inverse of the jacobian with no damping term
      mPinv_JvND_robot1 = mJv_robot1.transpose() * (mJv_robot1 * mJv_robot1.transpose()).llt().solve(Eigen::Matrix3d::Identity());
      // std::cout << "mJv_robot1 = \n" << mJv_robot1 << std::endl;

      //Compute the torque by doing feedback linearization
      //Adding just the Coriolis term since gravity term is already added in torque controller.
      // joint_torque_cmd = mM_robot1 * mPinv_Jv_robot1 * (mDesired_ddx_position - mdJv_dq) + mCg_robot1;

      // Pass the optimization and the scaling factor that you want your torque limit to be constraint.
      RESCLF(mScale);
      // std::cout << " Main Check 1" << std::endl;
      toc1 = std::chrono::high_resolution_clock::now();
      double tic1_time = std::chrono::duration_cast<std::chrono::microseconds>(toc1 - tic1).count();
      
      // spdlog::info("Total time = {}", tic1_time);
      maxtime = std::max(maxtime,tic1_time);
      if(maxtime_old > maxtime)
      {
        spdlog::info("max time = {}", maxtime);
      }
      if(tic1_time > 4500)
      {
        spdlog::info("Total time = {}", tic1_time);
      }
      maxtime_old = maxtime;

      //Declare the RESLCF body torque
      // Eigen::VectorXd bodyTorques = Eigen::VectorXd::Zero(7);
      // bodyTorques = mM_robot1 * pinv_Jv * (mDesired_ddx_position + mddq_RESCLF_robot - mdJv_dq) + mCg_robot1;
      
      // std::cout << "mE_robot = \n" << mE_robot << std::endl;
      // std::cout << "mdE_robot = \n" << mdE_robot << std::endl;
      // std::cout << "mDesired_ddx_position = \n" << mDesired_ddx_position << std::endl;
      // std::cout << "mEE_position_ = \n" << mEE_position_ << std::endl;
      // std::cout << "Feeback Linearization Torque = \n" << joint_torque_cmd << std::endl;
      // std::cout << "mBodyTorques_QPMAd_robot1 = \n" << mBodyTorques_QPMAd_robot1 << std::endl;
      //Passing the computed torque to iiwa command to apply on robot
      for (int joint = 0; joint < kNumJoints; joint++)
      {
        iiwa_command.joint_position[joint] = iiwa_status_.joint_position_measured[joint];
        iiwa_command.joint_torque[joint] = mBodyTorques_QPMAd_robot1[joint]; //mFBL_Control_robot1(joint);////joint_torque_cmd[joint];
      }
      
      lcm_.publish(kLcmCommandChannel, &iiwa_command);
      
      // const double cur_traj_time_s = static_cast<double>(cur_time_us - start_time_us) / 1e6;

      iiwa_command.utime = iiwa_status_.utime;

      for (int joint = 0; joint < kNumJoints; joint++) {
        // iiwa_command.joint_position[joint] = desired_next(joint);
      }

      lcm_.publish(kLcmCommandChannel, &iiwa_command);
      counter++;
    }
  }

 private:
  void InitAutoDiffXd()
  {
      mplant_autodiff = systems::System<double>::ToAutoDiffXd(mplant);
      mContext_autodiff = mplant_autodiff->CreateDefaultContext();

      // mContext is passed as a pointer becuase it is itself a pointer and we want to pass its value
      mContext_autodiff->SetTimeStateAndParametersFrom(*mContext);
      mFrame_EE_autodiff = &(mplant_autodiff->GetBodyByName(mEE_link).body_frame());
      mWorld_Frame_autodiff = &(mplant_autodiff->world_frame());
      
      // AutoDiffVecXd q_autodiff(mq_robot.rows()); 
      mq_autodiff = drake::math::InitializeAutoDiff(mq_robot);  
     
      mplant_autodiff->SetPositions(mContext_autodiff.get(), mq_autodiff); 
     
      // mJacobian_autodiff = drake::math::InitializeAutoDiff(mJv_robot1);
      mJacobian_autodiff = drake::math::InitializeAutoDiff(mJacobian);

      // // std::cout << "mJacobian_autodiff = " << mJacobian_autodiff1 << std::endl;

      // mplant_autodiff->CalcJacobianTranslationalVelocity(*mContext_autodiff,
      //                                 multibody::JacobianWrtVariable::kQDot,
      //                                 *mFrame_EE_autodiff,
      //                                 Eigen::Vector3d::Zero().cast<AutoDiffXd>(),
      //                                 *mWorld_Frame_autodiff,
      //                                 *mWorld_Frame_autodiff,
      //                                 &mJacobian_autodiff 
      //                                 );

      // // Calculate spatial Jacobian. first three are angular jacobian and last 3 are linear jacobian
      mplant_autodiff->CalcJacobianSpatialVelocity(*mContext_autodiff,
                                      multibody::JacobianWrtVariable::kQDot,
                                      *mFrame_EE_autodiff,
                                      Eigen::Vector3d::Zero().cast<AutoDiffXd>(),
                                      *mWorld_Frame_autodiff,
                                      *mWorld_Frame_autodiff,
                                      &mJacobian_autodiff 
                                      );

      // std::cout << "mJacobian_autodiff.rows() = " << mJacobian_autodiff.rows() << "mJacobian_autodiff.cols()" << mJacobian_autodiff.cols() << std::endl;
      // {
        std::vector<Eigen::MatrixXd> temp(mq_robot.rows());
        mdJdq_derivative = temp;
      // }


      for (int i = 0; i < mq_autodiff.rows(); ++i)
      {
        mdJdq_derivative[i].resize(mJacobian_autodiff.rows(), mJacobian_autodiff.cols());
      }

  }
  
   //Function to initialize the parameters
  void InitDynamicParam(double kp, double kd, double kpOr, double kdOr)
  {
    //Creating default context for the plant
    mContext = mplant.CreateDefaultContext();

    //Copying end-effector name from urdf
    mEE_link = "iiwa_link_ee";

    //Copying end-effector name from urdf
    mBase_link = "iiwa_link_0";

    //Defining the frame for end-effector
    mFrame_EE = &mplant.GetBodyByName(mEE_link).body_frame();

    //Initializing the joint position and velocity
    int nv = mplant.num_velocities();
    mq_robot = Eigen::VectorXd::Zero(nv); // Joint positions. We can also use kNumJoints
    mdq_robot = Eigen::VectorXd::Zero(nv); // Joint velocities.
    // mq_robot(1) = 30*3.14/180;
    // mq_robot(3) = -60*3.14/180;

//   mq_robot(0) = 0*3.14/180;
    mq_robot(1) = 40*3.14/180;
  // mq_robot(2) = 0*3.14/180;
    mq_robot(3) = -84*3.14/180;
  // mq_robot(4) = 0*3.14/180;
    mq_robot(5) = 58*3.14/180;
  // mq_robot(6) = 0*3.14/180;


    // std::cout << "mIiwa_q_ = " << mIiwa_q_.rows() << "*" << mIiwa_q_.cols() << std::endl;
    // std::cout << "iiwa_status_.num_joints = " << iiwa_status_.num_joints << std::endl;
    
    // This didnot work becuase iiwa_status is not initialized at the moment
    // for (int joint = 0; joint < iiwa_status_.num_joints; joint++) {
    //   mIiwa_q_[joint] = iiwa_status_.joint_position_measured[joint];
    //   mIiwa_qdot_[joint] = iiwa_status_.joint_velocity_estimated[joint];
    // }
   
    //Initialize mass and coriolis term
    mM_robot1 = Eigen::MatrixXd::Zero(nv, nv);
    mCg_robot1 = Eigen::VectorXd::Zero(nv, nv);
    
    int task_dim = 3; // Dimenstion of task space. 3 for position only
    int task_dim_ori = 3;  // Dimenstion of task space. 3 for orientation only

    //Initialize dimension for desired and actual position and velocity
    mE_robot = Eigen::VectorXd::Zero(task_dim);
    mDe_robot = Eigen::VectorXd::Zero(task_dim);

    mDesired_ee_ori = Eigen::VectorXd::Zero(task_dim_ori);
    mDesired_ee_ori_vel = Eigen::VectorXd::Zero(task_dim_ori);
    mEE_position = Eigen::VectorXd::Zero(task_dim);
    mEE_velocity = Eigen::VectorXd::Zero(task_dim);
    mEE_RPY = Eigen::VectorXd::Zero(task_dim);
    mDesired_ddx_position = Eigen::VectorXd::Zero(task_dim);

    // //Orientation of the robot
    mTargetRPY = Eigen::VectorXd::Zero(task_dim);
    // mTargetRot.set(Eigen::Matrix3d::Zero());
    mTargetRot.Identity();

    // std::cout << "mTargetRot = \n" << mTargetRot << std::endl;
    
    // mEE_w_robot = Eigen::VectorXd::Zero(task_dim);
    // mEE_dw_robot = Eigen::VectorXd::Zero(task_dim);

    //Initialize the Linear and Angular Jacobian matrix
    mJv_robot1 = Eigen::MatrixXd::Zero(task_dim, nv);
    mJw_robot1 = Eigen::MatrixXd::Zero(task_dim, nv);
    
    //Initialize the product dJ*dq for linear and angular jacobian
    mdJv_dq = Eigen::Vector3d::Zero();
    mdJw_dq = Eigen::Vector3d::Zero();

    //Initialize gains for position and velocity
    mKp.setZero();
    mKv.setZero();
    for (std::size_t i = 0; i < 3; ++i)
    {
      mKp(i,i) = kp;
      mKv(i,i) = kd;

      mKpOr(i,i) = kpOr;
      mKvOr(i,i) = kdOr;
    }
   
    // Define the plant to position given by iiwa_status
    // Since mContext is a pointer, we first get its value and then pass its address
    // since arguement expect the pointer so we have to pass its address.
    // We can also use mContext_.get() 
    mplant.SetPositions(&(*mContext), mq_robot); 
    mplant.SetVelocities(mContext.get(), mdq_robot);
    
    //Get the end-effector position. mContext is passed as a pointer becuase it is itself a pointer and we want to pass its value.
    mEE_link_pose = mplant.EvalBodyPoseInWorld(*mContext, mplant.GetBodyByName(mEE_link));

    //Get the end-effector velocity in spatial representation
    mEE_link_velocity = mplant.EvalBodySpatialVelocityInWorld(*mContext, mplant.GetBodyByName(mEE_link));
    
    //Define the position by taking the translation part.
    mEE_position = mEE_link_pose.translation();

    //Define the velocity by taking the translational part
    mEE_velocity = mEE_link_velocity.translational();
    
    //Define the target position by taking the end effector position
    mTargetPos = mEE_link_pose.translation();
    
    //Define the orientation by first converting math::rotationmatrix<double> to isometry
    mTargetRot = mEE_link_pose.GetAsIsometry3();
    
    mEE_Rot = mEE_link_pose.GetAsIsometry3();

    //Get the orientation part of the isometry and then represent it in XYZ euler angles. 
    // mEE_RPY = mEE_link_pose.GetAsIsometry3();

    // mTargetRPY = (mTargetRot.linear()).eulerAngles(0, 1, 2);
    mTargetRPY << 0, 1.57, 0;

    mEE_RPY = (mEE_Rot.linear()).eulerAngles(0, 1, 2);

    // dx = Eigen::VectorXd::Zero(3);
    // dx << 0.1, 0.02, -0.05;
    // mTargetPos = mTargetPos + dx;

    mTargetVel = Eigen::VectorXd::Zero(3);

    mFreq = 0.0002; //Set frequency for the sine wave of moving target

    mAmplitude = 0.2; //Set Amplitude for the moving target


    //Get the base position. mContext is passed as a pointer becuase it is itself a pointer and we want to pass its value.
    mBase_link_pose = mplant.EvalBodyPoseInWorld(*mContext, mplant.GetBodyByName(mBase_link));

    //Define the position by taking the translation part.
    mBase_position = mBase_link_pose.translation();
    
    //Reading parameters for the 
    // mP = readMatrix("/home/murtaza/drake/examples/kuka_iiwa_arm/P.txt");
    // mF = readMatrix("/home/murtaza/drake/examples/kuka_iiwa_arm/F.txt");
    // mG = readMatrix("/home/murtaza/drake/examples/kuka_iiwa_arm/G.txt"); 
    
    //Not using files anymore as mP, mF, and mG are fine and do not change.
    mP = Eigen::MatrixXd::Zero(6, 6);
    mF = Eigen::MatrixXd::Zero(6, 6);
    mG = Eigen::MatrixXd::Zero(6, 3);

    mP << 1.73205, 0, 0, 1, 0, 0,
          0, 1.73205, 0, 0, 1, 0,
          0, 0, 1.73205, 0, 0, 1,
          1, 0, 0, 1.73205, 0, 0,
          0, 1, 0, 0, 1.73205, 0,
          0, 0, 1, 0, 0, 1.73205;

    mF << 0, 0, 0, 1, 0, 0,
          0, 0, 0, 0, 1, 0,
          0, 0, 0, 0, 0, 1,
          0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0;

    mG << 0, 0, 0,
          0, 0, 0,
          0, 0, 0,
          1, 0, 0,
          0, 1, 0,
          0, 0, 1;

    mTauLim_robot1 << 30,30,20,20,15,5,5;//80, 80, 60, 60, 55, 15, 15;//60, 60, 60, 30, 12, 12, 6; //320, 320, 176, 176, 110, 40, 40; // Need to be adjusted when adding limits

    Eigen::Vector3d ObstacleOffset(1.75, 1.75, 0.75);
    mObstaclePos = mEE_position + ObstacleOffset;
    mObstacleVel << 0, 0, 0;

    // ******************************************Declaring variables for dumping values **********
    std::string s1 = "/home/sfam/Documents/GitHub/";
    ee_position.open(s1 + "drake/examples/kuka_iiwa_arm/experiment_data/ee_position.csv");
    ee_velocity.open(s1 + "drake/examples/kuka_iiwa_arm/experiment_data/ee_velocity.csv");
    joint_position.open(s1 + "drake/examples/kuka_iiwa_arm/experiment_data/joint_position.csv");
    joint_velocity.open(s1 + "drake/examples/kuka_iiwa_arm/experiment_data/joint_velocity.csv");
    torque_value.open(s1 + "drake/examples/kuka_iiwa_arm/experiment_data/torque_value.csv");
    target_position.open(s1 + "drake/examples/kuka_iiwa_arm/experiment_data/target_position.csv");
    barrier_position.open(s1 + "drake/examples/kuka_iiwa_arm/experiment_data/barrier_position.csv");
    out_barrier_position.open(s1 + "drake/examples/kuka_iiwa_arm/experiment_data/out_barrier_position.csv");
    ee_rpy.open(s1 + "drake/examples/kuka_iiwa_arm/experiment_data/ee_rpy.csv");
    target_rpy.open(s1 + "drake/examples/kuka_iiwa_arm/experiment_data/target_rpy.csv");
    mu_singular.open(s1 + "drake/examples/kuka_iiwa_arm/experiment_data/mu_singular.csv");


    ee_position << "ee_x" << ", " << "ee_y" << ", " << "ee_z" << "," << "time_us" << std::endl;
    ee_velocity <<  "v_ee_x" << ", " << "v_ee_y" << ", " << "v_ee_z" << std::endl;
    target_position << "target_x" << ", " << "target_y" << ", " << "target_z" << "," << "angle" << std::endl;
    barrier_position << "barrier_x" << ", " << "barrier_y" << ", " << "barrier_z"  << std:: endl;
    torque_value << "T1" << ", " << "T2" << ", "<< "T3" << ", "<< "T4" << ", "<< "T5" << ", "<< "T6" << ", "<< "T7" << std::endl;
    joint_position << "q1" << ", " << "q2" << ", "<< "q3" << ", "<< "q4" << ", "<< "q5" << ", "<< "q6" << ", "<< "q7" << std::endl;            
 
    joint_velocity << "dq1" << ", " << "dq2" << ", "<< "dq3" << ", "<< "dq4" << ", "<< "dq5" << ", "<< "dq6" << ", "<< "dq7" << std::endl;
    out_barrier_position << "out_barrier_x" << ", " << "out_barrier_x" << ", " << "out_barrier_x"  << ", " << "bool_value"  <<std:: endl;

    ee_rpy << "ee_rpy_x" << ", " << "ee_rpy_y" << ", " << "ee_rpy_z" << std::endl;
    target_rpy << "target_rpy_x" << ", " << "target_rpy_y" << ", " << "target_rpy_z" << std::endl;

    mu_singular << "mu_value" << std::endl;

    // ******************************************Task Position Constraint Variable *******************
    mOuterPower = 6;
    
    mOuterBoxRadius_x = 10;
    mOuterBoxRadius_y = 10;
    mOuterBoxRadius_z = 10;

    A_OuterBox = Eigen::MatrixXd::Zero(3, 3);


    // *******************************************Singularity Avoidance Variables *************************
    std::vector<Eigen::MatrixXd> temp(7);
    mJacobian_Derivative = temp;
    mJ_mu_old = Eigen::MatrixXd::Zero(1, 7);
    mJ_mu = mJ_mu_old;
    mdJ_mu = mJ_mu_old;

  // ******************************************** Joint Position variables ********************************
    mJoint_limit_max << 2.967, 2.094, 2.967, 2.09439510239, 2.96705972839, 2.09439510239, 3.05432619099;

  }

  void UpdateAutoDiff()
  {
    mContext_autodiff->SetTimeStateAndParametersFrom(*mContext);
    mplant_autodiff->SetPositions(mContext_autodiff.get(), mq_autodiff); 
    
    // mJacobian_autodiff = drake::math::InitializeAutoDiff(mJv_robot1);
    // // std::cout << "mJacobian_autodiff = " << mJacobian_autodiff1 << std::endl;

    // mplant_autodiff->CalcJacobianTranslationalVelocity(*mContext_autodiff,
    //                                 multibody::JacobianWrtVariable::kQDot,
    //                                 *mFrame_EE_autodiff,
    //                                 Eigen::Vector3d::Zero().cast<AutoDiffXd>(),
    //                                 *mWorld_Frame_autodiff,
    //                                 *mWorld_Frame_autodiff,
    //                                 &mJacobian_autodiff 
    //                                 );

    mplant_autodiff->CalcJacobianSpatialVelocity(*mContext_autodiff,
                                      multibody::JacobianWrtVariable::kQDot,
                                      *mFrame_EE_autodiff,
                                      Eigen::Vector3d::Zero().cast<AutoDiffXd>(),
                                      *mWorld_Frame_autodiff,
                                      *mWorld_Frame_autodiff,
                                      &mJacobian_autodiff 
                                      );

  }

  void UpdateJacobianDiff()
  {
    Eigen::VectorXd q_robot1_singularity = mq_robot;
    double diff = 1e-5;
    step_size = 5e-3;

    
    Eigen::MatrixXd pinv_JvND_robot1 = mJv_robot1.transpose() * (mJv_robot1 * mJv_robot1.transpose()).inverse();
    // Eigen::MatrixXd pinv_J_AugND_robot1 = J_Aug_robot1.transpose() * (J_Aug_robot1 * J_Aug_robot1.transpose()).inverse();

    Eigen::MatrixXd Jv_product =  mJv_robot1 * mJv_robot1.transpose();
    mMu = std::sqrt(Jv_product.determinant());
    Eigen::MatrixXd Jmu_product;
    Eigen::Matrix<double, 3, 7> mJv_robot1_singularity;

    for(int i = 0; i < mq_robot.rows();i++)
    {
      q_robot1_singularity = mq_robot;
      q_robot1_singularity(i) = mq_robot(i) + diff;
      mplant.SetPositions(&(*mContext), q_robot1_singularity);
      mplant.SetVelocities(mContext.get(), mdq_robot);

       // Calculate Linear Jacobian
      mplant.CalcJacobianTranslationalVelocity(*mContext,
                                      multibody::JacobianWrtVariable::kQDot,
                                      *mFrame_EE,
                                      Eigen::Vector3d::Zero(),
                                      mplant.world_frame(),
                                      mplant.world_frame(),
                                      &mJv_robot1_singularity 
                                      );
     
      mJacobian_Derivative[i] = (mJv_robot1_singularity - mJv_robot1)/diff;
      
      Jmu_product = mJacobian_Derivative[i] * pinv_JvND_robot1;
      mJ_mu(i) = mMu * Jmu_product.trace();
      mdJ_mu(i) = (mJ_mu(i) - mJ_mu_old(i) )/step_size;
    }
    mJ_mu_old = mJ_mu;

  }

  void UpdateDynamicParam()
  {
    // std::cout << "iiwa_status_.num_joints = " << iiwa_status_.num_joints << std::endl;
    for (int joint = 0; joint < iiwa_status_.num_joints; joint++)
    {
      mq_robot[joint] = iiwa_status_.joint_position_measured[joint];
      mdq_robot[joint] = iiwa_status_.joint_velocity_estimated[joint];
    }

     // Update context
    mplant.SetPositions(&(*mContext), mq_robot);
    mplant.SetVelocities(mContext.get(), mdq_robot);
    
    // Calculate mass matrix.
    mplant.CalcMassMatrix(*mContext, &mM_robot1);
    
    // Calculate Coriolis forces.
    mplant.CalcBiasTerm(*mContext, &mCg_robot1);

    // Calculate gravity forces.
    mG_robot1 =  mplant.CalcGravityGeneralizedForces(*mContext);

    //Calculate Coriolis and Gravity Term
    mH_robot1 = mCg_robot1 - mG_robot1;

    // Get the end-effector position. mContext is passed as a pointer becuase it is itself a pointer and we want to pass its value.
    mEE_link_pose = mplant.EvalBodyPoseInWorld(*mContext, mplant.GetBodyByName(mEE_link));
    
    //Define the position by taking the translation part.
    mEE_position = mEE_link_pose.translation();
    
    //Get the end-effector velocity
    mEE_link_velocity = mplant.EvalBodySpatialVelocityInWorld(*mContext, mplant.GetBodyByName(mEE_link));

    mEE_velocity = mEE_link_velocity.translational();

    //Define the orientation by first converting math::rotationmatrix<double> to isometry
    mEE_Rot = mEE_link_pose.GetAsIsometry3();
    
    //Defining the end effector orientation in Euler XYZ to analyze it
    mEE_RPY = (mEE_Rot.linear()).eulerAngles(0, 1, 2);


    // Calculate Linear Jacobian
    mplant.CalcJacobianTranslationalVelocity(*mContext,
                                      multibody::JacobianWrtVariable::kQDot,
                                      *mFrame_EE,
                                      Eigen::Vector3d::Zero(),
                                      mplant.world_frame(),
                                      mplant.world_frame(),
                                      &mJv_robot1 
                                      );

    // Calculate spatial Jacobian. first three are angular jacobian and last 3 are linear jacobian
    mplant.CalcJacobianSpatialVelocity(*mContext,
                                      multibody::JacobianWrtVariable::kQDot,
                                      *mFrame_EE,
                                      Eigen::Vector3d::Zero(),
                                      mplant.world_frame(),
                                      mplant.world_frame(),
                                      &mJacobian 
                                      );
    

    mJw_robot1 = mJacobian.block(0,0,3,7);

    //Calculates dJ*dq
    mdJv_dq = mplant.CalcBiasTranslationalAcceleration(*mContext,
                                      multibody::JacobianWrtVariable::kV,
                                      *mFrame_EE,
                                      Eigen::Vector3d::Zero(),
                                      mplant.world_frame(),
                                      mplant.world_frame()
                                      ); 
    

    // CalcBiasSpatialAccelerationmTargetRPY << 0, 1.57, 0;
    mdJ_Aug_dq_spatial1 = mplant.CalcBiasSpatialAcceleration(*mContext,
                                      multibody::JacobianWrtVariable::kV,
                                      *mFrame_EE,
                                      Eigen::Vector3d::Zero(),
                                      mplant.world_frame(),
                                      mplant.world_frame()
                                      ); 

    mdJ_Aug_dq_robot1 = mdJ_Aug_dq_spatial1.get_coeffs();

    // if(counter > 500)
    // {  
    //   //Update the target position if necassary. For now it is not being done
    //   // dx = mAmplitude * std::sin(2 * M_PI * mFreq * counter2 * 3e-3);//mCur_time_us * 3e-3); 
    //   // dx << 0.1,0.01,0.1;
    //   if(counter2 % 100 == 0)
    //   {
    //     if(check == 0)
    //     {
    //       dx = -0.005;
    //     }
    //     else if (check == 1)
    //     {
    //       dx  = 0.005;
    //     } 
    //   }
    //   if(mTargetPos(2) > 0.67)
    //   {
    //     check = 0;
    //   }
    //   else if(mTargetPos(2) < 0.20)
    //   {
    //     check = 1;
    //   }
    //   // mTargetPos(2) = mTargetPos(2) + dx;
    
    //   dx = 0;
    //   // mTargetVel(1) = mAmplitude * 2 * M_PI * mFreq * std::cos(2 * M_PI * mFreq * counter2 * 3e-3);//mCur_time_us * 3e-3);
    //   counter2++;
    // }


    // mTargetPos << x, y, z;
    
    // dy = 0.4*std::sin(angle);
    // dz = 0.25+0.15*std::cos(angle);
    // dy = 0.6+0.2*std::sin(angle);
    // dz = 0.25+0.15;
    
    dy = 0.4*std::sin(angle);
    dz = 0.25+0.15*std::cos(angle);
    if(angle > (loops+1)*M_PI/8)
    {
      loops = loops + 1;
    }

    // if(loops < 2*16){
    //   barrier_active = 1;
    // }
    // else if(loops < 3*16+4){
    //   barrier_active = 2;
    // }
    // else if(loops < 4*16+3){
    //   barrier_active = 3;
    // }
    // else if(loops < 5*16){
    //   barrier_active = 4;
    // }
    // else if(loops < 6*16){
    //   barrier_active = 5;
    // }
    // else if(loops < 7*16){
    if(loops < 2*16){
      barrier_active = 1;
    }
    else if(loops < 3*16){
      barrier_active = 2;
    }
    else if(loops < 4*16){
      barrier_active = 3;
    }
    else if(loops < 5*16){
      barrier_active = 4;
    }
    else if(loops < 6*16){
      barrier_active = 5;
    }
    else if(loops < 7*16+3){
      barrier_active = 6;
    }
    else if(loops < 8*16+4){
      barrier_active = 7;
    }
    else if(loops < 9*16){
      barrier_active = 8;
    }
    else if(loops < 10*16){
      barrier_active = 9;
    }
    else if(loops < 11*16){
      barrier_active = 10;
    }
    else if(loops < 12*16){
      barrier_active = 11;
    }
    else if(loops < 13*16){
      barrier_active = 12;
    }
    else if(loops < 14*16){
      barrier_active = 13;
    }
    else if(loops < 15*16){
      barrier_active = 14;
    }
    else if(loops < 16*16){
      barrier_active = 15;
    }
    else if(loops < 17*16){
      barrier_active = 16;
    }
    else if(loops < 18*16){
      barrier_active = 17;
    }
    else if(loops > 19*16){
      barrier_active = 17;
    }
    
    mObstaclePos << x, y+100, z+100;//  0.628842 -0.0160403   0.207691
     if(barrier_active == 1) // Barrier off for joint limits
    {
      mJoint_limit_max << 2.967, 2.094, 2.967, 2.094, 2.967, 2.094, 3.054;
      mTargetPos << 0.6, dy, dz;//0.188;

    }
    else if(barrier_active == 2){ // Barrier on for joint limits
      mJoint_limit_max << 0.5, 1.0, 0.5, 2.094, 0.5, 1.2, 0.1;
      mTargetPos << 0.6, dy, dz;//0.188;

    }
    else if(barrier_active == 3){ // Fast Trajectory reference for joint limits
      mJoint_limit_max << 0.5, 1.0, 0.5, 2.094, 0.5, 1.2, 0.1;
      mTargetPos << 0.6, dy, dz;//0.188;
      step_ref = 4.5*1e-3;
    }
    else if(barrier_active == 4){ // Normal Trajectory reference for joint limits
      mJoint_limit_max << 0.5, 1.0, 0.5, 2.094, 0.5, 1.2, 0.1;
      mTargetPos << 0.6, dy, dz;//0.188;
      step_ref = 3*1e-3;
    }
    else if(barrier_active == 5){ // Slow Trajectory reference for joint limits
      mJoint_limit_max << 0.5, 1.0, 0.5, 2.094, 0.5, 1.2, 0.1;
      mTargetPos << 0.6, dy, dz;//0.188;
      step_ref = 1*1e-3;
    }
    else if(barrier_active == 6){ // Normal Trajectory reference and no barrier
      mJoint_limit_max << 2.967, 2.094, 2.967, 2.094, 2.967, 2.094, 3.054;
      mOuterBoxRadius_y = 1;
      mTargetPos << 0.6, dy, dz;//0.188;
      step_ref = 3*1e-3;
    }
    // else if(barrier_active == 7){ // Outside Barrier off
    //   mOuterBoxRadius_y = 1;
    //   mTargetPos << 0.6, dy, dz;//0.188;
    // }
    if(barrier_active == 7) // Outside Barrier on and end-effector will move in
    {
      mOuterBoxRadius_y = 0.25;
      mTargetPos << 0.6, dy, dz;//0.188;
    }
    else if(barrier_active == 8) // Outside Barrier off and end-effector will move out
    {
      mOuterBoxRadius_y = 1;
      mTargetPos << 0.6, dy, dz;//0.188;
    }
    else if(barrier_active == 9){ // Fast Trajectory reference for outside barrier
      mOuterBoxRadius_y = 0.25;
      step_ref = 4.5*1e-3;
      mTargetPos << 0.6, dy, dz;//0.188;
    }
    else if(barrier_active == 10){ // Normal Trajectory reference for outside barrier
      mOuterBoxRadius_y = 0.25;
      step_ref = 3*1e-3;
      mTargetPos << 0.6, dy, dz;//0.188;
    }
    else if(barrier_active == 11){ // Slow Trajectory reference for outside barrier
      mOuterBoxRadius_y = 0.25;
      step_ref = 1*1e-3;
      mTargetPos << 0.6, dy, dz;//0.188;
    } 
    // if(barrier_active == 6) // Barrier off for joint position limit
    // {
    //   mOuterBoxRadius_y = 2.25;
    //   mJoint_limit_max << 2.967, 2.094, 2.967, 2.09439510239, 2.96705972839, 2.09439510239, 3.05432619099;
    //   step_ref = 3*1e-3;
    //   mTargetPos << 0.6, dy, dz;//0.188;
    // }
    // else if(barrier_active == 7){ // Barrier on for joint position limit
    //   mOuterBoxRadius_y = 2.5;
    //   step_ref = 3*1e-3;
    //   mJoint_limit_max << 0.5, 1.0, 0.5, 2.094, 0.5, 1.2, 0.1;
    //   mTargetPos << 0.6, dy, dz;//0.188;
    // }
    // else if(barrier_active == 8){ // Fast Trajectory reference without joint position limit
    //   mOuterBoxRadius_y = 2.25;
    //   step_ref = 4.5*1e-3;
    //   mJoint_limit_max << 2.967, 2.094, 2.967, 2.09439510239, 2.96705972839, 2.09439510239, 3.05432619099;
    //   mTargetPos << 0.6, dy, dz;//0.188;
    // }
    // else if(barrier_active == 9){ // Fast Trajectory reference with joint position limit
    //   mOuterBoxRadius_y = 2.25;
    //   step_ref = 4.5*1e-3;
    //   mJoint_limit_max << 0.5, 1.0, 0.5, 2.094, 0.5, 1.2, 0.1;
    //   mTargetPos << 0.6, dy, dz;//0.188;
    // }
    // else if(barrier_active == 10){ // Normal Trajectory reference for joint position limit
    //   mOuterBoxRadius_y = 2.25;
    //   step_ref = 3*1e-3;
    //   // mJoint_limit_max << 0.5, 2.094, 2.967, 2.094, 2.96705972839, 2.09, 3.054;
    //   mJoint_limit_max << 0.5, 1.0, 0.5, 2.094, 0.5, 1.2, 0.1;
    //   mTargetPos << 0.6, dy, dz;//0.188;
    // }
    // else if(barrier_active == 11){ // Slow Trajectory reference without joint position limit
    //   mOuterBoxRadius_y = 2.25;
    //   step_ref = 1*1e-3;
    //   mJoint_limit_max << 2.967, 2.094, 2.967, 2.09439510239, 2.96705972839, 2.09439510239, 3.05432619099;
    //   mTargetPos << 0.6, dy, dz;//0.188;
    // }
    // else if(barrier_active == 12){ // Slow Trajectory reference with joint position limit
    //   mOuterBoxRadius_y = 2.25;
    //   step_ref = 1*1e-3;
    //   // mJoint_limit_max << 0.5, 2.094, 2.967, 2.094, 2.96705972839, 2.09, 3.054;
    //   mJoint_limit_max << 0.5, 1.0, 0.5, 2.094, 0.5, 1.2, 0.1;
    //   mTargetPos << 0.6, dy, dz;//0.188;
    // }
    if(barrier_active == 12) // Barrier off for singularity avoidance
    {
      dy = 0.6+0.2*std::sin(angle);
      dz = 0.25+0.15;
      step_ref = 3*1e-3;
      mEpsilon = 0.01;
      mTargetPos << dy, 0, dz;//0.188;
    }
    else if(barrier_active == 13){ // Barrier on for singularity avoidance
      dy = 0.6+0.2*std::sin(angle);
      dz = 0.25+0.15;
      mEpsilon = 0.15;
      mTargetPos << dy, 0, dz;//0.188;
    }
    else if(barrier_active == 14){ // Fast Trajectory reference for singularity avoidance
      dy = 0.6+0.2*std::sin(angle);
      dz = 0.25+0.15;
      mEpsilon = 0.15;
      step_ref = 4.5*1e-3;
      mTargetPos << dy, 0, dz;//0.188;
    }
    else if(barrier_active == 15){ // Normal Trajectory reference for singularity avoidance
      dy = 0.6+0.2*std::sin(angle);
      dz = 0.25+0.15;
      mEpsilon = 0.15;
      step_ref = 3*1e-3;
      mTargetPos << dy, 0, dz;//0.188;
    }
    else if(barrier_active == 16){ // Slow Trajectory reference for singularity avoidance
      dy = 0.6+0.2*std::sin(angle);
      dz = 0.25+0.15;
      mEpsilon = 0.15;
      step_ref = 1.5*1e-3;
      mTargetPos << dy, 0, dz;//0.188;
    }    
    else if(barrier_active == 17){ // Final position defining the end of experiment
      dy = 0;
      dz = 0.25+0.15;
    }

    spdlog::info("barrier_active = {}", barrier_active);


    // else if(barrier_active == 6){ // weird trajectory
    //   mOuterBoxRadius_y = 1;
    //   dy = 0.4*std::sin(angle);
    //   dz = 0.25+0.15*std::cos(angle2);
    // }
    // else if(barrier_active == 7){ // expanding barrier
    //   mOuterBoxRadius_y = 0.25+0.000025*counter_barrier;
    //   counter_barrier = counter_barrier+1;
    //   dy = 0.4*std::sin(angle);
    //   dz = 0.25+0.15*std::cos(angle2);
    // }
    // else if(barrier_active == 8){ // expanding barrier
    //   mOuterBoxRadius_y = 0.25+0.000025*counter_barrier;
    //   counter_barrier = counter_barrier+1;
    //   dy = 0;
    //   dz = 0.25+0.15;
    // }

    angle2 = angle2 + 3*step_ref;
    angle = angle + step_ref; //3*1e-3

    angle2 = angle2 + 3*step_ref;
    angle = angle + step_ref; //3*1e-3
    // mTargetPos << 0.6, dy, dz;//0.188;

    // mTargetPos << dy, 0, dz;//0.188;
    // mTargetPos << 0.6, 0, 0.4;//0.188;


    //Error value between target position and end effector position
    mE_robot = mTargetPos - mEE_position;
    mDe_robot = mTargetVel - mEE_velocity; //Error value between target velocity and end effector velocity
    // std::cout << "mE_robot = \n" << mE_robot << std::endl;
    // spdlog::info("mE_robot = \n{}",mE_robot );
    // // // spdlog::info("mDe_robot = \n{}",mDe_robot );
    // spdlog::info("mTargetPos = \n{}",mTargetPos );
    // spdlog::info("mEE_position = \n{}",mEE_position );
    // spdlog::info("mObstaclePos = \n{}",mObstaclePos );

    // spdlog::info("dx = \n{}",dx );

    // spdlog::info("mTargetRPY = \n{}",mTargetRPY);
    // spdlog::info("mEE_RPY = \n{}",mEE_RPY);


    // spdlog::info("mTargetVel = \n{}",mTargetVel );
    // spdlog::info("mEE_velocity = \n{}",mEE_velocity );

    // std::cout << "mDe_robot = \n" << mDe_robot << std::endl;


    mDesired_ddx_position = mKp*mE_robot + mKv*mDe_robot; //Desired acceleration as PD control

    // Orientation control part
    // End-effector Orientation for robot
    Eigen::Quaterniond quat_robot1(mEE_Rot.linear());
    double quat_w_robot1 = quat_robot1.w();
    Eigen::Vector3d quat_xyz_robot1(
        quat_robot1.x(), quat_robot1.y(), quat_robot1.z());
    
    // spdlog::info(" quatee_w before = {}",quat_w_robot1);
    // spdlog::info(" quatee_xyz before = {}",quat_xyz_robot1.transpose());
   
    if (quat_w_robot1 < 0) {
        quat_w_robot1 *= -1.0;
        quat_xyz_robot1 *= -1.0;
    }
   
    // spdlog::info(" quatee_w after = {}",quat_w_robot1);
    // spdlog::info(" quatee_xyz after = {}",quat_xyz_robot1.transpose());

    Eigen::Quaterniond quatRef_robot1(
        Eigen::AngleAxisd(mTargetRPY(0), Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(mTargetRPY(1), Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(mTargetRPY(2), Eigen::Vector3d::UnitZ()));

    double quatRef_w_robot1 = quatRef_robot1.w();
    Eigen::Vector3d quatRef_xyz_robot1(
        quatRef_robot1.x(), quatRef_robot1.y(), quatRef_robot1.z());
   
    // spdlog::info(" quatRef_w before = {}",quatRef_w_robot1);
    // spdlog::info(" quatRef_xyz before = {}",quatRef_xyz_robot1.transpose());
   
    if (quatRef_w_robot1 < 0) {
        quatRef_w_robot1 *= -1.0;
        quatRef_xyz_robot1 *= -1.0;
    }
   
    // spdlog::info(" quatRef_w after = {}",quatRef_w_robot1);
    // spdlog::info(" quatRef_xyz after = {}",quatRef_xyz_robot1.transpose());

    if (std::pow(-quat_w_robot1 - quatRef_w_robot1, 2)
            + std::pow((-quat_xyz_robot1 - quatRef_xyz_robot1).norm(), 2)
        < std::pow(quat_w_robot1 - quatRef_w_robot1, 2)
              + std::pow((-quat_xyz_robot1 - quatRef_xyz_robot1).norm(), 2)) {
        quat_w_robot1 *= -1.0;
        quat_xyz_robot1 *= -1.0;
        // spdlog::info("Inside Loop");
    }
    
    mQuatError_xyz_robot1 = quatRef_w_robot1 * quat_xyz_robot1
                           - quat_w_robot1 * quatRef_xyz_robot1
                           + quatRef_xyz_robot1.cross(quat_xyz_robot1);
    // spdlog::info(" mQuatError_xyz_robot1 = {}",mQuatError_xyz_robot1.transpose());


    mEE_w_robot = mJw_robot1 * mdq_robot; // mEndEffector->getAngularVelocity() = Jw * dq
    mDwref_robot1 = -mKpOr * mQuatError_xyz_robot1 - mKvOr * mEE_w_robot;
    // std::cout << " Update DynamicPara Check 1" << std::endl;
    // spdlog::info(" Update Dynamic Parameter Done");

  }

  void RESCLF(double scaling)
  {
      // ***************** Writing our own controller  - Feedback Linearization *******************************
    Eigen::Matrix J_Aug_robot1 = mJacobian;
    
    //  Pseudo inverse augmented jacobian and product of mass and pseudo inverse jacobian for robot 1
    Eigen::MatrixXd pinv_J_Aug_robot1 = J_Aug_robot1.transpose() * (J_Aug_robot1 * J_Aug_robot1.transpose() + 0.0025 * Eigen::MatrixXd::Identity(6, 6)).inverse();
  
    //Augmented Jacobian without any damping part
    Eigen::MatrixXd pinv_J_AugND_robot1 = J_Aug_robot1.transpose() * (J_Aug_robot1 * J_Aug_robot1.transpose()).inverse();
    Eigen::MatrixXd M2_aug_robot1 = mM_robot1 * pinv_J_AugND_robot1;
    Eigen::MatrixXd M2_position_robot1 = mM_robot1 * mPinv_JvND_robot1;



    // // Inverse of Mass Matrix for robot1
    Eigen::MatrixXd invM_robot1 = mM_robot1.inverse();

    Eigen::Matrix<double, 6, 1> desired_ddx_Aug_robot1; 
    desired_ddx_Aug_robot1 << mDwref_robot1, mDesired_ddx_position ;
    // std::cout << "mDesired_ddx_position = \n" << mDesired_ddx_position << std::endl;
    // std::cout << "mDwref_robot1 = \n" << mDwref_robot1 << std::endl;
    

    //  Feedback linearization controller for robot1
    mFBL_Control_robot1 = M2_aug_robot1 * (desired_ddx_Aug_robot1 - mdJ_Aug_dq_robot1) + mCg_robot1;
    // mFBL_Control_robot1 = M2_position_robot1 * (mDesired_ddx_position - mdJv_dq) + mCg_robot1;

    size_t n = mF.rows();
    // std::cout << "n = mF.rows() = " << n <<  std::endl;

    //Define states for each operational space for robot1
    Eigen::Matrix<double, 6,1> EE_Eta_robot1, Or_Eta_robot1;
    EE_Eta_robot1 << mE_robot, mDe_robot; //x - targetPosition,dx;
    Or_Eta_robot1 << mQuatError_xyz_robot1, mEE_w_robot;

    // Defining eta for error in robot1
    Eigen::VectorXd mEta1_robot1(n), mEta2_robot1(n);
    mEta1_robot1 << EE_Eta_robot1;
    mEta2_robot1 << Or_Eta_robot1;

    // Defining the LfV_x, LgV_x and V_x for position in robot1
    Eigen::MatrixXd LfV_x_pos_robot1 = mEta1_robot1.transpose()*(mF.transpose()*mP+mP*mF)*mEta1_robot1;
    Eigen::MatrixXd LgV_x_pos_robot1 = 2*mEta1_robot1.transpose()*mP*mG;
    Eigen::MatrixXd V_x_pos_robot1 = mEta1_robot1.transpose()*mP*mEta1_robot1;

    // Defining the LfV_x, LgV_x and V_x for orientation in robot1
    Eigen::MatrixXd LfV_x_ori_robot1 = mEta2_robot1.transpose()*(mF.transpose()*mP+mP*mF)*mEta2_robot1;
    Eigen::MatrixXd LgV_x_ori_robot1 = 2*mEta2_robot1.transpose()*mP*mG;
    Eigen::MatrixXd V_x_ori_robot1 = mEta2_robot1.transpose()*mP*mEta2_robot1;
    // std::cout << " RESCLF Check 1" << std::endl;

    // //*****************************************************Barrier Part*************************************
    // //Adding Barrier Stuff
    Eigen::Vector3d x_robot1  = mEE_position;
    Eigen::Vector3d dx_robot1  = mEE_velocity;

    Eigen::Vector3d x_b = mObstaclePos; //mObstacle->getWorldTransform().translation();

    Eigen::Vector3d ddx_b(0,0,0); //= mObstacleAcceleration;

    Eigen::Vector3d e_b = x_robot1 - mObstaclePos;

    Eigen::Vector3d de_b = dx_robot1 - mObstacleVel;

    Eigen::Matrix3d A_Obstacle = Eigen::MatrixXd::Zero(3,3);
    // A_Obstacle.setZero();
    double mObstacleRadius = 0.20;

    A_Obstacle(0, 0) = 1 / (mObstacleRadius * mObstacleRadius);
    A_Obstacle(1, 1) = 1 / (mObstacleRadius * mObstacleRadius);
    A_Obstacle(2, 2) = 1 / (mObstacleRadius * mObstacleRadius);

    // Degine Eta for Exponential Barrier Function
    Eigen::Vector2d eta;
    eta << e_b.transpose() * A_Obstacle * e_b - 1, 2 * e_b.transpose() * A_Obstacle * de_b;

    // Define K matrix containing poles. Not using this formulation. Leaving it for reference.
    double p0, p1, h_0, h_1, temp1;
    h_0 = e_b.transpose() * A_Obstacle * e_b - 1;
    h_1 = 2 * e_b.transpose() * A_Obstacle * de_b;
    temp1 = h_1 / h_0;
    p1 = 50 + temp1;
    p0 = 500;

    //Dealing with K_b for Barrier Formualation
    // K_b << 500,80;
    Eigen::Matrix<double, 1, 2> K_b;
    K_b << 45, 15; //2000,800;


    // std::cout << " RESCLF Check 2" << std::endl;

    //L_G_CBF deals with formulation for input \Tau, whereas L_G_CBF_OS deals with \mu from operational space by expanding \Tau.
    //Similarly for L_F_CBF and L_F_CBF_OS.
    Eigen::Matrix<double, 1, 7> L_G_CBF = 2 * e_b.transpose() * A_Obstacle * mJv_robot1 * invM_robot1;
    Eigen::Matrix<double, 1, 3> L_G_CBF_OS = 2 * (e_b).transpose() * A_Obstacle * mJv_robot1 * invM_robot1 * mM_robot1 * mPinv_JvND_robot1;

    // Eigen::Matrix<double, 1, 1> L_F_CBF = 2 * de_b.transpose() * A_Obstacle * de_b + 2 * e_b.transpose() * A_Obstacle * (mdJv_dq - mJv_robot1 * invM_robot1 * mCg_robot1 - ddx_b);
    // Eigen::Matrix<double, 1, 1> L_F_CBF_OS = 2 * de_b.transpose() * A_Obstacle * de_b + 2 * e_b.transpose() * A_Obstacle * (mdJv_dq - mJv_robot1 * invM_robot1 * mCg_robot1 - ddx_b) + 2 * e_b.transpose() * A_Obstacle * mJv_robot1 * invM_robot1 * (mM_robot1 * mPinv_JvND_robot1 * (mDesired_ddx_position - mdJv_dq) + mCg_robot1);
    Eigen::Matrix<double, 1, 1> L_F_CBF = 2 * de_b.transpose() * A_Obstacle * de_b + 2 * e_b.transpose() * A_Obstacle * (mdJv_dq - mJv_robot1 * invM_robot1 * mH_robot1 - ddx_b);
    Eigen::Matrix<double, 1, 1> L_F_CBF_OS = 2 * de_b.transpose() * A_Obstacle * de_b + 2 * e_b.transpose() * A_Obstacle * (mdJv_dq - mJv_robot1 * invM_robot1 * mH_robot1 - ddx_b) + 2 * e_b.transpose() * A_Obstacle * mJv_robot1 * invM_robot1 * (mM_robot1 * mPinv_JvND_robot1 * (mDesired_ddx_position - mdJv_dq) + mH_robot1);
  
    Eigen::Vector2d eta_CBF = eta;
    Eigen::Matrix<double, 1, 2> K_CBF = K_b;

    // spdlog::info("L_G_CBF_OS = {}", L_G_CBF_OS);
    // spdlog::info("L_F_CBF_OS = {}", L_F_CBF_OS);
    // spdlog::info("L_F_CBF_OS(0,0) = {}", L_F_CBF_OS(0,0));
    // spdlog::info("K_CBF = {}", K_CBF);

    // spdlog::info("eta = {}", eta);
    // spdlog::info("K_CBF * eta_CBF = {}", K_b*eta);
    // spdlog::info("-(L_F_CBF_OS(0, 0) + K_b * eta) = {}", -(L_F_CBF_OS(0, 0) + K_b * eta));


// // Obtaining the obstacle information and EndEffector Information
//     Eigen::Vector3d x_b = mObstacle->getWorldTransform().translation();
 
//     Eigen::Vector3d ddx_b = mObstacle->getLinearAcceleration();

//     Eigen::Vector3d e_b =  mEndEffector->getWorldTransform()*mOffset - mObstacle->getWorldTransform().translation();

//     Eigen::Vector3d de_b = mEndEffector->getLinearVelocity() - mObstacle->getLinearVelocity();
  
//     Eigen::Matrix<double, 3,3> A_Obstacle = Eigen::MatrixXd::Zero(3,3);

//     double mObstacleRadius = 0.20;
 
//     A_Obstacle(0,0) = 1/(mObstacleRadius*mObstacleRadius);
//     A_Obstacle(1,1) = 1/(mObstacleRadius*mObstacleRadius);
//     A_Obstacle(2,2) = 1/(mObstacleRadius*mObstacleRadius);

//     // Degine Eta for Exponential Barrier Function
//     Eigen::Matrix<double,2,1> eta;
//     eta << e_b.transpose()*A_Obstacle*e_b - 1,2*e_b.transpose()*A_Obstacle*de_b;

//     // Define K matrix containing poles. Not using this formulation. Leaving it for reference.
//     double p0, p1, h_0, h_1, temp1;
//     h_0 = e_b.transpose()*A_Obstacle*e_b - 1;
//     h_1 = 2*e_b.transpose()*A_Obstacle*de_b;
//     temp1 = h_1/h_0;
//     p1 = 50 + temp1;
//     p0 = 500;

//     //Dealing with K_b for Barrier Formualation
//     Eigen::Matrix<double,1,2> K_b;
//     // K_b << 500,80;
//     K_b << 160,40;//2000,800;

//     //L_G_CBF deals with formulation for input \Tau, whereas L_G_CBF_OS deals with \mu from operational space by expanding \Tau.
//     //Similarly for L_F_CBF and L_F_CBF_OS.
//     Eigen::MatrixXd L_G_CBF = 2 * e_b.transpose() * A_Obstacle * Jv_robot1 * invM_robot1;
//     Eigen::MatrixXd L_G_CBF_OS = 2*(e_b).transpose()*A_Obstacle*Jv_robot1*invM_robot1*M_robot1*pinv_Jv_robot1;;
//     Eigen::MatrixXd L_F_CBF = 2 * de_b.transpose() * A_Obstacle * de_b + 2 * e_b.transpose() * A_Obstacle * (dJv_robot1*dq_robot1 - Jv_robot1*invM_robot1*Cg_robot1 - ddx_b);
//     Eigen::Matrix<double,1,1> L_F_CBF_OS = 2*de_b.transpose()*A_Obstacle*de_b + 2*e_b.transpose()*A_Obstacle*(dJv_robot1*dq_robot1 - Jv_robot1*invM_robot1*Cg_robot1 - ddx_b)
//                 + 2*e_b.transpose()*A_Obstacle*Jv_robot1*invM_robot1*(M_robot1*pinv_Jv_robot1*(desired_ddx_pos_robot1 - dJv_robot1*dq_robot1)+Cg_robot1);
//     Eigen::MatrixXd eta_CBF = eta;
//     Eigen::MatrixXd K_CBF = K_b;


// *************************** Task Position Constraint using Higher ORder Barrier ***********************

    //Defining the outer box for containment of the manipulator
    Eigen::Vector3d x_b_Outer = mBase_position;// = mOuterBoxOffset; //mOuterBox->getWorldTransform().translation(); //mOuterBox->getWorldTransform()*mOuterBoxOffset;//

    //Defining the error for end-effector and outer box position
    Eigen::Vector3d e_b_outer = x_robot1 - x_b_Outer; //mEndEffector->getWorldTransform().translation() - mOuterBox->getWorldTransform().translation();

    //Defining the error for end-effector and outer box velocity. Since outerbox velocity do not move, we are not considering it since its zero.
    Eigen::Vector3d de_b_outer = dx_robot1;

    //Acceleration of the outer box is zero but writing it here for the sake of completion.
    Eigen::Vector3d ddx_b_outer = Eigen::Vector3d::Zero();
    double x11 = std::pow(mOuterBoxRadius_x, mOuterPower);
    double y11 = std::pow(mOuterBoxRadius_y, mOuterPower);
    double z11 = std::pow(mOuterBoxRadius_z, mOuterPower);

    // spdlog::info("x11 = {}", x11);  
    // spdlog::info("y11 = {}", y11);
    // spdlog::info("z11 = {}", z11);

    A_OuterBox(0, 0) = 1 / x11;
    A_OuterBox(1, 1) = 1 / y11;
    A_OuterBox(2, 2) = 1 / z11;
    // spdlog::info("A_OuterBox = \n{}", A_OuterBox);  

    int pow_out =  mOuterPower / 2;
    //Writing the first term for nth term. First casting as array to do element wise power
    Eigen::Vector3d e_b_n_power = (e_b_outer.array().abs()).pow(pow_out);

    //Computing power with respect to n-1
    Eigen::Vector3d e_b_n_1_power = (pow_out) * e_b_outer.array().sign() * e_b_outer.array().pow((pow_out) - 1);
    Eigen::Vector3d de_b_outer_n_power =  e_b_n_1_power.array() * de_b_outer.array();

    //Computing power with respect to n-2
    Eigen::Vector3d e_b_n_2_power = e_b_outer.array().pow(pow_out - 2);
    Eigen::Vector3d dde_n_power = e_b_outer.array().sign() * (pow_out) * (pow_out - 1) * e_b_n_2_power.array() * (de_b_outer.array()*de_b_outer.array());

    // Defining the X_Dot in nth power formulation
    double h_nth_Outer = 1 - e_b_n_power.transpose() * A_OuterBox * e_b_n_power;
    double dh_nth_Outer = -2 * e_b_n_power.transpose() * A_OuterBox * de_b_outer_n_power;

    // Degine Eta for Exponential Barrier Function
    Eigen::Matrix<double, 2, 1> eta_Outer;
    eta_Outer << h_nth_Outer, dh_nth_Outer;

    Eigen::Matrix<double, 1, 2> K_b_Outer;
    K_b_Outer << 680, 120; //2000,800;

      // //Computing the terms for nth order for L_G to be used in final expression
    Eigen::Matrix<double, 3, 3> ddx_outer_input = mJv_robot1 * invM_robot1 * mM_robot1 * mPinv_JvND_robot1;
    Eigen::MatrixXd ddx_outer_product_LG = ddx_outer_input.array().colwise() * e_b_n_1_power.array();

    // // Computing the terms for nth order for L_F to be used in final expression
    // Eigen::MatrixXd ddx_outer_other = dJv_robot1 * dq_robot1 - mJv_robot1 * invM_robot1 * Cg_robot1 + mJv_robot1 * invM_robot1 * (mM_robot1 * mPinv_Jv_robot1 * (desired_ddx_pos_robot1 - dJv_robot1 * dq_robot1) + Cg_robot1);
    Eigen::MatrixXd ddx_outer_other = mdJv_dq - mJv_robot1 * invM_robot1 * mH_robot1 + mJv_robot1 * invM_robot1 * (mM_robot1 * mPinv_JvND_robot1 * (mDesired_ddx_position - mdJv_dq) + mH_robot1);
    Eigen::MatrixXd ddx_outer_product_LF = ddx_outer_other.array().colwise() * e_b_n_1_power.array();

    Eigen::MatrixXd L_G_CBF_OS_Outer = -2 * e_b_n_power.transpose() * A_OuterBox * ddx_outer_product_LG;
    Eigen::MatrixXd L_F_CBF_OS_Outer = -2 * de_b_outer_n_power.transpose() * A_OuterBox * de_b_outer_n_power - 2 * e_b_n_power.transpose() * A_OuterBox * (dde_n_power + ddx_outer_product_LF);
    // // 
    Eigen::MatrixXd K_eta_CBF_Outer = K_b_Outer * eta_Outer;

    // spdlog::info("L_G_CBF_OS_Outer = {}", L_G_CBF_OS_Outer);
    // spdlog::info("L_F_CBF_OS_Outer = {}", L_F_CBF_OS_Outer);
    // spdlog::info("q = {}", mq_robot.transpose());
    // spdlog::info("dq = {}", mdq_robot.transpose());
    // spdlog::info("mTargetRPY = {}", mTargetRPY.transpose());
    // spdlog::info("mEE_RPY = {}", mEE_RPY.transpose());
    // spdlog::info("LgV_x_ori_robot1 = {}", LgV_x_ori_robot1);
    // spdlog::info("-LfV_x_pos_robot1(0, 0) = {}", -LfV_x_pos_robot1(0, 0));
    
    // spdlog::info("- gamma * V_x_pos_robot1(0, 0); = {}", - 1e-3 * V_x_pos_robot1(0, 0));

    
    // spdlog::info("eta = {}", eta);
    // spdlog::info("K_CBF * eta_CBF = {}", K_b*eta);
    // spdlog::info("-(L_F_CBF_OS(0, 0) + K_b * eta) = {}", -(L_F_CBF_OS(0, 0) + K_b * eta));

// // *************************************** Joint Position Constraints *********************************************************

    //Here e_joint is just the position becuase the limit range is defined in A_Joint matrix and error do not make sense. 
    // Wrote it for the generalization.

    Eigen::Matrix<double,7,1> e_joint = mq_robot;
    Eigen::Matrix<double,7,1> de_joint = mdq_robot;

   //Define the power of the Joint. Must be even number for the ellipsoid
    mJointPower = 14;

    // mJoint_limit_min << -2.967, -2.094, -2.967, -2.094, -2.967, -2.094, -3.054;
    // mJoint_limit_max << 2.967, 2.094, 2.967, 2.09439510239, 2.96705972839, 2.09439510239, 3.05432619099;
    // mJoint_limit_max << 0.5, 1, 2.967, 2, 2.96705972839, 2.09, 3.054;

    // std::cout << "mJoint_limit_max = " << mJoint_limit_max << std::endl;

    Eigen::Matrix<double, 7,7> A_Joint = Eigen::MatrixXd::Zero(7,7);
    A_Joint(0,0) =  1/(std::pow(std::abs(mJoint_limit_max(0)),mJointPower));
    A_Joint(1,1) =  1/(std::pow(std::abs(mJoint_limit_max(1)),mJointPower));
    A_Joint(2,2) =  1/(std::pow(std::abs(mJoint_limit_max(2)),mJointPower));
    A_Joint(3,3) =  1/(std::pow(std::abs(mJoint_limit_max(3)),mJointPower));
    A_Joint(4,4) =  1/(std::pow(std::abs(mJoint_limit_max(4)),mJointPower));
    A_Joint(5,5) =  1/(std::pow(std::abs(mJoint_limit_max(5)),mJointPower));
    A_Joint(6,6) =  1/(std::pow(std::abs(mJoint_limit_max(6)),mJointPower));

    int pow_joint =  mJointPower / 2;
    //Writing the first term for nth term. First casting as array to do element wise power
    Eigen::Matrix<double,7,1> e_joint_n_power = (e_joint.array().abs()).pow(pow_joint);

    //Computing power with respect to n-1
    Eigen::Matrix<double,7,1> e_joint_n_1_power = (pow_joint) * e_joint.array().sign() * e_joint.array().pow(pow_joint - 1);
    Eigen::Matrix<double,7,1> de_joint_n_power =  e_joint_n_1_power.array() * de_joint.array();

    //Computing power with respect to n-2
    Eigen::Matrix<double,7,1> e_joint_n_2_power = e_joint.array().pow(pow_joint - 2);
    Eigen::Matrix<double,7,1> dde_joint_n_power = e_joint.array().sign() * (pow_joint) * (pow_joint - 1) * e_joint_n_2_power.array() * (de_joint.array()*de_joint.array());

    // Defining the X_Dot in nth power formulation
    double h_nth_joint = 1 - e_joint_n_power.transpose() * A_Joint * e_joint_n_power;
    double dh_nth_joint = -2 * e_joint_n_power.transpose() * A_Joint * de_joint_n_power;

  // Degine Eta for Exponential Barrier Function
    Eigen::Matrix<double, 2, 1> eta_Joint;
    eta_Joint << h_nth_joint, dh_nth_joint;

    Eigen::Matrix<double, 1, 2> K_b_Joint;
    K_b_Joint << 400, 100; //2000,800;

    // //Computing the terms for nth order for L_G to be used in final expression
    // Eigen::Matrix<double, 7, 3> ddx_joint_input = invM_robot1 * mM_robot1 * mPinv_JvND_robot1;
    Eigen::MatrixXd ddx_joint_input = invM_robot1 * mM_robot1 * pinv_J_AugND_robot1;
    Eigen::MatrixXd ddx_joint_product_LG = ddx_joint_input.array().colwise() * e_joint_n_1_power.array();

    // // Computing the terms for nth order for L_F to be used in final expression
    // Eigen::MatrixXd ddx_joint_other =  -invM_robot1 * mH_robot1 + invM_robot1 * (mM_robot1 * mPinv_JvND_robot1 * (mDesired_ddx_position - mdJv_dq) + mH_robot1);
    Eigen::MatrixXd ddx_joint_other =  -invM_robot1 * mH_robot1 + invM_robot1 * (mM_robot1 * pinv_J_AugND_robot1 * (desired_ddx_Aug_robot1 - mdJ_Aug_dq_robot1) + mH_robot1);
    Eigen::MatrixXd ddx_joint_product_LF = ddx_joint_other.array().colwise() * e_joint_n_1_power.array();

    Eigen::MatrixXd L_G_CBF_Joint = -2 * e_joint_n_power.transpose() * A_Joint * ddx_joint_product_LG;
    Eigen::MatrixXd L_F_CBF_Joint = -2 * de_joint_n_power.transpose() * A_Joint * de_joint_n_power - 2 * e_joint_n_power.transpose() * A_Joint * (dde_joint_n_power + ddx_joint_product_LF);
  // // 
    Eigen::MatrixXd K_eta_CBF_Joint = K_b_Joint * eta_Joint;

//   // /* ************************************************Singularity Avoidance ************************************** */

//     // Code copied from https://stackoverflow.com/questions/71721926/autodiff-for-jacobian-derivative-with-respect-to-individual-joint-angles/71722242#71722242
        
//     for (int i = 0; i < mq_autodiff.rows(); ++i)
//     {
//       // dJidq stores the gradient of the ∂J.col(i)/∂q, namely dJidq(j, k) = ∂J(j, i)/∂q(k)
//       auto dJidq = drake::math::ExtractGradient(mJacobian_autodiff.col(i));
//       for (int j = 0; j < static_cast<int>(mdJdq_derivative.size()); ++j) {
//         mdJdq_derivative[j].col(i) = dJidq.col(j);
//       }
//     }

//     //No Damping Pseudo Inverse
//     // Eigen::MatrixXd Jv_product =  mJv_robot1 * mJv_robot1.transpose();
//     Eigen::MatrixXd Jv_product =  mJacobian * mJacobian.transpose();
//     double mu = std::sqrt(Jv_product.determinant());

//     //Defining J_mu as the jacobian with respect to manipulability index
//     Eigen::Matrix<double, 1, 7> J_mu, dJ_mu;
//     double trace_mat;
//     Eigen::MatrixXd Jmu_product;
    
//     //Set the values for J_mu
//     for(int i = 0; i < mq_robot.rows();i++)
//     {
//       // Jmu_product = mdJdq_derivative[i] * mPinv_JvND_robot1;
//       Jmu_product = mdJdq_derivative[i] * pinv_J_AugND_robot1;
//       J_mu(i) = mu * Jmu_product.trace();
//     }
    
//     step_size = 5e-3;
//     if(check_Jmu)
//     {
//       dJ_mu = (J_mu - mJ_mu_old)/step_size;
//     }
//     else
//     {
//       dJ_mu.setZero();
//       check_Jmu = true;
//     }
//     mJ_mu_old = J_mu;

//     double epsilon = 0.08;
//  // Defining the X_Dot in nth power formulation
//     double h_singular = mu - epsilon;//mu - epsilon;
//     double dh_singular = J_mu * mdq_robot;

//   // Degine Eta for Exponential Barrier Function
//     Eigen::Matrix<double, 2, 1> eta_Singular;
//     eta_Singular << h_singular, dh_singular;

//     Eigen::Matrix<double, 1, 2> K_b_Singular;
//     K_b_Singular << 200, 140; //2000,800;


//     Eigen::Matrix<double, 1, 7> L_G_CBF_Singular = J_mu * invM_robot1; //2 * e_b.transpose() * A_Obstacle * mJv_robot1 * invM_robot1;
//     // Eigen::Matrix<double, 1, 3> L_G_CBF_Singular_OS = J_mu * invM_robot1 * mM_robot1 * mPinv_JvND_robot1;
//     Eigen::Matrix<double, 1, 3> L_G_CBF_Singular_OS = J_mu * invM_robot1 * mM_robot1 * pinv_J_AugND_robot1;

//     Eigen::Matrix<double, 1, 1> L_F_CBF_Singular = dJ_mu * mdq_robot +  J_mu * (-invM_robot1 * mH_robot1);
//     // Eigen::Matrix<double, 1, 1> L_F_CBF_Singular_OS = dJ_mu * mdq_robot +  J_mu * (-invM_robot1 * mH_robot1 + invM_robot1 * (mM_robot1 * mPinv_JvND_robot1 * (mDesired_ddx_position - mdJv_dq) + mH_robot1);
//     Eigen::Matrix<double, 1, 1> L_F_CBF_Singular_OS = dJ_mu * mdq_robot +  J_mu * (-invM_robot1 * mH_robot1) + J_mu * invM_robot1 * (mM_robot1 * pinv_J_AugND_robot1 * (desired_ddx_Aug_robot1 - mdJ_Aug_dq_robot1) + mH_robot1);

//     Eigen::MatrixXd K_eta_CBF_Singular = K_b_Singular * eta_Singular;

//     // spdlog::info("J_mu = {}",J_mu);
//     // spdlog::info("dJ_mu = {}",dJ_mu);

//     // std::cout << "dJ_mu = " << dJ_mu << std::endl;

//     // std::cout << "J_mu = \n" << J_mu << std::endl;
// // mPinv_JvND_robot1
    

  // Defining the X_Dot in nth power formulation
    double h_singular = mMu - mEpsilon;//mu - epsilon;
    Eigen::VectorXd dh_singular = mJ_mu * mdq_robot;

  // Degine Eta for Exponential Barrier Function
    Eigen::Matrix<double, 2, 1> eta_Singular;
    eta_Singular << h_singular, dh_singular;

    Eigen::Matrix<double, 1, 2> K_b_Singular;
    K_b_Singular << 200, 40; //2000,800;


    Eigen::Matrix<double, 1, 7> L_G_CBF_Singular = mJ_mu * invM_robot1; //2 * e_b.transpose() * A_Obstacle * mJv_robot1 * invM_robot1;
    Eigen::Matrix<double, 1, 3> L_G_CBF_Singular_OS = mJ_mu * invM_robot1 * mM_robot1 * mPinv_JvND_robot1;
    // Eigen::Matrix<double, 1, 6> L_G_CBF_Singular_OS = mJ_mu * invM_robot1 * mM_robot1 * pinv_J_AugND_robot1;

    Eigen::Matrix<double, 1, 1> L_F_CBF_Singular = mdJ_mu * mdq_robot +  mJ_mu * (-invM_robot1 * mH_robot1);
    Eigen::Matrix<double, 1, 1> L_F_CBF_Singular_OS = mdJ_mu * mdq_robot +  mJ_mu * (-invM_robot1 * mH_robot1 + invM_robot1 * (mM_robot1 * mPinv_JvND_robot1 * (mDesired_ddx_position - mdJv_dq) + mH_robot1));
    // Eigen::Matrix<double, 1, 1> L_F_CBF_Singular_OS = mdJ_mu * mdq_robot +  mJ_mu * (-invM_robot1 * mH_robot1) + mJ_mu * invM_robot1 * (mM_robot1 * pinv_J_AugND_robot1 * (desired_ddx_Aug_robot1 - mdJ_Aug_dq_robot1) + mH_robot1);

    Eigen::MatrixXd K_eta_CBF_Singular = K_b_Singular * eta_Singular;


    //     // Set Optimization parameters for robot1 and robot2
    qpmad::MatrixIndex size = 8;
    qpmad::MatrixIndex num_ctr = 13;
    // static constexpr size_t size_dv = 7;
    // static constexpr size_t num_ctr1 = 7;

    // Initialize the variables for QPMad
    Eigen::VectorXd x; // Decision variables
    Eigen::MatrixXd H; // Quadratic cost used in x'Hx
    Eigen::VectorXd h; // Linear cost used in h'*x
    Eigen::MatrixXd A_x; // Constraint A_x*x
    Eigen::VectorXd Alb; // Lower bound for the constraint -b < A_x * x
    Eigen::VectorXd Aub; // Upper bound for the constraint  A_x * x < b
    Eigen::VectorXd lb; // Lower Constraint on decision variables
    Eigen::VectorXd ub; // Lower Constraint on decision variable
    // std::cout << " RESCLF Check 3" << std::endl;

      //Declaring Quadratic cost for robot1. Total of seven variables with 3 \mu for position and 3 \mu for orientation and 1 input for relaxation.
    H.setIdentity(size, size);
    // H.block<6, 6>(0, 0) = Eigen::MatrixXd::Identity(6, 6);
    // H.setIdentity();
    double relaxation = 5;
    double gamma = 1e-3;

    H(6, 6) = relaxation;
    H(7, 7) = relaxation;


    //Declaring linear cost for roboti
    h.setZero(size);
    // h.setZero();

    //Declaring variable bounds
    lb.resize(size);
    ub.resize(size);


    //Declaring constraint for robot1
    //Initializing A_x matrices.
    A_x.setZero(num_ctr, size);
    // A_x.setZero();

    //Constraint for orientation
    A_x.block<1, 3>(0, 0) = LgV_x_ori_robot1;
    A_x(0, 7) = -1;

    //Constraint for position
    A_x.block<1, 3>(1, 3) = LgV_x_pos_robot1;
    A_x(1, 6) = -1;
    
    // //Constraint for Obstacle avoidance barrier
    A_x.block<1, 3>(2, 3) = L_G_CBF_OS;

    // // Constraint for the Outer Box barrier
    A_x.block<1, 3>(3, 3) = L_G_CBF_OS_Outer;

    // //Constraint for the the joints barrier
    // A_x.block<1, 3>(4, 3) = L_G_CBF_Joint;
    A_x.block<1, 6>(4, 0) = L_G_CBF_Joint;

    // //Constraint on the singularity avoidance
    A_x.block<1, 3>(5, 3)  = L_G_CBF_Singular_OS; 


    //Constraint for the Torque limits
    A_x.block<7, 6>(6, 0) = mM_robot1 * pinv_J_AugND_robot1;

    //Defining a vector to define the range of decision variables \mu and \delta(relaxation). Not used and will delete later.
    Eigen::Matrix<double, 7, 1> Huge_Limit_max;
    Huge_Limit_max << HUGE_VAL, HUGE_VAL, HUGE_VAL, HUGE_VAL, HUGE_VAL, HUGE_VAL, HUGE_VAL;
    // std::cout << " RESCLF Check 4" << std::endl;

    //Defining the size of constraints.
    Alb.resize(num_ctr);
    Aub.resize(num_ctr);

    //Lower range for position constraint.
    Alb(0) = -HUGE_VAL;

    //Lower range for orientation constraint.
    Alb(1) = -HUGE_VAL;

    // // // Lower Constraint for Obstacle Barriers
    Alb(2) = -(L_F_CBF_OS(0, 0) + K_b * eta);
    //     std::cout << "RESCLF: check 6" << std::endl;

    // //Lower Constraints for Outer Box Barrier
    Alb(3) = -(L_F_CBF_OS_Outer(0, 0) + K_eta_CBF_Outer(0, 0)); //-(L_F_CBF_OS_Outer(0,0)) + K_b_Outer * eta_CBF_Outer);

    // //Lower Constraints for Joints Barrier
    Alb(4) = -(L_F_CBF_Joint(0, 0) + K_eta_CBF_Joint(0, 0));

    // //Lower Constraints for Singularity  Barrier
    Alb(5) = -(L_F_CBF_Singular_OS(0, 0) + K_eta_CBF_Singular(0, 0));

    // //Lower Constraints values for torque
    Alb.segment<7>(6) = -mTauLim_robot1 - (mM_robot1 * pinv_J_AugND_robot1 * (desired_ddx_Aug_robot1 - mdJ_Aug_dq_robot1) + mCg_robot1);

    //Upper range for position constraint.
    Aub(0) = -LfV_x_ori_robot1(0, 0) - gamma * V_x_ori_robot1(0, 0);

    // //Upper range for orientation constraint.
    Aub(1) = -LfV_x_pos_robot1(0, 0) - gamma * V_x_pos_robot1(0, 0);

    // // //Upper range for obstacle avoidance barrier constraints
    Aub(2) = HUGE_VAL;

    // // // //Upper range for outer box barrier constraint.
    Aub(3) = HUGE_VAL;
    // std::cout << " RESCLF Check 5" << std::endl;

    // //Upper range for joints barrier constraint.
    Aub(4) = HUGE_VAL;

    // //Upper range for  Singular constraint.
    Aub(5) = HUGE_VAL;

    // //Upper range for torque constraints
    Aub.segment<7>(6) = mTauLim_robot1 - (mM_robot1 * pinv_J_AugND_robot1 * (desired_ddx_Aug_robot1 - mdJ_Aug_dq_robot1) + mCg_robot1);

    // //  If lb and ub is set to three variables with one constraint
    lb << -HUGE_VAL, -HUGE_VAL, -HUGE_VAL, -HUGE_VAL, -HUGE_VAL, -HUGE_VAL, -HUGE_VAL, -HUGE_VAL;
    ub << HUGE_VAL, HUGE_VAL, HUGE_VAL, HUGE_VAL, HUGE_VAL, HUGE_VAL, HUGE_VAL, HUGE_VAL;
    
    Eigen::VectorXcd eivals = H.eigenvalues();
    // std::cout << "Eigenvalues of H =\n" << eivals << std::endl;

    qpmad::Solver solver;
    try
    {
      qpmad::Solver::ReturnStatus status = solver.solve(x, H, h, lb, ub, A_x, Alb, Aub);
      // spdlog::info("x.head() = {}",x.head<6>());
      qp_fail_check = true;
    }
    catch(const std::exception& e)
    {
      // std::cerr << "qpmad failed: " << e.what() << std::endl;
      spdlog::info("qpmad failed = {}", e.what());
      qp_fail_check = false;
      fail_check++;
    }
       
    // if (status != qpmad::Solver::OK)
    // {
    //   std::cerr << "Error" << std::endl;
    // }
    // std::cout << " RESCLF Check 6" << std::endl;
    if(qp_fail_check){
      mBodyTorques_QPMAd_robot1 = mM_robot1 * pinv_J_AugND_robot1 * (desired_ddx_Aug_robot1 + x.head<6>() - mdJ_Aug_dq_robot1) + mCg_robot1;
      // mBodyTorques_QPMAd_robot1 = mM_robot1 * mPinv_Jv_robot1 * (mDesired_ddx_position + x.head<6>() - mdJ_Aug_dq_robot1) + mCg_robot1;
    }
    else{
      mBodyTorques_QPMAd_robot1 = Eigen::VectorXd::Zero(7);
    }
    // std::cout << " RESCLF Check 7" << std::endl;
    // spdlog::info("mBodyTorques_QPMAd_robot1 = {}",mBodyTorques_QPMAd_robot1);



    ee_position << x_robot1(0) << ", " << x_robot1(1) << ", " << x_robot1(2) << "," << mCur_time_us << std::endl;
    ee_velocity <<  dx_robot1(0) << ", " << dx_robot1(1) << ", " << dx_robot1(2) << std::endl;
    target_position << mTargetPos(0) << ", " << mTargetPos(1) << ", " << mTargetPos(2) << ", " << angle << std::endl;
    barrier_position << x_b(0) << ", " << x_b(1) << ", " << x_b(2) << std:: endl;
    torque_value << mBodyTorques_QPMAd_robot1(0) << ", " << mBodyTorques_QPMAd_robot1(1) << ", " << mBodyTorques_QPMAd_robot1(2) << ", "
                << mBodyTorques_QPMAd_robot1(3) << ", " << mBodyTorques_QPMAd_robot1(4) << ", " << mBodyTorques_QPMAd_robot1(5) << ", "
                << mBodyTorques_QPMAd_robot1(6) << std::endl;
    joint_position << mq_robot(0) << ", " << mq_robot(1) << ", " << mq_robot(2) << ", " 
                   << mq_robot(3) << ", " << mq_robot(4) << ", " << mq_robot(5) << ", " << mq_robot(6) << std::endl;            
 
    joint_velocity << mdq_robot(0) << ", " << mdq_robot(1) << ", " << mdq_robot(2) << ", " 
                   << mdq_robot(3) << ", " << mdq_robot(4) << ", " << mdq_robot(5) << ", " << mdq_robot(6) << std::endl;      

    out_barrier_position << mOuterBoxRadius_x << ", " << mOuterBoxRadius_y << ", " << mOuterBoxRadius_z << "," << barrier_active << std:: endl;
    ee_rpy << mEE_RPY(0) << "," << mEE_RPY(1) << "," << mEE_RPY(2) << "," << std::endl;
    target_rpy << mTargetRPY(0) << "," << mTargetRPY(1) << "," << mTargetRPY(2) << "," << std::endl;

    mu_singular << mMu << std::endl;

}


  void HandleStatus(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_iiwa_status* status) {
    iiwa_status_ = *status;
  }

  ::lcm::LCM lcm_;
  const multibody::MultibodyPlant<double>& mplant;
  lcmt_iiwa_status iiwa_status_;
  int64_t mCur_time_us, bias_ref;
  double mFreq, mAmplitude;

  //Variable for manual differentiation
  std::vector<Eigen::MatrixXd> mJacobian_Derivative;
  Eigen::MatrixXd mJ_mu_old; //Storing the old value of J_mu
  Eigen::MatrixXd mJ_mu, mdJ_mu; //Storing the old value of J_mu and dJ_Mu
  double mMu;
  double mEpsilon;



  //Variables for Autodiff in drake
  std::unique_ptr<multibody::MultibodyPlant<AutoDiffXd>> mplant_autodiff;
  std::unique_ptr<systems::Context<AutoDiffXd>> mContext_autodiff;
  const multibody::Frame<AutoDiffXd>* mFrame_EE_autodiff; // End effector frame
  const multibody::Frame<AutoDiffXd>* mWorld_Frame_autodiff; // End effector frame

  // MatrixX<AutoDiffXd> q_autodiff; //wrong my approach
  AutoDiffVecXd mq_autodiff;//(mq_robot.rows()); 
  MatrixX<AutoDiffXd> mJacobian_autodiff; // Linear Jacobian matrix.
  // Eigen::Matrix<double, 6, 7> mJacobian_autodiff; // Spatial Jacobian matrix.
  std::vector<Eigen::MatrixXd> mdJdq_derivative;


  //Manually defined variables for the controller
  std::unique_ptr<systems::Context<double>> mContext;
  std::string mEE_link; //end-effector link name
  std::string mBase_link; //base of manipulator name
  const multibody::Frame<double>* mFrame_EE; // End effector frame
  Eigen::Vector3d mdJv_dq; //Product of Linear Jacobian Derivative and joint velocity = Jdot*q_dot
  Eigen::Vector3d mdJw_dq; //Product of Angular Jacobian Derivative and joint velocity
  Eigen::VectorXd mq_robot; // Joint positions.
  Eigen::VectorXd mdq_robot; // Joint velocities.
  Eigen::MatrixXd mM_robot1; // Mass matrix.
  Eigen::VectorXd mCg_robot1; // Coriolis vector.
  Eigen::VectorXd mG_robot1; // Gravity vector.
  Eigen::VectorXd mH_robot1; // Coriolis and Gravity vector.  
  Eigen::VectorXd mE_robot; //Desired ee position
  Eigen::VectorXd mDe_robot; //Desired ee velocity
  Eigen::VectorXd mDesired_ee_ori; //Desired orientation position
  Eigen::VectorXd mDesired_ee_ori_vel; //Desired orientation velocity
  Eigen::VectorXd mDesired_ddx_position; //Desired ee acceleration
  Eigen::VectorXd mEE_position; //Current ee position
  Eigen::VectorXd mEE_velocity; //Current ee velocity
  Eigen::VectorXd mEE_RPY; //Current ee orientation
  Eigen::Isometry3d mEE_Rot; //Initial rotation matrix
  Eigen::VectorXd mBase_position; //Current Base position
  Eigen::Matrix<double,7,1> mJoint_limit_max;




  Eigen::Vector3d mEE_w_robot; //Current ee angular velocity (orientation)
  Eigen::Vector3d mQuatError_xyz_robot1; //Current ee angular position (orientation)
  Eigen::Vector3d mDwref_robot1; //Current ee angular velocity (orientation)
  Eigen::Matrix<double, 7,1> mTauLim_robot1; //Torque Limit of the robot
  Eigen::Vector3d mObstaclePos; //mObstacle->getWorldTransform().translation();
  Eigen::Vector3d mObstacleVel;
  Eigen::Matrix<double, 7,1> mBodyTorques_QPMAd_robot1;
  double mScale = 1;


  Eigen::MatrixXd mPinv_JvND_robot1; //Pseduo Inverse of Jacobian with no damping

  Eigen::MatrixXd mPinv_Jv_robot1; //Pseduo Inverse of Jacobian
  // Eigen::Vector3d mEE_dw_robot; //Current ee angular velocity (orientation velocity)
  multibody::SpatialAcceleration<double> mdJ_Aug_dq_spatial1; //Augmented Jacobian Derivative * dq
  Eigen::Matrix<double, 6, 1> mdJ_Aug_dq_robot1;
  Eigen::MatrixXd mFBL_Control_robot1;

  // Eigen::Vector3d dx; //Incremental distance for target
  double dx, dy, dz;
  Eigen::Vector3d mTargetPos; //Target Position
  Eigen::Vector3d mTargetVel; //Target Position
  Eigen::Vector3d mTargetRPY; // Target Orientation
  // math::RotationMatrix<double> mTargetRot; //Initial rotation matrix
  Eigen::Isometry3d mTargetRot; //Initial rotation matrix

  Eigen::MatrixXd mJv_robot1; // Linear Jacobian matrix.
  Eigen::MatrixXd mJw_robot1; // Angular Jacobian matrix.
  Eigen::Matrix<double, 6, 7> mJacobian; // Spatial Jacobian matrix.
  Eigen::Matrix3d mKp; //Defines gains for position in pd controller
  Eigen::Matrix3d mKv; //Define gains for velocity in pd controller

  Eigen::Matrix3d mKpOr; //Defines gains for angular position in pd controller
  Eigen::Matrix3d mKvOr; //Define gains for angular velocity in pd controller
  
  //Special matrices to load fator grpah
  Eigen::MatrixXd mF;
  Eigen::MatrixXd mG;
  Eigen::MatrixXd mP;
  math::RigidTransform<double> mEE_link_pose; // End effector pose
  math::RigidTransform<double> mBase_link_pose; // End effector pose
  multibody::SpatialVelocity<double> mEE_link_velocity; //end-effector velocity

  double step_size;
  double counter = 1;
  int64_t counter2 = 0, check = 0, fail_check = 0;
  double angle = 0;
  double angle2 = 0;
  double step_ref = 3*1e-3;
  double loops = 0;
  double counter_barrier = 0;
  double maxtime, maxtime_old;
  bool check_Jmu = false;
  

  //Task Position Constraint Variable
  int mOuterPower;         //Defining the power of ellipsoid
  double mOuterBoxRadius_x, mOuterBoxRadius_y, mOuterBoxRadius_z; //Defining the box containing variable.
  Eigen::Matrix<double, 3, 3> A_OuterBox;   //Defining  Matrix for ellipsoide variable
  
  //Joint Position Constraint
  double mJointPower; //Define the power of ellipsoid to approximate the limit of the joints

  //qp_fail check
  bool qp_fail_check = true;

  // Declaring output stream variable to store variables in file
  std::ofstream ee_position;
  std::ofstream ee_velocity;
  std::ofstream joint_position;
  std::ofstream joint_velocity;
  std::ofstream torque_value;
  std::ofstream target_position;
  std::ofstream barrier_position;
  std::ofstream out_barrier_position;
  std::ofstream ee_rpy;
  std::ofstream target_rpy;
  std::ofstream mu_singular;



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
