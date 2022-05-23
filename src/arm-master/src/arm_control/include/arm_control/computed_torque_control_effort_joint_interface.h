#include "ros/ros.h"
#include <kdl/jntarrayvel.hpp>
#include <arm_control/Efforts.h>
#include <kdl_parser/kdl_parser.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
// from kdl packages
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>              // inverse dynamics

#include <trajectory_interface/quintic_spline_segment.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <boost/scoped_ptr.hpp>
namespace joint_trajectory_controller
{
    typedef trajectory_interface::QuinticSplineSegment<double> SegmentImpl;
    typedef JointTrajectorySegment<SegmentImpl> Segment;
    typedef typename Segment::State State;
}

template<>
class HardwareInterfaceAdapter<hardware_interface::EffortJointInterface, joint_trajectory_controller::State>
{
public:
    HardwareInterfaceAdapter() :
            joint_handles_ptr(0)
    {}

    bool init(std::vector<hardware_interface::JointHandle> &joint_handles, ros::NodeHandle &nh)
    {
        // Store pointer to joint handles
        joint_handles_ptr = &joint_handles;

        // Parse the URDF string into a URDF model.
        urdf::Model urdf_model;
        if (!urdf_model.initParam("/robot_description")) {
            ROS_ERROR("Failed to parse urdf model from robot description");
            return false;
        }
        ROS_INFO("Parsed urdf model from robot description");

        // Compute the KDL tree of the robot from the URDF.
        KDL::Tree tree;
        if (!kdl_parser::treeFromUrdfModel(urdf_model, tree)) {
            ROS_ERROR("Failed to parse kdl tree from urdf model");
            return false;
        }
        ROS_INFO("Parsed kdl tree from urdf model");

        // Extract chain from KDL tree.
       // KDL::Chain chain;

               // 4.2 kdl chain
        std::string root_name, tip_name;
        if (!nh.getParam("root_link", root_name))
        {
            ROS_ERROR("Could not find root link name");
            return false;
        }
        if (!nh.getParam("tip_link", tip_name))
        {
            ROS_ERROR("Could not find tip link name");
            return false;
        }

        // if (!tree.getChain("toe", "finger", kdl_chain_)) {
        //     ROS_ERROR("Failed to extract chain from 'toe' to 'finger' in kdl tree");
        //     return false;
        // }

        // if (!tree.getChain(root_name, tip_name, kdl_chain_)) {
        //     ROS_ERROR("Failed to extract chain from 'toe' to 'finger' in kdl tree");
        //     return false;
        // }
        // ROS_INFO("Extracted chain from kdl tree");

        if (!tree.getChain(root_name, tip_name, kdl_chain_))
        {
            ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
            ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name);
            ROS_ERROR_STREAM("  Tree has " << tree.getNrOfJoints() << " joints");
            ROS_ERROR_STREAM("  Tree has " << tree.getNrOfSegments() << " segments");
            ROS_ERROR_STREAM("  The segments are:");

            KDL::SegmentMap segment_map = tree.getSegments();
            KDL::SegmentMap::iterator it;

            for (it = segment_map.begin(); it != segment_map.end(); it++)
                ROS_ERROR_STREAM("    " << (*it).first);

            return false;
        }
        else
        {
            ROS_INFO("Got kdl chain");
        }



        // Init effort command publisher.
        publisher.reset(new EffortsPublisher(nh, "efforts", 10));

        // links joints efforts to publisher message.
        joints_efforts = &(publisher->msg_.data);

        // Reset and resize joint states/controls.
        unsigned int n_joints = kdl_chain_.getNrOfJoints();
        inner_loop_control.resize(n_joints);
        outer_loop_control.resize(n_joints);
        joints_effort_limits.resize(n_joints);
        (*joints_efforts).resize(n_joints);
        joints_state.resize(n_joints);
        std::vector<std::string> joint_name_vec;
        for (unsigned int idx = 0; idx < kdl_chain_.getNrOfJoints(); idx++) {
            // Get joint name.
            std::string name = kdl_chain_.getSegment(idx).getJoint().getName();
            joint_name_vec.push_back(name);

            // Extract joint effort limits from urdf.
            if (!(urdf_model.getJoint(name)) ||
                !(urdf_model.getJoint(name)->limits) ||
                !(urdf_model.getJoint(name)->limits->effort)) {
                ROS_ERROR("No effort limit specified for joint '%s'", name.c_str());
                return false;
            }
            joints_effort_limits.data[idx] = urdf_model.getJoint(name)->limits->effort;
        }
        ROS_INFO("Extracted joint effort limits");

        // Init inverse dynamics solver.
        gravity_ = KDL::Vector::Zero(); // ?
        gravity_(2) = -9.81;            // 0: x-axis 1: y-axis 2: z-axis

        id_rne_solver_.reset(new KDL::ChainIdSolver_RNE(kdl_chain_, gravity_));
        ROS_INFO("Initialized kdl inverse dynamics solver");
   
        // 4.3 inverse dynamics solver 초기화
        id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));
        
        e_.data = Eigen::VectorXd::Zero(n_joints);
        e_dot_.data = Eigen::VectorXd::Zero(n_joints);

        // 1.2 Gain
        // 1.2.1 Joint Controller
        Kp_.resize(n_joints);
        Kd_.resize(n_joints);
        Ki_.resize(n_joints);

        std::vector<double> Kp(n_joints), Ki(n_joints), Kd(n_joints);
        for (size_t i = 0; i < n_joints; i++)
        {
            std::string si = boost::lexical_cast<std::string>(i + 1);
            std::string param_prefix = "/computed_torque_controller/gains/"+ joint_name_vec[i];
            //if (n.getParam("/elfin/computed_torque_controller/gains/elfin_joint" + si + "/pid/p", Kp[i]))
            std::string full_param = param_prefix  + "/pid/p";
            if (nh.getParam(full_param, Kp[i]))
            {
                Kp_(i) = Kp[i];
            }
            else
            {
                //std::cout << "/elfin/computed_torque_controller/gains/elfin_joint" + si + "/pid/p" << std::endl;
                std::cout << full_param.c_str() << std::endl;

                ROS_ERROR("Cannot find pid/p gain");
                return false;
            }

            full_param.clear();
            full_param = param_prefix  + "/pid/i";

            if (nh.getParam(full_param, Ki[i]))
            {
                Ki_(i) = Ki[i];
            }
            else
            {
                ROS_ERROR("Cannot find pid/i gain");
                return false;
            }

            full_param.clear();
            full_param = param_prefix  + "/pid/d";
            if (nh.getParam(full_param, Kd[i]))
            {
                Kd_(i) = Kd[i];
            }
            else
            {
                ROS_ERROR("Cannot find pid/d gain");
                return false;
            }
        }

        qd_.data = Eigen::VectorXd::Zero(n_joints);
        qd_dot_.data = Eigen::VectorXd::Zero(n_joints);
        qd_ddot_.data = Eigen::VectorXd::Zero(n_joints);


        // 5.2 Matrix 
        M_.resize(n_joints);
        C_.resize(n_joints);
        G_.resize(n_joints);
        
        return true;
    }

    void starting(const ros::Time & /*time*/)
    {
        if (!joint_handles_ptr) { return; }

        for (unsigned int idx = 0; idx < joint_handles_ptr->size(); ++idx) {
            // Write joint effort command.
            (*joint_handles_ptr)[idx].setCommand(0);
        }
    }

    void stopping(const ros::Time & /*time*/)
    {}

    void updateCommand(const ros::Time &     /*time*/,
                       const ros::Duration & period,
                       const joint_trajectory_controller::State &desired_state,
                       const joint_trajectory_controller::State &state_error)
    {
        if (!joint_handles_ptr) { return; }
        double dt = period.toSec();
        //std::cout<<"[ctc] dt:"<<dt<<std::endl;
        //std::cout<<"[effort_joint_interface] ctc dt:"<<dt<<std::endl;

        for (size_t idx = 0; idx < joint_handles_ptr->size(); ++idx) {

            // Update joint state with current position (q) and velocity (qdot).
            joints_state.q.data[idx] = (*joint_handles_ptr)[idx].getPosition();
            joints_state.qdot.data[idx] = (*joint_handles_ptr)[idx].getVelocity();

            // Compute outer loop control.
            // todo: dynamic reconfigure parameters.
            outer_loop_control.data[idx] = 100 * state_error.position[idx] + 10 * state_error.velocity[idx];
        }


        // *** 2.1 Error Definition in Joint Space ***
        for (size_t idx = 0; idx < joint_handles_ptr->size(); ++idx) {

            // Update joint state with current position (q) and velocity (qdot).
            e_.data[idx] = state_error.position[idx];
            e_dot_.data[idx] = state_error.velocity[idx];
        }

        for (size_t idx = 0; idx < joint_handles_ptr->size(); ++idx) {

            // Update joint state with current position (q) and velocity (qdot).
            qd_.data[idx] = desired_state.position[idx];
            qd_dot_.data[idx] = desired_state.velocity[idx];
            qd_ddot_.data[idx] = desired_state.acceleration[idx];
        }


        // *** 2.2 Compute model(M,C,G) ***
        id_solver_->JntToMass(joints_state.q, M_);
        id_solver_->JntToCoriolis(joints_state.q, joints_state.qdot, C_);
        id_solver_->JntToGravity(joints_state.q, G_); 


        // *** 2.3 Apply Torque Command to Actuator ***
        aux_d_.data = M_.data * (qd_ddot_.data + Kp_.data.cwiseProduct(e_.data) + Kd_.data.cwiseProduct(e_dot_.data));
        comp_d_.data = C_.data + G_.data;
        tau_d_.data = aux_d_.data + comp_d_.data;

        //std::cout<<"M_.data:\n"<<M_.data<<std::endl;



        // No external forces (except gravity).
        KDL::Wrenches external_forces(joint_handles_ptr->size());

        // Solve inverse dynamics (inner loop control).
        int ret = id_rne_solver_->CartToJnt(
                joints_state.q,
                joints_state.qdot,
                outer_loop_control,
                external_forces,
                inner_loop_control);
        if (ret != 0) {
            //ROS_ERROR("error solving inverse dynamics,ret is:%d\n",ret);
            return;
        };

        for (unsigned int idx = 0; idx < joint_handles_ptr->size(); ++idx) {
            //(*joints_efforts)[idx] = inner_loop_control.data[idx];
            (*joints_efforts)[idx] = tau_d_.data[idx];

            // Limit based on min/max efforts.
            // (*joints_efforts)[idx] = std::min((*joints_efforts)[idx], joints_effort_limits.data[idx]);
            // (*joints_efforts)[idx] = std::max((*joints_efforts)[idx], -joints_effort_limits.data[idx]);

            // Write joint effort command.
            (*joint_handles_ptr)[idx].setCommand((*joints_efforts)[idx]);

        }

        // Publish efforts.
        if (publisher->trylock()) {
            publisher->msg_.header.stamp = ros::Time::now();
            publisher->unlockAndPublish();
        }
    }

private:

    // Joints handles.
    std::vector<hardware_interface::JointHandle> *joint_handles_ptr;

    // Realtime effort command publisher.
    typedef realtime_tools::RealtimePublisher<arm_control::Efforts> EffortsPublisher;
    boost::scoped_ptr<EffortsPublisher> publisher;

    // Inverse Dynamics Solver.
    boost::scoped_ptr<KDL::ChainIdSolver_RNE> id_rne_solver_;
    boost::scoped_ptr<KDL::ChainDynParam> id_solver_;                  // Solver To compute the inverse dynamics

    KDL::JntArray e_, e_dot_;

    // gains
    KDL::JntArray Kp_, Ki_, Kd_;
    // Joints state.
    KDL::Chain kdl_chain_;//must have this member as private or public object ,should not use as a temp object ,see ,https://github.com/jmichiels/arm/issues/3

    KDL::JntArrayVel joints_state;

    // kdl M,C,G
    KDL::JntSpaceInertiaMatrix M_; // intertia matrix
    KDL::JntArray C_;              // coriolis
    KDL::JntArray G_;              // gravity torque vector
    KDL::Vector gravity_;

    KDL::JntArray qd_, qd_dot_, qd_ddot_;

    // Input
    KDL::JntArray aux_d_;
    KDL::JntArray comp_d_;
    KDL::JntArray tau_d_;

    // Joints commands.
    KDL::JntArray
            joints_effort_limits,
            inner_loop_control,
            outer_loop_control;

    // Joints efforts.
    std::vector<double> *joints_efforts;
};
