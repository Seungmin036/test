
#pragma once

/**< Necessary includes */
#include <NRMKFramework/AbstractComponent/ControlAlgorithm.h>
#include <NRMKFramework/GlobalDefinitions.h>
#include <Controller/PositionController.h>

#include <Poco/ClassLibrary.h>
#include <Poco/ClassLoader.h>
#include <Poco/MetaObject.h>


template<typename ROBOT>
class TaskPDFrictionObserver : public NRMKControl::ControlAlgorithm<ROBOT>
{
    /**< Type definitions for clarity and ease of maintenance */
    typedef NRMKControl::ControlAlgorithm<ROBOT> AlgorithmBase;
    typedef typename AlgorithmBase::ControlGains ControlGains;
    typedef typename AlgorithmBase::CustomControlGains CustomControlGains;
    typedef typename NRMKControl::ControlAlgorithm<ROBOT>::MotionData MotionData;
    typedef typename NRMKControl::ControlAlgorithm<ROBOT>::ControlData ControlData;
    typedef typename ROBOT::JointVec JointVec;
    typedef typename ROBOT::JointMat JointMat;
    typedef typename ROBOT::TaskPosition  TaskPosition;
    typedef typename ROBOT::TaskVelocity TaskVelocity;
    typedef typename ROBOT::ExtendedPosition ExtendedPosition;
    typedef typename ROBOT::ExtendedVelocity ExtendedVelocity;
    typedef typename ROBOT::ExtendedTaskJacobian ExtendedTaskJacobian;
    typedef typename ROBOT::ExtendedTaskVec ExtendedTaskVec;


public:
    /**< Constructor and member functions */
    TaskPDFrictionObserver() = default;
    virtual void initialize(ROBOT& robot, double delt) override;
    virtual void reset(ROBOT& robot) override;
    virtual void compute(ROBOT& robot, const LieGroup::Vector3D& gravityDir,
                         const MotionData& motionData, ControlData& controlData) override;

    /**< Algorithm specific variable definition */
    void initializeNominalRobot(ROBOT & robot);
    // JointVec friction_observer_L1_PD(ROBOT &robot, const LieGroup::Vector3D& gravDir,  const JointVec &Control_input,  const MotionData &motionData, ControlData &controlData);  
private:
    int debug_cnt;
    bool is_soft_estop_ = false;
    double _delT, currentT_;
    JointVec _kp;
    JointVec _kv;
    JointVec _ki;

    JointVec fric_gain_;
    Eigen::MatrixXd Gamma_, Gamma_p_, L_, Lp_, B_;
    JointVec Gamma_vec_, Gamma_p_vec_, L_vec_, Lp_vec_, K_lpf_vec_, B_vec_, J_lpf_vec_;
    JointVec tau_j_, tau_j_lpf_, tau_j_prev_;
    JointVec tau_f_prev_, theta_n_, dtheta_n_;
    
    /* Task-space controller variables */
    JointVec Kp_task_, Kd_task_;
    ExtendedPosition tpos_d_;
    ExtendedPosition tpos_;
    ExtendedVelocity tvel_;
    ExtendedTaskJacobian J_, Jdot_;
    /* Task-space controller variables end */

    /* JTS initialization variables */
    bool jts_initialized_ = false;
    int jts_init_cnt_ = 1;
    int avg_cnt_;
    JointVec tau_j_bias_;
    /* JTS initialization variables end */

    /* Friction Observer initialization variables */
    int fo_init_status_ = 0;
    double fo_init_duration_ = 2.0;
    int fo_init_cnt_ = 0;
    int fo_init_cnt_total_;
    JointVec dK_L_vec_;
    /* Friction Observer initialization variables end */

    /* Task trajectory variables */
    double traj_start_;
    double traj_duration_;
    Eigen::Vector3d traj_;

    /* Task trajectory variables end */

    JointVec Kp_, Kd_, PW_;
    std::ofstream file_;
    double printT_ = 0.0;
    double printT2_ = 0.0;
};

/**< Class creator to facilitate dynamic loading if necessary */
class TaskPDFrictionObserverCreator : public NRMKControl::ControlAlgorithmCreator
{
CONTROL_CREATOR_BODY(TaskPDFrictionObserver)
};