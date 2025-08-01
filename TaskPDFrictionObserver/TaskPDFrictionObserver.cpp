#include "TaskPDFrictionObserver.h"

template<typename ROBOT>
void TaskPDFrictionObserver<ROBOT>::initialize(ROBOT &robot, double delt)
{
    /**< Initialize the H-infinity controller with the given time step */
    robot.initHinfController(delt);
    /**< Reset the integral terms in the H-inifinity control algorithm */
    robot.resetHinfController();
    reset(robot);

    debug_cnt = 3;
    is_soft_estop_ = false;
    file_.open("LPF_log.csv");
    _delT = delt;
    currentT_ = 0;
        /* Friction Observer initialization reset */
    reset(robot);
    printT_ = 0.0;
    printT2_ = 0.0;

}

template<typename ROBOT>
void TaskPDFrictionObserver<ROBOT>::reset(ROBOT& robot)
{
    /**< Reset the integral terms in the H-inifinity control algorithm */
    robot.resetHinfController();
    
    /* Task-space controller reset */
    robot.computeFK(tpos_d_);
    /* Task-space controller reset end */
    theta_n_ = robot.q();
    dtheta_n_ = robot.qdot();
    fo_init_status_ = 0;
    fo_init_cnt_ = 0;
    L_vec_.setZero();

    L_.resize(6, 6); L_.setZero();
    L_.diagonal() << 80.0, 80.0, 80.0, 80.0, 80.0, 80.0;
    Lp_.resize(6, 6); Lp_.setZero();
    Lp_.diagonal() << 40.0, 40.0, 40.0, 40.0, 40.0, 40.0; 
    B_.resize(6, 6); B_.setZero();
    B_.diagonal() << 1.0, 1.0, 1.0, 0.8, 0.8, 0.8;
    tau_f_prev_ = Eigen::VectorXd::Zero(6);
    tau_j_prev_ = Eigen::VectorXd::Zero(6);    
    /* JTS initialization init */
    // robotNominal_ = std::make_unique<ROBOT>(robot);
    jts_initialized_ = false;
    jts_init_cnt_ = 1;
    avg_cnt_ = static_cast<int>(2.0 / _delT);
    tau_j_bias_.setZero();
    /* JTS initialization init end */
        /* Friction Observer initialization init */
    fo_init_status_ = 0;
    dK_L_vec_.setZero();
    fo_init_duration_ = 2.0;
    fo_init_cnt_ = 0;
    fo_init_cnt_total_ = static_cast<int>(fo_init_duration_ / _delT);
    /* Friction Observer initialization init end */

    /* Friction Observer initialization reset end */
    std::cout << "========== RESET HAS CALLED ==========" << std::endl;
}

template<typename ROBOT>
void TaskPDFrictionObserver<ROBOT>::compute(ROBOT &robot, const LieGroup::Vector3D &gravDir,
                                               const MotionData &motionData, ControlData &controlData)
{
    // ---------------------------------------------------------------------------------------  My Custom controller  --------------------------------------------------------------
    robot.idyn_gravity(gravDir);
    JointVec gravity_compensation_torque = robot.tau();

    JointVec control_motor_torque; control_motor_torque.setZero();
    JointVec friction_compensation_torque; friction_compensation_torque.setZero();
    JointVec PDinput; PDinput.setZero();
    JointVec new_tau; new_tau.setZero();
    
    Kp_ = this->gain3;
    Kd_ = this->gain4;
    J_lpf_vec_ = this->gain0;
    B_vec_ = this->gain8;
    PW_ = this->gain9;
    Lp_vec_ = this->gain6;

    if (Lp_vec_.minCoeff() > 0)
        Lp_.diagonal() << Lp_vec_;
    if (B_vec_.minCoeff() > 0)
        B_.diagonal() << B_vec_;
    
    const double pw_target_ = 6.29;
    const double pw_tol_ = 1e-6;
    if  (std::abs(PW_[0] - pw_target_) > pw_tol_) {  
        controlData.controlTorque.tau = gravity_compensation_torque;
        if (std::abs(PW_[0]) > pw_tol_)
            std::cout << "PW error" << std::endl;
        return;
    }
        
    if (is_soft_estop_ || std::abs(robot.qdot().maxCoeff()) > 1.3) {
        controlData.controlTorque.tau = gravity_compensation_torque;
        is_soft_estop_ = true;
        std::cout << "EMERGENCY!!! : qdot exceeded" << std::endl;
        return;
    }
    // std::cout << "######################################################################################" << std::endl;
    // std::cout << "time: " << currentT_ << std::endl;

    /* JTS initialization */
    if (!jts_initialized_) {
        const double alpha = (jts_init_cnt_ - 1.0) / jts_init_cnt_;
        tau_j_ = controlData.controlTorque.tauJTS;
        tau_j_(1) = - controlData.controlTorque.tauJTS(1);

        JointVec tau_j_nominal;
        robot.idyn_gravity(gravDir);
        tau_j_nominal= robot.tau();
        tau_j_bias_ = alpha * tau_j_bias_ + (1 - alpha) * (tau_j_ - tau_j_nominal);
        jts_init_cnt_++;
        
        if (jts_init_cnt_ > avg_cnt_) {
            jts_initialized_ = true;
            jts_init_cnt_ = 1;
            theta_n_ = robot.q();
            dtheta_n_ = robot.qdot();
            std::cout<<"Joint torque sensor bias is initialized"<<std::endl;
            std::cout<<"bias : " << tau_j_bias_.transpose()<<std::endl;
        }
        new_tau = gravity_compensation_torque;
    }
    /* JTS initialization end */
    // -------------------------------- JOINT CONTROLLER -------------------------------------------------------
    else {
        JointVec de_nr = dtheta_n_ - robot.qdot();
        JointVec e_nr = theta_n_ - robot.q();
        control_motor_torque = - Kp_.cwiseProduct(robot.q() - motionData.motionPoint.qd) - Kd_.cwiseProduct( robot.qdot() - motionData.motionPoint.qdotd) + gravity_compensation_torque;
        PDinput = - Kp_.cwiseProduct(theta_n_ - motionData.motionPoint.qd) - Kd_.cwiseProduct(dtheta_n_) + gravity_compensation_torque;
        
        // Calculate PD Observer // 
        if (this->gain5.minCoeff() > 0) {
            switch (fo_init_status_) {
                case 0:
                    dK_L_vec_ = this->gain5 / (fo_init_duration_ / _delT);
                    fo_init_cnt_ = 0;
                    fo_init_status_ = 1;
                    break;
                case 1:
                    L_vec_ += dK_L_vec_;
                    fo_init_cnt_++;
                    if (fo_init_cnt_ > fo_init_cnt_total_) {
                        fo_init_status_ = 2;
                    }
                    break;
                case 2:
                    L_vec_ = this->gain5;
                    break;
                default:
                    L_vec_.setZero();
            }
        }
        L_.diagonal() << L_vec_;
        
        friction_compensation_torque = -B_ * L_ * (de_nr + Lp_ * e_nr);
        tau_j_ = controlData.controlTorque.tauJTS;
        tau_j_(1) = - controlData.controlTorque.tauJTS(1);
        tau_j_ -= tau_j_bias_;   // eliminate bias 
        
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6,6);
        Eigen::MatrixXd Beta(6, 6); Beta.setZero();
        Beta.diagonal() = (- J_lpf_vec_ * _delT).array().exp();
        tau_j_lpf_ = Beta * tau_j_prev_ + (I - Beta) * tau_j_;
        tau_j_prev_ = tau_j_lpf_;

        JointVec qddotNom ;
        qddotNom.setZero();
        qddotNom = B_.inverse() * ( PDinput - tau_j_lpf_ );   
        dtheta_n_ += _delT * qddotNom;
        theta_n_ += _delT * dtheta_n_;
        new_tau = PDinput - friction_compensation_torque;

        // new_tau = control_motor_torque;
    }
    currentT_ += _delT;
    controlData.controlTorque.tau = new_tau;
}

/**< Manifest to allow dynamic loading of the control algorithm */
POCO_BEGIN_MANIFEST(NRMKControl::ControlAlgorithmCreator)
        POCO_EXPORT_CLASS(TaskPDFrictionObserverCreator)
POCO_END_MANIFEST

