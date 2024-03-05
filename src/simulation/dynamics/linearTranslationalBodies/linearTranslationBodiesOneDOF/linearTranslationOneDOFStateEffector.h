/*
 ISC License

 Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */


#ifndef LINEAR_TRANSLATION_ONE_DOF_STATE_EFFECTOR_H
#define LINEAR_TRANSLATION_ONE_DOF_STATE_EFFECTOR_H

#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/bskLogging.h"

#include "architecture/msgPayloadDefC/ArrayMotorForceMsgPayload.h"
#include "architecture/msgPayloadDefC/ArrayEffectorLockMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/LinearTranslationRigidBodyMsgPayload.h"
#include "architecture/messaging/messaging.h"

/*! @brief linear spring mass damper state effector class */
class linearTranslationOneDOFStateEffector :
	public StateEffector, public SysModel
{
public:
    Message<LinearTranslationRigidBodyMsgPayload> translatingBodyOutMsg;        //!< state output message
    Message<SCStatesMsgPayload> translatingBodyConfigLogOutMsg;           //!< translating body state config log message
    ReadFunctor<ArrayMotorForceMsgPayload> motorForceInMsg;               //!< -- (optional) motor force input message
    ReadFunctor<LinearTranslationRigidBodyMsgPayload> translatingBodyRefInMsg;  //!< -- (optional) reference state input message
    ReadFunctor<ArrayEffectorLockMsgPayload> motorLockInMsg;                   //!< -- (optional) lock flag input message

    linearTranslationOneDOFStateEffector();           //!< -- Contructor
	~linearTranslationOneDOFStateEffector();          //!< -- Destructor

    void setMass(double mass) {this->mass = mass;};
    void setK(double k) {this->k = k;};
    void setC(double c) {this->c = c;};
    void setRhoInit(double rhoInit) {this->rhoInit = rhoInit;};
    void setRhoDotInit(double rhoDotInit) {this->rhoDotInit = rhoDotInit;};
    void setFHat_B(Eigen::Vector3d fHat_B) {this->fHat_B = fHat_B;};
    void setR_FcF_F(Eigen::Vector3d r_FcF_F) {this->r_FcF_F = r_FcF_F;};
    void setR_F0B_B(Eigen::Vector3d r_F0B_B) {this->r_F0B_B = r_F0B_B;};
    void setIPntFc_F(Eigen::Matrix3d IPntFc_F) {this->IPntFc_F = IPntFc_F;};
    void setDCM_FB(Eigen::Matrix3d dcm_FB) {this->dcm_FB = dcm_FB;};

    double getMass() const {return this->mass;};
    double getK() const {return this->k;};
    double getC() const {return this->c;};
    double getRhoInit() const {return this->rhoInit;};
    double getRhoDotInit() const {return this->rhoDotInit;};
    Eigen::Vector3d getFHat_B() const {return this->fHat_B;};
    Eigen::Vector3d getR_FcF_F() const {return this->r_FcF_F;};
    Eigen::Vector3d getR_F0B_B() const {return this->r_F0B_B;};
    Eigen::Matrix3d getIPntFc_F() const {return IPntFc_F;};
    Eigen::Matrix3d getDCM_FB() const {return dcm_FB;};

private:
    double mass = 1.0;              //!< [kg] mass of effector
    double k = 0;                   //!< [N/m] linear spring constant
    double c = 0;                   //!< [N-s/m] linear damping term
    double rhoInit = 0;             //!< [m] initial displacement offset
    double rhoDotInit = 0;          //!< [m/s] Initial displacement rate offset
	Eigen::Vector3d fHat_B {1.0, 0.0, 0.0};         //!< -- axis of translation in B frame components.
    Eigen::Vector3d r_FcF_F = Eigen::Vector3d::Zero();        //!< [m] vector pointing from location F to FC in F frame components
    Eigen::Vector3d r_F0B_B = Eigen::Vector3d::Zero();        //!< [m] vector pointing from body frame B origin to point to F0 origin of F frame in B frame components
    Eigen::Matrix3d IPntFc_F = Eigen::Matrix3d::Identity();   //!< [kg-m^2] Inertia of pc about point Fc in F frame component
    Eigen::Matrix3d dcm_FB = Eigen::Matrix3d::Identity();     //!< -- DCM from the F frame to the body frame
    std::string nameOfRhoState;     //!< [-] Identifier for the rho state data container
    std::string nameOfRhoDotState;  //!< [-] Identifier for the rhoDot state data container

    int lockFlag = 0;                   //!< [] flag for locking the translation axis
    double rho;                         //!< [m] displacement from equilibrium
    double rhoDot;                      //!< [m/s] time derivative of displacement from equilibrium
    double rhoRef = 0.0;                //!< [m] translating body reference position
    double rhoDotRef = 0.0;             //!< [m/s] translating body reference velocity
    double motorForce = 0.0;            //!< [N] optional motor force
    Eigen::Vector3d r_FcB_B;            //!< [m] position vector from B to center of mass location of effector
    Eigen::Vector3d r_FcF0_B;           //!< [m] vector pointing from point p0 origin of F frame to center of mass location of effector in B frame components
    Eigen::Matrix3d rTilde_FcF_B;       //!< [m] tilde matrix of r_FcF_B
	Eigen::Vector3d rPrime_FcF_B;       //!< [m/s] Body time derivative of r_FcF_B
	Eigen::Matrix3d rPrimeTilde_FcF_B;  //!< [m/s] Tilde matrix of rPrime_FcF_B
	Eigen::Matrix3d rTilde_FcB_B;       //!< [m] tilde matrix of r_FcB_B
	Eigen::Vector3d rPrime_FcB_B;       //!< [m/s] Body time derivative of r_FcB_B
	Eigen::Matrix3d rPrimeTilde_FcB_B;  //!< [m/s] Tilde matrix of rPrime_FcB_B
	Eigen::Matrix3d IPntFc_B;           //!< [kg-m^2] Inertia of Fc about point B in B frame components
	Eigen::Matrix3d dcm_BN;             //!< -- DCM from the B frame to the N frame
    Eigen::Vector3d omega_BN_B;         //!< [rad/s] angular velocity of the B frame wrt the N frame in B frame components.
    Eigen::Matrix3d omegaTilde_BN_B;    //!< [rad/s] tilde matrix of omega_BN_B

    Eigen::Vector3d aRho;          //!< -- Term needed for back-sub method
    Eigen::Vector3d bRho;          //!< -- Term needed for back-sub method
    double cRho;                   //!< -- Term needed for back-sub method

    StateData *rhoState;		   //!< -- state data for displacement from equilibrium
    StateData *rhoDotState;		   //!< -- state data for time derivative of rho;
    Eigen::MatrixXd *g_N;          //!< [m/s^2] gravitational acceleration in N frame components
    Eigen::MatrixXd* inertialPositionProperty = nullptr;  //!< [m] r_N inertial position relative to system spice zeroBase/refBase
    Eigen::MatrixXd* inertialVelocityProperty = nullptr;  //!< [m] v_N inertial velocity relative to system spice zeroBase/refBase
    static uint64_t effectorID;    //!< [] ID number of this panel

    // Translating body properties
    Eigen::Vector3d r_FcN_N;            //!< [m] position vector of translating body's center of mass Fc relative to the inertial frame origin N
    Eigen::Vector3d v_FcN_N;            //!< [m/s] inertial velocity vector of Fc relative to inertial frame
    Eigen::Vector3d sigma_FN;           //!< -- MRP attitude of frame F relative to inertial frame
    Eigen::Vector3d omega_FN_F;         //!< [rad/s] inertial translating body frame angular velocity vector

    // Functions
    void Reset(uint64_t CurrentClock) override;
	void registerStates(DynParamManager& states);
	void linkInStates(DynParamManager& states);
    void writeOutputStateMessages(uint64_t CurrentSimNanos);
    void updateEffectorMassProps(double integTime);
    void updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N);
    void updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B,
                                              double & rotEnergyContr, Eigen::Vector3d omega_BN_B);
    void computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N,
                            Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN);
    void UpdateState(uint64_t CurrentSimNanos);
    void computeTranslatingBodyInertialStates();
    void computeBackSubContributions(BackSubMatrices& backSubContr, Eigen::Vector3d F_g);
};

#endif /* LINEAR_TRANSLATION_ONE_DOF_STATE_EFFECTOR_H */
