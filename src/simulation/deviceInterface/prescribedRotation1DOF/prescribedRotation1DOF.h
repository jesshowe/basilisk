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

#ifndef _PRESCRIBEDROTATION1DOF_
#define _PRESCRIBEDROTATION1DOF_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/bskLogging.h"
#include "cMsgCInterface/HingedRigidBodyMsg_C.h"
#include "cMsgCInterface/PrescribedRotationMsg_C.h"
#include <Eigen/Dense>
#include <cstdint>

/*! @brief Prescribed 1 DOF Rotation Profiler Class */
class PrescribedRotation1DOF: public SysModel{
public:
    PrescribedRotation1DOF() = default;                                    //!< Constructor
    ~PrescribedRotation1DOF() = default;                                   //!< Destructor

    void SelfInit() override;                                              //!< Member function to initialize the C-wrapped output message
    void Reset(uint64_t CurrentSimNanos) override;                         //!< Reset member function
    void UpdateState(uint64_t CurrentSimNanos) override;                   //!< Update member function
    void setCoastOptionRampDuration(double rampDuration);                  //!< Setter for the coast option ramp duration

    void setRotHat_M(const Eigen::Vector3d &rotHat_M);                     //!< Setter for the spinning body rotation axis
    void setSmoothingDuration(double smoothingDuration);                    //!< Setter method for the duration the acceleration is smoothed until reaching the given maximum acceleration value

    void setThetaDDotMax(double thetaDDotMax);                             //!< Setter for the ramp segment scalar angular acceleration
    void setThetaInit(double thetaInit);                                   //!< Setter for the initial spinning body angle
    double getCoastOptionRampDuration() const;                             //!< Getter for the coast option ramp duration
    const Eigen::Vector3d &getRotHat_M() const;                            //!< Getter for the spinning body rotation axis
    double getSmoothingDuration() const;                                    //!< Getter method for the duration the acceleration is smoothed until reaching the given maximum acceleration value
    double getThetaDDotMax() const;                                        //!< Getter for the ramp segment scalar angular acceleration
    double getThetaInit() const;                                           //!< Getter for the initial spinning body angle

    ReadFunctor<HingedRigidBodyMsgPayload> spinningBodyInMsg;              //!< Input msg for the spinning body reference angle and angle rate
    Message<HingedRigidBodyMsgPayload> spinningBodyOutMsg;                 //!< Output msg for the spinning body angle and angle rate
    Message<PrescribedRotationMsgPayload> prescribedRotationOutMsg;        //!< Output msg for the spinning body prescribed rotational states
    HingedRigidBodyMsg_C spinningBodyOutMsgC = {};                         //!< C-wrapped output msg for the spinning body angle and angle rate
    PrescribedRotationMsg_C prescribedRotationOutMsgC = {};                //!< C-wrapped output msg for the spinning body prescribed rotational states

    BSKLogger *bskLogger;                                                  //!< BSK Logging

private:

        /* Non-smoothed bang-bang option member functions */
        void computeBangBangParametersNoSmoothing();                        //!< Method for computing the required parameters for the rotation with a non-smoothed bang-bang acceleration profile

        /* Non-smoothed bang-coast-bang option member functions */
        void computeBangCoastBangParametersNoSmoothing();                   //!< Method for computing the required parameters for the non-smoothed bang-coast-bang option

        /* Smoothed bang-bang option member functions */
        void computeSmoothedBangBangParameters();                           //!< Method for computing the required parameters for the rotation with a smoothed bang-bang acceleration profile

        /* Smoothed bang-coast-bang member functions */
        void computeSmoothedBangCoastBangParameters();                      //!< Method for computing the required parameters for the smoothed bang-coast-bang option
        bool isInFourthSmoothedSegment(double time) const;                  //!< Method for determining if the current time is within the fourth smoothing segment for the smoothed bang-coast-bang option
        void computeFourthSmoothedSegment(double time);                     //!< Method for computing the fourth smoothing segment scalar rotational states for the smoothed bang-coast-bang option

        /* Shared member functions */
        void computeCoastSegment(double time);                      //!< Method for computing the coast segment scalar rotational states
        void computeFirstBangSegment(double time);                  //!< Method for computing the first bang segment scalar rotational states
        void computeFirstSmoothedSegment(double time);              //!< Method for computing the first smoothing segment scalar rotational states for the smoothed profiler options
        void computeSecondBangSegment(double time);                 //!< Method for computing the second bang segment scalar rotational states
        void computeSecondSmoothedSegment(double time);             //!< Method for computing the second smoothing segment scalar rotational states for the smoothed profiler options
        void computeThirdSmoothedSegment(double time);              //!< Method for computing the third smoothing segment scalar rotational states for the smoothed profiler options
        void computeRotationComplete();                             //!< Method for computing the scalar rotational states when the rotation is complete
        bool isInCoastSegment(double time) const;                   //!< Method for determining if the current time is within the coast segment
        bool isInFirstBangSegment(double time) const;               //!< Method for determining if the current time is within the first bang segment
        bool isInFirstSmoothedSegment(double time) const;           //!< Method for determining if the current time is within the first smoothing segment for the smoothed profiler options
        bool isInSecondBangSegment(double time) const;              //!< Method for determining if the current time is within the second bang segment
        bool isInSecondSmoothedSegment(double time) const;          //!< Method for determining if the current time is within the second smoothing segment for the smoothed profiler options
        bool isInThirdSmoothedSegment(double time) const;           //!< Method for determining if the current time is within the third smoothing segment for the smoothed profiler options
        Eigen::Vector3d computeSigma_FM();                          //!< Method for computing the current spinning body MRP attitude relative to the mount frame: sigma_FM

        /* User-configurable variables */
        double coastOptionRampDuration;                             //!< [s] Time used for the coast option bang segment
        double smoothingDuration;                                   //!< [s] Time the acceleration is smoothed to the given maximum acceleration value
        double thetaDDotMax;                                        //!< [rad/s^2] Maximum angular acceleration of spinning body used in the ramp segments
        Eigen::Vector3d rotHat_M;                                   //!< Spinning body rotation axis in M frame components

        /* Non-smoothed bang-coast-bang option variables */
        double theta_ts1;
        double thetaDot_ts1;
        double theta_tb1;                                         //!< [m] Position at the end of the first bang segment
        double thetaDot_tb1;                                         //!< [m/s] Velocity at the end of the first bang segment
        double theta_ts2;
        double thetaDot_ts2;
        double theta_tc;
        double thetaDot_tc;
        double theta_ts3;
        double thetaDot_ts3;
        double theta_tb2;
        double thetaDot_tb2;

        /* Smoothed bang-coast-bang option variables */
        double t_s3;

        /* Shared module variables */
        double theta;                                               //!< [rad] Current angle
        double thetaDot;                                            //!< [rad/s] Current angle rate
        double thetaDDot;                                           //!< [rad/s^2] Current angular acceleration
        bool convergence;                                           //!< Boolean variable is true when the rotation is complete
        double tInit;                                               //!< [s] Simulation time at the beginning of the rotation
        double thetaInit;                                           //!< [rad] Initial spinning body angle from frame M to frame F about rotHat_M
        double thetaDotInit;                                        //!< [rad/s] Initial spinning body angle rate between frame M to frame F
        double thetaRef;                                            //!< [rad] Spinning body reference angle from frame M to frame F about rotHat_M
        double t_b1;
        double t_b2;
        double t_c;
        double t_f;                                                  //!< [s] The simulation time when the rotation is complete
        double t_s1;
        double t_s2;
        double a;                                                   //!< Parabolic constant for the first half of the rotation
        double b;                                                   //!< Parabolic constant for the second half of the rotation
};

#endif
