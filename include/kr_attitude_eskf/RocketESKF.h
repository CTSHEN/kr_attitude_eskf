/**
* RocketESKF.hpp
*
* C.T. Shen, SRPL, IAA, NCKU
* 
* This file is part of srpl_eskf.
*
* This file is modified from AttitudeESKF.hpp in kr_attitude_eskf project.
*--------------------------------------------------------------
* AttitudeESKF.hpp
*
*  Copyright (c) 2013 Gareth Cross. Apache 2 License.
*
*  This file is part of kr_attitude_eskf.
*
*	Created on: 12/24/2013
*		  Author: gareth
*/

#ifndef SRPL_ESKF_H_
#define SRPL_ESKF_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include "AttitudeESKF.hpp"

namespace srpl {

    /**
     * @class RocketESKF
     * @brief Implementation of ESKF for attitude and position for sounding rockets
     * using quaternions.
     * @note Since the sounding rocket is in gravity-free condition during a 
     * parabolic flight phase, therefore, gravity vector is used as a reference
     * vector only in initialized phase. During the flight, magnetic field and sun
     * position vectors are reference vectors. Current states are:
     *      Quaternion (q1,q2,q3,q4)
     *      gyroBias (x,y,z) 
     *      Position (x,y,z)
     *      Velocity (x,y,z)
     *      accBias (x,y,z)
     * -Inputs are:
     *      gyroRaw (x,y,z)
     *      accRaw(x,y,z)
     * -Refernece vector:
     *      magneticField (x,y,z)
     *      sunPosition (x,y,z)
     *      gravity (x,y,z)
     * 
     */
    class RocketESKF : kr::AttitudeESKF {
        
        /**
         *  @brief Constructor
         */
        RocketESKF();

        /**
         * @brief prediction step.
         * @param wRaw Gyroscope raw data in body frame.
         * @param accRaw Accelerometer raw data in body frame.
         * @param dt Time step in second.
         * @param wCov Covariance of gyro measurement.
         * @param accCov Covariance of accelerometer measurement.
         * @param useRK4 If true, use RK4 integration - otherwise euler is used
         */
        void predict(const vec3 &wRaw, const vec3 &accRaw, scalar_t dt
                     const mat3 &wCov, const mat3 &accCov, bool userRK4=false);
    }
}