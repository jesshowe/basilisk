/*
 ISC License

 Copyright (c) 2016-2018, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef _SUN_SAFE_POINT_H_
#define _SUN_SAFE_POINT_H_

#include "messaging/static_messaging.h"
#include "fswMessages/attGuidFswMsg.h"
#include "simFswInterfaceMessages/navAttIntMsg.h"
#include "fswMessages/cssConfigFswMsg.h"
#include "fswMessages/imuSensorBodyFswMsg.h"
#include <stdint.h>

/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Top level structure for the sun-safe attitude guidance routine.
 This algorithm is intended to be incredibly simple and robust*/
typedef struct {
    char outputDataName[MAX_STAT_MSG_LENGTH]; /*!< The name of the output message*/
    char inputSunVecName[MAX_STAT_MSG_LENGTH]; /*!< The name of the Input message*/
    char inputIMUDataName[MAX_STAT_MSG_LENGTH]; /*!< The name of the incoming IMU message*/
    double minUnitMag;       /*!< -- The minimally acceptable sun body unit vector*/
    double sunAngleErr;      /*!< r  The current error between cmd and obs sun angle*/
    double sunMnvrVec[3];    /*!< -- The eigen axis that we want to rotate on to get sun*/
    double sHatBdyCmd[3];    /*!< -- Desired body vector to point at the sun*/
    int32_t outputMsgID;     /*!< -- ID for the outgoing body estimate message*/
    int32_t inputMsgID;      /*!< -- ID for the incoming CSS sensor message*/
    int32_t imuMsgID;        /*!< -- ID for the incoming CSS sensor message*/
    AttGuidFswMsg attOut;   /*!< -- The output data that we compute*/
}sunSafePointConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_sunSafePoint(sunSafePointConfig *ConfigData, uint64_t moduleID);
    void CrossInit_sunSafePoint(sunSafePointConfig *ConfigData, uint64_t moduleID);
    void Update_sunSafePoint(sunSafePointConfig *ConfigData, uint64_t callTime,
        uint64_t moduleID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif