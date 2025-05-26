/**
* Copyright (c) 2025 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file       bhy_multi_tap_param.c
* @date       2025-03-28
* @version    v2.1.0
*
*/

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <string.h>
#include <stdio.h>

/*********************************************************************/
/* BHI3 SensorAPI header files */
/*********************************************************************/

/*********************************************************************/
/* own header files */
/*********************************************************************/
#include "bhy_multi_tap_param.h"

#include "bhy.h"

#define BHY_ROUND_UP_4_MUL(x)  (((x) % 4) ? (uint16_t)((((x) / 4) + 1) * \
                                                       4) : (uint16_t)(x))

/*lint -e506, -e778*/

/*!
 * @brief This API writes to the configuration parameter
 *
 * @param[in] conf  Reference to store MULTI_TAP configuration
 * @param[in] dev   Device instance
 *
 * @return  API error codes
 */
int8_t bhy_multi_tap_param_set_config(const bhy_event_data_multi_tap *conf, struct bhy_dev *dev)
{
    int8_t rslt = BHY_OK;
    uint16_t param_id = BHY_MULTI_TAP_PARAM_PAGE_BASE + BHY_MULTI_TAP_PARAM_ENABLE_PARAM_ID;

    if ((conf == NULL) || (dev == NULL))
    {
        rslt = BHY_E_NULL_PTR;
    }
    else
    {
        rslt = bhy_set_parameter(param_id,
                                 (const uint8_t*)conf,
                                 BHY_ROUND_UP_4_MUL(BHY_MULTI_TAP_PARAM_ENABLE_LENGTH),
                                 dev);
    }

    return rslt;
}

/*!
 * @brief To get the MULTI-TAP configuration parameters
 *
 * @param[out] conf  Reference to store MULTI_TAP configuration
 * @param[in]  dev   Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhy_multi_tap_param_get_config(bhy_event_data_multi_tap *conf, struct bhy_dev *dev)
{
    int8_t rslt = BHY_OK;
    uint32_t ret_length;
    uint16_t param_id = BHY_MULTI_TAP_PARAM_PAGE_BASE + BHY_MULTI_TAP_PARAM_ENABLE_PARAM_ID;
    uint8_t buffer[BHY_ROUND_UP_4_MUL(BHY_MULTI_TAP_PARAM_ENABLE_LENGTH)] = { 0U };

    if ((conf == NULL) || (dev == NULL))
    {
        rslt = BHY_E_NULL_PTR;
    }
    else
    {
        rslt = bhy_get_parameter(param_id,
                                 (uint8_t *)buffer,
                                 BHY_ROUND_UP_4_MUL(BHY_MULTI_TAP_PARAM_ENABLE_LENGTH),
                                 &ret_length,
                                 dev);

        if (rslt == BHY_OK)
        {
            *conf = (bhy_event_data_multi_tap)buffer[0];
        }
        else
        {
            /*! Invalid number of parameters readout */
            rslt = BHY_E_INVALID_EVENT_SIZE;
        }
    }

    return rslt;
}

/*!
 * @brief This API writes to the tap detector configuration parameter
 *
 * @param[in] conf  Reference to store MULTI_TAP Detector configuration
 * @param[in] dev   Device instance
 *
 * @return  API error codes
 */
int8_t bhy_multi_tap_param_detector_set_config(const bhy_multi_tap_param_detector *conf, struct bhy_dev *dev)
{
    int8_t rslt = BHY_OK;
    uint16_t param_id = BHY_MULTI_TAP_PARAM_PAGE_BASE + BHY_MULTI_TAP_PARAM_DETECTOR_CONFIG_PARAM_ID;

    if ((conf == NULL) || (dev == NULL))
    {
        rslt = BHY_E_NULL_PTR;
    }
    else
    {
        rslt =
            bhy_set_parameter(param_id,
                              (const uint8_t*)conf,
                              BHY_ROUND_UP_4_MUL(BHY_MULTI_TAP_PARAM_DETECTOR_CONFIG_LENGTH),
                              dev);
    }

    return rslt;
}

/*!
 * @brief To get the MULTI-TAP Detector configuration parameters
 *
 * @param[out] conf  Reference to store MULTI_TAP Detector configuration
 * @param[in]  dev   Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhy_multi_tap_param_detector_get_config(bhy_multi_tap_param_detector *conf, struct bhy_dev *dev)
{
    int8_t rslt = BHY_OK;
    uint32_t ret_length;
    uint16_t param_id = BHY_MULTI_TAP_PARAM_PAGE_BASE + BHY_MULTI_TAP_PARAM_DETECTOR_CONFIG_PARAM_ID;
    uint8_t buffer[BHY_ROUND_UP_4_MUL(BHY_MULTI_TAP_PARAM_DETECTOR_CONFIG_LENGTH)] = { 0U };

    if ((conf == NULL) || (dev == NULL))
    {
        rslt = BHY_E_NULL_PTR;
    }
    else
    {
        rslt = bhy_get_parameter(param_id,
                                 buffer,
                                 BHY_ROUND_UP_4_MUL(BHY_MULTI_TAP_PARAM_DETECTOR_CONFIG_LENGTH),
                                 &ret_length,
                                 dev);

        if (rslt == BHY_OK)
        {
            conf->stap_setting.as_s.axis_sel = ((uint16_t)buffer[0] & BHY_MULTI_TAP_PARAM_SINGLE_TAP_AXIS_SEL_MASK) >>
                                               BHY_MULTI_TAP_PARAM_SINGLE_TAP_AXIS_SEL_SHIFT;
            conf->stap_setting.as_s.wait_for_timeout =
                (((uint16_t)buffer[0] & BHY_MULTI_TAP_PARAM_SINGLE_TAP_WAIT_TIMEOUT_MASK) >>
                 BHY_MULTI_TAP_PARAM_SINGLE_TAP_WAIT_TIMEOUT_SHIFT);
            conf->stap_setting.as_s.max_peaks_for_tap =
                (((uint16_t)buffer[0] & BHY_MULTI_TAP_PARAM_SINGLE_TAP_MAX_PEAKS_FOR_TAP_MASK) >>
                 BHY_MULTI_TAP_PARAM_SINGLE_TAP_MAX_PEAKS_FOR_TAP_SHIFT);
            conf->stap_setting.as_s.mode =
                (((uint16_t)buffer[0] & BHY_MULTI_TAP_PARAM_SINGLE_TAP_FILTER_MODE_MASK) >>
                 BHY_MULTI_TAP_PARAM_SINGLE_TAP_FILTER_MODE_SHIFT);
            conf->dtap_setting.as_s.tap_peak_thres =
                (BHY_LE2U16(&buffer[2]) & BHY_MULTI_TAP_PARAM_DOUBLE_TAP_TAP_PEAK_DUR_MASK) >>
                BHY_MULTI_TAP_PARAM_DOUBLE_TAP_TAP_PEAK_DUR_SHIFT;
            conf->dtap_setting.as_s.max_gesture_dur =
                ((BHY_LE2U16(&buffer[2]) & BHY_MULTI_TAP_PARAM_DOUBLE_TAP_MAX_GES_DUR_MASK) >>
                 BHY_MULTI_TAP_PARAM_DOUBLE_TAP_MAX_GES_DUR_SHIFT);
            conf->ttap_setting.as_s.max_dur_between_peaks =
                ((uint16_t)buffer[4] & BHY_MULTI_TAP_PARAM_TRIPLE_TAP_MAX_DUR_BW_PEAKS_MASK) >>
                BHY_MULTI_TAP_PARAM_TRIPLE_TAP_MAX_DUR_BW_PEAKS_SHIFT;
            conf->ttap_setting.as_s.tap_shock_settling_dur =
                (((uint16_t)buffer[4] & BHY_MULTI_TAP_PARAM_TRIPLE_TAP_TAP_SHOCK_SETL_DUR_MASK) >>
                 BHY_MULTI_TAP_PARAM_TRIPLE_TAP_TAP_SHOCK_SETL_DUR_SHIFT);
            conf->ttap_setting.as_s.min_quite_dur_between_taps =
                ((uint16_t)buffer[5] & BHY_MULTI_TAP_PARAM_TRIPLE_TAP_MIN_QT_DUR_BW_PEAKS_MASK) >>
                BHY_MULTI_TAP_PARAM_TRIPLE_TAP_MIN_QT_DUR_BW_PEAKS_SHIFT;
            conf->ttap_setting.as_s.quite_time_after_gesture =
                (((uint16_t)buffer[5] & BHY_MULTI_TAP_PARAM_TRIPLE_TAP_QT_TM_AFTER_GESTURE_MASK) >>
                 BHY_MULTI_TAP_PARAM_TRIPLE_TAP_QT_TM_AFTER_GESTURE_SHIFT);
        }
        else
        {
            /*! Invalid number of parameters readout */
            rslt = BHY_E_INVALID_EVENT_SIZE;
        }
    }

    return rslt;
}
