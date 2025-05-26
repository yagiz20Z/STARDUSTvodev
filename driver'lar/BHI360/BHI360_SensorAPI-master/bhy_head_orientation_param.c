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
* @file       bhy_head_orientation_param.c
* @date       2025-03-28
* @version    v2.1.0
*
*/

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <string.h>
#include <stdlib.h>

/*********************************************************************/
/* BHY SensorAPI header files */
/*********************************************************************/
#include "bhy.h"

/*********************************************************************/
/* own header files */
/*********************************************************************/
#include "bhy_head_orientation_param.h"

#define BHY_ROUND_UP_4_MUL(x)  (((x) % 4) ? (uint16_t)((((x) / 4) + 1) * \
                                                       4) : (uint16_t)(x))

/*lint -e506 */

/*!
 * @brief To trigger the Head Misalignment Calibration
 *
 * @param[in] dev  Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhy_head_orientation_param_trigger_hmc_calibration(struct bhy_dev *dev)
{
    int8_t rslt = BHY_OK;
    uint16_t param_id = BHY_HEAD_ORIENTATION_PARAM_PAGE_BASE + BHY_HEAD_ORIENTATION_PARAM_HMC_TRIGGER_CALIB_ID;
    uint8_t trig_set[BHY_HEAD_ORIENTATION_PARAM_HMC_TRIGGER_CALIB_LENGTH] = { 0 };

    if (dev == NULL)
    {
        rslt = BHY_E_NULL_PTR;
    }
    else
    {
        trig_set[0] = BHY_HEAD_ORIENTATION_PARAM_HMC_TRIGGER_CALIB_SET;
        rslt = bhy_set_parameter(param_id,
                                 (const uint8_t*)trig_set,
                                 BHY_HEAD_ORIENTATION_PARAM_HMC_TRIGGER_CALIB_LENGTH,
                                 dev);
    }

    return rslt;
}

/*!
 * @brief To set the Head Misalignment Calibration Configuration
 *
 * @param[in] config  Reference to hold the Head Misalignment Calibration Configuration
 * @param[in] dev     Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhy_head_orientation_param_set_hmc_configuration(const bhy_head_orientation_param_misalignment_config *config,
                                                        struct bhy_dev *dev)
{
    int8_t rslt = BHY_OK;
    uint16_t param_id = BHY_HEAD_ORIENTATION_PARAM_PAGE_BASE + BHY_HEAD_ORIENTATION_PARAM_HMC_CONFIG_ID;

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHY_E_NULL_PTR;
    }
    else
    {
        rslt = bhy_set_parameter(param_id, (const uint8_t*)config, BHY_HEAD_ORIENTATION_PARAM_HMC_CONFIG_LENGTH, dev);
    }

    return rslt;
}

/*!
 * @brief To get the Head Misalignment Calibration Configuration
 *
 * @param[out] config  Reference to hold the Head Misalignment Calibration Configuration
 * @param[in] dev      Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhy_head_orientation_param_get_hmc_configuration(bhy_head_orientation_param_misalignment_config *config,
                                                        struct bhy_dev *dev)
{
    int8_t rslt = BHY_OK;
    uint32_t act_len;
    uint8_t buffer[BHY_HEAD_ORIENTATION_PARAM_HMC_CONFIG_LENGTH] = { 0U };
    uint16_t param_id = BHY_HEAD_ORIENTATION_PARAM_PAGE_BASE + BHY_HEAD_ORIENTATION_PARAM_HMC_CONFIG_ID;

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHY_E_NULL_PTR;
    }
    else
    {
        rslt = bhy_get_parameter(param_id, buffer, BHY_HEAD_ORIENTATION_PARAM_HMC_CONFIG_LENGTH, &act_len, dev);
        if (rslt == BHY_OK)
        {
            config->still_phase_max_dur = buffer[0];
            config->still_phase_min_dur = buffer[1];
            config->still_phase_max_samples = buffer[2];
            config->acc_diff_threshold = BHY_LE2S32(&buffer[3]);
        }
    }

    return rslt;
}

/*!
 * @brief To set Head Misalignment Calibration Configuration to default
 *
 * @param[in] dev  Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhy_head_orientation_param_set_default_hmc_cfg(struct bhy_dev *dev)
{
    int8_t rslt = BHY_OK;
    uint16_t param_id = BHY_HEAD_ORIENTATION_PARAM_PAGE_BASE + BHY_HEAD_ORIENTATION_PARAM_HMC_SET_DEF_CONFIG_ID;
    uint8_t default_set[BHY_HEAD_ORIENTATION_PARAM_HMC_SET_DEF_CONFIG_LENGTH] = { 0 };

    if (dev == NULL)
    {
        rslt = BHY_E_NULL_PTR;
    }
    else
    {
        default_set[0] = BHY_HEAD_ORIENTATION_PARAM_HMC_SET_DEF_CONFIG_SET;
        rslt = bhy_set_parameter(param_id,
                                 (const uint8_t*)default_set,
                                 BHY_HEAD_ORIENTATION_PARAM_HMC_SET_DEF_CONFIG_LENGTH,
                                 dev);
    }

    return rslt;
}

/*!
 * @brief To get the Head Misalignment Calibrator Version
 *
 * @param[out] hmc_version  Reference to hold the Head Misalignment Calibrator Version
 * @param[in] dev           Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhy_head_orientation_param_get_hmc_version(bhy_head_orientation_param_ver *hmc_version, struct bhy_dev *dev)
{
    int8_t rslt = BHY_OK;
    uint32_t act_len;
    uint16_t param_id = BHY_HEAD_ORIENTATION_PARAM_PAGE_BASE + BHY_HEAD_ORIENTATION_PARAM_HMC_VERSION_ID;

    if ((hmc_version == NULL) || (dev == NULL))
    {
        rslt = BHY_E_NULL_PTR;
    }
    else
    {
        rslt = bhy_get_parameter(param_id,
                                 (uint8_t*)hmc_version,
                                 BHY_HEAD_ORIENTATION_PARAM_HMC_VERSION_LENGTH,
                                 &act_len,
                                 dev);
    }

    return rslt;
}

/*!
 * @brief To set the Head Misalignment Quaternion Calibration Correction Configuration
 *
 * @param[in] config  Reference to hold the Head Misalignment Quaternion Calibration Correction Configuration
 * @param[in] dev     Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhy_head_orientation_param_set_hmc_quat_cal_cor_cfg(
    const bhy_head_orientation_param_misalignment_quat_corr *config,
    struct bhy_dev *dev)
{
    int8_t rslt = BHY_OK;
    uint16_t param_id = BHY_HEAD_ORIENTATION_PARAM_PAGE_BASE + BHY_HEAD_ORIENTATION_PARAM_HMC_QUAT_CALIB_CORR_ID;

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHY_E_NULL_PTR;
    }
    else
    {
        rslt =
            bhy_set_parameter(param_id, (uint8_t*)config, BHY_HEAD_ORIENTATION_PARAM_HMC_QUAT_CALIB_CORR_WLENGTH, dev);
    }

    return rslt;
}

/*!
 * @brief To get the Head Misalignment Quaternion Calibration Correction Configuration
 *
 * @param[out] config  Reference to hold the Head Misalignment Quaternion Calibration Correction Configuration
 * @param[in] dev      Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhy_head_orientation_param_get_hmc_quat_cal_cor_cfg(bhy_head_orientation_param_misalignment_quat_corr *config,
                                                           struct bhy_dev *dev)
{
    int8_t rslt = BHY_OK;
    uint32_t act_len;
    uint8_t buffer[BHY_HEAD_ORIENTATION_PARAM_HMC_QUAT_CALIB_CORR_RLENGTH] = { 0U };
    uint16_t param_id = BHY_HEAD_ORIENTATION_PARAM_PAGE_BASE + BHY_HEAD_ORIENTATION_PARAM_HMC_QUAT_CALIB_CORR_ID;

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHY_E_NULL_PTR;
    }
    else
    {
        rslt =
            bhy_get_parameter(param_id, buffer, BHY_HEAD_ORIENTATION_PARAM_HMC_QUAT_CALIB_CORR_RLENGTH, &act_len, dev);
        if (rslt == BHY_OK)
        {
            config->quaternion_x.u32_val = BHY_LE2U32(&buffer[0]);
            config->quaternion_y.u32_val = BHY_LE2U32(&buffer[4]);
            config->quaternion_z.u32_val = BHY_LE2U32(&buffer[8]);
            config->quaternion_w.u32_val = BHY_LE2U32(&buffer[12]);
            config->accuracy.u32_val = BHY_LE2U32(&buffer[16]);
        }
    }

    return rslt;
}

/*!
 * @brief To set the Head Misalignment Mode and Vector X value
 *
 * @param[in] config  Reference to hold the Head Misalignment Mode and Vector X value
 * @param[in] dev     Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhy_head_orientation_param_set_hmc_mode_vector_x(const bhy_head_misalignment_mode_vector_x *config,
                                                        struct bhy_dev *dev)
{
    int8_t rslt = BHY_OK;
    uint16_t param_id = BHY_HEAD_ORIENTATION_PARAM_PAGE_BASE + BHY_HEAD_ORIENTATION_PARAM_HMC_PARAM_SET_MODE_ID;

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHY_E_NULL_PTR;
    }
    else
    {
        rslt =
            bhy_set_parameter(param_id,
                              (const uint8_t*)config,
                              BHY_ROUND_UP_4_MUL(BHY_HEAD_ORIENTATION_PARAM_HMC_PARAM_SET_MODE_LENGTH),
                              dev);
    }

    return rslt;
}

/*!
 * @brief To get the Head Misalignment Mode and Vector X value
 *
 * @param[out] config  Reference to hold the Head Misalignment Mode and Vector X value
 * @param[in] dev      Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhy_head_orientation_param_get_hmc_mode_vector_x(bhy_head_misalignment_mode_vector_x *config,
                                                        struct bhy_dev *dev)
{
    int8_t rslt = BHY_OK;
    uint32_t act_len;
    uint8_t buffer[BHY_ROUND_UP_4_MUL(BHY_HEAD_ORIENTATION_PARAM_HMC_PARAM_SET_MODE_LENGTH)] = { 0U };
    uint16_t param_id = BHY_HEAD_ORIENTATION_PARAM_PAGE_BASE + BHY_HEAD_ORIENTATION_PARAM_HMC_PARAM_SET_MODE_ID;

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHY_E_NULL_PTR;
    }
    else
    {
        rslt =
            bhy_get_parameter(param_id,
                              buffer,
                              BHY_ROUND_UP_4_MUL(BHY_HEAD_ORIENTATION_PARAM_HMC_PARAM_SET_MODE_LENGTH),
                              &act_len,
                              dev);
        if (rslt == BHY_OK)
        {
            config->mode = buffer[0];
            config->vector_x_0.u32_val = BHY_LE2U32(&buffer[1]);
            config->vector_x_1.u32_val = BHY_LE2U32(&buffer[5]);
            config->vector_x_2.u32_val = BHY_LE2U32(&buffer[9]);
        }
    }

    return rslt;
}

/*!
 * @brief To set the Head Misalignment Quaternion Initial Head Correction
 *
 * @param[in] config  Reference to hold the Head Misalignment Quaternion Initial Head Correction
 * @param[in] dev     Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhy_head_orientation_param_set_quat_init_head_corr(const uint8_t *config, struct bhy_dev *dev)
{
    int8_t rslt = BHY_OK;
    uint16_t param_id = BHY_HEAD_ORIENTATION_PARAM_PAGE_BASE + BHY_HEAD_ORIENTATION_PARAM_QUAT_INITIAL_HEAD_CORR_ID;

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHY_E_NULL_PTR;
    }
    else
    {
        rslt = bhy_set_parameter(param_id, config, BHY_HEAD_ORIENTATION_PARAM_QUAT_INITIAL_HEAD_CORR_LENGTH, dev);
    }

    return rslt;
}

/*!
 * @brief To get the Head Misalignment Quaternion Initial Head Correction
 *
 * @param[out] config  Reference to hold the Head Misalignment Quaternion Initial Head Correction
 * @param[in] dev      Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhy_head_orientation_param_get_quat_init_head_corr(uint8_t *config, struct bhy_dev *dev)
{
    int8_t rslt = BHY_OK;
    uint32_t act_len;
    uint8_t buffer[BHY_HEAD_ORIENTATION_PARAM_QUAT_INITIAL_HEAD_CORR_LENGTH] = { 0U };
    uint16_t param_id = BHY_HEAD_ORIENTATION_PARAM_PAGE_BASE + BHY_HEAD_ORIENTATION_PARAM_QUAT_INITIAL_HEAD_CORR_ID;

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHY_E_NULL_PTR;
    }
    else
    {
        rslt = bhy_get_parameter(param_id,
                                 buffer,
                                 BHY_HEAD_ORIENTATION_PARAM_QUAT_INITIAL_HEAD_CORR_LENGTH,
                                 &act_len,
                                 dev);
        if (rslt == BHY_OK)
        {
            *config = buffer[0];
        }
    }

    return rslt;
}

/*!
 * @brief To get the IMU/NDOF Head Orientation Version
 *
 * @param[out] ho_version Reference to hold the IMU/NDOF Head Orientation Version
 * @param[in] dev         Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhy_head_orientation_param_get_ho_version(bhy_head_orientation_param_ver *ho_version, struct bhy_dev *dev)
{
    int8_t rslt = BHY_OK;
    uint32_t act_len;
    uint16_t param_id = BHY_HEAD_ORIENTATION_PARAM_PAGE_BASE + BHY_HEAD_ORIENTATION_PARAM_VERSION_ID;

    if ((ho_version == NULL) || (dev == NULL))
    {
        rslt = BHY_E_NULL_PTR;
    }
    else
    {
        rslt = bhy_get_parameter(param_id,
                                 (uint8_t*)ho_version,
                                 BHY_HEAD_ORIENTATION_PARAM_VERSION_LENGTH,
                                 &act_len,
                                 dev);
    }

    return rslt;
}

/*!
 * @brief To set the Head Misalignment Euler Initial Head Correction
 *
 * @param[in] config  Reference to hold the Head Misalignment Euler Initial Head Correction
 * @param[in] dev     Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhy_head_orientation_param_set_eul_init_head_corr(const uint8_t *config, struct bhy_dev *dev)
{
    int8_t rslt = BHY_OK;
    uint16_t param_id = BHY_HEAD_ORIENTATION_PARAM_PAGE_BASE + BHY_HEAD_ORIENTATION_PARAM_EUL_INITIAL_HEAD_CORR_ID;

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHY_E_NULL_PTR;
    }
    else
    {
        rslt = bhy_set_parameter(param_id,
                                 (const uint8_t*)config,
                                 BHY_HEAD_ORIENTATION_PARAM_EUL_INITIAL_HEAD_CORR_LENGTH,
                                 dev);
    }

    return rslt;
}

/*!
 * @brief To get the Head Misalignment Euler Initial Head Correction
 *
 * @param[out] config  Reference to hold the Head Misalignment Euler Initial Head Correction
 * @param[in] dev      Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhy_head_orientation_param_get_eul_init_head_corr(uint8_t *config, struct bhy_dev *dev)
{
    int8_t rslt = BHY_OK;
    uint32_t act_len;
    uint8_t buffer[BHY_HEAD_ORIENTATION_PARAM_EUL_INITIAL_HEAD_CORR_LENGTH] = { 0U };
    uint16_t param_id = BHY_HEAD_ORIENTATION_PARAM_PAGE_BASE + BHY_HEAD_ORIENTATION_PARAM_EUL_INITIAL_HEAD_CORR_ID;

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHY_E_NULL_PTR;
    }
    else
    {
        rslt = bhy_get_parameter(param_id,
                                 buffer,
                                 BHY_HEAD_ORIENTATION_PARAM_EUL_INITIAL_HEAD_CORR_LENGTH,
                                 &act_len,
                                 dev);
        if (rslt == BHY_OK)
        {
            *config = buffer[0];
        }
    }

    return rslt;
}
