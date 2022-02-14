/**
 * Copyright (c) 2017 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "sdk_common.h"
#if NRF_MODULE_ENABLED(NRF_SDH)

#include "nrf_sdh.h"

#include <stdint.h>

#include "nrf_sdm.h"
//#include "nrf_nvic.h"
#include "sdk_config.h"
#include "app_error.h"

#ifdef BLE_USE_SECTION
// Create section "sdh_stack_observers".
NRF_SECTION_SET_DEF(sdh_stack_observers, nrf_sdh_stack_observer_t, NRF_SDH_STACK_OBSERVER_PRIO_LEVELS);
#endif

static bool m_nrf_sdh_enabled;   /**< Variable to indicate whether the SoftDevice is enabled. */
static bool m_nrf_sdh_suspended; /**< Variable to indicate whether this module is suspended. */
static bool m_nrf_sdh_continue;  /**< Variable to indicate whether enable/disable process was started. */


ret_code_t nrf_sdh_enable_request(void)
{
    ret_code_t ret_code;

    if (m_nrf_sdh_enabled)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    m_nrf_sdh_continue = true;

    ret_code = sd_softdevice_enable(NULL, NULL);
    if (ret_code != NRF_SUCCESS)
    {
        return ret_code;
    }

    m_nrf_sdh_enabled   = true;
    m_nrf_sdh_continue  = false;
    m_nrf_sdh_suspended = false;

    return NRF_SUCCESS;
}


ret_code_t nrf_sdh_disable_request(void)
{
    ret_code_t ret_code;

    if (!m_nrf_sdh_enabled)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    m_nrf_sdh_continue = true;

    ret_code          = sd_softdevice_disable();
    if (ret_code != NRF_SUCCESS)
    {
        return ret_code;
    }

    m_nrf_sdh_enabled  = false;
    m_nrf_sdh_continue = false;

    return NRF_SUCCESS;
}


ret_code_t nrf_sdh_request_continue(void)
{
    if (!m_nrf_sdh_continue)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (m_nrf_sdh_enabled)
    {
        return nrf_sdh_disable_request();
    }
    else
    {
        return nrf_sdh_enable_request();
    }
}


bool nrf_sdh_is_enabled(void)
{
    return m_nrf_sdh_enabled;
}


void nrf_sdh_suspend(void)
{
    if (!m_nrf_sdh_enabled)
    {
        return;
    }
    m_nrf_sdh_suspended = true;
}


void nrf_sdh_resume(void)
{
    if ((!m_nrf_sdh_suspended) || (!m_nrf_sdh_enabled))
    {
        return;
    }

    m_nrf_sdh_suspended = false;
}


bool nrf_sdh_is_suspended(void)
{
    return (!m_nrf_sdh_enabled) || (m_nrf_sdh_suspended);
}


#ifdef BLE_USE_SECTION
void nrf_sdh_evts_poll(void)
{
    nrf_section_iter_t iter;

    // Notify observers about pending SoftDevice event.
    for (nrf_section_iter_init(&iter, &sdh_stack_observers);
         nrf_section_iter_get(&iter) != NULL;
         nrf_section_iter_next(&iter))
    {
        nrf_sdh_stack_observer_t    * p_observer;
        nrf_sdh_stack_evt_handler_t   handler;

        p_observer = (nrf_sdh_stack_observer_t *) nrf_section_iter_get(&iter);
        handler    = p_observer->handler;

        handler(p_observer->p_context);
    }
}
#else
extern void nrf_sdh_ble_evts_poll(void * p_context);
void nrf_sdh_evts_poll(void)
{
    nrf_sdh_ble_evts_poll(NULL);
}
#endif

#endif // NRF_MODULE_ENABLED(NRF_SDH)
