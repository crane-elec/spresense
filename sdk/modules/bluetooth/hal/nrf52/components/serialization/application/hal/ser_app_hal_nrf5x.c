/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
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
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#include "app_util_platform.h"
#include "ser_app_hal.h"
//#include "nrf.h"
//#include "nrf_gpio.h"
#include "nrf_soc.h"
//#include "nrf_delay.h"
//#include "nrf_nvmc.h"
//#include "boards.h"
#include "ser_phy.h"
//#include "ser_phy_config_app.h"
//#include "nrf_drv_clock.h"

//#define SOFTDEVICE_EVT_IRQ      SD_EVT_IRQn         /**< SoftDevice Event IRQ number. Used for both protocol events and SoC events. */
//#define FLASH_WRITE_MAX_LENGTH  ((uint16_t)NRF_FICR->CODEPAGESIZE)
//#define BLE_FLASH_PAGE_SIZE     ((uint16_t)NRF_FICR->CODEPAGESIZE)  /**< Size of one flash page. */

#define CXD56_SYS_MIRROR  0x04000000
#define CXD56_TOPREG_BASE (CXD56_SYS_MIRROR + 0x00100000)
#define CXD56_IOGP_HIF_IRQ_OUT (CXD56_TOPREG_BASE + 0x2078)
#define REG32Write(addr, data) (*((volatile uint32_t*)(addr)) = (uint32_t)(data))

//static ser_app_hal_flash_op_done_handler_t m_flash_op_handler;
uint32_t ser_app_hal_hw_init(ser_app_hal_flash_op_done_handler_t handler)
{
    return NRF_SUCCESS;
}

void ser_app_hal_delay(uint32_t ms)
{
    usleep(ms * 1000);
}

void ser_app_hal_nrf_reset_pin_clear(void)
{
//    nrf_gpio_pin_clear(CONN_CHIP_RESET_PIN_NO);
}

void ser_app_hal_nrf_reset_pin_set(void)
{
//    nrf_gpio_pin_set(CONN_CHIP_RESET_PIN_NO);
}

void ser_app_hal_nrf_evt_irq_priority_set(void)
{
//    NVIC_SetPriority(SOFTDEVICE_EVT_IRQ, APP_IRQ_PRIORITY_LOWEST);
}

void ser_app_hal_nrf_evt_pending(void)
{
//    NVIC_SetPendingIRQ(SOFTDEVICE_EVT_IRQ);
}
