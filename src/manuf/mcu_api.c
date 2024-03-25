/*!*****************************************************************
 * \file    mcu_api.c
 * \brief   MCU drivers.
 *******************************************************************
 * \copyright
 *
 * Copyright (c) 2022, UnaBiz SAS
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1 Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  2 Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  3 Neither the name of UnaBiz SAS nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************/

#include "manuf/mcu_api.h"
#include "manuf/TI_aes_128_encr_only.h"

#ifdef USE_SIGFOX_EP_FLAGS_H
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_types.h"
#include "sigfox_error.h"

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/settings/settings.h>

#define EP_ID CONFIG_SIGFOX_EP_ID
#define INITIAL_PAC CONFIG_SIGFOX_INITIAL_PAC
#define NAK CONFIG_SIGFOX_NAK

/*** MCU API functions ***/

#ifdef TIMER_REQUIRED
struct work_timer {
	struct k_work_delayable work;
	MCU_API_timer_t timer;
};

static struct work_timer work_timer_1;
static struct work_timer work_timer_2;

static void work_timer_expire_fn(struct k_work *work)
{
	struct work_timer *work_timer = CONTAINER_OF(work, struct work_timer, work);
	work_timer->timer.cplt_cb();
}
#endif

static sfx_u8 current_nvm_data[SIGFOX_NVM_DATA_SIZE_BYTES] = { 0 };

static int sigfox_settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	const char *next;
	int err;

	if (settings_name_steq(name, "nvm_data", &next) && !next) {
		if (len != sizeof(current_nvm_data)) {
			return -EINVAL;
		}

		err = read_cb(cb_arg, &current_nvm_data, sizeof(current_nvm_data));
		if (err >= 0) {
			return 0;
		}

		return err;
	}

	return -ENOENT;
}

struct settings_handler sigfox_settings_handler_config = {
	.name = "sigfox",
	.h_set = sigfox_settings_set
};

#if (defined ASYNCHRONOUS) || (defined LOW_LEVEL_OPEN_CLOSE)
/*******************************************************************/
MCU_API_status_t MCU_API_open(MCU_API_config_t *mcu_api_config) {
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif

	int err;
	
	err = settings_subsys_init();
	if (err) {
		printk("settings subsys init err: %d\n", err);
		status = MCU_API_ERROR;
		goto errors;
	} else {
		printk("settings subsys init ok\n");
	}

	err = settings_register(&sigfox_settings_handler_config);
	if (err) {
		printk("settings register err: %d\n", err);
		status = MCU_API_ERROR;
		goto errors;
	} else {
		printk("settings register ok\n");
	}

	err = settings_load();
	if (err) {
		printk("settings load err: %d\n", err);
		status = MCU_API_ERROR;
		goto errors;
	} else {
		printk("settings load ok\n");
	}

#ifdef TIMER_REQUIRED
	k_work_init_delayable(&work_timer_1.work, work_timer_expire_fn);
	k_work_init_delayable(&work_timer_2.work, work_timer_expire_fn);
#endif

errors:
	RETURN();
}
#endif

#ifdef LOW_LEVEL_OPEN_CLOSE
/*******************************************************************/
MCU_API_status_t MCU_API_close(void) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	RETURN();
}
#endif

#ifdef ASYNCHRONOUS
/*******************************************************************/
MCU_API_status_t MCU_API_process(void) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	RETURN();
}
#endif

#ifdef TIMER_REQUIRED
/*******************************************************************/
MCU_API_status_t MCU_API_timer_start(MCU_API_timer_t *timer) {
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif

	switch (timer->instance) {
	case MCU_API_TIMER_1:
		work_timer_1.timer = *timer;
		k_work_reschedule(&work_timer_1.work, K_MSEC(timer->duration_ms));
		break;
	case MCU_API_TIMER_2:
		work_timer_2.timer = *timer;
		k_work_reschedule(&work_timer_1.work, K_MSEC(timer->duration_ms));
		break;
	default:
		status = MCU_API_ERROR;
	}
	
	RETURN();
}
#endif

#ifdef TIMER_REQUIRED
/*******************************************************************/
MCU_API_status_t MCU_API_timer_stop(MCU_API_timer_instance_t timer_instance) {
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif

	RETURN();
}
#endif

#if (defined TIMER_REQUIRED) && !(defined ASYNCHRONOUS)
/*******************************************************************/
MCU_API_status_t MCU_API_timer_status(MCU_API_timer_instance_t timer_instance, sfx_bool *timer_has_elapsed) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	RETURN();
}
#endif

#if (defined TIMER_REQUIRED) && !(defined ASYNCHRONOUS)
/*******************************************************************/
MCU_API_status_t MCU_API_timer_wait_cplt(MCU_API_timer_instance_t timer_instance) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	RETURN();
}
#endif

/*******************************************************************/
MCU_API_status_t MCU_API_aes_128_cbc_encrypt(MCU_API_encryption_data_t *aes_data) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif

	uint8_t nak[SIGFOX_EP_KEY_SIZE_BYTES];
	size_t len = hex2bin(NAK, strlen(NAK), nak, sizeof(nak));
	if (len == 0) {
			status = MCU_API_ERROR;
	}

#ifdef PUBLIC_KEY_CAPABLE
    if (aes_data->key == SIGFOX_EP_KEY_PRIVATE) {
        aes_encrypt(aes_data->data, (sfx_u8 *)nak);
    } else if ((aes_data->key == SIGFOX_EP_KEY_PUBLIC)) {
        aes_encrypt(aes_data->data, (sfx_u8 *)SIGFOX_EP_PUBLIC_KEY);
    }
#else
    aes_encrypt(aes_data->data, (sfx_u8 *)nak);
#endif

	RETURN();
}

#ifdef CRC_HW
/*******************************************************************/
MCU_API_status_t MCU_API_compute_crc16(sfx_u8 *data, sfx_u8 data_size, sfx_u16 polynom, sfx_u16 *crc) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	RETURN();
}
#endif

#if (defined CRC_HW) && (defined BIDIRECTIONAL)
/*******************************************************************/
MCU_API_status_t MCU_API_compute_crc8(sfx_u8 *data, sfx_u8 data_size, sfx_u16 polynom, sfx_u8 *crc) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	RETURN();
}
#endif

/*******************************************************************/
MCU_API_status_t MCU_API_get_ep_id(sfx_u8 *ep_id, sfx_u8 ep_id_size_bytes) {
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif

	size_t len = hex2bin(EP_ID, strlen(EP_ID), ep_id, ep_id_size_bytes);
	if (len == 0) {
		status = MCU_API_ERROR;
	}

	RETURN();
}

/*******************************************************************/
MCU_API_status_t MCU_API_get_nvm(sfx_u8 *nvm_data, sfx_u8 nvm_data_size_bytes) {
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif

	memcpy(nvm_data, current_nvm_data, nvm_data_size_bytes);

	printk("get_nvm_data: %02x%02x%02x%02x\n", current_nvm_data[0], current_nvm_data[1], current_nvm_data[2], current_nvm_data[3]);

	RETURN();
}

/*******************************************************************/
MCU_API_status_t MCU_API_set_nvm(sfx_u8 *nvm_data, sfx_u8 nvm_data_size_bytes) {
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif

	int err;

	memcpy(current_nvm_data, nvm_data, sizeof(current_nvm_data));

	printk("set_nvm_data: %02x%02x%02x%02x\n", current_nvm_data[0], current_nvm_data[1], current_nvm_data[2], current_nvm_data[3]);

	err = settings_save_one("sigfox/nvm_data", nvm_data, nvm_data_size_bytes);
	if (err) {
		status = MCU_API_ERROR;
	}

	RETURN();
}

#if (defined CONTROL_KEEP_ALIVE_MESSAGE) || (defined BIDIRECTIONAL)
/*******************************************************************/
MCU_API_status_t MCU_API_get_voltage_temperature(sfx_u16 *voltage_idle_mv, sfx_u16 *voltage_tx_mv, sfx_s16 *temperature_tenth_degrees) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	RETURN();
}
#endif

#if (defined CERTIFICATION) && (defined BIDIRECTIONAL)
/*******************************************************************/
MCU_API_status_t MCU_API_print_dl_payload(sfx_u8 *dl_payload, sfx_u8 dl_payload_size, sfx_s16 rssi_dbm) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	RETURN();
}
#endif

#ifdef VERBOSE
/*******************************************************************/
MCU_API_status_t MCU_API_get_initial_pac(sfx_u8 *initial_pac, sfx_u8 initial_pac_size_bytes) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif

	size_t len = hex2bin(INITIAL_PAC, strlen(INITIAL_PAC), initial_pac, 
			initial_pac_size_bytes);
	if (len == 0) {
		status = MCU_API_ERROR;
	}

	RETURN();
}
#endif

#if (defined TIMER_REQUIRED) && (defined LATENCY_COMPENSATION) && (defined BIDIRECTIONAL)
/*******************************************************************/
MCU_API_status_t MCU_API_get_latency(MCU_API_latency_t latency_type, sfx_u32 *latency_ms) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	RETURN();
}
#endif

#ifdef VERBOSE
/*******************************************************************/
MCU_API_status_t MCU_API_get_version(sfx_u8 **version, sfx_u8 *version_size_char) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	RETURN();
}
#endif

#ifdef ERROR_CODES
/*******************************************************************/
void MCU_API_error(void) {
	/* To be implemented by the device manufacturer */
}
#endif
