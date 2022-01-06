/*
 * Copyright (c) 2021 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr.h>
#include <logging/log_ctrl.h>
#include <logging/log.h>

#include <tfm_veneers.h>
#include <tfm_ns_interface.h>
#include "tfm_tflm_service_api.h"

#include "util_app_log.h"

#include <math.h>

/** Declare a reference to the application logging interface. */
LOG_MODULE_DECLARE(app, CONFIG_LOG_DEFAULT_LEVEL);

void main(void)
{
	psa_status_t status;

	const float PI = 3.14159265359f;
	float deg = PI / 180.0;

	float x_value, y_value;

	/* Initialise the logger subsys and dump the current buffer. */
	log_init();

	for (int i = 0; i <= 360; i++) {

		x_value = (float)i * deg;
		status = al_psa_status(
			psa_secure_inference_tflm_hello(&x_value,
							sizeof(x_value),
							&y_value,
							sizeof(y_value)),
			__func__);

		if (status != PSA_SUCCESS) {
			LOG_ERR("Failed to get sine value using secure inference");
			goto err;
		}

		printf("Model: Sine of %d deg is: %f\t", i, y_value);
		printf("C Mathlib: Sine of %d deg is: %f\t", i, sin(x_value));
		printf("Deviation: %f\n", fabs(sin(x_value) - y_value));
		al_dump_log();

		k_msleep(500);
	}

err:
	/* Dump any queued log messages, and wait for system events. */
	al_dump_log();
}
