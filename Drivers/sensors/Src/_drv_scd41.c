/*
 * _drv_scd41.c
 *
 *  Created on: Jun 5, 2025
 *      Author: admin
 */

#include "_drv_scd41.h"
#include "scd4x_i2c.h"
#include "sensirion_i2c_hal.h"

#include "smtc_hal_dbg_trace.h"

static int16_t error = NO_ERROR;

void DRV_SCD41_run(drv_SCD41_Context_t* context, drv_SCD41_results* results, uint32_t timer, drv_Drv_status* status) {

	SCD41_State_t next_state = context->driver_state;
	bool data_ready = false;

	switch (context->driver_state) {
		case S_START:
			*status = DRV_Wait;
			//sensirion_i2c_hal_init(context->i2c_handle);
			scd4x_init(SCD41_I2C_ADDR_62);
			error = scd4x_wake_up();
			error = scd4x_stop_periodic_measurement();
			error = scd4x_reinit();
			context->soft_timer_start = timer;
			next_state = S_AWAIT_START;
			SMTC_HAL_TRACE_INFO("Waking up sensor\n");
			if (error != NO_ERROR) {
				SMTC_HAL_TRACE_INFO("Fuck!\n");
			}
			break;

		case S_AWAIT_START:
			*status = DRV_Wait;
			if (timer - context->soft_timer_start >=20) {
				context->soft_timer_start = timer;
				error = scd4x_measure_single_shot();

				next_state = S_AWAIT_STABILIZE_MEASURE;
				SMTC_HAL_TRACE_INFO("Sensor awaken\n");
				if (error != NO_ERROR) {
					SMTC_HAL_TRACE_INFO("Fuck!\n");
				}
			}
			break;


		case S_AWAIT_STABILIZE_MEASURE:
			*status = DRV_Wait;
			error = scd4x_get_data_ready_status(&data_ready);
			if (timer - context->soft_timer_start >= 5000 || data_ready) {
				context->soft_timer_start = timer;
				error = scd4x_read_measurement(&results->co2_concentration, &results->temperature,
													 &results->relative_humidity);
				error = scd4x_measure_single_shot();
				next_state = S_AWAIT_DATA;
				data_ready = false;
				SMTC_HAL_TRACE_INFO("Got first measurement\n");
				if (error != NO_ERROR) {
					SMTC_HAL_TRACE_INFO("Fuck!\n");
				}
			}
			break;

		case S_AWAIT_DATA:
			error = scd4x_get_data_ready_status(&data_ready);
			if (timer - context->soft_timer_start >= 5000 || data_ready) {
				context->soft_timer_start = timer;
				error = scd4x_read_measurement(&results->co2_concentration, &results->temperature,
																	 &results->relative_humidity);
				error = scd4x_power_down();
				*status = DRV_DataReady;

				SMTC_HAL_TRACE_INFO("Got second measurement, going to sleep\n");
				if (error != NO_ERROR) {
					SMTC_HAL_TRACE_INFO("Fuck!\n");
				}

				SMTC_HAL_TRACE_INFO("CO2: %d ppm\n", results->co2_concentration);

				next_state = S_SLEEP;
			}
			break;

		case S_SLEEP:
			*status = DRV_Sleep;
			HAL_Delay(1);
			SMTC_HAL_TRACE_INFO("Till next: %d\n", 60*1000 -(timer - context->soft_timer_start));
			if (timer - context->soft_timer_start >= 60*1000) {
				HAL_Delay(1);

				next_state = S_START;
				context->soft_timer_start = timer;
			}

			break;

		default:
			break;
	}

	context->driver_state = next_state;

	return;
}

