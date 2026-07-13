// SPDX-License-Identifier: BSD-3-Clause

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <stddef.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>

#include "acc_pm.h"

#include "acc_algorithm.h"
#include "acc_definitions_a121.h"
#include "acc_detector_distance.h"
#include "acc_hal_definitions_a121.h"
#include "acc_hal_integration_a121.h"
#include "acc_integration.h"
#include "acc_integration_log.h"
#include "acc_rss_a121.h"
#include "acc_sensor.h"
#include "acc_version.h"

#include "acc_battery_info.h"
#include "acc_mcu_temperature.h"
#include "acc_bthome_ble.h"
#include "acc_serial_console_commands.h"
#include "acc_tank_config_ble.h"

#define SENSOR_ID         1U
#define SENSOR_TIMEOUT_MS 1000U
#define CLOSE_RANGE_START 0.07f

/* How often the radar takes a measurement. A tank only changes level while its pump
 * is running (briefly, e.g. once/day), so this can be far slower than a continuous
 * 1 Hz poll and still catch a fill/drain event within a minute of it starting.
 */
#define MEASURE_PERIOD_MS (60U * 1000U)

/* Radar sampling is cheap compared to a BLE broadcast, so it runs on the period
 * above regardless -- but a broadcast is only actually sent when the volume has
 * moved by at least this much (filters out measurement noise around a static level,
 * not real pump activity)...
 */
#define BROADCAST_VOLUME_THRESHOLD_ML (2000U) /* 2 L */

/* ...or at least this often regardless of change, so Home Assistant always sees a
 * reasonably fresh packet even while the level is genuinely static for a long time.
 */
#define BROADCAST_HEARTBEAT_MS (15U * 60U * 1000U) /* 15 min */

/* Not a libc/newlib-nano guarantee, so a local constant is used instead of M_PI. */
#define TANK_VOLUME_PI 3.14159265358979323846f

// Set this variable in order to achieve the lowest possible power consumpion in idle
// With this set no results will be printed on the UART

static const bool suspend_uart = false;

/* Broadcast-throttling state (see volume_broadcast_needed()). Zero-initialized, so the
 * very first measurement always broadcasts.
 */
static bool     have_broadcast_once;
static bool     last_broadcast_valid;
static uint32_t last_broadcast_volume_ml;
static int64_t  last_broadcast_uptime_ms;

// Settings for a small tank
#define SMALL_TANK_MEDIAN_FILTER_LENGTH             3U
#define SMALL_TANK_NUM_MEDIANS_TO_AVERAGE           2U
#define SMALL_TANK_RANGE_START_M                    0.03f
#define SMALL_TANK_RANGE_END_M                      0.5f
#define SMALL_TANK_MAX_STEP_LENGTH                  12U
#define SMALL_TANK_MAX_PROFILE                      ACC_CONFIG_PROFILE_3
#define SMALL_TANK_NUM_FRAMES_REC                   50U
#define SMALL_TANK_PEAK_SORTING                     ACC_DETECTOR_DISTANCE_PEAK_SORTING_CLOSEST
#define SMALL_TANK_REFLECTOR_SHAPE                  ACC_DETECTOR_DISTANCE_REFLECTOR_SHAPE_PLANAR
#define SMALL_TANK_THRESHOLD_METHOD                 ACC_DETECTOR_DISTANCE_THRESHOLD_METHOD_CFAR
#define SMALL_TANK_FIXED_AMPLITUDE_THRESHOLD        100.0f
#define SMALL_TANK_FIXED_STRENGTH_THRESHOLD         0.0f
#define SMALL_TANK_THRESHOLD_SENSITIVITY            0.0f
#define SMALL_TANK_SIGNAL_QUALITY                   3.0f
#define SMALL_TANK_CLOSE_RANGE_LEAKAGE_CANCELLATION false

// Settings for a medium tank
#define MEDIUM_TANK_MEDIAN_FILTER_LENGTH             3U
#define MEDIUM_TANK_NUM_MEDIANS_TO_AVERAGE           1U
#define MEDIUM_TANK_RANGE_START_M                    0.05f
#define MEDIUM_TANK_RANGE_END_M                      6.0f
#define MEDIUM_TANK_MAX_STEP_LENGTH                  0U // 0 means no limit
#define MEDIUM_TANK_MAX_PROFILE                      ACC_CONFIG_PROFILE_5
#define MEDIUM_TANK_NUM_FRAMES_REC                   50U
#define MEDIUM_TANK_PEAK_SORTING                     ACC_DETECTOR_DISTANCE_PEAK_SORTING_STRONGEST
#define MEDIUM_TANK_REFLECTOR_SHAPE                  ACC_DETECTOR_DISTANCE_REFLECTOR_SHAPE_PLANAR
#define MEDIUM_TANK_THRESHOLD_METHOD                 ACC_DETECTOR_DISTANCE_THRESHOLD_METHOD_CFAR
#define MEDIUM_TANK_FIXED_AMPLITUDE_THRESHOLD        100.0f
#define MEDIUM_TANK_FIXED_STRENGTH_THRESHOLD         3.0f
#define MEDIUM_TANK_THRESHOLD_SENSITIVITY            0.0f
#define MEDIUM_TANK_SIGNAL_QUALITY                   19.0f
#define MEDIUM_TANK_CLOSE_RANGE_LEAKAGE_CANCELLATION false

// Settings for a large tank
#define LARGE_TANK_MEDIAN_FILTER_LENGTH             1U
#define LARGE_TANK_NUM_MEDIANS_TO_AVERAGE           1U
#define LARGE_TANK_RANGE_START_M                    0.1f
#define LARGE_TANK_RANGE_END_M                      15.0f
#define LARGE_TANK_MAX_STEP_LENGTH                  0U // 0 means no limit
#define LARGE_TANK_MAX_PROFILE                      ACC_CONFIG_PROFILE_5
#define LARGE_TANK_NUM_FRAMES_REC                   50U
#define LARGE_TANK_PEAK_SORTING                     ACC_DETECTOR_DISTANCE_PEAK_SORTING_STRONGEST
#define LARGE_TANK_REFLECTOR_SHAPE                  ACC_DETECTOR_DISTANCE_REFLECTOR_SHAPE_PLANAR
#define LARGE_TANK_THRESHOLD_METHOD                 ACC_DETECTOR_DISTANCE_THRESHOLD_METHOD_CFAR
#define LARGE_TANK_FIXED_AMPLITUDE_THRESHOLD        100.0f
#define LARGE_TANK_FIXED_STRENGTH_THRESHOLD         5.0f
#define LARGE_TANK_THRESHOLD_SENSITIVITY            0.0f
#define LARGE_TANK_SIGNAL_QUALITY                   20.0f
#define LARGE_TANK_CLOSE_RANGE_LEAKAGE_CANCELLATION false

/** \example xm126_bthome_tank.c
 * @brief This is a reference application for measuring the volume of liquid in a
 * vertical cylindrical tank, publishing it as a BTHome v2 BLE broadcast (natively
 * understood by Home Assistant's Bluetooth integration), and supporting wireless BLE
 * firmware updates (mcumgr SMP) plus BLE-configurable tank geometry.
 * @n
 * This reference application executes as follows:
 *   - Retrieve HAL integration
 *   - Load tank geometry (diameter, sensor-to-floor offset), persisted or defaults
 *   - Initialize application resources:
 *     + Create application configuration
 *     + Create distance detector configuration
 *     + Update configuration settings
 *     + Create distance detector handle
 *     + Create buffer for detector calibration data
 *     + Create buffer for sensor data
 *   - Create and calibrate the sensor
 *   - Calibrate the detector
 *   - Start the BTHome broadcaster and the periodic BLE DFU/config window
 *   - Measure distances with the detector (loop):
 *     + Prepare sensor with the detector
 *     + Measure and wait until a read can be done
 *     + Process sensor measurement and get distance detector result
 *     + Process distance detector result, compute volume, and publish over BTHome
 *     + Handle "calibration_needed" indication
 *     + Reconfigure and recalibrate the detector if the tank geometry was changed
 *       over BLE
 *   - Cleanup:
 *     + Destroy detector configuration
 *     + Destroy detector handle
 *     + Destroy sensor data buffer
 *     + Destroy detector calibration data buffer
 */

typedef enum
{
	TANK_LEVEL_PRESET_CONFIG_NONE = 0,
	TANK_LEVEL_PRESET_CONFIG_SMALL_TANK,
	TANK_LEVEL_PRESET_CONFIG_MEDIUM_TANK,
	TANK_LEVEL_PRESET_CONFIG_LARGE_TANK,
} tank_level_preset_config_t;

/* Selects the detector's profile/threshold tuning (not just its range -- the BLE
 * tank-config service only reconfigures floor_offset_mm/diameter_mm, see
 * acc_tank_config_ble.h). Must match the real tank's rough depth or the detector may
 * never get a reliable in-range reading even with the correct floor offset configured
 * -- e.g. SMALL_TANK's close-range tuning pointed several meters out can end up stuck
 * at NO_DETECTION/OUT_OF_RANGE indefinitely, which silently suppresses the BTHome
 * distance/volume objects (see acc_bthome_ble_update()) since they're only sent when
 * valid.
 */
#define DEFAULT_PRESET_CONFIG TANK_LEVEL_PRESET_CONFIG_MEDIUM_TANK

/* Fallback tank geometry, used only until configured over BLE (see
 * acc_tank_config_ble.h) -- after that, the persisted values from flash win.
 * DEFAULT_FLOOR_OFFSET_MM matches DEFAULT_PRESET_CONFIG above; update it if you change
 * the preset. DEFAULT_TANK_DIAMETER_MM has no preset equivalent -- pick whatever is a
 * reasonable starting point for your deployment, it is not tied to the preset.
 */
#define DEFAULT_FLOOR_OFFSET_MM  ((uint32_t)(MEDIUM_TANK_RANGE_END_M * 1000.0f))
#define DEFAULT_TANK_DIAMETER_MM (1000U)

typedef struct
{
	float                           tank_range_start_m;
	float                           tank_range_end_m;
	uint16_t                        median_filter_length;
	uint16_t                        num_medians_to_average;
	acc_detector_distance_config_t *distance_config;
} acc_ref_app_tank_level_config_t;

typedef struct
{
	acc_ref_app_tank_level_config_t  *app_config;
	acc_sensor_t                     *sensor;
	float                            *level_history;
	uint16_t                          level_history_length;
	float                            *median_vector;
	uint16_t                          median_vector_length;
	uint16_t                          median_counter;
	uint16_t                          mean_counter;
	uint16_t                          median_edge_status_count;
	uint16_t                          mean_edge_status_count;
	acc_detector_distance_handle_t   *detector_handle;
	void                             *buffer;
	uint32_t                          buffer_size;
	uint8_t                          *detector_cal_result_static;
	uint32_t                          detector_cal_result_static_size;
	acc_detector_cal_result_dynamic_t detector_cal_result_dynamic;
} app_context_t;

typedef enum
{
	PEAK_STATUS_IN_RANGE,
	PEAK_STATUS_NO_DETECTION,
	PEAK_STATUS_OVERFLOW,
	PEAK_STATUS_OUT_OF_RANGE
} peak_status_t;

typedef struct
{
	bool          peak_detected;
	peak_status_t peak_status;
	float         level;
	bool          result_ready;
} app_result_t;

static void cleanup(app_context_t *context);

static float get_detector_start_m(acc_ref_app_tank_level_config_t *app_config);

static float get_detector_end_m(acc_ref_app_tank_level_config_t *app_config);

static void set_config(acc_ref_app_tank_level_config_t *app_config, tank_level_preset_config_t preset, float tank_range_end_m);

static bool volume_broadcast_needed(bool valid, uint32_t volume_ml);

static uint8_t battery_voltage_to_percent(float voltage);

static bool initialize_application_resources(app_context_t *context);

static bool sensor_calibration(acc_sensor_t *sensor, acc_cal_result_t *cal_result, void *buffer, uint32_t buffer_size);

static bool full_detector_calibration(app_context_t *context, const acc_cal_result_t *sensor_cal_result);

static bool update_detector_calibration(app_context_t *context, const acc_cal_result_t *sensor_cal_result);

static bool reconfigure_detector_for_new_geometry(app_context_t *context, acc_cal_result_t *sensor_cal_result);

static bool detector_get_next(app_context_t *context, const acc_cal_result_t *sensor_cal_result, acc_detector_distance_result_t *detector_result);

static float median(float *array, uint16_t array_length);

static float nanmean(float *array, uint16_t array_length);

static void process_detector_result(const acc_detector_distance_result_t *distance_result, app_result_t *app_result, app_context_t *context);

static void print_result(const app_result_t *app_result, const acc_ref_app_tank_level_config_t *app_config, uint32_t volume_milliliters);

int main(int argc, char *argv[]);

int main(int argc, char *argv[])
{
	(void)argc;
	(void)argv;
	app_context_t                   context    = {0};
	acc_ref_app_tank_level_config_t app_config = {0};
	context.app_config                         = &app_config;
	context.sensor                             = NULL;

	printf("Acconeer software version %s\n", acc_version_get());

	const acc_hal_a121_t *hal = acc_hal_rss_integration_get_implementation();

	if (!acc_rss_hal_register(hal))
	{
		return EXIT_FAILURE;
	}

	acc_tank_config_ble_init(DEFAULT_TANK_DIAMETER_MM, DEFAULT_FLOOR_OFFSET_MM);

	if (!acc_battery_info_init())
	{
		printf("acc_battery_info_init() failed, battery reporting will be unavailable\n");
	}

	context.app_config->distance_config = acc_detector_distance_config_create();
	if (context.app_config->distance_config == NULL)
	{
		printf("acc_detector_distance_config_create() failed\n");
		cleanup(&context);
		return EXIT_FAILURE;
	}

	set_config(&app_config, DEFAULT_PRESET_CONFIG, acc_tank_config_ble_get_floor_offset_mm() / 1000.0f);

	acc_integration_set_periodic_wakeup(MEASURE_PERIOD_MS);

	if (!initialize_application_resources(&context))
	{
		printf("Initializing detector context failed\n");
		cleanup(&context);
		return EXIT_FAILURE;
	}

	/* Turn the sensor on */
	acc_hal_integration_sensor_supply_on(SENSOR_ID);
	acc_hal_integration_sensor_enable(SENSOR_ID);

	context.sensor = acc_sensor_create(SENSOR_ID);
	if (context.sensor == NULL)
	{
		printf("acc_sensor_create() failed\n");
		cleanup(&context);
		return EXIT_FAILURE;
	}

	acc_cal_result_t cal_result;

	if (!sensor_calibration(context.sensor, &cal_result, context.buffer, context.buffer_size))
	{
		printf("Sensor calibration failed\n");
		cleanup(&context);
		return EXIT_FAILURE;
	}

	if (!full_detector_calibration(&context, &cal_result))
	{
		printf("Detector calibration failed\n");
		cleanup(&context);
		return EXIT_FAILURE;
	}

	if (suspend_uart)
	{
		const struct device *const uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
		if (acc_pm_suspend_uart(uart) != 0)
		{
			printf("Failed to suspend uart, continuing");
		}
	}
	else
	{
		/* Only useful while the console UART is actually up -- type "dfu" + Enter to
		 * reboot straight into MCUboot serial recovery (no DFU button needed), or
		 * "resetcfg" + Enter to clear a bad persisted tank configuration.
		 */
		acc_serial_console_commands_init();
	}

	acc_bthome_ble_init();

	while (true)
	{
		acc_detector_distance_result_t detector_result = {0};
		app_result_t                   app_result      = {0};

		if (!detector_get_next(&context, &cal_result, &detector_result))
		{
			printf("Could not get next result\n");
			cleanup(&context);
			return EXIT_FAILURE;
		}

		process_detector_result(&detector_result, &app_result, &context);

		/* If "calibration needed" is indicated, the sensor needs to be recalibrated and the detector calibration updated */
		if (detector_result.calibration_needed)
		{
			printf("Sensor recalibration and detector calibration update needed ... \n");

			if (!sensor_calibration(context.sensor, &cal_result, context.buffer, context.buffer_size))
			{
				printf("Sensor calibration failed\n");
				cleanup(&context);
				return EXIT_FAILURE;
			}

			/* Once the sensor is recalibrated, the detector calibration should be updated and measuring can continue. */
			if (!update_detector_calibration(&context, &cal_result))
			{
				printf("Detector calibration update failed\n");
				cleanup(&context);
				return EXIT_FAILURE;
			}

			printf("Sensor recalibration and detector calibration update done!\n");
		}

		if (app_result.result_ready)
		{
			acc_hal_integration_sensor_disable(SENSOR_ID);

			bool     valid       = (app_result.peak_status == PEAK_STATUS_IN_RANGE);
			uint16_t distance_mm = 0U;
			uint32_t volume_ml   = 0U;

			if (valid)
			{
				float diameter_m = (float)acc_tank_config_ble_get_diameter_mm() / 1000.0f;
				float area_m2     = TANK_VOLUME_PI * (diameter_m / 2.0f) * (diameter_m / 2.0f);

				volume_ml   = (uint32_t)(app_result.level * area_m2 * 1000000.0f); // m^3 -> mL
				distance_mm = (uint16_t)((context.app_config->tank_range_end_m - app_result.level) * 1000.0f);
			}

			float    battery_voltage_v    = 0.0f;
			uint8_t  battery_percent      = 0U;
			uint16_t battery_voltage_mv   = 0U;
			float    mcu_temperature_c    = 0.0f;
			int16_t  temperature_centi_c  = 0;

			if (acc_battery_info_sample_voltage(&battery_voltage_v, false))
			{
				battery_voltage_mv = (uint16_t)(battery_voltage_v * 1000.0f);
				battery_percent    = battery_voltage_to_percent(battery_voltage_v);
			}
			else
			{
				printf("Failed to sample battery voltage\n");
			}

			if (acc_mcu_temperature_read_die_temp(&mcu_temperature_c))
			{
				temperature_centi_c = (int16_t)(mcu_temperature_c * 100.0f);
			}
			else
			{
				printf("Failed to read MCU temperature\n");
			}

			if (!suspend_uart)
			{
				print_result(&app_result, context.app_config, volume_ml);
				printf("Battery: %" PRIfloat " V, %u %%\n", ACC_LOG_FLOAT_TO_INTEGER(battery_voltage_v), (unsigned int)battery_percent);
				printf("MCU temperature: %" PRIfloat " degC\n", ACC_LOG_FLOAT_TO_INTEGER(mcu_temperature_c));
			}

			if (volume_broadcast_needed(valid, volume_ml))
			{
				acc_bthome_ble_update(distance_mm, volume_ml, valid, battery_percent, battery_voltage_mv, temperature_centi_c);
			}

			if (acc_tank_config_ble_consume_dirty_flag())
			{
				printf("Tank configuration changed over BLE, reconfiguring detector ... \n");

				if (!reconfigure_detector_for_new_geometry(&context, &cal_result))
				{
					printf("Detector reconfiguration failed\n");
					cleanup(&context);
					return EXIT_FAILURE;
				}

				printf("Detector reconfiguration done!\n");
			}

			acc_integration_sleep_until_periodic_wakeup();

			acc_hal_integration_sensor_enable(SENSOR_ID);
		}
	}

	cleanup(&context);

	printf("Application finished OK\n");

	return EXIT_SUCCESS;
}

static void cleanup(app_context_t *context)
{
	acc_hal_integration_sensor_disable(SENSOR_ID);
	acc_hal_integration_sensor_supply_off(SENSOR_ID);

	acc_detector_distance_config_destroy(context->app_config->distance_config);
	acc_detector_distance_destroy(context->detector_handle);

	acc_integration_mem_free(context->buffer);
	acc_integration_mem_free(context->detector_cal_result_static);
	acc_integration_mem_free(context->level_history);
	acc_integration_mem_free(context->median_vector);

	if (context->sensor != NULL)
	{
		acc_sensor_destroy(context->sensor);
	}
}

static float get_detector_start_m(acc_ref_app_tank_level_config_t *app_config)
{
	// Decrease start point by 15 mm to make sure that start of the tank is fully covered
	return app_config->tank_range_start_m - 0.015f;
}

static float get_detector_end_m(acc_ref_app_tank_level_config_t *app_config)
{
	// Increase end point by 5% to make sure that the bottom of the tank is fully covered
	return fminf(app_config->tank_range_end_m * 1.05f, 23.0f);
}

static void set_config(acc_ref_app_tank_level_config_t *app_config, tank_level_preset_config_t preset, float tank_range_end_m)
{
	switch (preset)
	{
		case TANK_LEVEL_PRESET_CONFIG_NONE:
			break;
		case TANK_LEVEL_PRESET_CONFIG_SMALL_TANK:
			app_config->tank_range_start_m     = SMALL_TANK_RANGE_START_M;
			app_config->tank_range_end_m       = tank_range_end_m;
			app_config->median_filter_length   = SMALL_TANK_MEDIAN_FILTER_LENGTH;
			app_config->num_medians_to_average = SMALL_TANK_NUM_MEDIANS_TO_AVERAGE;

			acc_detector_distance_config_start_set(app_config->distance_config, get_detector_start_m(app_config));
			acc_detector_distance_config_end_set(app_config->distance_config, get_detector_end_m(app_config));
			acc_detector_distance_config_max_step_length_set(app_config->distance_config, SMALL_TANK_MAX_STEP_LENGTH);
			acc_detector_distance_config_max_profile_set(app_config->distance_config, SMALL_TANK_MAX_PROFILE);
			acc_detector_distance_config_num_frames_recorded_threshold_set(app_config->distance_config, SMALL_TANK_NUM_FRAMES_REC);
			acc_detector_distance_config_peak_sorting_set(app_config->distance_config, SMALL_TANK_PEAK_SORTING);
			acc_detector_distance_config_reflector_shape_set(app_config->distance_config, SMALL_TANK_REFLECTOR_SHAPE);
			acc_detector_distance_config_threshold_method_set(app_config->distance_config, SMALL_TANK_THRESHOLD_METHOD);
			acc_detector_distance_config_fixed_amplitude_threshold_value_set(app_config->distance_config, SMALL_TANK_FIXED_AMPLITUDE_THRESHOLD);
			acc_detector_distance_config_fixed_strength_threshold_value_set(app_config->distance_config, SMALL_TANK_FIXED_STRENGTH_THRESHOLD);
			acc_detector_distance_config_threshold_sensitivity_set(app_config->distance_config, SMALL_TANK_THRESHOLD_SENSITIVITY);
			acc_detector_distance_config_signal_quality_set(app_config->distance_config, SMALL_TANK_SIGNAL_QUALITY);
			acc_detector_distance_config_close_range_leakage_cancellation_set(app_config->distance_config,
			                                                                  SMALL_TANK_CLOSE_RANGE_LEAKAGE_CANCELLATION);
			break;
		case TANK_LEVEL_PRESET_CONFIG_MEDIUM_TANK:
			app_config->tank_range_start_m     = MEDIUM_TANK_RANGE_START_M;
			app_config->tank_range_end_m       = tank_range_end_m;
			app_config->median_filter_length   = MEDIUM_TANK_MEDIAN_FILTER_LENGTH;
			app_config->num_medians_to_average = MEDIUM_TANK_NUM_MEDIANS_TO_AVERAGE;

			acc_detector_distance_config_start_set(app_config->distance_config, get_detector_start_m(app_config));
			acc_detector_distance_config_end_set(app_config->distance_config, get_detector_end_m(app_config));
			acc_detector_distance_config_max_step_length_set(app_config->distance_config, MEDIUM_TANK_MAX_STEP_LENGTH);
			acc_detector_distance_config_max_profile_set(app_config->distance_config, MEDIUM_TANK_MAX_PROFILE);
			acc_detector_distance_config_num_frames_recorded_threshold_set(app_config->distance_config, MEDIUM_TANK_NUM_FRAMES_REC);
			acc_detector_distance_config_peak_sorting_set(app_config->distance_config, MEDIUM_TANK_PEAK_SORTING);
			acc_detector_distance_config_reflector_shape_set(app_config->distance_config, MEDIUM_TANK_REFLECTOR_SHAPE);
			acc_detector_distance_config_threshold_method_set(app_config->distance_config, MEDIUM_TANK_THRESHOLD_METHOD);
			acc_detector_distance_config_fixed_amplitude_threshold_value_set(app_config->distance_config, MEDIUM_TANK_FIXED_AMPLITUDE_THRESHOLD);
			acc_detector_distance_config_fixed_strength_threshold_value_set(app_config->distance_config, MEDIUM_TANK_FIXED_STRENGTH_THRESHOLD);
			acc_detector_distance_config_threshold_sensitivity_set(app_config->distance_config, MEDIUM_TANK_THRESHOLD_SENSITIVITY);
			acc_detector_distance_config_signal_quality_set(app_config->distance_config, MEDIUM_TANK_SIGNAL_QUALITY);
			acc_detector_distance_config_close_range_leakage_cancellation_set(app_config->distance_config,
			                                                                  MEDIUM_TANK_CLOSE_RANGE_LEAKAGE_CANCELLATION);
			break;
		case TANK_LEVEL_PRESET_CONFIG_LARGE_TANK:
			app_config->tank_range_start_m     = LARGE_TANK_RANGE_START_M;
			app_config->tank_range_end_m       = tank_range_end_m;
			app_config->median_filter_length   = LARGE_TANK_MEDIAN_FILTER_LENGTH;
			app_config->num_medians_to_average = LARGE_TANK_NUM_MEDIANS_TO_AVERAGE;

			acc_detector_distance_config_start_set(app_config->distance_config, get_detector_start_m(app_config));
			acc_detector_distance_config_end_set(app_config->distance_config, get_detector_end_m(app_config));
			acc_detector_distance_config_max_step_length_set(app_config->distance_config, LARGE_TANK_MAX_STEP_LENGTH);
			acc_detector_distance_config_max_profile_set(app_config->distance_config, LARGE_TANK_MAX_PROFILE);
			acc_detector_distance_config_num_frames_recorded_threshold_set(app_config->distance_config, LARGE_TANK_NUM_FRAMES_REC);
			acc_detector_distance_config_peak_sorting_set(app_config->distance_config, LARGE_TANK_PEAK_SORTING);
			acc_detector_distance_config_reflector_shape_set(app_config->distance_config, LARGE_TANK_REFLECTOR_SHAPE);
			acc_detector_distance_config_threshold_method_set(app_config->distance_config, LARGE_TANK_THRESHOLD_METHOD);
			acc_detector_distance_config_fixed_amplitude_threshold_value_set(app_config->distance_config, LARGE_TANK_FIXED_AMPLITUDE_THRESHOLD);
			acc_detector_distance_config_fixed_strength_threshold_value_set(app_config->distance_config, LARGE_TANK_FIXED_STRENGTH_THRESHOLD);
			acc_detector_distance_config_threshold_sensitivity_set(app_config->distance_config, LARGE_TANK_THRESHOLD_SENSITIVITY);
			acc_detector_distance_config_signal_quality_set(app_config->distance_config, LARGE_TANK_SIGNAL_QUALITY);
			acc_detector_distance_config_close_range_leakage_cancellation_set(app_config->distance_config,
			                                                                  LARGE_TANK_CLOSE_RANGE_LEAKAGE_CANCELLATION);
			break;
	}
}

static bool initialize_application_resources(app_context_t *context)
{
	context->detector_handle = acc_detector_distance_create(context->app_config->distance_config);
	if (context->detector_handle == NULL)
	{
		printf("acc_detector_distance_create() failed\n");
		return false;
	}

	if (!acc_detector_distance_get_sizes(context->detector_handle, &(context->buffer_size), &(context->detector_cal_result_static_size)))
	{
		printf("acc_detector_distance_get_sizes() failed\n");
		return false;
	}

	context->buffer = acc_integration_mem_alloc(context->buffer_size);
	if (context->buffer == NULL)
	{
		printf("sensor buffer allocation failed\n");
		return false;
	}

	context->detector_cal_result_static = acc_integration_mem_alloc(context->detector_cal_result_static_size);
	if (context->detector_cal_result_static == NULL)
	{
		printf("calibration buffer allocation failed\n");
		return false;
	}

	context->level_history_length = context->app_config->median_filter_length;

	context->level_history = acc_integration_mem_alloc(context->level_history_length * sizeof(*context->level_history));
	if (context->level_history == NULL)
	{
		printf("level history buffer allocation failed\n");
		return false;
	}

	context->median_vector_length = context->app_config->num_medians_to_average;

	context->median_vector = acc_integration_mem_alloc(context->median_vector_length * sizeof(*context->median_vector));
	if (context->median_vector == NULL)
	{
		printf("median vector allocation failed\n");
		return false;
	}

	context->median_counter           = 0U;
	context->mean_counter             = 0U;
	context->median_edge_status_count = 0U;
	context->mean_edge_status_count   = 0U;

	return true;
}

static bool sensor_calibration(acc_sensor_t *sensor, acc_cal_result_t *sensor_cal_result, void *buffer, uint32_t buffer_size)
{
	bool           status              = false;
	bool           cal_complete        = false;
	const uint16_t calibration_retries = 1U;

	// Random disturbances may cause the calibration to fail. At failure, retry at least once.
	for (uint16_t i = 0; !status && (i <= calibration_retries); i++)
	{
		// Reset sensor before calibration by disabling/enabling it
		acc_hal_integration_sensor_disable(SENSOR_ID);
		acc_hal_integration_sensor_enable(SENSOR_ID);

		do
		{
			status = acc_sensor_calibrate(sensor, &cal_complete, sensor_cal_result, buffer, buffer_size);

			if (status && !cal_complete)
			{
				status = acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS);
			}
		} while (status && !cal_complete);
	}

	if (status)
	{
		/* Reset sensor after calibration by disabling/enabling it */
		acc_hal_integration_sensor_disable(SENSOR_ID);
		acc_hal_integration_sensor_enable(SENSOR_ID);
	}
	else
	{
		printf("acc_sensor_calibrate() failed\n");
		acc_sensor_status(sensor);
	}

	return status;
}

static bool full_detector_calibration(app_context_t *context, const acc_cal_result_t *sensor_cal_result)
{
	bool done = false;
	bool status;

	do
	{
		status = acc_detector_distance_calibrate(context->sensor,
		                                         context->detector_handle,
		                                         sensor_cal_result,
		                                         context->buffer,
		                                         context->buffer_size,
		                                         context->detector_cal_result_static,
		                                         context->detector_cal_result_static_size,
		                                         &context->detector_cal_result_dynamic,
		                                         &done);
		if (done)
		{
			break;
		}

		if (status)
		{
			status = acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS);
		}
	} while (status);

	return status;
}

static bool update_detector_calibration(app_context_t *context, const acc_cal_result_t *sensor_cal_result)
{
	bool done = false;
	bool status;

	do
	{
		status = acc_detector_distance_update_calibration(context->sensor,
		                                                  context->detector_handle,
		                                                  sensor_cal_result,
		                                                  context->buffer,
		                                                  context->buffer_size,
		                                                  &context->detector_cal_result_dynamic,
		                                                  &done);
		if (done)
		{
			break;
		}

		if (status)
		{
			status = acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS);
		}
	} while (status);

	return status;
}

static bool reconfigure_detector_for_new_geometry(app_context_t *context, acc_cal_result_t *sensor_cal_result)
{
	float new_floor_offset_m = (float)acc_tank_config_ble_get_floor_offset_mm() / 1000.0f;

	acc_detector_distance_destroy(context->detector_handle);

	set_config(context->app_config, DEFAULT_PRESET_CONFIG, new_floor_offset_m);

	context->detector_handle = acc_detector_distance_create(context->app_config->distance_config);
	if (context->detector_handle == NULL)
	{
		printf("acc_detector_distance_create() failed during reconfiguration\n");
		return false;
	}

	uint32_t new_buffer_size;
	uint32_t new_cal_result_static_size;

	if (!acc_detector_distance_get_sizes(context->detector_handle, &new_buffer_size, &new_cal_result_static_size))
	{
		printf("acc_detector_distance_get_sizes() failed during reconfiguration\n");
		return false;
	}

	if (new_buffer_size != context->buffer_size)
	{
		acc_integration_mem_free(context->buffer);
		context->buffer_size = new_buffer_size;
		context->buffer      = acc_integration_mem_alloc(context->buffer_size);
		if (context->buffer == NULL)
		{
			printf("sensor buffer allocation failed during reconfiguration\n");
			return false;
		}
	}

	if (new_cal_result_static_size != context->detector_cal_result_static_size)
	{
		acc_integration_mem_free(context->detector_cal_result_static);
		context->detector_cal_result_static_size = new_cal_result_static_size;
		context->detector_cal_result_static      = acc_integration_mem_alloc(context->detector_cal_result_static_size);
		if (context->detector_cal_result_static == NULL)
		{
			printf("calibration buffer allocation failed during reconfiguration\n");
			return false;
		}
	}

	return full_detector_calibration(context, sensor_cal_result);
}

static bool detector_get_next(app_context_t *context, const acc_cal_result_t *sensor_cal_result, acc_detector_distance_result_t *detector_result)
{
	bool result_available = false;

	do
	{
		if (!acc_detector_distance_prepare(context->detector_handle,
		                                   context->app_config->distance_config,
		                                   context->sensor,
		                                   sensor_cal_result,
		                                   context->buffer,
		                                   context->buffer_size))
		{
			printf("acc_detector_distance_prepare() failed\n");
			return false;
		}

		if (!acc_sensor_measure(context->sensor))
		{
			printf("acc_sensor_measure() failed\n");
			return false;
		}

		if (!acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS))
		{
			printf("Sensor interrupt timeout\n");
			return false;
		}

		if (!acc_sensor_read(context->sensor, context->buffer, context->buffer_size))
		{
			printf("acc_sensor_read() failed\n");
			return false;
		}

		if (!acc_detector_distance_process(context->detector_handle,
		                                   context->buffer,
		                                   context->detector_cal_result_static,
		                                   &context->detector_cal_result_dynamic,
		                                   &result_available,
		                                   detector_result))
		{
			printf("acc_detector_distance_process() failed\n");
			return false;
		}
	} while (!result_available);

	return true;
}

static float median(float *array, uint16_t array_length)
{
	bool  status = true;
	float result = 0.0f;

	for (uint16_t i = 0U; i < array_length && status; i++)
	{
		if (isnan(array[i]))
		{
			result = (float)NAN;
			status = false;
		}
	}

	if (status)
	{
		result = acc_algorithm_median_f32(array, array_length);
	}

	return result;
}

static float nanmean(float *array, uint16_t array_length)
{
	uint16_t samples = 0U;
	float    sum     = 0.0f;

	for (uint16_t i = 0U; i < array_length; i++)
	{
		if (!isnan(array[i]))
		{
			samples++;
			sum += array[i];
		}
	}

	return samples > 0U ? sum / (float)samples : (float)NAN;
}

static void process_detector_result(const acc_detector_distance_result_t *distance_result, app_result_t *app_result, app_context_t *context)
{
	app_result->peak_detected = false;
	app_result->peak_status   = PEAK_STATUS_NO_DETECTION;
	app_result->level         = 0.0f;
	app_result->result_ready  = false;
	float level               = 0.0f;

	if (distance_result->num_distances > 0)
	{
		app_result->peak_detected = true;
		level                     = context->app_config->tank_range_end_m - distance_result->distances[0];
	}
	else
	{
		level = (float)NAN;
	}

	if (distance_result->near_start_edge_status)
	{
		context->median_edge_status_count++;
	}

	context->level_history[context->median_counter++] = level;

	if (context->median_counter == context->level_history_length)
	{
		float med = median(context->level_history, context->level_history_length);

		context->median_vector[context->mean_counter++] = med;

		if (context->median_edge_status_count > context->level_history_length / 2)
		{
			context->mean_edge_status_count++;
		}

		context->median_counter           = 0U;
		context->median_edge_status_count = 0U;
	}

	if (context->mean_counter == context->median_vector_length)
	{
		level = nanmean(context->median_vector, context->median_vector_length);

		if (!isnan(level))
		{
			if (level < 0.0f)
			{
				app_result->peak_status = PEAK_STATUS_OUT_OF_RANGE;
			}
			else if ((level > (context->app_config->tank_range_end_m - context->app_config->tank_range_start_m)) &&
			         (context->app_config->tank_range_start_m >= CLOSE_RANGE_START))
			{
				app_result->peak_status = PEAK_STATUS_OVERFLOW;
			}
			else
			{
				app_result->peak_status = PEAK_STATUS_IN_RANGE;
				app_result->level       = level;
			}
		}
		else if ((context->mean_edge_status_count > context->median_vector_length / 2) &&
		         (context->app_config->tank_range_start_m >= CLOSE_RANGE_START))
		{
			app_result->peak_status = PEAK_STATUS_OVERFLOW;
		}

		if (app_result->peak_status == PEAK_STATUS_OVERFLOW || app_result->peak_status == PEAK_STATUS_OUT_OF_RANGE)
		{
			app_result->level = (float)NAN;
		}

		app_result->result_ready        = true;
		context->mean_counter           = 0U;
		context->mean_edge_status_count = 0U;
	}
}

static void print_result(const app_result_t *app_result, const acc_ref_app_tank_level_config_t *app_config, uint32_t volume_milliliters)
{
	switch (app_result->peak_status)
	{
		case PEAK_STATUS_IN_RANGE:
			printf("Level within range\n");
			printf("Level: %" PRIfloat " cm, %" PRIfloat " %%, Volume: %u mL\n",
			       ACC_LOG_FLOAT_TO_INTEGER(app_result->level * 100.0f),
			       ACC_LOG_FLOAT_TO_INTEGER((app_result->level / (app_config->tank_range_end_m - app_config->tank_range_start_m) * 100.f)),
			       (unsigned int)volume_milliliters);
			break;
		case PEAK_STATUS_NO_DETECTION:
			printf("No detection\n");
			break;
		case PEAK_STATUS_OVERFLOW:
			printf("Overflow!\n");
			break;
		case PEAK_STATUS_OUT_OF_RANGE:
			printf("Out of range\n");
			break;
		default:
			break;
	}
}

/* Decides whether the current measurement is worth a BLE broadcast, and if so,
 * updates the "last broadcast" bookkeeping used for the next decision. Keeping radar
 * sampling on MEASURE_PERIOD_MS but broadcasting only on real change (or the
 * heartbeat) is what actually saves power -- a BLE TX is far more expensive than a
 * radar sample sitting idle in between.
 */
static bool volume_broadcast_needed(bool valid, uint32_t volume_ml)
{
	int64_t  now_ms       = k_uptime_get();
	uint32_t volume_delta = (volume_ml > last_broadcast_volume_ml) ? (volume_ml - last_broadcast_volume_ml)
	                                                                : (last_broadcast_volume_ml - volume_ml);

	bool should_send = !have_broadcast_once || (valid != last_broadcast_valid) ||
	                    (valid && (volume_delta >= BROADCAST_VOLUME_THRESHOLD_ML)) ||
	                    ((now_ms - last_broadcast_uptime_ms) >= BROADCAST_HEARTBEAT_MS);

	if (should_send)
	{
		have_broadcast_once      = true;
		last_broadcast_valid     = valid;
		last_broadcast_volume_ml = volume_ml;
		last_broadcast_uptime_ms = now_ms;
	}

	return should_send;
}

struct battery_curve_point
{
	float   voltage;
	uint8_t percent;
};

/* Approximate 1S (single-cell) LiPo open-circuit discharge curve, highest voltage
 * first. LiPo cells sit fairly flat around 3.7-4.0 V for most of their capacity, then
 * drop off steeply near empty -- a straight-line 2-point mapping (the previous
 * placeholder, meant for a generic 2.0-3.0V source) would read very inaccurately here.
 * Values are widely-cited approximations for standard LiPo chemistry (4.2V full,
 * ~3.7V nominal); actual voltage sags somewhat under load and shifts slightly with
 * temperature/cell age, so treat the percentage as indicative, not precise -- the raw
 * voltage is always published alongside it (BTHome object 0x0C) for anyone who wants a
 * more exact curve downstream instead.
 */
static const struct battery_curve_point lipo_1s_curve[] = {
    {4.20f, 100U}, {4.15f, 95U}, {4.11f, 90U}, {4.08f, 85U}, {4.02f, 80U}, {3.98f, 75U}, {3.95f, 70U}, {3.91f, 65U},
    {3.87f, 60U},  {3.85f, 55U}, {3.84f, 50U}, {3.82f, 45U}, {3.80f, 40U}, {3.79f, 35U}, {3.77f, 30U}, {3.75f, 25U},
    {3.73f, 20U},  {3.71f, 15U}, {3.69f, 10U}, {3.61f, 5U},  {3.27f, 0U},
};

static uint8_t battery_voltage_to_percent(float voltage)
{
	size_t count = ARRAY_SIZE(lipo_1s_curve);

	if (voltage >= lipo_1s_curve[0].voltage)
	{
		return 100U;
	}

	if (voltage <= lipo_1s_curve[count - 1].voltage)
	{
		return 0U;
	}

	for (size_t i = 0; i + 1 < count; i++)
	{
		const struct battery_curve_point *hi = &lipo_1s_curve[i];
		const struct battery_curve_point *lo = &lipo_1s_curve[i + 1];

		if (voltage <= hi->voltage && voltage >= lo->voltage)
		{
			float fraction = (voltage - lo->voltage) / (hi->voltage - lo->voltage);

			return (uint8_t)((float)lo->percent + fraction * (float)(hi->percent - lo->percent));
		}
	}

	return 0U; /* unreachable: bounds already checked above */
}
