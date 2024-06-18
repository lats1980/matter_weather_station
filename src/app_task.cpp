/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <ei_wrapper.h>

#include "app_task.h"

#include "battery.h"
#include "buzzer.h"

#include "app/matter_init.h"
#include "app/task_executor.h"
#include "board/board.h"
#include "board/led_widget.h"

#ifdef CONFIG_CHIP_OTA_REQUESTOR
#include "dfu/ota/ota_util.h"
#endif

#ifdef CONFIG_MCUMGR_TRANSPORT_BT
#include "dfu/smp/dfu_over_smp.h"
#endif

#include <app-common/zap-generated/attributes/Accessors.h>
#include <app-common/zap-generated/cluster-objects.h>
#include <app/server/OnboardingCodesUtil.h>
#include <app/server/Server.h>

#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(app, CONFIG_CHIP_APP_LOG_LEVEL);

using namespace ::chip;
using namespace ::chip::DeviceLayer;
using namespace ::chip::app;

namespace
{
enum class FunctionTimerMode { kDisabled, kFactoryResetTrigger, kFactoryResetComplete };
enum class LedState { kAlive, kAdvertisingBle, kConnectedBle, kProvisioned };

#if CONFIG_AVERAGE_CURRENT_CONSUMPTION <= 0
#error Invalid CONFIG_AVERAGE_CURRENT_CONSUMPTION value set
#endif

#define SHIFT_WINDOWS		0
#define SHIFT_FRAMES		100

constexpr size_t kMeasurementsIntervalMs = 3000;
constexpr uint8_t kTemperatureMeasurementEndpointId = 1;
constexpr int16_t kTemperatureMeasurementAttributeMaxValue = 0x7fff;
constexpr int16_t kTemperatureMeasurementAttributeMinValue = 0x954d;
constexpr int16_t kTemperatureMeasurementAttributeInvalidValue = 0x8000;
constexpr uint8_t kHumidityMeasurementEndpointId = 2;
constexpr uint16_t kHumidityMeasurementAttributeMaxValue = 0x2710;
constexpr uint16_t kHumidityMeasurementAttributeMinValue = 0;
constexpr uint16_t kHumidityMeasurementAttributeInvalidValue = 0xffff;
constexpr uint8_t kPressureMeasurementEndpointId = 3;
constexpr int16_t kPressureMeasurementAttributeMaxValue = 0x7fff;
constexpr int16_t kPressureMeasurementAttributeMinValue = 0x8001;
constexpr int16_t kPressureMeasurementAttributeInvalidValue = 0x8000;
constexpr uint8_t kPowerSourceEndpointId = 0;
constexpr int16_t kMinimalOperatingVoltageMv = 3200;
constexpr int16_t kMaximalOperatingVoltageMv = 4050;
constexpr int16_t kWarningThresholdVoltageMv = 3450;
constexpr int16_t kCriticalThresholdVoltageMv = 3250;
constexpr uint8_t kMinBatteryPercentage = 0;
/* Value is expressed in half percent units ranging from 0 to 200. */
constexpr uint8_t kMaxBatteryPercentage = 200;
/* Battery capacity in uAh */
constexpr uint32_t kBatteryCapacityUaH = 1350000;
/* Average device current consumption in uA */
constexpr uint32_t kDeviceAverageCurrentConsumptionUa = CONFIG_AVERAGE_CURRENT_CONSUMPTION;
/* Fully charged battery operation time in seconds */
constexpr uint32_t kFullBatteryOperationTime = kBatteryCapacityUaH / kDeviceAverageCurrentConsumptionUa * 3600;
/* It is recommended to toggle the signalled state with 0.5 s interval. */
constexpr size_t kIdentifyTimerIntervalMs = 500;
/* Number of accelerometer channels. */
constexpr uint8_t kAccelerometerChannels = 3;
/* Accelerometer sample interval in ms. */
constexpr size_t kAccMeasurementsIntervalMs = 10;
/* Label of Normal. */
constexpr char kNormalLabel[] = "Normal";
/* Label of Unbalance. */
constexpr char kUnbalanceLabel[] = "Unbala";

k_timer sMeasurementsTimer;
k_timer sIdentifyTimer;
k_timer sAccMeasurementsTimer;

enum {
	ML_DROP_RESULT			= BIT(0),
	ML_CLEANUP_REQUIRED		= BIT(1),
	ML_FIRST_PREDICTION		= BIT(2),
	ML_RUNNING			= BIT(3),
};
uint8_t ml_control;

const device *sBme688SensorDev = DEVICE_DT_GET_ONE(bosch_bme680);
const device *sAdxl362SensorDev = DEVICE_DT_GET_ONE(adi_adxl362);

/* Add identify for all endpoints */
Identify sIdentifyTemperature = { chip::EndpointId{ kTemperatureMeasurementEndpointId }, AppTask::OnIdentifyStart,
				  AppTask::OnIdentifyStop, Clusters::Identify::IdentifyTypeEnum::kAudibleBeep };
Identify sIdentifyHumidity = { chip::EndpointId{ kHumidityMeasurementEndpointId }, AppTask::OnIdentifyStart,
			       AppTask::OnIdentifyStop, Clusters::Identify::IdentifyTypeEnum::kAudibleBeep };
Identify sIdentifyPressure = { chip::EndpointId{ kPressureMeasurementEndpointId }, AppTask::OnIdentifyStart,
			       AppTask::OnIdentifyStop, Clusters::Identify::IdentifyTypeEnum::kAudibleBeep };

} /* namespace */

static int buf_cleanup(void)
{
	bool cancelled = false;
	int err = ei_wrapper_clear_data(&cancelled);

	if (!err) {
		if (cancelled) {
			ml_control &= ~ML_RUNNING;
		}
		if (ml_control & ML_RUNNING) {
			ml_control |= ML_DROP_RESULT;
		}
		ml_control &= ~ML_CLEANUP_REQUIRED;
		ml_control |= ML_FIRST_PREDICTION;
	} else if (err == -EBUSY) {
		LOG_ERR("Cannot cleanup buffer (err: %d)", err);
		ml_control |= ML_DROP_RESULT;
		ml_control |= ML_CLEANUP_REQUIRED;
	} else {
		LOG_ERR("Cannot cleanup buffer (err: %d)", err);
	}

	return err;
}

static void start_prediction(void)
{
	int err;
	size_t window_shift;
	size_t frame_shift;

	if (ml_control & ML_RUNNING) {
		return;
	}

	if (ml_control & ML_CLEANUP_REQUIRED) {
		err = buf_cleanup();
		if (err) {
			return;
		}
	}

	if (ml_control & ML_FIRST_PREDICTION) {
		window_shift = 0;
		frame_shift = 0;
	} else {
		window_shift = SHIFT_WINDOWS;
		frame_shift = SHIFT_FRAMES;
	}

	err = ei_wrapper_start_prediction(window_shift, frame_shift);
	if (!err) {
		ml_control |= ML_RUNNING;
		ml_control &= ~ML_FIRST_PREDICTION;
	} else {
		LOG_ERR("Cannot start prediction (err: %d)", err);
	}
}

void AppTask::ResultReadyHandler(int err)
{
	bool drop_result = (err) || (ml_control & ML_DROP_RESULT);

	if (err) {
		LOG_ERR("Result ready callback returned error (err: %d)", err);
	} else {
		ml_control &= ~ML_DROP_RESULT;
		ml_control &= ~ML_RUNNING;
		start_prediction();
	}

	if (!drop_result) {
		int ret;
		const char *label;
		float value;
		size_t idx;
		float anomaly;

		ret = ei_wrapper_get_next_classification_result(&label, &value, &idx);
		if (!ret) {
			if (!label) {
				LOG_ERR("Returned label is NULL");
				return;
			}
			LOG_ERR("%s, %f", label, value);
			if (strncmp(label, kNormalLabel, strlen(kNormalLabel)) == 0) {
				LOG_INF("Normal");
			} else if (strncmp(label, kUnbalanceLabel, strlen(kUnbalanceLabel)) == 0) {
				LOG_INF("Unbalance");
			} else {
				LOG_INF("Ignore result");
			}
		} else {
			ret = ei_wrapper_get_anomaly(&anomaly);
			if (!ret) {
				LOG_ERR("anomaly:%f", anomaly);
			} else {
				LOG_ERR("Fail to retrieve anomaly");
			}
		}
	} else {
		LOG_ERR("drop_result");
	}
}

void AppTask::MeasurementsTimerHandler()
{
	Instance().UpdateClustersState();
}

void AppTask::AccMeasurementsTimerHandler()
{
	struct sensor_value data[kAccelerometerChannels];
	int ret;
	float float_data[kAccelerometerChannels];

	ret = sensor_sample_fetch(sAdxl362SensorDev);
	if (ret != 0) {
		LOG_ERR("Fetching data from ADXL362 sensor failed with: %d", ret);
	} else {
		ret = sensor_channel_get(sAdxl362SensorDev, SENSOR_CHAN_ACCEL_XYZ, &data[0]);
		if (ret) {
			LOG_ERR("sensor_channel_get, error: %d", ret);
		} else {
			for (size_t i = 0; i < kAccelerometerChannels; i++) {
				float_data[i] = sensor_value_to_double(&data[i]);
			}
			ret = ei_wrapper_add_data(float_data, kAccelerometerChannels);
			if (ret) {
				LOG_ERR("Cannot add data for EI wrapper (err %d)", ret);
			}
		}
	}
}

void AppTask::OnIdentifyStart(Identify *)
{
	Nrf::PostTask(
		[] { Nrf::GetBoard().GetLED(Nrf::DeviceLeds::LED2).Blink(Nrf::LedConsts::kIdentifyBlinkRate_ms); });
	k_timer_start(&sIdentifyTimer, K_MSEC(kIdentifyTimerIntervalMs), K_MSEC(kIdentifyTimerIntervalMs));
}

void AppTask::OnIdentifyStop(Identify *)
{
	k_timer_stop(&sIdentifyTimer);
	BuzzerSetState(false);
}

void AppTask::IdentifyTimerHandler()
{
	BuzzerToggleState();
}

void AppTask::UpdateTemperatureClusterState()
{
	struct sensor_value sTemperature;
	EmberAfStatus status;
	int result = sensor_channel_get(sBme688SensorDev, SENSOR_CHAN_AMBIENT_TEMP, &sTemperature);
	if (result == 0) {
		/* Defined by cluster temperature measured value = 100 x temperature in degC with resolution of
		 * 0.01 degC. val1 is an integer part of the value and val2 is fractional part in one-millionth
		 * parts. To achieve resolution of 0.01 degC val2 needs to be divided by 10000. */
		int16_t newValue = static_cast<int16_t>(sTemperature.val1 * 100 + sTemperature.val2 / 10000);

		if (newValue > kTemperatureMeasurementAttributeMaxValue ||
		    newValue < kTemperatureMeasurementAttributeMinValue) {
			/* Read value exceeds permitted limits, so assign invalid value code to it. */
			newValue = kTemperatureMeasurementAttributeInvalidValue;
		}

		status = Clusters::TemperatureMeasurement::Attributes::MeasuredValue::Set(
			kTemperatureMeasurementEndpointId, newValue);
		if (status != EMBER_ZCL_STATUS_SUCCESS) {
			LOG_ERR("Updating temperature measurement %x", status);
		}
	} else {
		LOG_ERR("Getting temperature measurement data from BME688 failed with: %d", result);
	}
}

void AppTask::UpdatePressureClusterState()
{
	struct sensor_value sPressure;
	EmberAfStatus status;
	int result = sensor_channel_get(sBme688SensorDev, SENSOR_CHAN_PRESS, &sPressure);
	if (result == 0) {
		/* Defined by cluster pressure measured value = 10 x pressure in kPa with resolution of 0.1 kPa.
		 * val1 is an integer part of the value and val2 is fractional part in one-millionth parts.
		 * To achieve resolution of 0.1 kPa val2 needs to be divided by 100000. */
		int16_t newValue = static_cast<int16_t>(sPressure.val1 * 10 + sPressure.val2 / 100000);

		if (newValue > kPressureMeasurementAttributeMaxValue ||
		    newValue < kPressureMeasurementAttributeMinValue) {
			/* Read value exceeds permitted limits, so assign invalid value code to it. */
			newValue = kPressureMeasurementAttributeInvalidValue;
		}

		status = Clusters::PressureMeasurement::Attributes::MeasuredValue::Set(kPressureMeasurementEndpointId,
										       newValue);
		if (status != EMBER_ZCL_STATUS_SUCCESS) {
			LOG_ERR("Updating pressure measurement %x", status);
		}
	} else {
		LOG_ERR("Getting pressure measurement data from BME688 failed with: %d", result);
	}
}

void AppTask::UpdateRelativeHumidityClusterState()
{
	struct sensor_value sHumidity;
	EmberAfStatus status;
	int result = sensor_channel_get(sBme688SensorDev, SENSOR_CHAN_HUMIDITY, &sHumidity);
	if (result == 0) {
		/* Defined by cluster humidity measured value = 100 x humidity in %RH with resolution of 0.01 %.
		 * val1 is an integer part of the value and val2 is fractional part in one-millionth parts.
		 * To achieve resolution of 0.01 % val2 needs to be divided by 10000. */
		uint16_t newValue = static_cast<int16_t>(sHumidity.val1 * 100 + sHumidity.val2 / 10000);

		if (newValue > kHumidityMeasurementAttributeMaxValue ||
		    newValue < kHumidityMeasurementAttributeMinValue) {
			/* Read value exceeds permitted limits, so assign invalid value code to it. */
			newValue = kHumidityMeasurementAttributeInvalidValue;
		}

		status = Clusters::RelativeHumidityMeasurement::Attributes::MeasuredValue::Set(
			kHumidityMeasurementEndpointId, newValue);
		if (status != EMBER_ZCL_STATUS_SUCCESS) {
			LOG_ERR("Updating relative humidity measurement %x", status);
		}
	} else {
		LOG_ERR("Getting humidity measurement data from BME688 failed with: %d", result);
	}
}

void AppTask::UpdatePowerSourceClusterState()
{
	EmberAfStatus status;
	int32_t voltage = BatteryMeasurementReadVoltageMv();
	/* Value is expressed in half percent units ranging from 0 to 200. */
	uint8_t batteryPercentage;
	uint32_t batteryTimeRemaining;
	Clusters::PowerSource::PowerSourceStatusEnum batteryStatus;
	Clusters::PowerSource::BatChargeLevelEnum batteryChargeLevel;
	bool batteryPresent;
	Clusters::PowerSource::BatChargeStateEnum batteryCharged;

	if (voltage < 0) {
		voltage = 0;
		batteryPercentage = 0;
		batteryStatus = Clusters::PowerSource::PowerSourceStatusEnum::kUnavailable;
		batteryPresent = false;

		LOG_ERR("Battery level measurement failed %d", voltage);
	} else {
		batteryStatus = Clusters::PowerSource::PowerSourceStatusEnum::kActive;
		batteryPresent = true;
	}

	if (voltage <= kMinimalOperatingVoltageMv) {
		batteryPercentage = kMinBatteryPercentage;
	} else if (voltage >= kMaximalOperatingVoltageMv) {
		batteryPercentage = kMaxBatteryPercentage;
	} else {
		batteryPercentage = kMaxBatteryPercentage * (voltage - kMinimalOperatingVoltageMv) /
				    (kMaximalOperatingVoltageMv - kMinimalOperatingVoltageMv);
	}

	batteryTimeRemaining = kFullBatteryOperationTime * batteryPercentage / kMaxBatteryPercentage;

	if (voltage < kCriticalThresholdVoltageMv) {
		batteryChargeLevel = Clusters::PowerSource::BatChargeLevelEnum::kCritical;
	} else if (voltage < kWarningThresholdVoltageMv) {
		batteryChargeLevel = Clusters::PowerSource::BatChargeLevelEnum::kWarning;
	} else {
		batteryChargeLevel = Clusters::PowerSource::BatChargeLevelEnum::kOk;
	}

	if (BatteryCharged()) {
		batteryCharged = Clusters::PowerSource::BatChargeStateEnum::kIsCharging;
	} else {
		batteryCharged = Clusters::PowerSource::BatChargeStateEnum::kIsNotCharging;
	}

	status = Clusters::PowerSource::Attributes::BatVoltage::Set(kPowerSourceEndpointId, voltage);
	if (status != EMBER_ZCL_STATUS_SUCCESS) {
		LOG_ERR("Updating battery voltage failed %x", status);
	}

	status = Clusters::PowerSource::Attributes::BatPercentRemaining::Set(kPowerSourceEndpointId, batteryPercentage);
	if (status != EMBER_ZCL_STATUS_SUCCESS) {
		LOG_ERR("Updating battery percentage failed %x", status);
	}

	status = Clusters::PowerSource::Attributes::BatTimeRemaining::Set(kPowerSourceEndpointId, batteryTimeRemaining);
	if (status != EMBER_ZCL_STATUS_SUCCESS) {
		LOG_ERR("Updating battery time remaining failed %x", status);
	}

	status = Clusters::PowerSource::Attributes::BatChargeLevel::Set(kPowerSourceEndpointId, batteryChargeLevel);
	if (status != EMBER_ZCL_STATUS_SUCCESS) {
		LOG_ERR("Updating battery charge level failed %x", status);
	}

	status = Clusters::PowerSource::Attributes::Status::Set(kPowerSourceEndpointId, batteryStatus);
	if (status != EMBER_ZCL_STATUS_SUCCESS) {
		LOG_ERR("Updating battery status failed %x", status);
	}

	status = Clusters::PowerSource::Attributes::BatPresent::Set(kPowerSourceEndpointId, batteryPresent);
	if (status != EMBER_ZCL_STATUS_SUCCESS) {
		LOG_ERR("Updating battery present failed %x", status);
	}

	status = Clusters::PowerSource::Attributes::BatChargeState::Set(kPowerSourceEndpointId, batteryCharged);
	if (status != EMBER_ZCL_STATUS_SUCCESS) {
		LOG_ERR("Updating battery charge failed %x", status);
	}
}

void AppTask::UpdateClustersState()
{
	const int result = sensor_sample_fetch(sBme688SensorDev);

	if (result == 0) {
		UpdateTemperatureClusterState();
		UpdatePressureClusterState();
		UpdateRelativeHumidityClusterState();
	} else {
		LOG_ERR("Fetching data from BME688 sensor failed with: %d", result);
	}

	UpdatePowerSourceClusterState();
}

void AppTask::UpdateLedState()
{
	if (!Instance().mGreenLED || !Instance().mBlueLED || !Instance().mRedLED) {
		return;
	}

	Instance().mGreenLED->Set(false);
	Instance().mBlueLED->Set(false);
	Instance().mRedLED->Set(false);

	switch (Nrf::GetBoard().GetDeviceState()) {
	case Nrf::DeviceState::DeviceAdvertisingBLE:
		Instance().mBlueLED->Blink(Nrf::LedConsts::StatusLed::Disconnected::kOn_ms,
					   Nrf::LedConsts::StatusLed::Disconnected::kOff_ms);
		break;
	case Nrf::DeviceState::DeviceDisconnected:
		Instance().mGreenLED->Blink(Nrf::LedConsts::StatusLed::Disconnected::kOn_ms,
					    Nrf::LedConsts::StatusLed::Disconnected::kOff_ms);
		break;
	case Nrf::DeviceState::DeviceConnectedBLE:
		Instance().mBlueLED->Blink(Nrf::LedConsts::StatusLed::BleConnected::kOn_ms,
					   Nrf::LedConsts::StatusLed::BleConnected::kOff_ms);
		break;
	case Nrf::DeviceState::DeviceProvisioned:
		Instance().mRedLED->Blink(Nrf::LedConsts::StatusLed::Disconnected::kOn_ms,
					  Nrf::LedConsts::StatusLed::Disconnected::kOff_ms);
		Instance().mBlueLED->Blink(Nrf::LedConsts::StatusLed::Disconnected::kOn_ms,
					   Nrf::LedConsts::StatusLed::Disconnected::kOff_ms);
		break;
	default:
		break;
	}
}

CHIP_ERROR AppTask::Init()
{
	/* Initialize Matter stack */
#ifdef CONFIG_MCUMGR_TRANSPORT_BT
	ReturnErrorOnFailure(
		Nrf::Matter::PrepareServer(Nrf::Matter::InitData{ .mPostServerInitClbk = []() -> CHIP_ERROR {
			Nrf::GetDFUOverSMP().StartServer();
			return CHIP_NO_ERROR;
		} }));
#else
	ReturnErrorOnFailure(Nrf::Matter::PrepareServer());
#endif

	/* Set references for RGB LED */
	mRedLED = &Nrf::GetBoard().GetLED(Nrf::DeviceLeds::LED1);
	mGreenLED = &Nrf::GetBoard().GetLED(Nrf::DeviceLeds::LED2);
	mBlueLED = &Nrf::GetBoard().GetLED(Nrf::DeviceLeds::LED3);

	if (!Nrf::GetBoard().Init(nullptr, UpdateLedState)) {
		LOG_ERR("User interface initialization failed.");
		return CHIP_ERROR_INCORRECT_STATE;
	}

	/* Register Matter event handler that controls the connectivity status LED based on the captured Matter network
	 * state. */
	ReturnErrorOnFailure(Nrf::Matter::RegisterEventHandler(Nrf::Board::DefaultMatterEventHandler, 0));

	if (!device_is_ready(sBme688SensorDev)) {
		LOG_ERR("BME688 sensor device not ready");
		return chip::System::MapErrorZephyr(-ENODEV);
	}

	if (!device_is_ready(sAdxl362SensorDev)) {
		LOG_ERR("ADXL362 sensor device not ready");
		return chip::System::MapErrorZephyr(-ENODEV);
	}

	int ret = BatteryMeasurementInit();
	if (ret) {
		LOG_ERR("Battery measurement init failed");
		return chip::System::MapErrorZephyr(ret);
	}

	ret = BatteryMeasurementEnable();
	if (ret) {
		LOG_ERR("Enabling battery measurement failed");
		return chip::System::MapErrorZephyr(ret);
	}

	ret = BatteryChargeControlInit();
	if (ret) {
		LOG_ERR("Battery charge control init failed");
		return chip::System::MapErrorZephyr(ret);
	}

	ret = BuzzerInit();
	if (ret) {
		LOG_ERR("Buzzer init failed");
		return chip::System::MapErrorZephyr(ret);
	}

	ml_control |= ML_FIRST_PREDICTION;
	ret = ei_wrapper_init(ResultReadyHandler);
	if (ret) {
		LOG_ERR("ei_wrapper_init() failed");
		return chip::System::MapErrorZephyr(ret);
	}

	/* Initialize timers */
	k_timer_init(
		&sMeasurementsTimer, [](k_timer *) { Nrf::PostTask([] { MeasurementsTimerHandler(); }); }, nullptr);
	k_timer_init(
		&sIdentifyTimer, [](k_timer *) { Nrf::PostTask([] { IdentifyTimerHandler(); }); }, nullptr);
	k_timer_start(&sMeasurementsTimer, K_MSEC(kMeasurementsIntervalMs), K_MSEC(kMeasurementsIntervalMs));
	k_timer_init(
		&sAccMeasurementsTimer, [](k_timer *) { Nrf::PostTask([] { AccMeasurementsTimerHandler(); }); }, nullptr);
	k_timer_start(&sAccMeasurementsTimer, K_MSEC(kAccMeasurementsIntervalMs), K_MSEC(kAccMeasurementsIntervalMs));
	start_prediction();

	return Nrf::Matter::StartServer();
}

CHIP_ERROR AppTask::StartApp()
{
	ReturnErrorOnFailure(Init());

	while (true) {
		Nrf::DispatchNextTask();
	}

	return CHIP_NO_ERROR;
}
