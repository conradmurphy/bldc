/*
	Copyright 2017 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include <math.h>

#include "app.h"
#include "buffer.h"
#include "ch.h"
#include "comm_can.h"
#include "conf_general.h"
#include "hal.h"
#include "hw.h"
#include "mc_interface.h"
#include "stm32f4xx_conf.h"
#include "timeout.h"
#include "utils.h"

// Settings
#define CANID_DCStatus      0x250
#define CANID_EnergyWh      0x251
#define CANID_EnergyAh      0x252
#define CANID_Temperature   0x253
#define CANID_Current       0x254
#define CANID_Speed         0x255
#define CANID_Throttle      0x256

#define CANID_RD_MC_ACTUAL_OPERATION_DATA     386
#define CANID_RD_REAL_TIME_DATA                     0x184 // 388
#define CANID_RD_BMS_CELL_VOLTAGE1            657
#define CANID_RD_BMS_CELL_VOLTAGE2            658
#define CANID_RD_BMS_CELL_VOLTAGE3            659

#define MAX_CAN_AGE                     0.1
#define MIN_MS_WITHOUT_POWER            500
#define FILTER_SAMPLES                  5
#define RPM_FILTER_SAMPLES              8

#define BRAKE_RPM_MIN                   2500
#define BRAKE_RPM_DERATE_WINDOW         3000

// Threads
static THD_FUNCTION(adc_thread, arg);
static THD_WORKING_AREA(adc_thread_wa, 1024);

static THD_FUNCTION(can_status, arg);
static THD_WORKING_AREA(can_status_wa, 1024);

// Private variables
static volatile adc_config config;
static volatile float ms_without_power = 0.0;
static volatile float decoded_level = 0.0;
static volatile float read_voltage = 0.0;
static volatile float decoded_level2 = 0.0;
static volatile float read_voltage2 = 0.0;
static volatile bool use_rx_tx_as_buttons = false;
static volatile bool stop_now = true;
static volatile bool is_running = false;

static volatile bool canThread_stop_now = true;
static volatile bool canThread_is_running = false;

static volatile float current_out_can_buffer; 

void app_custom_start(void) {
    canThread_stop_now = false;
    chThdCreateStatic(can_status_wa, sizeof(can_status_wa), NORMALPRIO, can_status, NULL);

    stop_now = false;
    chThdCreateStatic(adc_thread_wa, sizeof(adc_thread_wa), NORMALPRIO, adc_thread, NULL);
}

void app_custom_stop(void) {
    canThread_stop_now = true;
    while (canThread_is_running) {
        chThdSleepMilliseconds(1);
    }

    stop_now = true;
    while (is_running) {
        chThdSleepMilliseconds(1);
    }
}

void app_custom_configure(app_configuration *conf) {
    config = conf->app_adc_conf;
    ms_without_power = 0.0;
}


/* This function sends CAN status messages on a loop
 */
static THD_FUNCTION(can_status, arg) {
    (void)arg;

    chRegSetThreadName("Lyft CAN");

    for(;;) {
        // If we've been told to stop do so
        if (canThread_stop_now) {
            canThread_is_running = false;
            return;
        }

        // Send Battery status message
        int32_t send_index = 0;
        uint8_t buffer[8];
        buffer_append_float16(buffer, mc_interface_get_tot_current_in_filtered(), 1e3, &send_index);
        buffer_append_float16(buffer, mc_interface_get_tot_current(), 1e3, &send_index);
        buffer_append_float16(buffer, GET_INPUT_VOLTAGE(), 1e3, &send_index);
        comm_can_transmit_sid(CANID_DCStatus, buffer, send_index);

        // Send Energy status message
        send_index = 0;
        buffer_append_float32(buffer, mc_interface_get_watt_hours(false), 1e3, &send_index);
        buffer_append_float32(buffer, mc_interface_get_watt_hours_charged(false), 1e3, &send_index);
        comm_can_transmit_sid(CANID_EnergyWh, buffer, send_index);

        // Send Ah status message
        send_index = 0;
        buffer_append_float32(buffer, mc_interface_get_amp_hours(false), 1e3, &send_index);
        buffer_append_float32(buffer, mc_interface_get_amp_hours_charged(false), 1e3, &send_index);
        comm_can_transmit_sid(CANID_EnergyAh, buffer, send_index);

        // Send Temperature status message
        send_index = 0;
        buffer_append_int16(buffer, (int16_t)(mc_interface_temp_motor_filtered() * 1.0), &send_index);
        buffer_append_int16(buffer, (int16_t)(mc_interface_temp_fet_filtered() * 1.0), &send_index);
        comm_can_transmit_sid(CANID_Temperature, buffer, send_index);

        // Send Current status message
        send_index = 0;
        buffer_append_float16(buffer, mc_interface_read_reset_avg_motor_current(), 1e3, &send_index);
        buffer_append_float16(buffer, mc_interface_read_reset_avg_id(), 1e3, &send_index);
        buffer_append_float16(buffer, mc_interface_read_reset_avg_iq(), 1e3, &send_index);
        comm_can_transmit_sid(CANID_Current, buffer, send_index);

        // Send Speed status message
        send_index = 0;
        buffer_append_float16(buffer, mc_interface_get_rpm(), 1, &send_index);
        buffer_append_int16(buffer, (int16_t)mc_interface_get_tachometer_value(false), &send_index);
        buffer_append_int16(buffer, (int16_t)mc_interface_get_tachometer_abs_value(false), &send_index);
        comm_can_transmit_sid(CANID_Speed, buffer, send_index);

        // Send Throttle status message
        send_index = 0;
        buffer_append_float16(buffer, mc_interface_get_duty_cycle_now(), 1e3, &send_index);
        buffer_append_float16(buffer, current_out_can_buffer, 1e3, &send_index);
        comm_can_transmit_sid(CANID_Throttle, buffer, send_index);
 
        //
        chThdSleepMilliseconds(50);

        comm_can_transmit_sid_rtr(CANID_RD_MC_ACTUAL_OPERATION_DATA);
        comm_can_transmit_sid_rtr(CANID_RD_REAL_TIME_DATA);
        comm_can_transmit_sid_rtr(CANID_RD_BMS_CELL_VOLTAGE1);
        comm_can_transmit_sid_rtr(CANID_RD_BMS_CELL_VOLTAGE2);
        comm_can_transmit_sid_rtr(CANID_RD_BMS_CELL_VOLTAGE3);
        
        chThdSleepMilliseconds(50);
    }
}

/*
float app_adc_get_decoded_level(void) {
    return decoded_level;
}

float app_adc_get_voltage(void) {
    return read_voltage;
}

float app_adc_get_decoded_level2(void) {
    return decoded_level2;
}

float app_adc_get_voltage2(void) {
    return read_voltage2;
}*/


static THD_FUNCTION(adc_thread, arg) {
    (void)arg;

    chRegSetThreadName("APP_ADC");

    // Set servo pin as an input with pullup
    if (use_rx_tx_as_buttons) {
        palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_INPUT_PULLUP);
        palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_INPUT_PULLUP);
    } else {
        palSetPadMode(HW_ICU_GPIO, HW_ICU_PIN, PAL_MODE_INPUT_PULLUP);
    }

    is_running = true;

    for(;;) {
        // Sleep for a time according to the specified rate
        systime_t sleep_time = CH_CFG_ST_FREQUENCY / config.update_rate_hz;

        // At least one tick should be slept to not block the other threads
        if (sleep_time == 0) {
            sleep_time = 1;
        }
        chThdSleep(sleep_time);

        if (stop_now) {
            is_running = false;
            return;
        }

        // For safe start when fault codes occur
        if (mc_interface_get_fault() != FAULT_CODE_NONE) {
            ms_without_power = 0;
        }

        // Read the external ADC pin and convert the value to a voltage.
        float pwr = (float)ADC_Value[ADC_IND_EXT];
        pwr /= 4095;
        pwr *= V_REG;

        read_voltage = pwr;

        // Optionally apply a mean value filter
        if (config.use_filter) {
            static float filter_buffer[FILTER_SAMPLES];
            static int filter_ptr = 0;

            filter_buffer[filter_ptr++] = pwr;
            if (filter_ptr >= FILTER_SAMPLES) {
                filter_ptr = 0;
            }

            pwr = 0.0;
            for (int i = 0;i < FILTER_SAMPLES;i++) {
                pwr += filter_buffer[i];
            }
            pwr /= FILTER_SAMPLES;
        }

        // Map the read voltage
        switch (config.ctrl_type) {
        case ADC_CTRL_TYPE_CURRENT_REV_CENTER:
        case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER:
        case ADC_CTRL_TYPE_DUTY_REV_CENTER:
        case ADC_CTRL_TYPE_PID_REV_CENTER:
            // Mapping with respect to center voltage
            if (pwr < config.voltage_center) {
                pwr = utils_map(pwr, config.voltage_start,
                        config.voltage_center, 0.0, 0.5);
            } else {
                pwr = utils_map(pwr, config.voltage_center,
                        config.voltage_end, 0.5, 1.0);
            }
            break;

        default:
            // Linear mapping between the start and end voltage
            pwr = utils_map(pwr, config.voltage_start, config.voltage_end, 0.0, 1.0);
            break;
        }

        // Truncate the read voltage
        utils_truncate_number(&pwr, 0.0, 1.0);

        // Optionally invert the read voltage
        if (config.voltage_inverted) {
            pwr = 1.0 - pwr;
        }

        decoded_level = pwr;

        // Read the external ADC pin and convert the value to a voltage.
#ifdef ADC_IND_EXT2
        float brake = (float)ADC_Value[ADC_IND_EXT2];
        brake /= 4095;
        brake *= V_REG;
#else
        float brake = 0.0;
#endif

        read_voltage2 = brake;

        // Optionally apply a mean value filter
        if (config.use_filter) {
            static float filter_buffer2[FILTER_SAMPLES];
            static int filter_ptr2 = 0;

            filter_buffer2[filter_ptr2++] = brake;
            if (filter_ptr2 >= FILTER_SAMPLES) {
                filter_ptr2 = 0;
            }

            brake = 0.0;
            for (int i = 0;i < FILTER_SAMPLES;i++) {
                brake += filter_buffer2[i];
            }
            brake /= FILTER_SAMPLES;
        }

        // Map and truncate the read voltage
        brake = utils_map(brake, config.voltage2_start, config.voltage2_end, 0.0, 1.0);
        utils_truncate_number(&brake, 0.0, 1.0);

        // Optionally invert the read voltage
        if (config.voltage2_inverted) {
            brake = 1.0 - brake;
        }

        decoded_level2 = brake;

        // Read the button pins
        bool cc_button = false;
        bool rev_button = false;
        if (use_rx_tx_as_buttons) {
            cc_button = !palReadPad(HW_UART_TX_PORT, HW_UART_TX_PIN);
            if (config.cc_button_inverted) {
                cc_button = !cc_button;
            }
            rev_button = !palReadPad(HW_UART_RX_PORT, HW_UART_RX_PIN);
            if (config.rev_button_inverted) {
                rev_button = !rev_button;
            }
        } else {
            // When only one button input is available, use it differently depending on the control mode
            if (config.ctrl_type == ADC_CTRL_TYPE_CURRENT_REV_BUTTON ||
                    config.ctrl_type == ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON ||
                    config.ctrl_type == ADC_CTRL_TYPE_DUTY_REV_BUTTON) {
                rev_button = !palReadPad(HW_ICU_GPIO, HW_ICU_PIN);
                if (config.rev_button_inverted) {
                    rev_button = !rev_button;
                }
            } else {
                cc_button = !palReadPad(HW_ICU_GPIO, HW_ICU_PIN);
                if (config.cc_button_inverted) {
                    cc_button = !cc_button;
                }
            }
        }

        switch (config.ctrl_type) {
        case ADC_CTRL_TYPE_CURRENT_REV_CENTER:
        case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER:
        case ADC_CTRL_TYPE_DUTY_REV_CENTER:
        case ADC_CTRL_TYPE_PID_REV_CENTER:
            // Scale the voltage and set 0 at the center
            pwr *= 2.0;
            pwr -= 1.0;
            break;

        case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC:
        case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC:
            pwr -= brake;
            break;

        case ADC_CTRL_TYPE_CURRENT_REV_BUTTON:
        case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON:
        case ADC_CTRL_TYPE_DUTY_REV_BUTTON:
        case ADC_CTRL_TYPE_PID_REV_BUTTON:
            // Invert the voltage if the button is pressed
            if (rev_button) {
                pwr = -pwr;
            }
            break;

        default:
            break;
        }

        // Apply deadband
        utils_deadband(&pwr, config.hyst, 1.0);

        // Apply throttle curve
        pwr = utils_throttle_curve(pwr, config.throttle_exp, config.throttle_exp_brake, config.throttle_exp_mode);

        // Apply RPM-based ramping for breaking
        const float rpm_now = mc_interface_get_rpm();
        if (config.ctrl_type == ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER && pwr<0){
            float rpm_scalar = (rpm_now-BRAKE_RPM_MIN)/BRAKE_RPM_DERATE_WINDOW;
            utils_truncate_number(&rpm_scalar, 0, 1);
            pwr = rpm_scalar * pwr;
        }

        // Apply ramping
        static systime_t last_time = 0;
        static float pwr_ramp = 0.0;
        const float ramp_time = fabsf(pwr) > fabsf(pwr_ramp) ? config.ramp_time_pos : config.ramp_time_neg;

        if (ramp_time > 0.01) {
            const float ramp_step = (float)ST2MS(chVTTimeElapsedSinceX(last_time)) / (ramp_time * 1000.0);
            utils_step_towards(&pwr_ramp, pwr, ramp_step);
            last_time = chVTGetSystemTimeX();
            pwr = pwr_ramp;
        }

        float current_rel = 0.0;
        bool current_mode = false;
        bool current_mode_brake = false;
        const volatile mc_configuration *mcconf = mc_interface_get_configuration();
        bool send_duty = false;

        // Use the filtered and mapped voltage for control according to the configuration.
        switch (config.ctrl_type) {
        case ADC_CTRL_TYPE_CURRENT:
        case ADC_CTRL_TYPE_CURRENT_REV_CENTER:
        case ADC_CTRL_TYPE_CURRENT_REV_BUTTON:
            current_mode = true;
            if ((pwr >= 0.0 && rpm_now > 0.0) || (pwr < 0.0 && rpm_now < 0.0)) {
                current_rel = pwr;
            } else {
                current_rel = pwr;
            }

            if (fabsf(pwr) < 0.001) {
                ms_without_power += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
            }
            break;

        case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER:
        case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON:
        case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC:
        case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC:
            current_mode = true;
            if (pwr >= 0.0) {
                current_rel = pwr;
            } else {
                current_rel = fabsf(pwr);
                current_mode_brake = true;
            }

            if (pwr < 0.001) {
                ms_without_power += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
            }

            if (config.ctrl_type == ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC && rev_button) {
                current_rel = -current_rel;
            }
            break;

        case ADC_CTRL_TYPE_DUTY:
        case ADC_CTRL_TYPE_DUTY_REV_CENTER:
        case ADC_CTRL_TYPE_DUTY_REV_BUTTON:
            if (fabsf(pwr) < 0.001) {
                ms_without_power += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
            }

            if (!(ms_without_power < MIN_MS_WITHOUT_POWER && config.safe_start)) {
                mc_interface_set_duty(utils_map(pwr, -1.0, 1.0, -mcconf->l_max_duty, mcconf->l_max_duty));
                send_duty = true;
            }
            break;

        case ADC_CTRL_TYPE_PID:
        case ADC_CTRL_TYPE_PID_REV_CENTER:
        case ADC_CTRL_TYPE_PID_REV_BUTTON:
            if ((pwr >= 0.0 && rpm_now > 0.0) || (pwr < 0.0 && rpm_now < 0.0)) {
                current_rel = pwr;
            } else {
                current_rel = pwr;
            }

            if (!(ms_without_power < MIN_MS_WITHOUT_POWER && config.safe_start)) {
                float speed = 0.0;
                if (pwr >= 0.0) {
                    speed = pwr * mcconf->l_max_erpm;
                } else {
                    speed = pwr * fabsf(mcconf->l_min_erpm);
                }

                mc_interface_set_pid_speed(speed);
                send_duty = true;
            }

            if (fabsf(pwr) < 0.001) {
                ms_without_power += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
            }
            break;

        default:
            continue;
        }

        // If safe start is enabled and the output has not been zero for long enough
        if (ms_without_power < MIN_MS_WITHOUT_POWER && config.safe_start) {
            static int pulses_without_power_before = 0;
            if (ms_without_power == pulses_without_power_before) {
                ms_without_power = 0;
            }
            pulses_without_power_before = ms_without_power;
            mc_interface_set_brake_current(timeout_get_brake_current());

            if (config.multi_esc) {
                for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
                    can_status_msg *msg = comm_can_get_status_msg_index(i);

                    if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
                        comm_can_set_current_brake(msg->id, timeout_get_brake_current());
                    }
                }
            }

            continue;
        }

        // Reset timeout
        timeout_reset();

        // If c is pressed and no throttle is used, maintain the current speed with PID control
        static bool was_pid = false;

        // Filter RPM to avoid glitches
        static float filter_buffer[RPM_FILTER_SAMPLES];
        static int filter_ptr = 0;
        filter_buffer[filter_ptr++] = mc_interface_get_rpm();
        if (filter_ptr >= RPM_FILTER_SAMPLES) {
            filter_ptr = 0;
        }

        float rpm_filtered = 0.0;
        for (int i = 0;i < RPM_FILTER_SAMPLES;i++) {
            rpm_filtered += filter_buffer[i];
        }
        rpm_filtered /= RPM_FILTER_SAMPLES;

        if (current_mode && cc_button && fabsf(pwr) < 0.001) {
            static float pid_rpm = 0.0;

            if (!was_pid) {
                was_pid = true;
                pid_rpm = rpm_filtered;
            }

            mc_interface_set_pid_speed(pid_rpm);

            // Send the same duty cycle to the other controllers
            if (config.multi_esc) {
                float current = mc_interface_get_tot_current_directional_filtered();

                for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
                    can_status_msg *msg = comm_can_get_status_msg_index(i);

                    if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
                        comm_can_set_current(msg->id, current);
                    }
                }
            }

            continue;
        }

        was_pid = false;

        // Find lowest RPM (for traction control)
        float rpm_local = mc_interface_get_rpm();
        float rpm_lowest = rpm_local;
        if (config.multi_esc) {
            for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
                can_status_msg *msg = comm_can_get_status_msg_index(i);

                if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
                    float rpm_tmp = msg->rpm;

                    if (fabsf(rpm_tmp) < fabsf(rpm_lowest)) {
                        rpm_lowest = rpm_tmp;
                    }
                }
            }
        }

        // Optionally send the duty cycles to the other ESCs seen on the CAN-bus
        if (send_duty && config.multi_esc) {
            float duty = mc_interface_get_duty_cycle_now();

            for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
                can_status_msg *msg = comm_can_get_status_msg_index(i);

                if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
                    comm_can_set_duty(msg->id, duty);
                }
            }
        }

        if (current_mode) {
            if (current_mode_brake) {
                mc_interface_set_brake_current_rel(current_rel);

                // Send brake command to all ESCs seen recently on the CAN bus
                for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
                    can_status_msg *msg = comm_can_get_status_msg_index(i);

                    if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
                        comm_can_set_current_brake_rel(msg->id, current_rel);
                    }
                }
            } else {
                float current_out = current_rel;
                bool is_reverse = false;
                if (current_out < 0.0) {
                    is_reverse = true;
                    current_out = -current_out;
                    current_rel = -current_rel;
                    rpm_local = -rpm_local;
                    rpm_lowest = -rpm_lowest;
                }

                // Traction control
                if (config.multi_esc) {
                    for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
                        can_status_msg *msg = comm_can_get_status_msg_index(i);

                        if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
                            if (config.tc) {
                                float rpm_tmp = msg->rpm;
                                if (is_reverse) {
                                    rpm_tmp = -rpm_tmp;
                                }

                                float diff = rpm_tmp - rpm_lowest;
                                current_out = utils_map(diff, 0.0, config.tc_max_diff, current_rel, 0.0);
                            }

                            if (is_reverse) {
                                comm_can_set_current_rel(msg->id, -current_out);
                            } else {
                                comm_can_set_current_rel(msg->id, current_out);
                            }
                        }
                    }

                    if (config.tc) {
                        float diff = rpm_local - rpm_lowest;
                        current_out = utils_map(diff, 0.0, config.tc_max_diff, current_rel, 0.0);
                    }
                }

                if (is_reverse) {
                    mc_interface_set_current_rel(-current_out);
                    current_out_can_buffer = -current_out;
                } else {
                    mc_interface_set_current_rel(current_out);
                    current_out_can_buffer = current_out;
                }
            }
        }
    }
}

