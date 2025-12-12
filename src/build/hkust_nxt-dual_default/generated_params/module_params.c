
/**
 * Empty cell voltage
 *
 * Defines the voltage where a single cell of the battery is considered empty.
 * The voltage should be chosen above the steep dropoff at 3.5V. A typical
 * lithium battery can only be discharged under high load down to 10% before
 * it drops off to a voltage level damaging the cells.
 * 
 *
 * @group Battery Calibration
 * @decimal 2
 * @increment 0.01
 * @unit V
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(BAT1_V_EMPTY, 3.6);

/**
 * Empty cell voltage
 *
 * Defines the voltage where a single cell of the battery is considered empty.
 * The voltage should be chosen above the steep dropoff at 3.5V. A typical
 * lithium battery can only be discharged under high load down to 10% before
 * it drops off to a voltage level damaging the cells.
 * 
 *
 * @group Battery Calibration
 * @decimal 2
 * @increment 0.01
 * @unit V
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(BAT2_V_EMPTY, 3.6);

/**
 * Empty cell voltage
 *
 * Defines the voltage where a single cell of the battery is considered empty.
 * The voltage should be chosen above the steep dropoff at 3.5V. A typical
 * lithium battery can only be discharged under high load down to 10% before
 * it drops off to a voltage level damaging the cells.
 * 
 *
 * @group Battery Calibration
 * @decimal 2
 * @increment 0.01
 * @unit V
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(BAT3_V_EMPTY, 3.6);

/**
 * Full cell voltage
 *
 * Defines the voltage where a single cell of the battery is considered full.
 * For a more accurate estimate set this below the nominal voltage of e.g. 4.2V
 * 
 *
 * @group Battery Calibration
 * @decimal 2
 * @increment 0.01
 * @unit V
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(BAT1_V_CHARGED, 4.05);

/**
 * Full cell voltage
 *
 * Defines the voltage where a single cell of the battery is considered full.
 * For a more accurate estimate set this below the nominal voltage of e.g. 4.2V
 * 
 *
 * @group Battery Calibration
 * @decimal 2
 * @increment 0.01
 * @unit V
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(BAT2_V_CHARGED, 4.05);

/**
 * Full cell voltage
 *
 * Defines the voltage where a single cell of the battery is considered full.
 * For a more accurate estimate set this below the nominal voltage of e.g. 4.2V
 * 
 *
 * @group Battery Calibration
 * @decimal 2
 * @increment 0.01
 * @unit V
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(BAT3_V_CHARGED, 4.05);

/**
 * Explicitly defines the per cell internal resistance for battery 1
 *
 * If non-negative, then this will be used instead of the online estimated internal resistance.
 * 
 *
 * @group Battery Calibration
 * @decimal 4
 * @increment 0.0005
 * @min -1.0
 * @max 0.2
 * @unit Ohm
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(BAT1_R_INTERNAL, -1.0);

/**
 * Explicitly defines the per cell internal resistance for battery 2
 *
 * If non-negative, then this will be used instead of the online estimated internal resistance.
 * 
 *
 * @group Battery Calibration
 * @decimal 4
 * @increment 0.0005
 * @min -1.0
 * @max 0.2
 * @unit Ohm
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(BAT2_R_INTERNAL, -1.0);

/**
 * Explicitly defines the per cell internal resistance for battery 3
 *
 * If non-negative, then this will be used instead of the online estimated internal resistance.
 * 
 *
 * @group Battery Calibration
 * @decimal 4
 * @increment 0.0005
 * @min -1.0
 * @max 0.2
 * @unit Ohm
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(BAT3_R_INTERNAL, -1.0);

/**
 * Number of cells for battery 1.
 *
 * Defines the number of cells the attached battery consists of.
 * 
 *
 * @group Battery Calibration
 * @value 0 Unknown
 * @value 1 1S Battery
 * @value 2 2S Battery
 * @value 3 3S Battery
 * @value 4 4S Battery
 * @value 5 5S Battery
 * @value 6 6S Battery
 * @value 7 7S Battery
 * @value 8 8S Battery
 * @value 9 9S Battery
 * @value 10 10S Battery
 * @value 11 11S Battery
 * @value 12 12S Battery
 * @value 13 13S Battery
 * @value 14 14S Battery
 * @value 15 15S Battery
 * @value 16 16S Battery
 */
PARAM_DEFINE_INT32(BAT1_N_CELLS, 0);

/**
 * Number of cells for battery 2.
 *
 * Defines the number of cells the attached battery consists of.
 * 
 *
 * @group Battery Calibration
 * @value 0 Unknown
 * @value 1 1S Battery
 * @value 2 2S Battery
 * @value 3 3S Battery
 * @value 4 4S Battery
 * @value 5 5S Battery
 * @value 6 6S Battery
 * @value 7 7S Battery
 * @value 8 8S Battery
 * @value 9 9S Battery
 * @value 10 10S Battery
 * @value 11 11S Battery
 * @value 12 12S Battery
 * @value 13 13S Battery
 * @value 14 14S Battery
 * @value 15 15S Battery
 * @value 16 16S Battery
 */
PARAM_DEFINE_INT32(BAT2_N_CELLS, 0);

/**
 * Number of cells for battery 3.
 *
 * Defines the number of cells the attached battery consists of.
 * 
 *
 * @group Battery Calibration
 * @value 0 Unknown
 * @value 1 1S Battery
 * @value 2 2S Battery
 * @value 3 3S Battery
 * @value 4 4S Battery
 * @value 5 5S Battery
 * @value 6 6S Battery
 * @value 7 7S Battery
 * @value 8 8S Battery
 * @value 9 9S Battery
 * @value 10 10S Battery
 * @value 11 11S Battery
 * @value 12 12S Battery
 * @value 13 13S Battery
 * @value 14 14S Battery
 * @value 15 15S Battery
 * @value 16 16S Battery
 */
PARAM_DEFINE_INT32(BAT3_N_CELLS, 0);

/**
 * Battery 1 capacity.
 *
 * Defines the capacity of battery 1 in mAh.
 * 
 *
 * @group Battery Calibration
 * @decimal 0
 * @increment 50
 * @min -1.0
 * @max 100000
 * @unit mAh
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(BAT1_CAPACITY, -1.0);

/**
 * Battery 2 capacity.
 *
 * Defines the capacity of battery 2 in mAh.
 * 
 *
 * @group Battery Calibration
 * @decimal 0
 * @increment 50
 * @min -1.0
 * @max 100000
 * @unit mAh
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(BAT2_CAPACITY, -1.0);

/**
 * Battery 3 capacity.
 *
 * Defines the capacity of battery 3 in mAh.
 * 
 *
 * @group Battery Calibration
 * @decimal 0
 * @increment 50
 * @min -1.0
 * @max 100000
 * @unit mAh
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(BAT3_CAPACITY, -1.0);

/**
 * Battery 1 monitoring source.
 *
 * This parameter controls the source of battery data. The value 'Power Module'
 * means that measurements are expected to come from a power module. If the value is set to
 * 'External' then the system expects to receive mavlink battery status messages.
 * If the value is set to 'ESCs', the battery information are taken from the esc_status message.
 * This requires the ESC to provide both voltage as well as current.
 * 
 *
 * @group Battery Calibration
 * @value -1 Disabled
 * @value 0 Power Module
 * @value 1 External
 * @value 2 ESCs
 * @reboot_required True
 */
PARAM_DEFINE_INT32(BAT1_SOURCE, 0);

/**
 * Battery 2 monitoring source.
 *
 * This parameter controls the source of battery data. The value 'Power Module'
 * means that measurements are expected to come from a power module. If the value is set to
 * 'External' then the system expects to receive mavlink battery status messages.
 * If the value is set to 'ESCs', the battery information are taken from the esc_status message.
 * This requires the ESC to provide both voltage as well as current.
 * 
 *
 * @group Battery Calibration
 * @value -1 Disabled
 * @value 0 Power Module
 * @value 1 External
 * @value 2 ESCs
 * @reboot_required True
 */
PARAM_DEFINE_INT32(BAT2_SOURCE, -1);

/**
 * Battery 3 monitoring source.
 *
 * This parameter controls the source of battery data. The value 'Power Module'
 * means that measurements are expected to come from a power module. If the value is set to
 * 'External' then the system expects to receive mavlink battery status messages.
 * If the value is set to 'ESCs', the battery information are taken from the esc_status message.
 * This requires the ESC to provide both voltage as well as current.
 * 
 *
 * @group Battery Calibration
 * @value -1 Disabled
 * @value 0 Power Module
 * @value 1 External
 * @value 2 ESCs
 * @reboot_required True
 */
PARAM_DEFINE_INT32(BAT3_SOURCE, -1);

/**
 * Low threshold.
 *
 * Sets the threshold when the battery will be reported as low.
 * This has to be higher than the critical threshold.
 * 
 *
 * @group Battery Calibration
 * @decimal 2
 * @increment 0.01
 * @min 0.12
 * @max 0.5
 * @unit norm
 */
PARAM_DEFINE_FLOAT(BAT_LOW_THR, 0.15);

/**
 * Critical threshold.
 *
 * Sets the threshold when the battery will be reported as critically low.
 * This has to be lower than the low threshold. This threshold commonly
 * will trigger RTL.
 * 
 *
 * @group Battery Calibration
 * @decimal 2
 * @increment 0.01
 * @min 0.05
 * @max 0.5
 * @unit norm
 */
PARAM_DEFINE_FLOAT(BAT_CRIT_THR, 0.07);

/**
 * Emergency threshold.
 *
 * Sets the threshold when the battery will be reported as dangerously low.
 * This has to be lower than the critical threshold. This threshold commonly
 * will trigger landing.
 * 
 *
 * @group Battery Calibration
 * @decimal 2
 * @increment 0.01
 * @min 0.03
 * @max 0.5
 * @unit norm
 */
PARAM_DEFINE_FLOAT(BAT_EMERGEN_THR, 0.05);

/**
 * Expected battery current in flight.
 *
 * This value is used to initialize the in-flight average current estimation,
 * which in turn is used for estimating remaining flight time and RTL triggering.
 * 
 *
 * @group Battery Calibration
 * @decimal 2
 * @increment 0.1
 * @min 0
 * @max 500
 * @unit A
 */
PARAM_DEFINE_FLOAT(BAT_AVRG_CURRENT, 15);

/**
 * Distance Sensor Rotation
 *
 * Distance Sensor Rotation as MAV_SENSOR_ORIENTATION enum
 * 
 *
 * @group Sensors
 * @value 25 ROTATION_DOWNWARD_FACING
 * @value 24 ROTATION_UPWARD_FACING
 * @value 4 ROTATION_BACKWARD_FACING
 * @value 0 ROTATION_FORWARD_FACING
 * @value 6 ROTATION_LEFT_FACING
 * @value 2 ROTATION_RIGHT_FACING
 * @reboot_required True
 */
PARAM_DEFINE_INT32(SENS_CM8JL65_R_0, 25);

/**
 * Hardware Model
 *
 * Models differ in range and FoV.
 *
 * @group Sensors
 * @value 1 TFMINI
 * @value 2 ISTRA24
 * @value 3 ISTRA24_100m
 * @min 1
 * @max 3
 * @reboot_required True
 */
PARAM_DEFINE_INT32(SENS_TFMINI_HW, 1);

/**
 * Minimum DShot Motor Output
 *
 * Minimum Output Value for DShot in percent. The value depends on the ESC. Make
 * sure to set this high enough so that the motors are always spinning while
 * armed.
 * 
 *
 * @group DShot
 * @decimal 2
 * @increment 0.01
 * @min 0
 * @max 1
 * @unit %
 */
PARAM_DEFINE_FLOAT(DSHOT_MIN, 0.055);

/**
 * Allows for 3d mode when using DShot and suitable mixer
 *
 * WARNING: ESC must be configured for 3D mode, and DSHOT_MIN set to 0.
 * This splits the throttle ranges in two.
 * Direction 1) 48 is the slowest, 1047 is the fastest.
 * Direction 2) 1049 is the slowest, 2047 is the fastest.
 * When mixer outputs 1000 or value inside DSHOT 3D deadband, DShot 0 is sent.
 * 
 *
 * @group DShot
 * @boolean
 */
PARAM_DEFINE_INT32(DSHOT_3D_ENABLE, 0);

/**
 * Enable bidirectional DShot
 *
 * This parameter enables bidirectional DShot which provides RPM feedback.
 * Note that this requires ESCs that support bidirectional DSHot, e.g. BlHeli32.
 * This is not the same as DShot telemetry which requires an additional serial connection.
 * 
 *
 * @group DShot
 * @boolean
 * @reboot_required True
 */
PARAM_DEFINE_INT32(DSHOT_BIDIR_EN, 0);

/**
 * DSHOT 3D deadband high
 *
 * When the actuator_output is between DSHOT_3D_DEAD_L and DSHOT_3D_DEAD_H, motor will not spin.
 * This value is with respect to the mixer_module range (0-1999), not the DSHOT values.
 * 
 *
 * @group DShot
 * @min 1000
 * @max 1999
 */
PARAM_DEFINE_INT32(DSHOT_3D_DEAD_H, 1000);

/**
 * DSHOT 3D deadband low
 *
 * When the actuator_output is between DSHOT_3D_DEAD_L and DSHOT_3D_DEAD_H, motor will not spin.
 * This value is with respect to the mixer_module range (0-1999), not the DSHOT values.
 * 
 *
 * @group DShot
 * @min 0
 * @max 1000
 */
PARAM_DEFINE_INT32(DSHOT_3D_DEAD_L, 1000);

/**
 * Number of magnetic poles of the motors
 *
 * Specify the number of magnetic poles of the motors.
 * It is required to compute the RPM value from the eRPM returned with the ESC telemetry.
 * 
 * Either get the number from the motor spec sheet or count the magnets on the bell of the motor (not the stator magnets).
 * Typical motors for 5 inch props have 14 poles.
 * 
 *
 * @group DShot
 */
PARAM_DEFINE_INT32(MOT_POLE_COUNT, 14);

/**
 * OSD Symbol Selection
 *
 * Configure / toggle support display options.
 * 
 *
 * @group OSD
 * @bit 0 CRAFT_NAME
 * @bit 1 DISARMED
 * @bit 2 GPS_LAT
 * @bit 3 GPS_LON
 * @bit 4 GPS_SATS
 * @bit 5 GPS_SPEED
 * @bit 6 HOME_DIST
 * @bit 7 HOME_DIR
 * @bit 8 MAIN_BATT_VOLTAGE
 * @bit 9 CURRENT_DRAW
 * @bit 10 MAH_DRAWN
 * @bit 11 RSSI_VALUE
 * @bit 12 ALTITUDE
 * @bit 13 NUMERICAL_VARIO
 * @bit 14 (unused) FLYMODE
 * @bit 15 (unused) ESC_TMP
 * @bit 16 (unused) PITCH_ANGLE
 * @bit 17 (unused) ROLL_ANGLE
 * @bit 18 CROSSHAIRS
 * @bit 19 AVG_CELL_VOLTAGE
 * @bit 20 (unused) HORIZON_SIDEBARS
 * @bit 21 POWER
 * @min 0
 * @max 4194303
 */
PARAM_DEFINE_INT32(OSD_SYMBOLS, 16383);

/**
 * OSD Crosshairs Height
 *
 * Controls the vertical position of the crosshair display.
 * Resolution is limited by OSD to 15 discrete values. Negative
 * values will display the crosshairs below the horizon
 * 
 *
 * @group OSD
 * @min -8
 * @max 8
 */
PARAM_DEFINE_INT32(OSD_CH_HEIGHT, 0);

/**
 * OSD Warning Level
 *
 * Minimum security of log level to display on the OSD.
 * 
 *
 * @group OSD
 */
PARAM_DEFINE_INT32(OSD_LOG_LEVEL, 3);

/**
 * OSD Scroll Rate (ms)
 *
 * Scroll rate in milliseconds for OSD messages longer than available character width.
 * This is lower-bounded by the nominal loop rate of this module.
 * 
 *
 * @group OSD
 * @min 100
 * @max 1000
 */
PARAM_DEFINE_INT32(OSD_SCROLL_RATE, 125);

/**
 * OSD Dwell Time (ms)
 *
 * Amount of time in milliseconds to dwell at the beginning of the display, when scrolling.
 * 
 *
 * @group OSD
 * @min 100
 * @max 10000
 */
PARAM_DEFINE_INT32(OSD_DWELL_TIME, 500);

/**
 * OSD RC Stick commands
 *
 * Forward RC stick input to VTX when disarmed
 * 
 *
 * @group OSD
 * @min 0
 * @max 1
 */
PARAM_DEFINE_INT32(OSD_RC_STICK, 1);

/**
 * Output Protocol Configuration for PWM Main 1-4
 *
 * Select which Output Protocol to use for outputs PWM Main 1-4.
 * 
 * Custom PWM rates can be used by directly setting any value >0.
 * 
 *
 * @group Actuator Outputs
 * @value -5 DShot150
 * @value -4 DShot300
 * @value -3 DShot600
 * @value -1 OneShot
 * @value 50 PWM 50 Hz
 * @value 100 PWM 100 Hz
 * @value 200 PWM 200 Hz
 * @value 400 PWM 400 Hz
 * @reboot_required True
 */
PARAM_DEFINE_INT32(PWM_MAIN_TIM0, 400);

/**
 * Output Protocol Configuration for PWM Main 5-6
 *
 * Select which Output Protocol to use for outputs PWM Main 5-6.
 * 
 * Custom PWM rates can be used by directly setting any value >0.
 * 
 *
 * @group Actuator Outputs
 * @value -5 DShot150
 * @value -4 DShot300
 * @value -3 DShot600
 * @value -1 OneShot
 * @value 50 PWM 50 Hz
 * @value 100 PWM 100 Hz
 * @value 200 PWM 200 Hz
 * @value 400 PWM 400 Hz
 * @reboot_required True
 */
PARAM_DEFINE_INT32(PWM_MAIN_TIM1, 400);

/**
 * Output Protocol Configuration for PWM Main 7-8
 *
 * Select which Output Protocol to use for outputs PWM Main 7-8.
 * 
 * Custom PWM rates can be used by directly setting any value >0.
 * 
 *
 * @group Actuator Outputs
 * @value -5 DShot150
 * @value -4 DShot300
 * @value -3 DShot600
 * @value -1 OneShot
 * @value 50 PWM 50 Hz
 * @value 100 PWM 100 Hz
 * @value 200 PWM 200 Hz
 * @value 400 PWM 400 Hz
 * @reboot_required True
 */
PARAM_DEFINE_INT32(PWM_MAIN_TIM2, 400);

/**
 * PWM Main 1 Output Function
 *
 * Select what should be output on PWM Main 1.
 * 
 * The default failsafe value is set according to the selected function:
 * - 'Min' for ConstantMin
 * - 'Max' for ConstantMax
 * - 'Max' for Parachute
 * - ('Max'+'Min')/2 for Servos
 * - 'Disarmed' for the rest
 * 
 *
 * @group Actuator Outputs
 * @value 0 Disabled
 * @value 1 Constant Min
 * @value 2 Constant Max
 * @value 101 Motor 1
 * @value 102 Motor 2
 * @value 103 Motor 3
 * @value 104 Motor 4
 * @value 105 Motor 5
 * @value 106 Motor 6
 * @value 107 Motor 7
 * @value 108 Motor 8
 * @value 109 Motor 9
 * @value 110 Motor 10
 * @value 111 Motor 11
 * @value 112 Motor 12
 * @value 201 Servo 1
 * @value 202 Servo 2
 * @value 203 Servo 3
 * @value 204 Servo 4
 * @value 205 Servo 5
 * @value 206 Servo 6
 * @value 207 Servo 7
 * @value 208 Servo 8
 * @value 301 Peripheral via Actuator Set 1
 * @value 302 Peripheral via Actuator Set 2
 * @value 303 Peripheral via Actuator Set 3
 * @value 304 Peripheral via Actuator Set 4
 * @value 305 Peripheral via Actuator Set 5
 * @value 306 Peripheral via Actuator Set 6
 * @value 400 Landing Gear
 * @value 401 Parachute
 * @value 402 RC Roll
 * @value 403 RC Pitch
 * @value 404 RC Throttle
 * @value 405 RC Yaw
 * @value 406 RC Flaps
 * @value 407 RC AUX 1
 * @value 408 RC AUX 2
 * @value 409 RC AUX 3
 * @value 410 RC AUX 4
 * @value 411 RC AUX 5
 * @value 412 RC AUX 6
 * @value 420 Gimbal Roll
 * @value 421 Gimbal Pitch
 * @value 422 Gimbal Yaw
 * @value 430 Gripper
 * @value 440 Landing Gear Wheel
 * @value 450 IC Engine Ignition
 * @value 451 IC Engine Throttle
 * @value 452 IC Engine Choke
 * @value 453 IC Engine Starter
 * @value 2000 Camera Trigger
 * @value 2032 Camera Capture
 * @value 2064 PPS Input
 * @value 2070 RPM Input
 */
PARAM_DEFINE_INT32(PWM_MAIN_FUNC1, 0);

/**
 * PWM Main 2 Output Function
 *
 * Select what should be output on PWM Main 2.
 * 
 * The default failsafe value is set according to the selected function:
 * - 'Min' for ConstantMin
 * - 'Max' for ConstantMax
 * - 'Max' for Parachute
 * - ('Max'+'Min')/2 for Servos
 * - 'Disarmed' for the rest
 * 
 *
 * @group Actuator Outputs
 * @value 0 Disabled
 * @value 1 Constant Min
 * @value 2 Constant Max
 * @value 101 Motor 1
 * @value 102 Motor 2
 * @value 103 Motor 3
 * @value 104 Motor 4
 * @value 105 Motor 5
 * @value 106 Motor 6
 * @value 107 Motor 7
 * @value 108 Motor 8
 * @value 109 Motor 9
 * @value 110 Motor 10
 * @value 111 Motor 11
 * @value 112 Motor 12
 * @value 201 Servo 1
 * @value 202 Servo 2
 * @value 203 Servo 3
 * @value 204 Servo 4
 * @value 205 Servo 5
 * @value 206 Servo 6
 * @value 207 Servo 7
 * @value 208 Servo 8
 * @value 301 Peripheral via Actuator Set 1
 * @value 302 Peripheral via Actuator Set 2
 * @value 303 Peripheral via Actuator Set 3
 * @value 304 Peripheral via Actuator Set 4
 * @value 305 Peripheral via Actuator Set 5
 * @value 306 Peripheral via Actuator Set 6
 * @value 400 Landing Gear
 * @value 401 Parachute
 * @value 402 RC Roll
 * @value 403 RC Pitch
 * @value 404 RC Throttle
 * @value 405 RC Yaw
 * @value 406 RC Flaps
 * @value 407 RC AUX 1
 * @value 408 RC AUX 2
 * @value 409 RC AUX 3
 * @value 410 RC AUX 4
 * @value 411 RC AUX 5
 * @value 412 RC AUX 6
 * @value 420 Gimbal Roll
 * @value 421 Gimbal Pitch
 * @value 422 Gimbal Yaw
 * @value 430 Gripper
 * @value 440 Landing Gear Wheel
 * @value 450 IC Engine Ignition
 * @value 451 IC Engine Throttle
 * @value 452 IC Engine Choke
 * @value 453 IC Engine Starter
 * @value 2000 Camera Trigger
 * @value 2032 Camera Capture
 * @value 2064 PPS Input
 * @value 2070 RPM Input
 */
PARAM_DEFINE_INT32(PWM_MAIN_FUNC2, 0);

/**
 * PWM Main 3 Output Function
 *
 * Select what should be output on PWM Main 3.
 * 
 * The default failsafe value is set according to the selected function:
 * - 'Min' for ConstantMin
 * - 'Max' for ConstantMax
 * - 'Max' for Parachute
 * - ('Max'+'Min')/2 for Servos
 * - 'Disarmed' for the rest
 * 
 *
 * @group Actuator Outputs
 * @value 0 Disabled
 * @value 1 Constant Min
 * @value 2 Constant Max
 * @value 101 Motor 1
 * @value 102 Motor 2
 * @value 103 Motor 3
 * @value 104 Motor 4
 * @value 105 Motor 5
 * @value 106 Motor 6
 * @value 107 Motor 7
 * @value 108 Motor 8
 * @value 109 Motor 9
 * @value 110 Motor 10
 * @value 111 Motor 11
 * @value 112 Motor 12
 * @value 201 Servo 1
 * @value 202 Servo 2
 * @value 203 Servo 3
 * @value 204 Servo 4
 * @value 205 Servo 5
 * @value 206 Servo 6
 * @value 207 Servo 7
 * @value 208 Servo 8
 * @value 301 Peripheral via Actuator Set 1
 * @value 302 Peripheral via Actuator Set 2
 * @value 303 Peripheral via Actuator Set 3
 * @value 304 Peripheral via Actuator Set 4
 * @value 305 Peripheral via Actuator Set 5
 * @value 306 Peripheral via Actuator Set 6
 * @value 400 Landing Gear
 * @value 401 Parachute
 * @value 402 RC Roll
 * @value 403 RC Pitch
 * @value 404 RC Throttle
 * @value 405 RC Yaw
 * @value 406 RC Flaps
 * @value 407 RC AUX 1
 * @value 408 RC AUX 2
 * @value 409 RC AUX 3
 * @value 410 RC AUX 4
 * @value 411 RC AUX 5
 * @value 412 RC AUX 6
 * @value 420 Gimbal Roll
 * @value 421 Gimbal Pitch
 * @value 422 Gimbal Yaw
 * @value 430 Gripper
 * @value 440 Landing Gear Wheel
 * @value 450 IC Engine Ignition
 * @value 451 IC Engine Throttle
 * @value 452 IC Engine Choke
 * @value 453 IC Engine Starter
 * @value 2000 Camera Trigger
 * @value 2032 Camera Capture
 * @value 2064 PPS Input
 * @value 2070 RPM Input
 */
PARAM_DEFINE_INT32(PWM_MAIN_FUNC3, 0);

/**
 * PWM Main 4 Output Function
 *
 * Select what should be output on PWM Main 4.
 * 
 * The default failsafe value is set according to the selected function:
 * - 'Min' for ConstantMin
 * - 'Max' for ConstantMax
 * - 'Max' for Parachute
 * - ('Max'+'Min')/2 for Servos
 * - 'Disarmed' for the rest
 * 
 *
 * @group Actuator Outputs
 * @value 0 Disabled
 * @value 1 Constant Min
 * @value 2 Constant Max
 * @value 101 Motor 1
 * @value 102 Motor 2
 * @value 103 Motor 3
 * @value 104 Motor 4
 * @value 105 Motor 5
 * @value 106 Motor 6
 * @value 107 Motor 7
 * @value 108 Motor 8
 * @value 109 Motor 9
 * @value 110 Motor 10
 * @value 111 Motor 11
 * @value 112 Motor 12
 * @value 201 Servo 1
 * @value 202 Servo 2
 * @value 203 Servo 3
 * @value 204 Servo 4
 * @value 205 Servo 5
 * @value 206 Servo 6
 * @value 207 Servo 7
 * @value 208 Servo 8
 * @value 301 Peripheral via Actuator Set 1
 * @value 302 Peripheral via Actuator Set 2
 * @value 303 Peripheral via Actuator Set 3
 * @value 304 Peripheral via Actuator Set 4
 * @value 305 Peripheral via Actuator Set 5
 * @value 306 Peripheral via Actuator Set 6
 * @value 400 Landing Gear
 * @value 401 Parachute
 * @value 402 RC Roll
 * @value 403 RC Pitch
 * @value 404 RC Throttle
 * @value 405 RC Yaw
 * @value 406 RC Flaps
 * @value 407 RC AUX 1
 * @value 408 RC AUX 2
 * @value 409 RC AUX 3
 * @value 410 RC AUX 4
 * @value 411 RC AUX 5
 * @value 412 RC AUX 6
 * @value 420 Gimbal Roll
 * @value 421 Gimbal Pitch
 * @value 422 Gimbal Yaw
 * @value 430 Gripper
 * @value 440 Landing Gear Wheel
 * @value 450 IC Engine Ignition
 * @value 451 IC Engine Throttle
 * @value 452 IC Engine Choke
 * @value 453 IC Engine Starter
 * @value 2000 Camera Trigger
 * @value 2032 Camera Capture
 * @value 2064 PPS Input
 * @value 2070 RPM Input
 */
PARAM_DEFINE_INT32(PWM_MAIN_FUNC4, 0);

/**
 * PWM Main 5 Output Function
 *
 * Select what should be output on PWM Main 5.
 * 
 * The default failsafe value is set according to the selected function:
 * - 'Min' for ConstantMin
 * - 'Max' for ConstantMax
 * - 'Max' for Parachute
 * - ('Max'+'Min')/2 for Servos
 * - 'Disarmed' for the rest
 * 
 *
 * @group Actuator Outputs
 * @value 0 Disabled
 * @value 1 Constant Min
 * @value 2 Constant Max
 * @value 101 Motor 1
 * @value 102 Motor 2
 * @value 103 Motor 3
 * @value 104 Motor 4
 * @value 105 Motor 5
 * @value 106 Motor 6
 * @value 107 Motor 7
 * @value 108 Motor 8
 * @value 109 Motor 9
 * @value 110 Motor 10
 * @value 111 Motor 11
 * @value 112 Motor 12
 * @value 201 Servo 1
 * @value 202 Servo 2
 * @value 203 Servo 3
 * @value 204 Servo 4
 * @value 205 Servo 5
 * @value 206 Servo 6
 * @value 207 Servo 7
 * @value 208 Servo 8
 * @value 301 Peripheral via Actuator Set 1
 * @value 302 Peripheral via Actuator Set 2
 * @value 303 Peripheral via Actuator Set 3
 * @value 304 Peripheral via Actuator Set 4
 * @value 305 Peripheral via Actuator Set 5
 * @value 306 Peripheral via Actuator Set 6
 * @value 400 Landing Gear
 * @value 401 Parachute
 * @value 402 RC Roll
 * @value 403 RC Pitch
 * @value 404 RC Throttle
 * @value 405 RC Yaw
 * @value 406 RC Flaps
 * @value 407 RC AUX 1
 * @value 408 RC AUX 2
 * @value 409 RC AUX 3
 * @value 410 RC AUX 4
 * @value 411 RC AUX 5
 * @value 412 RC AUX 6
 * @value 420 Gimbal Roll
 * @value 421 Gimbal Pitch
 * @value 422 Gimbal Yaw
 * @value 430 Gripper
 * @value 440 Landing Gear Wheel
 * @value 450 IC Engine Ignition
 * @value 451 IC Engine Throttle
 * @value 452 IC Engine Choke
 * @value 453 IC Engine Starter
 * @value 2000 Camera Trigger
 * @value 2032 Camera Capture
 * @value 2064 PPS Input
 * @value 2070 RPM Input
 */
PARAM_DEFINE_INT32(PWM_MAIN_FUNC5, 0);

/**
 * PWM Main 6 Output Function
 *
 * Select what should be output on PWM Main 6.
 * 
 * The default failsafe value is set according to the selected function:
 * - 'Min' for ConstantMin
 * - 'Max' for ConstantMax
 * - 'Max' for Parachute
 * - ('Max'+'Min')/2 for Servos
 * - 'Disarmed' for the rest
 * 
 *
 * @group Actuator Outputs
 * @value 0 Disabled
 * @value 1 Constant Min
 * @value 2 Constant Max
 * @value 101 Motor 1
 * @value 102 Motor 2
 * @value 103 Motor 3
 * @value 104 Motor 4
 * @value 105 Motor 5
 * @value 106 Motor 6
 * @value 107 Motor 7
 * @value 108 Motor 8
 * @value 109 Motor 9
 * @value 110 Motor 10
 * @value 111 Motor 11
 * @value 112 Motor 12
 * @value 201 Servo 1
 * @value 202 Servo 2
 * @value 203 Servo 3
 * @value 204 Servo 4
 * @value 205 Servo 5
 * @value 206 Servo 6
 * @value 207 Servo 7
 * @value 208 Servo 8
 * @value 301 Peripheral via Actuator Set 1
 * @value 302 Peripheral via Actuator Set 2
 * @value 303 Peripheral via Actuator Set 3
 * @value 304 Peripheral via Actuator Set 4
 * @value 305 Peripheral via Actuator Set 5
 * @value 306 Peripheral via Actuator Set 6
 * @value 400 Landing Gear
 * @value 401 Parachute
 * @value 402 RC Roll
 * @value 403 RC Pitch
 * @value 404 RC Throttle
 * @value 405 RC Yaw
 * @value 406 RC Flaps
 * @value 407 RC AUX 1
 * @value 408 RC AUX 2
 * @value 409 RC AUX 3
 * @value 410 RC AUX 4
 * @value 411 RC AUX 5
 * @value 412 RC AUX 6
 * @value 420 Gimbal Roll
 * @value 421 Gimbal Pitch
 * @value 422 Gimbal Yaw
 * @value 430 Gripper
 * @value 440 Landing Gear Wheel
 * @value 450 IC Engine Ignition
 * @value 451 IC Engine Throttle
 * @value 452 IC Engine Choke
 * @value 453 IC Engine Starter
 * @value 2000 Camera Trigger
 * @value 2032 Camera Capture
 * @value 2064 PPS Input
 * @value 2070 RPM Input
 */
PARAM_DEFINE_INT32(PWM_MAIN_FUNC6, 0);

/**
 * PWM Main 7 Output Function
 *
 * Select what should be output on PWM Main 7.
 * 
 * The default failsafe value is set according to the selected function:
 * - 'Min' for ConstantMin
 * - 'Max' for ConstantMax
 * - 'Max' for Parachute
 * - ('Max'+'Min')/2 for Servos
 * - 'Disarmed' for the rest
 * 
 *
 * @group Actuator Outputs
 * @value 0 Disabled
 * @value 1 Constant Min
 * @value 2 Constant Max
 * @value 101 Motor 1
 * @value 102 Motor 2
 * @value 103 Motor 3
 * @value 104 Motor 4
 * @value 105 Motor 5
 * @value 106 Motor 6
 * @value 107 Motor 7
 * @value 108 Motor 8
 * @value 109 Motor 9
 * @value 110 Motor 10
 * @value 111 Motor 11
 * @value 112 Motor 12
 * @value 201 Servo 1
 * @value 202 Servo 2
 * @value 203 Servo 3
 * @value 204 Servo 4
 * @value 205 Servo 5
 * @value 206 Servo 6
 * @value 207 Servo 7
 * @value 208 Servo 8
 * @value 301 Peripheral via Actuator Set 1
 * @value 302 Peripheral via Actuator Set 2
 * @value 303 Peripheral via Actuator Set 3
 * @value 304 Peripheral via Actuator Set 4
 * @value 305 Peripheral via Actuator Set 5
 * @value 306 Peripheral via Actuator Set 6
 * @value 400 Landing Gear
 * @value 401 Parachute
 * @value 402 RC Roll
 * @value 403 RC Pitch
 * @value 404 RC Throttle
 * @value 405 RC Yaw
 * @value 406 RC Flaps
 * @value 407 RC AUX 1
 * @value 408 RC AUX 2
 * @value 409 RC AUX 3
 * @value 410 RC AUX 4
 * @value 411 RC AUX 5
 * @value 412 RC AUX 6
 * @value 420 Gimbal Roll
 * @value 421 Gimbal Pitch
 * @value 422 Gimbal Yaw
 * @value 430 Gripper
 * @value 440 Landing Gear Wheel
 * @value 450 IC Engine Ignition
 * @value 451 IC Engine Throttle
 * @value 452 IC Engine Choke
 * @value 453 IC Engine Starter
 * @value 2000 Camera Trigger
 * @value 2032 Camera Capture
 * @value 2064 PPS Input
 * @value 2070 RPM Input
 */
PARAM_DEFINE_INT32(PWM_MAIN_FUNC7, 0);

/**
 * PWM Main 8 Output Function
 *
 * Select what should be output on PWM Main 8.
 * 
 * The default failsafe value is set according to the selected function:
 * - 'Min' for ConstantMin
 * - 'Max' for ConstantMax
 * - 'Max' for Parachute
 * - ('Max'+'Min')/2 for Servos
 * - 'Disarmed' for the rest
 * 
 *
 * @group Actuator Outputs
 * @value 0 Disabled
 * @value 1 Constant Min
 * @value 2 Constant Max
 * @value 101 Motor 1
 * @value 102 Motor 2
 * @value 103 Motor 3
 * @value 104 Motor 4
 * @value 105 Motor 5
 * @value 106 Motor 6
 * @value 107 Motor 7
 * @value 108 Motor 8
 * @value 109 Motor 9
 * @value 110 Motor 10
 * @value 111 Motor 11
 * @value 112 Motor 12
 * @value 201 Servo 1
 * @value 202 Servo 2
 * @value 203 Servo 3
 * @value 204 Servo 4
 * @value 205 Servo 5
 * @value 206 Servo 6
 * @value 207 Servo 7
 * @value 208 Servo 8
 * @value 301 Peripheral via Actuator Set 1
 * @value 302 Peripheral via Actuator Set 2
 * @value 303 Peripheral via Actuator Set 3
 * @value 304 Peripheral via Actuator Set 4
 * @value 305 Peripheral via Actuator Set 5
 * @value 306 Peripheral via Actuator Set 6
 * @value 400 Landing Gear
 * @value 401 Parachute
 * @value 402 RC Roll
 * @value 403 RC Pitch
 * @value 404 RC Throttle
 * @value 405 RC Yaw
 * @value 406 RC Flaps
 * @value 407 RC AUX 1
 * @value 408 RC AUX 2
 * @value 409 RC AUX 3
 * @value 410 RC AUX 4
 * @value 411 RC AUX 5
 * @value 412 RC AUX 6
 * @value 420 Gimbal Roll
 * @value 421 Gimbal Pitch
 * @value 422 Gimbal Yaw
 * @value 430 Gripper
 * @value 440 Landing Gear Wheel
 * @value 450 IC Engine Ignition
 * @value 451 IC Engine Throttle
 * @value 452 IC Engine Choke
 * @value 453 IC Engine Starter
 * @value 2000 Camera Trigger
 * @value 2032 Camera Capture
 * @value 2064 PPS Input
 * @value 2070 RPM Input
 */
PARAM_DEFINE_INT32(PWM_MAIN_FUNC8, 0);

/**
 * PWM Main 1 Disarmed Value
 *
 * This is the output value that is set when not armed.
 * 
 * Note that non-motor outputs might already be active in prearm state if COM_PREARM_MODE is set.
 * 
 *
 * @group Actuator Outputs
 * @min 800
 * @max 2200
 */
PARAM_DEFINE_INT32(PWM_MAIN_DIS1, 1000);

/**
 * PWM Main 2 Disarmed Value
 *
 * This is the output value that is set when not armed.
 * 
 * Note that non-motor outputs might already be active in prearm state if COM_PREARM_MODE is set.
 * 
 *
 * @group Actuator Outputs
 * @min 800
 * @max 2200
 */
PARAM_DEFINE_INT32(PWM_MAIN_DIS2, 1000);

/**
 * PWM Main 3 Disarmed Value
 *
 * This is the output value that is set when not armed.
 * 
 * Note that non-motor outputs might already be active in prearm state if COM_PREARM_MODE is set.
 * 
 *
 * @group Actuator Outputs
 * @min 800
 * @max 2200
 */
PARAM_DEFINE_INT32(PWM_MAIN_DIS3, 1000);

/**
 * PWM Main 4 Disarmed Value
 *
 * This is the output value that is set when not armed.
 * 
 * Note that non-motor outputs might already be active in prearm state if COM_PREARM_MODE is set.
 * 
 *
 * @group Actuator Outputs
 * @min 800
 * @max 2200
 */
PARAM_DEFINE_INT32(PWM_MAIN_DIS4, 1000);

/**
 * PWM Main 5 Disarmed Value
 *
 * This is the output value that is set when not armed.
 * 
 * Note that non-motor outputs might already be active in prearm state if COM_PREARM_MODE is set.
 * 
 *
 * @group Actuator Outputs
 * @min 800
 * @max 2200
 */
PARAM_DEFINE_INT32(PWM_MAIN_DIS5, 1000);

/**
 * PWM Main 6 Disarmed Value
 *
 * This is the output value that is set when not armed.
 * 
 * Note that non-motor outputs might already be active in prearm state if COM_PREARM_MODE is set.
 * 
 *
 * @group Actuator Outputs
 * @min 800
 * @max 2200
 */
PARAM_DEFINE_INT32(PWM_MAIN_DIS6, 1000);

/**
 * PWM Main 7 Disarmed Value
 *
 * This is the output value that is set when not armed.
 * 
 * Note that non-motor outputs might already be active in prearm state if COM_PREARM_MODE is set.
 * 
 *
 * @group Actuator Outputs
 * @min 800
 * @max 2200
 */
PARAM_DEFINE_INT32(PWM_MAIN_DIS7, 1000);

/**
 * PWM Main 8 Disarmed Value
 *
 * This is the output value that is set when not armed.
 * 
 * Note that non-motor outputs might already be active in prearm state if COM_PREARM_MODE is set.
 * 
 *
 * @group Actuator Outputs
 * @min 800
 * @max 2200
 */
PARAM_DEFINE_INT32(PWM_MAIN_DIS8, 1000);

/**
 * PWM Main 1 Minimum Value
 *
 * Minimum output value (when not disarmed).
 * 
 *
 * @group Actuator Outputs
 * @min 800
 * @max 1400
 */
PARAM_DEFINE_INT32(PWM_MAIN_MIN1, 1000);

/**
 * PWM Main 2 Minimum Value
 *
 * Minimum output value (when not disarmed).
 * 
 *
 * @group Actuator Outputs
 * @min 800
 * @max 1400
 */
PARAM_DEFINE_INT32(PWM_MAIN_MIN2, 1000);

/**
 * PWM Main 3 Minimum Value
 *
 * Minimum output value (when not disarmed).
 * 
 *
 * @group Actuator Outputs
 * @min 800
 * @max 1400
 */
PARAM_DEFINE_INT32(PWM_MAIN_MIN3, 1000);

/**
 * PWM Main 4 Minimum Value
 *
 * Minimum output value (when not disarmed).
 * 
 *
 * @group Actuator Outputs
 * @min 800
 * @max 1400
 */
PARAM_DEFINE_INT32(PWM_MAIN_MIN4, 1000);

/**
 * PWM Main 5 Minimum Value
 *
 * Minimum output value (when not disarmed).
 * 
 *
 * @group Actuator Outputs
 * @min 800
 * @max 1400
 */
PARAM_DEFINE_INT32(PWM_MAIN_MIN5, 1000);

/**
 * PWM Main 6 Minimum Value
 *
 * Minimum output value (when not disarmed).
 * 
 *
 * @group Actuator Outputs
 * @min 800
 * @max 1400
 */
PARAM_DEFINE_INT32(PWM_MAIN_MIN6, 1000);

/**
 * PWM Main 7 Minimum Value
 *
 * Minimum output value (when not disarmed).
 * 
 *
 * @group Actuator Outputs
 * @min 800
 * @max 1400
 */
PARAM_DEFINE_INT32(PWM_MAIN_MIN7, 1000);

/**
 * PWM Main 8 Minimum Value
 *
 * Minimum output value (when not disarmed).
 * 
 *
 * @group Actuator Outputs
 * @min 800
 * @max 1400
 */
PARAM_DEFINE_INT32(PWM_MAIN_MIN8, 1000);

/**
 * PWM Main 1 Maximum Value
 *
 * Maxmimum output value (when not disarmed).
 * 
 *
 * @group Actuator Outputs
 * @min 1600
 * @max 2200
 */
PARAM_DEFINE_INT32(PWM_MAIN_MAX1, 2000);

/**
 * PWM Main 2 Maximum Value
 *
 * Maxmimum output value (when not disarmed).
 * 
 *
 * @group Actuator Outputs
 * @min 1600
 * @max 2200
 */
PARAM_DEFINE_INT32(PWM_MAIN_MAX2, 2000);

/**
 * PWM Main 3 Maximum Value
 *
 * Maxmimum output value (when not disarmed).
 * 
 *
 * @group Actuator Outputs
 * @min 1600
 * @max 2200
 */
PARAM_DEFINE_INT32(PWM_MAIN_MAX3, 2000);

/**
 * PWM Main 4 Maximum Value
 *
 * Maxmimum output value (when not disarmed).
 * 
 *
 * @group Actuator Outputs
 * @min 1600
 * @max 2200
 */
PARAM_DEFINE_INT32(PWM_MAIN_MAX4, 2000);

/**
 * PWM Main 5 Maximum Value
 *
 * Maxmimum output value (when not disarmed).
 * 
 *
 * @group Actuator Outputs
 * @min 1600
 * @max 2200
 */
PARAM_DEFINE_INT32(PWM_MAIN_MAX5, 2000);

/**
 * PWM Main 6 Maximum Value
 *
 * Maxmimum output value (when not disarmed).
 * 
 *
 * @group Actuator Outputs
 * @min 1600
 * @max 2200
 */
PARAM_DEFINE_INT32(PWM_MAIN_MAX6, 2000);

/**
 * PWM Main 7 Maximum Value
 *
 * Maxmimum output value (when not disarmed).
 * 
 *
 * @group Actuator Outputs
 * @min 1600
 * @max 2200
 */
PARAM_DEFINE_INT32(PWM_MAIN_MAX7, 2000);

/**
 * PWM Main 8 Maximum Value
 *
 * Maxmimum output value (when not disarmed).
 * 
 *
 * @group Actuator Outputs
 * @min 1600
 * @max 2200
 */
PARAM_DEFINE_INT32(PWM_MAIN_MAX8, 2000);

/**
 * PWM Main 1 Failsafe Value
 *
 * This is the output value that is set when in failsafe mode.
 * 
 * When set to -1 (default), the value depends on the function (see PWM_MAIN_FUNC1).
 * 
 *
 * @group Actuator Outputs
 * @min -1
 * @max 2200
 */
PARAM_DEFINE_INT32(PWM_MAIN_FAIL1, -1);

/**
 * PWM Main 2 Failsafe Value
 *
 * This is the output value that is set when in failsafe mode.
 * 
 * When set to -1 (default), the value depends on the function (see PWM_MAIN_FUNC2).
 * 
 *
 * @group Actuator Outputs
 * @min -1
 * @max 2200
 */
PARAM_DEFINE_INT32(PWM_MAIN_FAIL2, -1);

/**
 * PWM Main 3 Failsafe Value
 *
 * This is the output value that is set when in failsafe mode.
 * 
 * When set to -1 (default), the value depends on the function (see PWM_MAIN_FUNC3).
 * 
 *
 * @group Actuator Outputs
 * @min -1
 * @max 2200
 */
PARAM_DEFINE_INT32(PWM_MAIN_FAIL3, -1);

/**
 * PWM Main 4 Failsafe Value
 *
 * This is the output value that is set when in failsafe mode.
 * 
 * When set to -1 (default), the value depends on the function (see PWM_MAIN_FUNC4).
 * 
 *
 * @group Actuator Outputs
 * @min -1
 * @max 2200
 */
PARAM_DEFINE_INT32(PWM_MAIN_FAIL4, -1);

/**
 * PWM Main 5 Failsafe Value
 *
 * This is the output value that is set when in failsafe mode.
 * 
 * When set to -1 (default), the value depends on the function (see PWM_MAIN_FUNC5).
 * 
 *
 * @group Actuator Outputs
 * @min -1
 * @max 2200
 */
PARAM_DEFINE_INT32(PWM_MAIN_FAIL5, -1);

/**
 * PWM Main 6 Failsafe Value
 *
 * This is the output value that is set when in failsafe mode.
 * 
 * When set to -1 (default), the value depends on the function (see PWM_MAIN_FUNC6).
 * 
 *
 * @group Actuator Outputs
 * @min -1
 * @max 2200
 */
PARAM_DEFINE_INT32(PWM_MAIN_FAIL6, -1);

/**
 * PWM Main 7 Failsafe Value
 *
 * This is the output value that is set when in failsafe mode.
 * 
 * When set to -1 (default), the value depends on the function (see PWM_MAIN_FUNC7).
 * 
 *
 * @group Actuator Outputs
 * @min -1
 * @max 2200
 */
PARAM_DEFINE_INT32(PWM_MAIN_FAIL7, -1);

/**
 * PWM Main 8 Failsafe Value
 *
 * This is the output value that is set when in failsafe mode.
 * 
 * When set to -1 (default), the value depends on the function (see PWM_MAIN_FUNC8).
 * 
 *
 * @group Actuator Outputs
 * @min -1
 * @max 2200
 */
PARAM_DEFINE_INT32(PWM_MAIN_FAIL8, -1);

/**
 * Reverse Output Range for PWM MAIN
 *
 * Allows to reverse the output range for each channel.
 * Note: this is only useful for servos.
 * 
 *
 * @group Actuator Outputs
 * @bit 0 PWM Main 1
 * @bit 1 PWM Main 2
 * @bit 2 PWM Main 3
 * @bit 3 PWM Main 4
 * @bit 4 PWM Main 5
 * @bit 5 PWM Main 6
 * @bit 6 PWM Main 7
 * @bit 7 PWM Main 8
 * @min 0
 * @max 255
 */
PARAM_DEFINE_INT32(PWM_MAIN_REV, 0);

/**
 * RC input protocol
 *
 * Select your RC input protocol or auto to scan.
 * 
 *
 * @group RC Input
 * @value -1 Auto
 * @value 0 None
 * @value 1 PPM
 * @value 2 SBUS
 * @value 3 DSM
 * @value 4 ST24
 * @value 5 SUMD
 * @value 6 CRSF
 * @value 7 GHST
 * @category System
 * @min -1
 * @max 7
 */
PARAM_DEFINE_INT32(RC_INPUT_PROTO, -1);

/**
 * Battery 1 voltage divider (V divider)
 *
 * This is the divider from battery 1 voltage to ADC voltage.
 * If using e.g. Mauch power modules the value from the datasheet
 * can be applied straight here. A value of -1 means to use
 * the board default.
 * 
 *
 * @group Battery Calibration
 * @decimal 8
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(BAT1_V_DIV, -1.0);

/**
 * Battery 2 voltage divider (V divider)
 *
 * This is the divider from battery 2 voltage to ADC voltage.
 * If using e.g. Mauch power modules the value from the datasheet
 * can be applied straight here. A value of -1 means to use
 * the board default.
 * 
 *
 * @group Battery Calibration
 * @decimal 8
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(BAT2_V_DIV, -1.0);

/**
 * Battery 1 current per volt (A/V)
 *
 * The voltage seen by the ADC multiplied by this factor
 * will determine the battery current. A value of -1 means to use
 * the board default.
 * 
 *
 * @group Battery Calibration
 * @decimal 8
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(BAT1_A_PER_V, -1.0);

/**
 * Battery 2 current per volt (A/V)
 *
 * The voltage seen by the ADC multiplied by this factor
 * will determine the battery current. A value of -1 means to use
 * the board default.
 * 
 *
 * @group Battery Calibration
 * @decimal 8
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(BAT2_A_PER_V, -1.0);

/**
 * Battery 1 Voltage ADC Channel
 *
 * This parameter specifies the ADC channel used to monitor voltage of main power battery.
 * A value of -1 means to use the board default.
 * 
 *
 * @group Battery Calibration
 * @reboot_required True
 */
PARAM_DEFINE_INT32(BAT1_V_CHANNEL, -1);

/**
 * Battery 2 Voltage ADC Channel
 *
 * This parameter specifies the ADC channel used to monitor voltage of main power battery.
 * A value of -1 means to use the board default.
 * 
 *
 * @group Battery Calibration
 * @reboot_required True
 */
PARAM_DEFINE_INT32(BAT2_V_CHANNEL, -1);

/**
 * Battery 1 Current ADC Channel
 *
 * This parameter specifies the ADC channel used to monitor current of main power battery.
 * A value of -1 means to use the board default.
 * 
 *
 * @group Battery Calibration
 * @reboot_required True
 */
PARAM_DEFINE_INT32(BAT1_I_CHANNEL, -1);

/**
 * Battery 2 Current ADC Channel
 *
 * This parameter specifies the ADC channel used to monitor current of main power battery.
 * A value of -1 means to use the board default.
 * 
 *
 * @group Battery Calibration
 * @reboot_required True
 */
PARAM_DEFINE_INT32(BAT2_I_CHANNEL, -1);

/**
 * Battery 1 idle current overwrite
 *
 * This parameter allows to overwrite the current measured during
 * idle (unarmed) state with a user-defined constant value (expressed in amperes).
 * When the system is armed, the measured current is used. This is useful
 * because on certain ESCs current measurements are inaccurate in case of no load.
 * Negative values are ignored and will cause the measured current to be used.
 * The default value of 0 disables the overwrite, in which case the measured value
 * is always used.
 * 
 *
 * @group Battery Calibration
 * @decimal 8
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(BAT1_I_OVERWRITE, 0);

/**
 * Battery 2 idle current overwrite
 *
 * This parameter allows to overwrite the current measured during
 * idle (unarmed) state with a user-defined constant value (expressed in amperes).
 * When the system is armed, the measured current is used. This is useful
 * because on certain ESCs current measurements are inaccurate in case of no load.
 * Negative values are ignored and will cause the measured current to be used.
 * The default value of 0 disables the overwrite, in which case the measured value
 * is always used.
 * 
 *
 * @group Battery Calibration
 * @decimal 8
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(BAT2_I_OVERWRITE, 0);

/**
 * External mode identifier 0
 *
 * This parameter is automatically set to identify external modes. It ensures that modes
 * get assigned to the same index independent from their startup order,
 * which is required when mapping an external mode to an RC switch.
 * 
 *
 * @group Commander
 * @category System
 * @volatile True
 */
PARAM_DEFINE_INT32(COM_MODE0_HASH, 0);

/**
 * External mode identifier 1
 *
 * This parameter is automatically set to identify external modes. It ensures that modes
 * get assigned to the same index independent from their startup order,
 * which is required when mapping an external mode to an RC switch.
 * 
 *
 * @group Commander
 * @category System
 * @volatile True
 */
PARAM_DEFINE_INT32(COM_MODE1_HASH, 0);

/**
 * External mode identifier 2
 *
 * This parameter is automatically set to identify external modes. It ensures that modes
 * get assigned to the same index independent from their startup order,
 * which is required when mapping an external mode to an RC switch.
 * 
 *
 * @group Commander
 * @category System
 * @volatile True
 */
PARAM_DEFINE_INT32(COM_MODE2_HASH, 0);

/**
 * External mode identifier 3
 *
 * This parameter is automatically set to identify external modes. It ensures that modes
 * get assigned to the same index independent from their startup order,
 * which is required when mapping an external mode to an RC switch.
 * 
 *
 * @group Commander
 * @category System
 * @volatile True
 */
PARAM_DEFINE_INT32(COM_MODE3_HASH, 0);

/**
 * External mode identifier 4
 *
 * This parameter is automatically set to identify external modes. It ensures that modes
 * get assigned to the same index independent from their startup order,
 * which is required when mapping an external mode to an RC switch.
 * 
 *
 * @group Commander
 * @category System
 * @volatile True
 */
PARAM_DEFINE_INT32(COM_MODE4_HASH, 0);

/**
 * External mode identifier 5
 *
 * This parameter is automatically set to identify external modes. It ensures that modes
 * get assigned to the same index independent from their startup order,
 * which is required when mapping an external mode to an RC switch.
 * 
 *
 * @group Commander
 * @category System
 * @volatile True
 */
PARAM_DEFINE_INT32(COM_MODE5_HASH, 0);

/**
 * External mode identifier 6
 *
 * This parameter is automatically set to identify external modes. It ensures that modes
 * get assigned to the same index independent from their startup order,
 * which is required when mapping an external mode to an RC switch.
 * 
 *
 * @group Commander
 * @category System
 * @volatile True
 */
PARAM_DEFINE_INT32(COM_MODE6_HASH, 0);

/**
 * External mode identifier 7
 *
 * This parameter is automatically set to identify external modes. It ensures that modes
 * get assigned to the same index independent from their startup order,
 * which is required when mapping an external mode to an RC switch.
 * 
 *
 * @group Commander
 * @category System
 * @volatile True
 */
PARAM_DEFINE_INT32(COM_MODE7_HASH, 0);

/**
 * Mode slot 1
 *
 * If the main switch channel is in this range the
 * selected flight mode will be applied.
 * 
 *
 * @group Commander
 * @value -1 Unassigned
 * @value 0 Manual
 * @value 1 Altitude
 * @value 2 Position
 * @value 9 Position Slow
 * @value 3 Mission
 * @value 4 Hold
 * @value 10 Takeoff
 * @value 11 Land
 * @value 5 Return
 * @value 6 Acro
 * @value 7 Offboard
 * @value 8 Stabilized
 * @value 12 Follow Me
 * @value 13 Precision Land
 * @value 100 External Mode 1
 * @value 101 External Mode 2
 * @value 102 External Mode 3
 * @value 103 External Mode 4
 * @value 104 External Mode 5
 * @value 105 External Mode 6
 * @value 106 External Mode 7
 * @value 107 External Mode 8
 */
PARAM_DEFINE_INT32(COM_FLTMODE1, -1);

/**
 * Mode slot 2
 *
 * If the main switch channel is in this range the
 * selected flight mode will be applied.
 * 
 *
 * @group Commander
 * @value -1 Unassigned
 * @value 0 Manual
 * @value 1 Altitude
 * @value 2 Position
 * @value 9 Position Slow
 * @value 3 Mission
 * @value 4 Hold
 * @value 10 Takeoff
 * @value 11 Land
 * @value 5 Return
 * @value 6 Acro
 * @value 7 Offboard
 * @value 8 Stabilized
 * @value 12 Follow Me
 * @value 13 Precision Land
 * @value 100 External Mode 1
 * @value 101 External Mode 2
 * @value 102 External Mode 3
 * @value 103 External Mode 4
 * @value 104 External Mode 5
 * @value 105 External Mode 6
 * @value 106 External Mode 7
 * @value 107 External Mode 8
 */
PARAM_DEFINE_INT32(COM_FLTMODE2, -1);

/**
 * Mode slot 3
 *
 * If the main switch channel is in this range the
 * selected flight mode will be applied.
 * 
 *
 * @group Commander
 * @value -1 Unassigned
 * @value 0 Manual
 * @value 1 Altitude
 * @value 2 Position
 * @value 9 Position Slow
 * @value 3 Mission
 * @value 4 Hold
 * @value 10 Takeoff
 * @value 11 Land
 * @value 5 Return
 * @value 6 Acro
 * @value 7 Offboard
 * @value 8 Stabilized
 * @value 12 Follow Me
 * @value 13 Precision Land
 * @value 100 External Mode 1
 * @value 101 External Mode 2
 * @value 102 External Mode 3
 * @value 103 External Mode 4
 * @value 104 External Mode 5
 * @value 105 External Mode 6
 * @value 106 External Mode 7
 * @value 107 External Mode 8
 */
PARAM_DEFINE_INT32(COM_FLTMODE3, -1);

/**
 * Mode slot 4
 *
 * If the main switch channel is in this range the
 * selected flight mode will be applied.
 * 
 *
 * @group Commander
 * @value -1 Unassigned
 * @value 0 Manual
 * @value 1 Altitude
 * @value 2 Position
 * @value 9 Position Slow
 * @value 3 Mission
 * @value 4 Hold
 * @value 10 Takeoff
 * @value 11 Land
 * @value 5 Return
 * @value 6 Acro
 * @value 7 Offboard
 * @value 8 Stabilized
 * @value 12 Follow Me
 * @value 13 Precision Land
 * @value 100 External Mode 1
 * @value 101 External Mode 2
 * @value 102 External Mode 3
 * @value 103 External Mode 4
 * @value 104 External Mode 5
 * @value 105 External Mode 6
 * @value 106 External Mode 7
 * @value 107 External Mode 8
 */
PARAM_DEFINE_INT32(COM_FLTMODE4, -1);

/**
 * Mode slot 5
 *
 * If the main switch channel is in this range the
 * selected flight mode will be applied.
 * 
 *
 * @group Commander
 * @value -1 Unassigned
 * @value 0 Manual
 * @value 1 Altitude
 * @value 2 Position
 * @value 9 Position Slow
 * @value 3 Mission
 * @value 4 Hold
 * @value 10 Takeoff
 * @value 11 Land
 * @value 5 Return
 * @value 6 Acro
 * @value 7 Offboard
 * @value 8 Stabilized
 * @value 12 Follow Me
 * @value 13 Precision Land
 * @value 100 External Mode 1
 * @value 101 External Mode 2
 * @value 102 External Mode 3
 * @value 103 External Mode 4
 * @value 104 External Mode 5
 * @value 105 External Mode 6
 * @value 106 External Mode 7
 * @value 107 External Mode 8
 */
PARAM_DEFINE_INT32(COM_FLTMODE5, -1);

/**
 * Mode slot 6
 *
 * If the main switch channel is in this range the
 * selected flight mode will be applied.
 * 
 *
 * @group Commander
 * @value -1 Unassigned
 * @value 0 Manual
 * @value 1 Altitude
 * @value 2 Position
 * @value 9 Position Slow
 * @value 3 Mission
 * @value 4 Hold
 * @value 10 Takeoff
 * @value 11 Land
 * @value 5 Return
 * @value 6 Acro
 * @value 7 Offboard
 * @value 8 Stabilized
 * @value 12 Follow Me
 * @value 13 Precision Land
 * @value 100 External Mode 1
 * @value 101 External Mode 2
 * @value 102 External Mode 3
 * @value 103 External Mode 4
 * @value 104 External Mode 5
 * @value 105 External Mode 6
 * @value 106 External Mode 7
 * @value 107 External Mode 8
 */
PARAM_DEFINE_INT32(COM_FLTMODE6, -1);

/**
 * Airframe selection
 *
 * Defines which mixer implementation to use.
 * Some are generic, while others are specifically fit to a certain vehicle with a fixed set of actuators.
 * 
 * 'Custom' should only be used if noting else can be used.
 * 
 *
 * @group Geometry
 * @value 0 Multirotor
 * @value 1 Fixed-wing
 * @value 2 Standard VTOL
 * @value 3 Tiltrotor VTOL
 * @value 4 Tailsitter VTOL
 * @value 5 Rover (Ackermann)
 * @value 6 Rover (Differential)
 * @value 7 Motors (6DOF)
 * @value 8 Multirotor with Tilt
 * @value 9 Custom
 * @value 10 Helicopter (tail ESC)
 * @value 11 Helicopter (tail Servo)
 * @value 12 Helicopter (Coaxial)
 * @value 13 Rover (Mecanum)
 * @value 14 Spacecraft 2D
 * @value 15 Spacecraft 3D
 */
PARAM_DEFINE_INT32(CA_AIRFRAME, 0);

/**
 * Control allocation method
 *
 * Selects the algorithm and desaturation method.
 * If set to Automatic, the selection is based on the airframe (CA_AIRFRAME).
 * 
 *
 * @group Geometry
 * @value 0 Pseudo-inverse with output clipping
 * @value 1 Pseudo-inverse with sequential desaturation technique
 * @value 2 Automatic
 */
PARAM_DEFINE_INT32(CA_METHOD, 2);

/**
 * Bidirectional/Reversible motors
 *
 * Configure motors to be bidirectional/reversible. Note that the output driver needs to support this as well.
 * 
 *
 * @group Geometry
 * @bit 0 Motor 1
 * @bit 1 Motor 2
 * @bit 2 Motor 3
 * @bit 3 Motor 4
 * @bit 4 Motor 5
 * @bit 5 Motor 6
 * @bit 6 Motor 7
 * @bit 7 Motor 8
 * @bit 8 Motor 9
 * @bit 9 Motor 10
 * @bit 10 Motor 11
 * @bit 11 Motor 12
 * @min 0
 * @max 4095
 */
PARAM_DEFINE_INT32(CA_R_REV, 0);

/**
 * Motor 0 slew rate limit
 *
 * Forces the motor output signal to take at least the configured time (in seconds)
 * to traverse its full range (normally [0, 1], or if reversible [-1, 1]).
 * 
 * Zero means that slew rate limiting is disabled.
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.01
 * @min 0
 * @max 10
 * @unit s
 */
PARAM_DEFINE_FLOAT(CA_R0_SLEW, 0.0);

/**
 * Motor 1 slew rate limit
 *
 * Forces the motor output signal to take at least the configured time (in seconds)
 * to traverse its full range (normally [0, 1], or if reversible [-1, 1]).
 * 
 * Zero means that slew rate limiting is disabled.
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.01
 * @min 0
 * @max 10
 * @unit s
 */
PARAM_DEFINE_FLOAT(CA_R1_SLEW, 0.0);

/**
 * Motor 2 slew rate limit
 *
 * Forces the motor output signal to take at least the configured time (in seconds)
 * to traverse its full range (normally [0, 1], or if reversible [-1, 1]).
 * 
 * Zero means that slew rate limiting is disabled.
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.01
 * @min 0
 * @max 10
 * @unit s
 */
PARAM_DEFINE_FLOAT(CA_R2_SLEW, 0.0);

/**
 * Motor 3 slew rate limit
 *
 * Forces the motor output signal to take at least the configured time (in seconds)
 * to traverse its full range (normally [0, 1], or if reversible [-1, 1]).
 * 
 * Zero means that slew rate limiting is disabled.
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.01
 * @min 0
 * @max 10
 * @unit s
 */
PARAM_DEFINE_FLOAT(CA_R3_SLEW, 0.0);

/**
 * Motor 4 slew rate limit
 *
 * Forces the motor output signal to take at least the configured time (in seconds)
 * to traverse its full range (normally [0, 1], or if reversible [-1, 1]).
 * 
 * Zero means that slew rate limiting is disabled.
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.01
 * @min 0
 * @max 10
 * @unit s
 */
PARAM_DEFINE_FLOAT(CA_R4_SLEW, 0.0);

/**
 * Motor 5 slew rate limit
 *
 * Forces the motor output signal to take at least the configured time (in seconds)
 * to traverse its full range (normally [0, 1], or if reversible [-1, 1]).
 * 
 * Zero means that slew rate limiting is disabled.
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.01
 * @min 0
 * @max 10
 * @unit s
 */
PARAM_DEFINE_FLOAT(CA_R5_SLEW, 0.0);

/**
 * Motor 6 slew rate limit
 *
 * Forces the motor output signal to take at least the configured time (in seconds)
 * to traverse its full range (normally [0, 1], or if reversible [-1, 1]).
 * 
 * Zero means that slew rate limiting is disabled.
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.01
 * @min 0
 * @max 10
 * @unit s
 */
PARAM_DEFINE_FLOAT(CA_R6_SLEW, 0.0);

/**
 * Motor 7 slew rate limit
 *
 * Forces the motor output signal to take at least the configured time (in seconds)
 * to traverse its full range (normally [0, 1], or if reversible [-1, 1]).
 * 
 * Zero means that slew rate limiting is disabled.
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.01
 * @min 0
 * @max 10
 * @unit s
 */
PARAM_DEFINE_FLOAT(CA_R7_SLEW, 0.0);

/**
 * Motor 8 slew rate limit
 *
 * Forces the motor output signal to take at least the configured time (in seconds)
 * to traverse its full range (normally [0, 1], or if reversible [-1, 1]).
 * 
 * Zero means that slew rate limiting is disabled.
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.01
 * @min 0
 * @max 10
 * @unit s
 */
PARAM_DEFINE_FLOAT(CA_R8_SLEW, 0.0);

/**
 * Motor 9 slew rate limit
 *
 * Forces the motor output signal to take at least the configured time (in seconds)
 * to traverse its full range (normally [0, 1], or if reversible [-1, 1]).
 * 
 * Zero means that slew rate limiting is disabled.
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.01
 * @min 0
 * @max 10
 * @unit s
 */
PARAM_DEFINE_FLOAT(CA_R9_SLEW, 0.0);

/**
 * Motor 10 slew rate limit
 *
 * Forces the motor output signal to take at least the configured time (in seconds)
 * to traverse its full range (normally [0, 1], or if reversible [-1, 1]).
 * 
 * Zero means that slew rate limiting is disabled.
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.01
 * @min 0
 * @max 10
 * @unit s
 */
PARAM_DEFINE_FLOAT(CA_R10_SLEW, 0.0);

/**
 * Motor 11 slew rate limit
 *
 * Forces the motor output signal to take at least the configured time (in seconds)
 * to traverse its full range (normally [0, 1], or if reversible [-1, 1]).
 * 
 * Zero means that slew rate limiting is disabled.
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.01
 * @min 0
 * @max 10
 * @unit s
 */
PARAM_DEFINE_FLOAT(CA_R11_SLEW, 0.0);

/**
 * Servo 0 slew rate limit
 *
 * Forces the servo output signal to take at least the configured time (in seconds)
 * to traverse its full range [-100%, 100%].
 * 
 * Zero means that slew rate limiting is disabled.
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.05
 * @min 0
 * @max 10
 * @unit s
 */
PARAM_DEFINE_FLOAT(CA_SV0_SLEW, 0.0);

/**
 * Servo 1 slew rate limit
 *
 * Forces the servo output signal to take at least the configured time (in seconds)
 * to traverse its full range [-100%, 100%].
 * 
 * Zero means that slew rate limiting is disabled.
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.05
 * @min 0
 * @max 10
 * @unit s
 */
PARAM_DEFINE_FLOAT(CA_SV1_SLEW, 0.0);

/**
 * Servo 2 slew rate limit
 *
 * Forces the servo output signal to take at least the configured time (in seconds)
 * to traverse its full range [-100%, 100%].
 * 
 * Zero means that slew rate limiting is disabled.
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.05
 * @min 0
 * @max 10
 * @unit s
 */
PARAM_DEFINE_FLOAT(CA_SV2_SLEW, 0.0);

/**
 * Servo 3 slew rate limit
 *
 * Forces the servo output signal to take at least the configured time (in seconds)
 * to traverse its full range [-100%, 100%].
 * 
 * Zero means that slew rate limiting is disabled.
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.05
 * @min 0
 * @max 10
 * @unit s
 */
PARAM_DEFINE_FLOAT(CA_SV3_SLEW, 0.0);

/**
 * Servo 4 slew rate limit
 *
 * Forces the servo output signal to take at least the configured time (in seconds)
 * to traverse its full range [-100%, 100%].
 * 
 * Zero means that slew rate limiting is disabled.
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.05
 * @min 0
 * @max 10
 * @unit s
 */
PARAM_DEFINE_FLOAT(CA_SV4_SLEW, 0.0);

/**
 * Servo 5 slew rate limit
 *
 * Forces the servo output signal to take at least the configured time (in seconds)
 * to traverse its full range [-100%, 100%].
 * 
 * Zero means that slew rate limiting is disabled.
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.05
 * @min 0
 * @max 10
 * @unit s
 */
PARAM_DEFINE_FLOAT(CA_SV5_SLEW, 0.0);

/**
 * Servo 6 slew rate limit
 *
 * Forces the servo output signal to take at least the configured time (in seconds)
 * to traverse its full range [-100%, 100%].
 * 
 * Zero means that slew rate limiting is disabled.
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.05
 * @min 0
 * @max 10
 * @unit s
 */
PARAM_DEFINE_FLOAT(CA_SV6_SLEW, 0.0);

/**
 * Servo 7 slew rate limit
 *
 * Forces the servo output signal to take at least the configured time (in seconds)
 * to traverse its full range [-100%, 100%].
 * 
 * Zero means that slew rate limiting is disabled.
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.05
 * @min 0
 * @max 10
 * @unit s
 */
PARAM_DEFINE_FLOAT(CA_SV7_SLEW, 0.0);

/**
 * Total number of rotors
 *
 * 
 *
 * @group Geometry
 * @value 0 0
 * @value 1 1
 * @value 2 2
 * @value 3 3
 * @value 4 4
 * @value 5 5
 * @value 6 6
 * @value 7 7
 * @value 8 8
 * @value 9 9
 * @value 10 10
 * @value 11 11
 * @value 12 12
 */
PARAM_DEFINE_INT32(CA_ROTOR_COUNT, 0);

/**
 * Position of rotor 0 along X body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR0_PX, 0.0);

/**
 * Position of rotor 1 along X body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR1_PX, 0.0);

/**
 * Position of rotor 2 along X body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR2_PX, 0.0);

/**
 * Position of rotor 3 along X body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR3_PX, 0.0);

/**
 * Position of rotor 4 along X body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR4_PX, 0.0);

/**
 * Position of rotor 5 along X body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR5_PX, 0.0);

/**
 * Position of rotor 6 along X body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR6_PX, 0.0);

/**
 * Position of rotor 7 along X body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR7_PX, 0.0);

/**
 * Position of rotor 8 along X body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR8_PX, 0.0);

/**
 * Position of rotor 9 along X body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR9_PX, 0.0);

/**
 * Position of rotor 10 along X body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR10_PX, 0.0);

/**
 * Position of rotor 11 along X body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR11_PX, 0.0);

/**
 * Position of rotor 0 along Y body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR0_PY, 0.0);

/**
 * Position of rotor 1 along Y body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR1_PY, 0.0);

/**
 * Position of rotor 2 along Y body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR2_PY, 0.0);

/**
 * Position of rotor 3 along Y body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR3_PY, 0.0);

/**
 * Position of rotor 4 along Y body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR4_PY, 0.0);

/**
 * Position of rotor 5 along Y body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR5_PY, 0.0);

/**
 * Position of rotor 6 along Y body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR6_PY, 0.0);

/**
 * Position of rotor 7 along Y body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR7_PY, 0.0);

/**
 * Position of rotor 8 along Y body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR8_PY, 0.0);

/**
 * Position of rotor 9 along Y body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR9_PY, 0.0);

/**
 * Position of rotor 10 along Y body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR10_PY, 0.0);

/**
 * Position of rotor 11 along Y body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR11_PY, 0.0);

/**
 * Position of rotor 0 along Z body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR0_PZ, 0.0);

/**
 * Position of rotor 1 along Z body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR1_PZ, 0.0);

/**
 * Position of rotor 2 along Z body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR2_PZ, 0.0);

/**
 * Position of rotor 3 along Z body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR3_PZ, 0.0);

/**
 * Position of rotor 4 along Z body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR4_PZ, 0.0);

/**
 * Position of rotor 5 along Z body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR5_PZ, 0.0);

/**
 * Position of rotor 6 along Z body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR6_PZ, 0.0);

/**
 * Position of rotor 7 along Z body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR7_PZ, 0.0);

/**
 * Position of rotor 8 along Z body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR8_PZ, 0.0);

/**
 * Position of rotor 9 along Z body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR9_PZ, 0.0);

/**
 * Position of rotor 10 along Z body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR10_PZ, 0.0);

/**
 * Position of rotor 11 along Z body axis relative to center of gravity
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(CA_ROTOR11_PZ, 0.0);

/**
 * Axis of rotor 0 thrust vector, X body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR0_AX, 0.0);

/**
 * Axis of rotor 1 thrust vector, X body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR1_AX, 0.0);

/**
 * Axis of rotor 2 thrust vector, X body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR2_AX, 0.0);

/**
 * Axis of rotor 3 thrust vector, X body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR3_AX, 0.0);

/**
 * Axis of rotor 4 thrust vector, X body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR4_AX, 0.0);

/**
 * Axis of rotor 5 thrust vector, X body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR5_AX, 0.0);

/**
 * Axis of rotor 6 thrust vector, X body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR6_AX, 0.0);

/**
 * Axis of rotor 7 thrust vector, X body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR7_AX, 0.0);

/**
 * Axis of rotor 8 thrust vector, X body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR8_AX, 0.0);

/**
 * Axis of rotor 9 thrust vector, X body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR9_AX, 0.0);

/**
 * Axis of rotor 10 thrust vector, X body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR10_AX, 0.0);

/**
 * Axis of rotor 11 thrust vector, X body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR11_AX, 0.0);

/**
 * Axis of rotor 0 thrust vector, Y body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR0_AY, 0.0);

/**
 * Axis of rotor 1 thrust vector, Y body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR1_AY, 0.0);

/**
 * Axis of rotor 2 thrust vector, Y body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR2_AY, 0.0);

/**
 * Axis of rotor 3 thrust vector, Y body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR3_AY, 0.0);

/**
 * Axis of rotor 4 thrust vector, Y body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR4_AY, 0.0);

/**
 * Axis of rotor 5 thrust vector, Y body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR5_AY, 0.0);

/**
 * Axis of rotor 6 thrust vector, Y body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR6_AY, 0.0);

/**
 * Axis of rotor 7 thrust vector, Y body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR7_AY, 0.0);

/**
 * Axis of rotor 8 thrust vector, Y body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR8_AY, 0.0);

/**
 * Axis of rotor 9 thrust vector, Y body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR9_AY, 0.0);

/**
 * Axis of rotor 10 thrust vector, Y body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR10_AY, 0.0);

/**
 * Axis of rotor 11 thrust vector, Y body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR11_AY, 0.0);

/**
 * Axis of rotor 0 thrust vector, Z body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR0_AZ, -1.0);

/**
 * Axis of rotor 1 thrust vector, Z body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR1_AZ, -1.0);

/**
 * Axis of rotor 2 thrust vector, Z body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR2_AZ, -1.0);

/**
 * Axis of rotor 3 thrust vector, Z body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR3_AZ, -1.0);

/**
 * Axis of rotor 4 thrust vector, Z body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR4_AZ, -1.0);

/**
 * Axis of rotor 5 thrust vector, Z body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR5_AZ, -1.0);

/**
 * Axis of rotor 6 thrust vector, Z body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR6_AZ, -1.0);

/**
 * Axis of rotor 7 thrust vector, Z body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR7_AZ, -1.0);

/**
 * Axis of rotor 8 thrust vector, Z body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR8_AZ, -1.0);

/**
 * Axis of rotor 9 thrust vector, Z body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR9_AZ, -1.0);

/**
 * Axis of rotor 10 thrust vector, Z body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR10_AZ, -1.0);

/**
 * Axis of rotor 11 thrust vector, Z body axis component
 *
 * Only the direction is considered (the vector is normalized).
 *
 * @group Geometry
 * @decimal 2
 * @increment 0.1
 * @min -100
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR11_AZ, -1.0);

/**
 * Thrust coefficient of rotor 0
 *
 * The thrust coefficient if defined as Thrust = CT * u^2,
 * where u (with value between actuator minimum and maximum)
 * is the output signal sent to the motor controller.
 * 
 *
 * @group Geometry
 * @decimal 1
 * @increment 1
 * @min 0
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR0_CT, 6.5);

/**
 * Thrust coefficient of rotor 1
 *
 * The thrust coefficient if defined as Thrust = CT * u^2,
 * where u (with value between actuator minimum and maximum)
 * is the output signal sent to the motor controller.
 * 
 *
 * @group Geometry
 * @decimal 1
 * @increment 1
 * @min 0
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR1_CT, 6.5);

/**
 * Thrust coefficient of rotor 2
 *
 * The thrust coefficient if defined as Thrust = CT * u^2,
 * where u (with value between actuator minimum and maximum)
 * is the output signal sent to the motor controller.
 * 
 *
 * @group Geometry
 * @decimal 1
 * @increment 1
 * @min 0
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR2_CT, 6.5);

/**
 * Thrust coefficient of rotor 3
 *
 * The thrust coefficient if defined as Thrust = CT * u^2,
 * where u (with value between actuator minimum and maximum)
 * is the output signal sent to the motor controller.
 * 
 *
 * @group Geometry
 * @decimal 1
 * @increment 1
 * @min 0
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR3_CT, 6.5);

/**
 * Thrust coefficient of rotor 4
 *
 * The thrust coefficient if defined as Thrust = CT * u^2,
 * where u (with value between actuator minimum and maximum)
 * is the output signal sent to the motor controller.
 * 
 *
 * @group Geometry
 * @decimal 1
 * @increment 1
 * @min 0
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR4_CT, 6.5);

/**
 * Thrust coefficient of rotor 5
 *
 * The thrust coefficient if defined as Thrust = CT * u^2,
 * where u (with value between actuator minimum and maximum)
 * is the output signal sent to the motor controller.
 * 
 *
 * @group Geometry
 * @decimal 1
 * @increment 1
 * @min 0
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR5_CT, 6.5);

/**
 * Thrust coefficient of rotor 6
 *
 * The thrust coefficient if defined as Thrust = CT * u^2,
 * where u (with value between actuator minimum and maximum)
 * is the output signal sent to the motor controller.
 * 
 *
 * @group Geometry
 * @decimal 1
 * @increment 1
 * @min 0
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR6_CT, 6.5);

/**
 * Thrust coefficient of rotor 7
 *
 * The thrust coefficient if defined as Thrust = CT * u^2,
 * where u (with value between actuator minimum and maximum)
 * is the output signal sent to the motor controller.
 * 
 *
 * @group Geometry
 * @decimal 1
 * @increment 1
 * @min 0
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR7_CT, 6.5);

/**
 * Thrust coefficient of rotor 8
 *
 * The thrust coefficient if defined as Thrust = CT * u^2,
 * where u (with value between actuator minimum and maximum)
 * is the output signal sent to the motor controller.
 * 
 *
 * @group Geometry
 * @decimal 1
 * @increment 1
 * @min 0
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR8_CT, 6.5);

/**
 * Thrust coefficient of rotor 9
 *
 * The thrust coefficient if defined as Thrust = CT * u^2,
 * where u (with value between actuator minimum and maximum)
 * is the output signal sent to the motor controller.
 * 
 *
 * @group Geometry
 * @decimal 1
 * @increment 1
 * @min 0
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR9_CT, 6.5);

/**
 * Thrust coefficient of rotor 10
 *
 * The thrust coefficient if defined as Thrust = CT * u^2,
 * where u (with value between actuator minimum and maximum)
 * is the output signal sent to the motor controller.
 * 
 *
 * @group Geometry
 * @decimal 1
 * @increment 1
 * @min 0
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR10_CT, 6.5);

/**
 * Thrust coefficient of rotor 11
 *
 * The thrust coefficient if defined as Thrust = CT * u^2,
 * where u (with value between actuator minimum and maximum)
 * is the output signal sent to the motor controller.
 * 
 *
 * @group Geometry
 * @decimal 1
 * @increment 1
 * @min 0
 * @max 100
 */
PARAM_DEFINE_FLOAT(CA_ROTOR11_CT, 6.5);

/**
 * Moment coefficient of rotor 0
 *
 * The moment coefficient if defined as Torque = KM * Thrust.
 * 
 * Use a positive value for a rotor with CCW rotation.
 * Use a negative value for a rotor with CW rotation.
 * 
 *
 * @group Geometry
 * @decimal 3
 * @increment 0.01
 * @min -1
 * @max 1
 */
PARAM_DEFINE_FLOAT(CA_ROTOR0_KM, 0.05);

/**
 * Moment coefficient of rotor 1
 *
 * The moment coefficient if defined as Torque = KM * Thrust.
 * 
 * Use a positive value for a rotor with CCW rotation.
 * Use a negative value for a rotor with CW rotation.
 * 
 *
 * @group Geometry
 * @decimal 3
 * @increment 0.01
 * @min -1
 * @max 1
 */
PARAM_DEFINE_FLOAT(CA_ROTOR1_KM, 0.05);

/**
 * Moment coefficient of rotor 2
 *
 * The moment coefficient if defined as Torque = KM * Thrust.
 * 
 * Use a positive value for a rotor with CCW rotation.
 * Use a negative value for a rotor with CW rotation.
 * 
 *
 * @group Geometry
 * @decimal 3
 * @increment 0.01
 * @min -1
 * @max 1
 */
PARAM_DEFINE_FLOAT(CA_ROTOR2_KM, 0.05);

/**
 * Moment coefficient of rotor 3
 *
 * The moment coefficient if defined as Torque = KM * Thrust.
 * 
 * Use a positive value for a rotor with CCW rotation.
 * Use a negative value for a rotor with CW rotation.
 * 
 *
 * @group Geometry
 * @decimal 3
 * @increment 0.01
 * @min -1
 * @max 1
 */
PARAM_DEFINE_FLOAT(CA_ROTOR3_KM, 0.05);

/**
 * Moment coefficient of rotor 4
 *
 * The moment coefficient if defined as Torque = KM * Thrust.
 * 
 * Use a positive value for a rotor with CCW rotation.
 * Use a negative value for a rotor with CW rotation.
 * 
 *
 * @group Geometry
 * @decimal 3
 * @increment 0.01
 * @min -1
 * @max 1
 */
PARAM_DEFINE_FLOAT(CA_ROTOR4_KM, 0.05);

/**
 * Moment coefficient of rotor 5
 *
 * The moment coefficient if defined as Torque = KM * Thrust.
 * 
 * Use a positive value for a rotor with CCW rotation.
 * Use a negative value for a rotor with CW rotation.
 * 
 *
 * @group Geometry
 * @decimal 3
 * @increment 0.01
 * @min -1
 * @max 1
 */
PARAM_DEFINE_FLOAT(CA_ROTOR5_KM, 0.05);

/**
 * Moment coefficient of rotor 6
 *
 * The moment coefficient if defined as Torque = KM * Thrust.
 * 
 * Use a positive value for a rotor with CCW rotation.
 * Use a negative value for a rotor with CW rotation.
 * 
 *
 * @group Geometry
 * @decimal 3
 * @increment 0.01
 * @min -1
 * @max 1
 */
PARAM_DEFINE_FLOAT(CA_ROTOR6_KM, 0.05);

/**
 * Moment coefficient of rotor 7
 *
 * The moment coefficient if defined as Torque = KM * Thrust.
 * 
 * Use a positive value for a rotor with CCW rotation.
 * Use a negative value for a rotor with CW rotation.
 * 
 *
 * @group Geometry
 * @decimal 3
 * @increment 0.01
 * @min -1
 * @max 1
 */
PARAM_DEFINE_FLOAT(CA_ROTOR7_KM, 0.05);

/**
 * Moment coefficient of rotor 8
 *
 * The moment coefficient if defined as Torque = KM * Thrust.
 * 
 * Use a positive value for a rotor with CCW rotation.
 * Use a negative value for a rotor with CW rotation.
 * 
 *
 * @group Geometry
 * @decimal 3
 * @increment 0.01
 * @min -1
 * @max 1
 */
PARAM_DEFINE_FLOAT(CA_ROTOR8_KM, 0.05);

/**
 * Moment coefficient of rotor 9
 *
 * The moment coefficient if defined as Torque = KM * Thrust.
 * 
 * Use a positive value for a rotor with CCW rotation.
 * Use a negative value for a rotor with CW rotation.
 * 
 *
 * @group Geometry
 * @decimal 3
 * @increment 0.01
 * @min -1
 * @max 1
 */
PARAM_DEFINE_FLOAT(CA_ROTOR9_KM, 0.05);

/**
 * Moment coefficient of rotor 10
 *
 * The moment coefficient if defined as Torque = KM * Thrust.
 * 
 * Use a positive value for a rotor with CCW rotation.
 * Use a negative value for a rotor with CW rotation.
 * 
 *
 * @group Geometry
 * @decimal 3
 * @increment 0.01
 * @min -1
 * @max 1
 */
PARAM_DEFINE_FLOAT(CA_ROTOR10_KM, 0.05);

/**
 * Moment coefficient of rotor 11
 *
 * The moment coefficient if defined as Torque = KM * Thrust.
 * 
 * Use a positive value for a rotor with CCW rotation.
 * Use a negative value for a rotor with CW rotation.
 * 
 *
 * @group Geometry
 * @decimal 3
 * @increment 0.01
 * @min -1
 * @max 1
 */
PARAM_DEFINE_FLOAT(CA_ROTOR11_KM, 0.05);

/**
 * Rotor 0 tilt assignment
 *
 * If not set to None, this motor is tilted by the configured tilt servo.
 *
 * @group Geometry
 * @value 0 None
 * @value 1 Tilt 1
 * @value 2 Tilt 2
 * @value 3 Tilt 3
 * @value 4 Tilt 4
 */
PARAM_DEFINE_INT32(CA_ROTOR0_TILT, 0);

/**
 * Rotor 1 tilt assignment
 *
 * If not set to None, this motor is tilted by the configured tilt servo.
 *
 * @group Geometry
 * @value 0 None
 * @value 1 Tilt 1
 * @value 2 Tilt 2
 * @value 3 Tilt 3
 * @value 4 Tilt 4
 */
PARAM_DEFINE_INT32(CA_ROTOR1_TILT, 0);

/**
 * Rotor 2 tilt assignment
 *
 * If not set to None, this motor is tilted by the configured tilt servo.
 *
 * @group Geometry
 * @value 0 None
 * @value 1 Tilt 1
 * @value 2 Tilt 2
 * @value 3 Tilt 3
 * @value 4 Tilt 4
 */
PARAM_DEFINE_INT32(CA_ROTOR2_TILT, 0);

/**
 * Rotor 3 tilt assignment
 *
 * If not set to None, this motor is tilted by the configured tilt servo.
 *
 * @group Geometry
 * @value 0 None
 * @value 1 Tilt 1
 * @value 2 Tilt 2
 * @value 3 Tilt 3
 * @value 4 Tilt 4
 */
PARAM_DEFINE_INT32(CA_ROTOR3_TILT, 0);

/**
 * Rotor 4 tilt assignment
 *
 * If not set to None, this motor is tilted by the configured tilt servo.
 *
 * @group Geometry
 * @value 0 None
 * @value 1 Tilt 1
 * @value 2 Tilt 2
 * @value 3 Tilt 3
 * @value 4 Tilt 4
 */
PARAM_DEFINE_INT32(CA_ROTOR4_TILT, 0);

/**
 * Rotor 5 tilt assignment
 *
 * If not set to None, this motor is tilted by the configured tilt servo.
 *
 * @group Geometry
 * @value 0 None
 * @value 1 Tilt 1
 * @value 2 Tilt 2
 * @value 3 Tilt 3
 * @value 4 Tilt 4
 */
PARAM_DEFINE_INT32(CA_ROTOR5_TILT, 0);

/**
 * Rotor 6 tilt assignment
 *
 * If not set to None, this motor is tilted by the configured tilt servo.
 *
 * @group Geometry
 * @value 0 None
 * @value 1 Tilt 1
 * @value 2 Tilt 2
 * @value 3 Tilt 3
 * @value 4 Tilt 4
 */
PARAM_DEFINE_INT32(CA_ROTOR6_TILT, 0);

/**
 * Rotor 7 tilt assignment
 *
 * If not set to None, this motor is tilted by the configured tilt servo.
 *
 * @group Geometry
 * @value 0 None
 * @value 1 Tilt 1
 * @value 2 Tilt 2
 * @value 3 Tilt 3
 * @value 4 Tilt 4
 */
PARAM_DEFINE_INT32(CA_ROTOR7_TILT, 0);

/**
 * Rotor 8 tilt assignment
 *
 * If not set to None, this motor is tilted by the configured tilt servo.
 *
 * @group Geometry
 * @value 0 None
 * @value 1 Tilt 1
 * @value 2 Tilt 2
 * @value 3 Tilt 3
 * @value 4 Tilt 4
 */
PARAM_DEFINE_INT32(CA_ROTOR8_TILT, 0);

/**
 * Rotor 9 tilt assignment
 *
 * If not set to None, this motor is tilted by the configured tilt servo.
 *
 * @group Geometry
 * @value 0 None
 * @value 1 Tilt 1
 * @value 2 Tilt 2
 * @value 3 Tilt 3
 * @value 4 Tilt 4
 */
PARAM_DEFINE_INT32(CA_ROTOR9_TILT, 0);

/**
 * Rotor 10 tilt assignment
 *
 * If not set to None, this motor is tilted by the configured tilt servo.
 *
 * @group Geometry
 * @value 0 None
 * @value 1 Tilt 1
 * @value 2 Tilt 2
 * @value 3 Tilt 3
 * @value 4 Tilt 4
 */
PARAM_DEFINE_INT32(CA_ROTOR10_TILT, 0);

/**
 * Rotor 11 tilt assignment
 *
 * If not set to None, this motor is tilted by the configured tilt servo.
 *
 * @group Geometry
 * @value 0 None
 * @value 1 Tilt 1
 * @value 2 Tilt 2
 * @value 3 Tilt 3
 * @value 4 Tilt 4
 */
PARAM_DEFINE_INT32(CA_ROTOR11_TILT, 0);

/**
 * Total number of Control Surfaces
 *
 * 
 *
 * @group Geometry
 * @value 0 0
 * @value 1 1
 * @value 2 2
 * @value 3 3
 * @value 4 4
 * @value 5 5
 * @value 6 6
 * @value 7 7
 * @value 8 8
 */
PARAM_DEFINE_INT32(CA_SV_CS_COUNT, 0);

/**
 * Control Surface 0 type
 *
 * 
 *
 * @group Geometry
 * @value 0 (Not set)
 * @value 1 Left Aileron
 * @value 2 Right Aileron
 * @value 3 Elevator
 * @value 4 Rudder
 * @value 5 Left Elevon
 * @value 6 Right Elevon
 * @value 7 Left V-Tail
 * @value 8 Right V-Tail
 * @value 9 Left Flap
 * @value 10 Right Flap
 * @value 11 Airbrake
 * @value 12 Custom
 * @value 13 Left A-tail
 * @value 14 Right A-tail
 * @value 15 Single Channel Aileron
 * @value 16 Steering Wheel
 * @value 17 Left Spoiler
 * @value 18 Right Spoiler
 */
PARAM_DEFINE_INT32(CA_SV_CS0_TYPE, 0);

/**
 * Control Surface 1 type
 *
 * 
 *
 * @group Geometry
 * @value 0 (Not set)
 * @value 1 Left Aileron
 * @value 2 Right Aileron
 * @value 3 Elevator
 * @value 4 Rudder
 * @value 5 Left Elevon
 * @value 6 Right Elevon
 * @value 7 Left V-Tail
 * @value 8 Right V-Tail
 * @value 9 Left Flap
 * @value 10 Right Flap
 * @value 11 Airbrake
 * @value 12 Custom
 * @value 13 Left A-tail
 * @value 14 Right A-tail
 * @value 15 Single Channel Aileron
 * @value 16 Steering Wheel
 * @value 17 Left Spoiler
 * @value 18 Right Spoiler
 */
PARAM_DEFINE_INT32(CA_SV_CS1_TYPE, 0);

/**
 * Control Surface 2 type
 *
 * 
 *
 * @group Geometry
 * @value 0 (Not set)
 * @value 1 Left Aileron
 * @value 2 Right Aileron
 * @value 3 Elevator
 * @value 4 Rudder
 * @value 5 Left Elevon
 * @value 6 Right Elevon
 * @value 7 Left V-Tail
 * @value 8 Right V-Tail
 * @value 9 Left Flap
 * @value 10 Right Flap
 * @value 11 Airbrake
 * @value 12 Custom
 * @value 13 Left A-tail
 * @value 14 Right A-tail
 * @value 15 Single Channel Aileron
 * @value 16 Steering Wheel
 * @value 17 Left Spoiler
 * @value 18 Right Spoiler
 */
PARAM_DEFINE_INT32(CA_SV_CS2_TYPE, 0);

/**
 * Control Surface 3 type
 *
 * 
 *
 * @group Geometry
 * @value 0 (Not set)
 * @value 1 Left Aileron
 * @value 2 Right Aileron
 * @value 3 Elevator
 * @value 4 Rudder
 * @value 5 Left Elevon
 * @value 6 Right Elevon
 * @value 7 Left V-Tail
 * @value 8 Right V-Tail
 * @value 9 Left Flap
 * @value 10 Right Flap
 * @value 11 Airbrake
 * @value 12 Custom
 * @value 13 Left A-tail
 * @value 14 Right A-tail
 * @value 15 Single Channel Aileron
 * @value 16 Steering Wheel
 * @value 17 Left Spoiler
 * @value 18 Right Spoiler
 */
PARAM_DEFINE_INT32(CA_SV_CS3_TYPE, 0);

/**
 * Control Surface 4 type
 *
 * 
 *
 * @group Geometry
 * @value 0 (Not set)
 * @value 1 Left Aileron
 * @value 2 Right Aileron
 * @value 3 Elevator
 * @value 4 Rudder
 * @value 5 Left Elevon
 * @value 6 Right Elevon
 * @value 7 Left V-Tail
 * @value 8 Right V-Tail
 * @value 9 Left Flap
 * @value 10 Right Flap
 * @value 11 Airbrake
 * @value 12 Custom
 * @value 13 Left A-tail
 * @value 14 Right A-tail
 * @value 15 Single Channel Aileron
 * @value 16 Steering Wheel
 * @value 17 Left Spoiler
 * @value 18 Right Spoiler
 */
PARAM_DEFINE_INT32(CA_SV_CS4_TYPE, 0);

/**
 * Control Surface 5 type
 *
 * 
 *
 * @group Geometry
 * @value 0 (Not set)
 * @value 1 Left Aileron
 * @value 2 Right Aileron
 * @value 3 Elevator
 * @value 4 Rudder
 * @value 5 Left Elevon
 * @value 6 Right Elevon
 * @value 7 Left V-Tail
 * @value 8 Right V-Tail
 * @value 9 Left Flap
 * @value 10 Right Flap
 * @value 11 Airbrake
 * @value 12 Custom
 * @value 13 Left A-tail
 * @value 14 Right A-tail
 * @value 15 Single Channel Aileron
 * @value 16 Steering Wheel
 * @value 17 Left Spoiler
 * @value 18 Right Spoiler
 */
PARAM_DEFINE_INT32(CA_SV_CS5_TYPE, 0);

/**
 * Control Surface 6 type
 *
 * 
 *
 * @group Geometry
 * @value 0 (Not set)
 * @value 1 Left Aileron
 * @value 2 Right Aileron
 * @value 3 Elevator
 * @value 4 Rudder
 * @value 5 Left Elevon
 * @value 6 Right Elevon
 * @value 7 Left V-Tail
 * @value 8 Right V-Tail
 * @value 9 Left Flap
 * @value 10 Right Flap
 * @value 11 Airbrake
 * @value 12 Custom
 * @value 13 Left A-tail
 * @value 14 Right A-tail
 * @value 15 Single Channel Aileron
 * @value 16 Steering Wheel
 * @value 17 Left Spoiler
 * @value 18 Right Spoiler
 */
PARAM_DEFINE_INT32(CA_SV_CS6_TYPE, 0);

/**
 * Control Surface 7 type
 *
 * 
 *
 * @group Geometry
 * @value 0 (Not set)
 * @value 1 Left Aileron
 * @value 2 Right Aileron
 * @value 3 Elevator
 * @value 4 Rudder
 * @value 5 Left Elevon
 * @value 6 Right Elevon
 * @value 7 Left V-Tail
 * @value 8 Right V-Tail
 * @value 9 Left Flap
 * @value 10 Right Flap
 * @value 11 Airbrake
 * @value 12 Custom
 * @value 13 Left A-tail
 * @value 14 Right A-tail
 * @value 15 Single Channel Aileron
 * @value 16 Steering Wheel
 * @value 17 Left Spoiler
 * @value 18 Right Spoiler
 */
PARAM_DEFINE_INT32(CA_SV_CS7_TYPE, 0);

/**
 * Control Surface 0 roll torque scaling
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CA_SV_CS0_TRQ_R, 0.0);

/**
 * Control Surface 1 roll torque scaling
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CA_SV_CS1_TRQ_R, 0.0);

/**
 * Control Surface 2 roll torque scaling
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CA_SV_CS2_TRQ_R, 0.0);

/**
 * Control Surface 3 roll torque scaling
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CA_SV_CS3_TRQ_R, 0.0);

/**
 * Control Surface 4 roll torque scaling
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CA_SV_CS4_TRQ_R, 0.0);

/**
 * Control Surface 5 roll torque scaling
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CA_SV_CS5_TRQ_R, 0.0);

/**
 * Control Surface 6 roll torque scaling
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CA_SV_CS6_TRQ_R, 0.0);

/**
 * Control Surface 7 roll torque scaling
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CA_SV_CS7_TRQ_R, 0.0);

/**
 * Control Surface 0 pitch torque scaling
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CA_SV_CS0_TRQ_P, 0.0);

/**
 * Control Surface 1 pitch torque scaling
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CA_SV_CS1_TRQ_P, 0.0);

/**
 * Control Surface 2 pitch torque scaling
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CA_SV_CS2_TRQ_P, 0.0);

/**
 * Control Surface 3 pitch torque scaling
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CA_SV_CS3_TRQ_P, 0.0);

/**
 * Control Surface 4 pitch torque scaling
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CA_SV_CS4_TRQ_P, 0.0);

/**
 * Control Surface 5 pitch torque scaling
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CA_SV_CS5_TRQ_P, 0.0);

/**
 * Control Surface 6 pitch torque scaling
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CA_SV_CS6_TRQ_P, 0.0);

/**
 * Control Surface 7 pitch torque scaling
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CA_SV_CS7_TRQ_P, 0.0);

/**
 * Control Surface 0 yaw torque scaling
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CA_SV_CS0_TRQ_Y, 0.0);

/**
 * Control Surface 1 yaw torque scaling
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CA_SV_CS1_TRQ_Y, 0.0);

/**
 * Control Surface 2 yaw torque scaling
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CA_SV_CS2_TRQ_Y, 0.0);

/**
 * Control Surface 3 yaw torque scaling
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CA_SV_CS3_TRQ_Y, 0.0);

/**
 * Control Surface 4 yaw torque scaling
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CA_SV_CS4_TRQ_Y, 0.0);

/**
 * Control Surface 5 yaw torque scaling
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CA_SV_CS5_TRQ_Y, 0.0);

/**
 * Control Surface 6 yaw torque scaling
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CA_SV_CS6_TRQ_Y, 0.0);

/**
 * Control Surface 7 yaw torque scaling
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CA_SV_CS7_TRQ_Y, 0.0);

/**
 * Control Surface 0 trim
 *
 * Can be used to add an offset to the servo control.
 *
 * @group Geometry
 * @decimal 2
 * @min -1.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(CA_SV_CS0_TRIM, 0.0);

/**
 * Control Surface 1 trim
 *
 * Can be used to add an offset to the servo control.
 *
 * @group Geometry
 * @decimal 2
 * @min -1.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(CA_SV_CS1_TRIM, 0.0);

/**
 * Control Surface 2 trim
 *
 * Can be used to add an offset to the servo control.
 *
 * @group Geometry
 * @decimal 2
 * @min -1.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(CA_SV_CS2_TRIM, 0.0);

/**
 * Control Surface 3 trim
 *
 * Can be used to add an offset to the servo control.
 *
 * @group Geometry
 * @decimal 2
 * @min -1.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(CA_SV_CS3_TRIM, 0.0);

/**
 * Control Surface 4 trim
 *
 * Can be used to add an offset to the servo control.
 *
 * @group Geometry
 * @decimal 2
 * @min -1.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(CA_SV_CS4_TRIM, 0.0);

/**
 * Control Surface 5 trim
 *
 * Can be used to add an offset to the servo control.
 *
 * @group Geometry
 * @decimal 2
 * @min -1.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(CA_SV_CS5_TRIM, 0.0);

/**
 * Control Surface 6 trim
 *
 * Can be used to add an offset to the servo control.
 *
 * @group Geometry
 * @decimal 2
 * @min -1.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(CA_SV_CS6_TRIM, 0.0);

/**
 * Control Surface 7 trim
 *
 * Can be used to add an offset to the servo control.
 *
 * @group Geometry
 * @decimal 2
 * @min -1.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(CA_SV_CS7_TRIM, 0.0);

/**
 * Control Surface 0 configuration as flap
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @min -1.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(CA_SV_CS0_FLAP, 0);

/**
 * Control Surface 1 configuration as flap
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @min -1.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(CA_SV_CS1_FLAP, 0);

/**
 * Control Surface 2 configuration as flap
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @min -1.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(CA_SV_CS2_FLAP, 0);

/**
 * Control Surface 3 configuration as flap
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @min -1.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(CA_SV_CS3_FLAP, 0);

/**
 * Control Surface 4 configuration as flap
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @min -1.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(CA_SV_CS4_FLAP, 0);

/**
 * Control Surface 5 configuration as flap
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @min -1.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(CA_SV_CS5_FLAP, 0);

/**
 * Control Surface 6 configuration as flap
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @min -1.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(CA_SV_CS6_FLAP, 0);

/**
 * Control Surface 7 configuration as flap
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @min -1.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(CA_SV_CS7_FLAP, 0);

/**
 * Control Surface 0 configuration as spoiler
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @min -1.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(CA_SV_CS0_SPOIL, 0);

/**
 * Control Surface 1 configuration as spoiler
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @min -1.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(CA_SV_CS1_SPOIL, 0);

/**
 * Control Surface 2 configuration as spoiler
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @min -1.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(CA_SV_CS2_SPOIL, 0);

/**
 * Control Surface 3 configuration as spoiler
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @min -1.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(CA_SV_CS3_SPOIL, 0);

/**
 * Control Surface 4 configuration as spoiler
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @min -1.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(CA_SV_CS4_SPOIL, 0);

/**
 * Control Surface 5 configuration as spoiler
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @min -1.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(CA_SV_CS5_SPOIL, 0);

/**
 * Control Surface 6 configuration as spoiler
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @min -1.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(CA_SV_CS6_SPOIL, 0);

/**
 * Control Surface 7 configuration as spoiler
 *
 * 
 *
 * @group Geometry
 * @decimal 2
 * @min -1.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(CA_SV_CS7_SPOIL, 0);

/**
 * Total number of Tilt Servos
 *
 * 
 *
 * @group Geometry
 * @value 0 0
 * @value 1 1
 * @value 2 2
 * @value 3 3
 * @value 4 4
 */
PARAM_DEFINE_INT32(CA_SV_TL_COUNT, 0);

/**
 * Tilt 0 is used for control
 *
 * Define if this servo is used for additional control.
 *
 * @group Geometry
 * @value 0 None
 * @value 1 Yaw
 * @value 2 Pitch
 * @value 3 Yaw and Pitch
 */
PARAM_DEFINE_INT32(CA_SV_TL0_CT, 1);

/**
 * Tilt 1 is used for control
 *
 * Define if this servo is used for additional control.
 *
 * @group Geometry
 * @value 0 None
 * @value 1 Yaw
 * @value 2 Pitch
 * @value 3 Yaw and Pitch
 */
PARAM_DEFINE_INT32(CA_SV_TL1_CT, 1);

/**
 * Tilt 2 is used for control
 *
 * Define if this servo is used for additional control.
 *
 * @group Geometry
 * @value 0 None
 * @value 1 Yaw
 * @value 2 Pitch
 * @value 3 Yaw and Pitch
 */
PARAM_DEFINE_INT32(CA_SV_TL2_CT, 1);

/**
 * Tilt 3 is used for control
 *
 * Define if this servo is used for additional control.
 *
 * @group Geometry
 * @value 0 None
 * @value 1 Yaw
 * @value 2 Pitch
 * @value 3 Yaw and Pitch
 */
PARAM_DEFINE_INT32(CA_SV_TL3_CT, 1);

/**
 * Tilt Servo 0 Tilt Angle at Minimum
 *
 * Defines the tilt angle when the servo is at the minimum.
 * An angle of zero means upwards.
 * 
 *
 * @group Geometry
 * @decimal 0
 * @min -90.0
 * @max 90.0
 * @unit deg
 */
PARAM_DEFINE_FLOAT(CA_SV_TL0_MINA, 0.0);

/**
 * Tilt Servo 1 Tilt Angle at Minimum
 *
 * Defines the tilt angle when the servo is at the minimum.
 * An angle of zero means upwards.
 * 
 *
 * @group Geometry
 * @decimal 0
 * @min -90.0
 * @max 90.0
 * @unit deg
 */
PARAM_DEFINE_FLOAT(CA_SV_TL1_MINA, 0.0);

/**
 * Tilt Servo 2 Tilt Angle at Minimum
 *
 * Defines the tilt angle when the servo is at the minimum.
 * An angle of zero means upwards.
 * 
 *
 * @group Geometry
 * @decimal 0
 * @min -90.0
 * @max 90.0
 * @unit deg
 */
PARAM_DEFINE_FLOAT(CA_SV_TL2_MINA, 0.0);

/**
 * Tilt Servo 3 Tilt Angle at Minimum
 *
 * Defines the tilt angle when the servo is at the minimum.
 * An angle of zero means upwards.
 * 
 *
 * @group Geometry
 * @decimal 0
 * @min -90.0
 * @max 90.0
 * @unit deg
 */
PARAM_DEFINE_FLOAT(CA_SV_TL3_MINA, 0.0);

/**
 * Tilt Servo 0 Tilt Angle at Maximum
 *
 * Defines the tilt angle when the servo is at the maximum.
 * An angle of zero means upwards.
 * 
 *
 * @group Geometry
 * @decimal 0
 * @min -90.0
 * @max 90.0
 * @unit deg
 */
PARAM_DEFINE_FLOAT(CA_SV_TL0_MAXA, 90.0);

/**
 * Tilt Servo 1 Tilt Angle at Maximum
 *
 * Defines the tilt angle when the servo is at the maximum.
 * An angle of zero means upwards.
 * 
 *
 * @group Geometry
 * @decimal 0
 * @min -90.0
 * @max 90.0
 * @unit deg
 */
PARAM_DEFINE_FLOAT(CA_SV_TL1_MAXA, 90.0);

/**
 * Tilt Servo 2 Tilt Angle at Maximum
 *
 * Defines the tilt angle when the servo is at the maximum.
 * An angle of zero means upwards.
 * 
 *
 * @group Geometry
 * @decimal 0
 * @min -90.0
 * @max 90.0
 * @unit deg
 */
PARAM_DEFINE_FLOAT(CA_SV_TL2_MAXA, 90.0);

/**
 * Tilt Servo 3 Tilt Angle at Maximum
 *
 * Defines the tilt angle when the servo is at the maximum.
 * An angle of zero means upwards.
 * 
 *
 * @group Geometry
 * @decimal 0
 * @min -90.0
 * @max 90.0
 * @unit deg
 */
PARAM_DEFINE_FLOAT(CA_SV_TL3_MAXA, 90.0);

/**
 * Tilt Servo 0 Tilt Direction
 *
 * Defines the direction the servo tilts towards when moving towards the maximum tilt angle.
 * For example if the minimum tilt angle is -90, the maximum 90, and the direction 'Towards Front',
 * the motor axis aligns with the XZ-plane, points towards -X at the minimum and +X at the maximum tilt.
 * 
 *
 * @group Geometry
 * @value 0 Towards Front
 * @value 90 Towards Right
 * @min 0
 * @max 359
 */
PARAM_DEFINE_INT32(CA_SV_TL0_TD, 0);

/**
 * Tilt Servo 1 Tilt Direction
 *
 * Defines the direction the servo tilts towards when moving towards the maximum tilt angle.
 * For example if the minimum tilt angle is -90, the maximum 90, and the direction 'Towards Front',
 * the motor axis aligns with the XZ-plane, points towards -X at the minimum and +X at the maximum tilt.
 * 
 *
 * @group Geometry
 * @value 0 Towards Front
 * @value 90 Towards Right
 * @min 0
 * @max 359
 */
PARAM_DEFINE_INT32(CA_SV_TL1_TD, 0);

/**
 * Tilt Servo 2 Tilt Direction
 *
 * Defines the direction the servo tilts towards when moving towards the maximum tilt angle.
 * For example if the minimum tilt angle is -90, the maximum 90, and the direction 'Towards Front',
 * the motor axis aligns with the XZ-plane, points towards -X at the minimum and +X at the maximum tilt.
 * 
 *
 * @group Geometry
 * @value 0 Towards Front
 * @value 90 Towards Right
 * @min 0
 * @max 359
 */
PARAM_DEFINE_INT32(CA_SV_TL2_TD, 0);

/**
 * Tilt Servo 3 Tilt Direction
 *
 * Defines the direction the servo tilts towards when moving towards the maximum tilt angle.
 * For example if the minimum tilt angle is -90, the maximum 90, and the direction 'Towards Front',
 * the motor axis aligns with the XZ-plane, points towards -X at the minimum and +X at the maximum tilt.
 * 
 *
 * @group Geometry
 * @value 0 Towards Front
 * @value 90 Towards Right
 * @min 0
 * @max 359
 */
PARAM_DEFINE_INT32(CA_SV_TL3_TD, 0);

/**
 * Number of swash plates servos
 *
 * 
 *
 * @group Geometry
 * @value 2 2
 * @value 3 3
 * @value 4 4
 */
PARAM_DEFINE_INT32(CA_SP0_COUNT, 3);

/**
 * Angle for swash plate servo 0
 *
 * The angle is measured clockwise (as seen from top), with 0 pointing forwards (X axis).
 * 
 *
 * @group Geometry
 * @decimal 0
 * @increment 10
 * @min 0
 * @max 360
 * @unit deg
 */
PARAM_DEFINE_FLOAT(CA_SP0_ANG0, 0);

/**
 * Angle for swash plate servo 1
 *
 * The angle is measured clockwise (as seen from top), with 0 pointing forwards (X axis).
 * 
 *
 * @group Geometry
 * @decimal 0
 * @increment 10
 * @min 0
 * @max 360
 * @unit deg
 */
PARAM_DEFINE_FLOAT(CA_SP0_ANG1, 140);

/**
 * Angle for swash plate servo 2
 *
 * The angle is measured clockwise (as seen from top), with 0 pointing forwards (X axis).
 * 
 *
 * @group Geometry
 * @decimal 0
 * @increment 10
 * @min 0
 * @max 360
 * @unit deg
 */
PARAM_DEFINE_FLOAT(CA_SP0_ANG2, 220);

/**
 * Angle for swash plate servo 3
 *
 * The angle is measured clockwise (as seen from top), with 0 pointing forwards (X axis).
 * 
 *
 * @group Geometry
 * @decimal 0
 * @increment 10
 * @min 0
 * @max 360
 * @unit deg
 */
PARAM_DEFINE_FLOAT(CA_SP0_ANG3, 0);

/**
 * Arm length for swash plate servo 0
 *
 * This is relative to the other arm lengths.
 * 
 *
 * @group Geometry
 * @decimal 3
 * @increment 0.1
 * @min 0
 * @max 10
 */
PARAM_DEFINE_FLOAT(CA_SP0_ARM_L0, 1.0);

/**
 * Arm length for swash plate servo 1
 *
 * This is relative to the other arm lengths.
 * 
 *
 * @group Geometry
 * @decimal 3
 * @increment 0.1
 * @min 0
 * @max 10
 */
PARAM_DEFINE_FLOAT(CA_SP0_ARM_L1, 1.0);

/**
 * Arm length for swash plate servo 2
 *
 * This is relative to the other arm lengths.
 * 
 *
 * @group Geometry
 * @decimal 3
 * @increment 0.1
 * @min 0
 * @max 10
 */
PARAM_DEFINE_FLOAT(CA_SP0_ARM_L2, 1.0);

/**
 * Arm length for swash plate servo 3
 *
 * This is relative to the other arm lengths.
 * 
 *
 * @group Geometry
 * @decimal 3
 * @increment 0.1
 * @min 0
 * @max 10
 */
PARAM_DEFINE_FLOAT(CA_SP0_ARM_L3, 1.0);

/**
 * Throttle curve at position 0
 *
 * Defines the output throttle at the interval position 0.
 * 
 *
 * @group Geometry
 * @decimal 3
 * @increment 0.1
 * @min 0
 * @max 1
 */
PARAM_DEFINE_FLOAT(CA_HELI_THR_C0, 1);

/**
 * Throttle curve at position 1
 *
 * Defines the output throttle at the interval position 1.
 * 
 *
 * @group Geometry
 * @decimal 3
 * @increment 0.1
 * @min 0
 * @max 1
 */
PARAM_DEFINE_FLOAT(CA_HELI_THR_C1, 1);

/**
 * Throttle curve at position 2
 *
 * Defines the output throttle at the interval position 2.
 * 
 *
 * @group Geometry
 * @decimal 3
 * @increment 0.1
 * @min 0
 * @max 1
 */
PARAM_DEFINE_FLOAT(CA_HELI_THR_C2, 1);

/**
 * Throttle curve at position 3
 *
 * Defines the output throttle at the interval position 3.
 * 
 *
 * @group Geometry
 * @decimal 3
 * @increment 0.1
 * @min 0
 * @max 1
 */
PARAM_DEFINE_FLOAT(CA_HELI_THR_C3, 1);

/**
 * Throttle curve at position 4
 *
 * Defines the output throttle at the interval position 4.
 * 
 *
 * @group Geometry
 * @decimal 3
 * @increment 0.1
 * @min 0
 * @max 1
 */
PARAM_DEFINE_FLOAT(CA_HELI_THR_C4, 1);

/**
 * Collective pitch curve at position 0
 *
 * Defines the collective pitch at the interval position 0 for a given thrust setpoint.
 * 
 * Use negative values if the swash plate needs to move down to provide upwards thrust.
 * 
 *
 * @group Geometry
 * @decimal 3
 * @increment 0.1
 * @min -1
 * @max 1
 */
PARAM_DEFINE_FLOAT(CA_HELI_PITCH_C0, -0.05);

/**
 * Collective pitch curve at position 1
 *
 * Defines the collective pitch at the interval position 1 for a given thrust setpoint.
 * 
 * Use negative values if the swash plate needs to move down to provide upwards thrust.
 * 
 *
 * @group Geometry
 * @decimal 3
 * @increment 0.1
 * @min -1
 * @max 1
 */
PARAM_DEFINE_FLOAT(CA_HELI_PITCH_C1, 0.0725);

/**
 * Collective pitch curve at position 2
 *
 * Defines the collective pitch at the interval position 2 for a given thrust setpoint.
 * 
 * Use negative values if the swash plate needs to move down to provide upwards thrust.
 * 
 *
 * @group Geometry
 * @decimal 3
 * @increment 0.1
 * @min -1
 * @max 1
 */
PARAM_DEFINE_FLOAT(CA_HELI_PITCH_C2, 0.2);

/**
 * Collective pitch curve at position 3
 *
 * Defines the collective pitch at the interval position 3 for a given thrust setpoint.
 * 
 * Use negative values if the swash plate needs to move down to provide upwards thrust.
 * 
 *
 * @group Geometry
 * @decimal 3
 * @increment 0.1
 * @min -1
 * @max 1
 */
PARAM_DEFINE_FLOAT(CA_HELI_PITCH_C3, 0.325);

/**
 * Collective pitch curve at position 4
 *
 * Defines the collective pitch at the interval position 4 for a given thrust setpoint.
 * 
 * Use negative values if the swash plate needs to move down to provide upwards thrust.
 * 
 *
 * @group Geometry
 * @decimal 3
 * @increment 0.1
 * @min -1
 * @max 1
 */
PARAM_DEFINE_FLOAT(CA_HELI_PITCH_C4, 0.45);

/**
 * Scale for yaw compensation based on collective pitch
 *
 * This allows to add a proportional factor of the collective pitch command to the yaw command.
 * A negative value is needed when positive thrust of the tail rotor rotates the vehicle opposite to the main rotor turn direction.
 * 
 * tail_output += CA_HELI_YAW_CP_S * abs(collective_pitch - CA_HELI_YAW_CP_O)
 * 
 *
 * @group Geometry
 * @decimal 3
 * @increment 0.1
 * @min -2
 * @max 2
 */
PARAM_DEFINE_FLOAT(CA_HELI_YAW_CP_S, 0.0);

/**
 * Offset for yaw compensation based on collective pitch
 *
 * This allows to specify which collective pitch command results in the least amount of rotor drag.
 * This is used to increase the accuracy of the yaw drag torque compensation based on collective pitch
 * by aligning the lowest rotor drag with zero compensation.
 * For symmetric profile blades this is the command that results in exactly 0° collective blade angle.
 * For lift profile blades this is typically a command resulting in slightly negative collective blade angle.
 * 
 * tail_output += CA_HELI_YAW_CP_S * abs(collective_pitch - CA_HELI_YAW_CP_O)
 * 
 *
 * @group Geometry
 * @decimal 3
 * @increment 0.1
 * @min -2
 * @max 2
 */
PARAM_DEFINE_FLOAT(CA_HELI_YAW_CP_O, 0.0);

/**
 * Scale for yaw compensation based on throttle
 *
 * This allows to add a proportional factor of the throttle command to the yaw command.
 * A negative value is needed when positive thrust of the tail rotor rotates the vehicle opposite to the main rotor turn direction.
 * 
 * tail_output += CA_HELI_YAW_TH_S * throttle
 * 
 *
 * @group Geometry
 * @decimal 3
 * @increment 0.1
 * @min -2
 * @max 2
 */
PARAM_DEFINE_FLOAT(CA_HELI_YAW_TH_S, 0.0);

/**
 * Main rotor turns counter-clockwise
 *
 * Default configuration is for a clockwise turning main rotor and
 * positive thrust of the tail rotor is expected to rotate the vehicle clockwise.
 * Set this parameter to true if the tail rotor provides thrust in counter-clockwise direction
 * which is mostly the case when the main rotor turns counter-clockwise.
 * 
 *
 * @group Geometry
 * @boolean
 */
PARAM_DEFINE_INT32(CA_HELI_YAW_CCW, 0);

/**
 * Setpoint for main rotor rpm
 *
 * Requires rpm feedback for the controller.
 * 
 *
 * @group Geometry
 * @decimal 0
 * @increment 1
 * @min 100
 * @max 10000
 */
PARAM_DEFINE_FLOAT(CA_HELI_RPM_SP, 1500);

/**
 * Proportional gain for rpm control
 *
 * Ratio between rpm error devided by 1000 to how much normalized output gets added to correct for it.
 * 
 * motor_command = throttle_curve + CA_HELI_RPM_P * (rpm_setpoint - rpm_measurement) / 1000
 * 
 *
 * @group Geometry
 * @decimal 3
 * @increment 0.1
 * @min 0
 * @max 10
 */
PARAM_DEFINE_FLOAT(CA_HELI_RPM_P, 0.0);

/**
 * Integral gain for rpm control
 *
 * Same definition as the proportional gain but for integral.
 * 
 *
 * @group Geometry
 * @decimal 3
 * @increment 0.1
 * @min 0
 * @max 10
 */
PARAM_DEFINE_FLOAT(CA_HELI_RPM_I, 0.0);

/**
 * Throw angle of swashplate servo at maximum commands for linearization
 *
 * Used to linearize mechanical output of swashplate servos to avoid axis coupling and binding with 4 servo redundancy.
 * This requires a symmetric setup where the servo horn is exactly centered with a 0 command.
 * Setting to zero disables feature.
 * 
 *
 * @group Geometry
 * @decimal 1
 * @increment 0.1
 * @min 0
 * @max 75
 * @unit deg
 */
PARAM_DEFINE_FLOAT(CA_MAX_SVO_THROW, 0.0);

/**
 * Motor failure handling mode
 *
 * This is used to specify how to handle motor failures
 * reported by failure detector.
 * 
 *
 * @group Geometry
 * @value 0 Ignore
 * @value 1 Remove first failed motor from effectiveness
 */
PARAM_DEFINE_INT32(CA_FAILURE_MODE, 0);

/**
 * EKF2 enable
 *
 * 
 *
 * @group EKF2
 * @boolean
 */
PARAM_DEFINE_INT32(EKF2_EN, 1);

/**
 * Verbose logging
 *
 * 
 *
 * @group EKF2
 * @boolean
 */
PARAM_DEFINE_INT32(EKF2_LOG_VERBOSE, 1);

/**
 * EKF prediction period
 *
 * EKF prediction period in microseconds. This should ideally be an integer multiple of the IMU time delta. Actual filter update will be an integer multiple of IMU update.
 *
 * @group EKF2
 * @min 1000
 * @max 20000
 * @unit us
 */
PARAM_DEFINE_INT32(EKF2_PREDICT_US, 10000);

/**
 * Maximum delay of all the aiding sensors
 *
 * Defines the delay between the current time and the delayed-time horizon. This value should be at least as large as the largest EKF2_XXX_DELAY parameter.
 *
 * @group EKF2
 * @decimal 1
 * @min 0
 * @max 1000
 * @unit ms
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(EKF2_DELAY_MAX, 200);

/**
 * 1-sigma tilt angle uncertainty after gravity vector alignment
 *
 * 
 *
 * @group EKF2
 * @decimal 3
 * @min 0.0
 * @max 0.5
 * @unit rad
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(EKF2_ANGERR_INIT, 0.1);

/**
 * Gate size for heading fusion
 *
 * Sets the number of standard deviations used by the innovation consistency test.
 *
 * @group EKF2
 * @decimal 1
 * @min 1.0
 * @unit SD
 */
PARAM_DEFINE_FLOAT(EKF2_HDG_GATE, 2.6);

/**
 * Measurement noise for magnetic heading fusion
 *
 * 
 *
 * @group EKF2
 * @decimal 2
 * @min 0.01
 * @max 1.0
 * @unit rad
 */
PARAM_DEFINE_FLOAT(EKF2_HEAD_NOISE, 0.3);

/**
 * Measurement noise for non-aiding position hold
 *
 * 
 *
 * @group EKF2
 * @decimal 1
 * @min 0.5
 * @max 50.0
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_NOAID_NOISE, 10.0);

/**
 * Maximum inertial dead-reckoning time
 *
 * Maximum lapsed time from last fusion of measurements that constrain velocity drift before the EKF will report the horizontal nav solution as invalid
 *
 * @group EKF2
 * @min 500000
 * @max 10000000
 * @unit us
 */
PARAM_DEFINE_INT32(EKF2_NOAID_TOUT, 5000000);

/**
 * Determines the reference source of height data used by the EKF
 *
 * When multiple height sources are enabled at the same time, the height estimate will always converge towards the reference height source selected by this parameter. The range sensor and vision options should only be used when for operation over a flat surface as the local NED origin will move up and down with ground level. If GPS is set as reference but altitude fusion is disabled in EKF2_GPS_CTRL, the GPS altitude is still used to initiaize the bias of the other height sensors.
 *
 * @group EKF2
 * @value 0 Barometric pressure
 * @value 1 GPS
 * @value 2 Range sensor
 * @value 3 Vision
 * @reboot_required True
 */
PARAM_DEFINE_INT32(EKF2_HGT_REF, 1);

/**
 * IMU control
 *
 * 
 *
 * @group EKF2
 * @bit 0 Gyro Bias
 * @bit 1 Accel Bias
 * @bit 2 Gravity vector fusion
 * @min 0
 * @max 7
 * @min 0
 * @max 7
 */
PARAM_DEFINE_INT32(EKF2_IMU_CTRL, 7);

/**
 * Rate gyro noise for covariance prediction
 *
 * 
 *
 * @group EKF2
 * @decimal 4
 * @min 0.0001
 * @max 0.1
 * @unit rad/s
 */
PARAM_DEFINE_FLOAT(EKF2_GYR_NOISE, 0.015);

/**
 * Accelerometer noise for covariance prediction
 *
 * 
 *
 * @group EKF2
 * @decimal 2
 * @min 0.01
 * @max 1.0
 * @unit m/s^2
 */
PARAM_DEFINE_FLOAT(EKF2_ACC_NOISE, 0.35);

/**
 * X position of IMU in body frame
 *
 * Forward axis with origin relative to vehicle centre of gravity
 *
 * @group EKF2
 * @decimal 3
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_IMU_POS_X, 0.0);

/**
 * Y position of IMU in body frame
 *
 * Forward axis with origin relative to vehicle centre of gravity
 *
 * @group EKF2
 * @decimal 3
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_IMU_POS_Y, 0.0);

/**
 * Z position of IMU in body frame
 *
 * Forward axis with origin relative to vehicle centre of gravity
 *
 * @group EKF2
 * @decimal 3
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_IMU_POS_Z, 0.0);

/**
 * Time constant of the velocity output prediction and smoothing filter
 *
 * 
 *
 * @group EKF2
 * @decimal 2
 * @max 1.0
 * @unit s
 */
PARAM_DEFINE_FLOAT(EKF2_TAU_VEL, 0.25);

/**
 * Output predictor position time constant
 *
 * Controls how tightly the output track the EKF states
 *
 * @group EKF2
 * @decimal 2
 * @min 0.1
 * @max 1.0
 * @unit s
 */
PARAM_DEFINE_FLOAT(EKF2_TAU_POS, 0.25);

/**
 * Velocity limit
 *
 * 
 *
 * @group EKF2
 * @decimal 1
 * @max 299792458
 * @unit m/s
 */
PARAM_DEFINE_FLOAT(EKF2_VEL_LIM, 100);

/**
 * 1-sigma IMU gyro switch-on bias
 *
 * 
 *
 * @group EKF2
 * @decimal 2
 * @min 0.0
 * @max 0.2
 * @unit rad/s
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(EKF2_GBIAS_INIT, 0.1);

/**
 * Process noise for IMU rate gyro bias prediction
 *
 * 
 *
 * @group EKF2
 * @decimal 6
 * @min 0.0
 * @max 0.01
 * @unit rad/s^2
 */
PARAM_DEFINE_FLOAT(EKF2_GYR_B_NOISE, 0.001);

/**
 * Gyro bias learning limit
 *
 * The ekf gyro bias states will be limited to within a range equivalent to +- of this value.
 *
 * @group EKF2
 * @decimal 3
 * @min 0.0
 * @max 0.4
 * @unit rad/s
 */
PARAM_DEFINE_FLOAT(EKF2_GYR_B_LIM, 0.15);

/**
 * 1-sigma IMU accelerometer switch-on bias
 *
 * 
 *
 * @group EKF2
 * @decimal 2
 * @min 0.0
 * @max 0.5
 * @unit m/s^2
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(EKF2_ABIAS_INIT, 0.2);

/**
 * Process noise for IMU accelerometer bias prediction
 *
 * 
 *
 * @group EKF2
 * @decimal 6
 * @min 0.0
 * @max 0.01
 * @unit m/s^3
 */
PARAM_DEFINE_FLOAT(EKF2_ACC_B_NOISE, 0.003);

/**
 * Accelerometer bias learning limit
 *
 * The ekf accel bias states will be limited to within a range equivalent to +- of this value.
 *
 * @group EKF2
 * @decimal 2
 * @min 0.0
 * @max 0.8
 * @unit m/s^2
 */
PARAM_DEFINE_FLOAT(EKF2_ABL_LIM, 0.4);

/**
 * Maximum IMU accel magnitude that allows IMU bias learning
 *
 * If the magnitude of the IMU accelerometer vector exceeds this value, the EKF accel bias state estimation will be inhibited. This reduces the adverse effect of high manoeuvre accelerations and IMU nonlinerity and scale factor errors on the accel bias estimates.
 *
 * @group EKF2
 * @decimal 1
 * @min 20.0
 * @max 200.0
 * @unit m/s^2
 */
PARAM_DEFINE_FLOAT(EKF2_ABL_ACCLIM, 25.0);

/**
 * Maximum IMU gyro angular rate magnitude that allows IMU bias learning
 *
 * If the magnitude of the IMU angular rate vector exceeds this value, the EKF accel bias state estimation will be inhibited. This reduces the adverse effect of rapid rotation rates and associated errors on the accel bias estimates.
 *
 * @group EKF2
 * @decimal 1
 * @min 2.0
 * @max 20.0
 * @unit rad/s
 */
PARAM_DEFINE_FLOAT(EKF2_ABL_GYRLIM, 3.0);

/**
 * Accel bias learning inhibit time constant
 *
 * The vector magnitude of angular rate and acceleration used to check if learning should be inhibited has a peak hold filter applied to it with an exponential decay. This parameter controls the time constant of the decay.
 *
 * @group EKF2
 * @decimal 2
 * @min 0.1
 * @max 1.0
 * @unit s
 */
PARAM_DEFINE_FLOAT(EKF2_ABL_TAU, 0.5);

/**
 * Multi-EKF IMUs
 *
 * Maximum number of IMUs to use for Multi-EKF. Set 0 to disable. Requires SENS_IMU_MODE 0.
 *
 * @group EKF2
 * @min 0
 * @max 4
 * @reboot_required True
 */
PARAM_DEFINE_INT32(EKF2_MULTI_IMU, 0);

/**
 * Multi-EKF Magnetometers
 *
 * Maximum number of magnetometers to use for Multi-EKF. Set 0 to disable. Requires SENS_MAG_MODE 0.
 *
 * @group EKF2
 * @min 0
 * @max 4
 * @reboot_required True
 */
PARAM_DEFINE_INT32(EKF2_MULTI_MAG, 0);

/**
 * Magnetic declination
 *
 * 
 *
 * @group EKF2
 * @decimal 1
 * @category System
 * @volatile True
 * @unit deg
 */
PARAM_DEFINE_FLOAT(EKF2_MAG_DECL, 0);

/**
 * Selector error reduce threshold
 *
 * EKF2 instances have to be better than the selected by at least this amount before their relative score can be reduced.
 *
 * @group EKF2
 */
PARAM_DEFINE_FLOAT(EKF2_SEL_ERR_RED, 0.2);

/**
 * Selector angular rate threshold
 *
 * EKF2 selector angular rate error threshold for comparing gyros. Angular rate vector differences larger than this will result in accumulated angular error.
 *
 * @group EKF2
 * @unit deg/s
 */
PARAM_DEFINE_FLOAT(EKF2_SEL_IMU_RAT, 7.0);

/**
 * Selector angular threshold
 *
 * EKF2 selector maximum accumulated angular error threshold for comparing gyros. Accumulated angular error larger than this will result in the sensor being declared faulty.
 *
 * @group EKF2
 * @unit deg
 */
PARAM_DEFINE_FLOAT(EKF2_SEL_IMU_ANG, 15.0);

/**
 * Selector acceleration threshold
 *
 * EKF2 selector acceleration error threshold for comparing accelerometers. Acceleration vector differences larger than this will result in accumulated velocity error.
 *
 * @group EKF2
 * @unit m/s^2
 */
PARAM_DEFINE_FLOAT(EKF2_SEL_IMU_ACC, 1.0);

/**
 * Selector angular threshold
 *
 * EKF2 selector maximum accumulated velocity threshold for comparing accelerometers. Accumulated velocity error larger than this will result in the sensor being declared faulty.
 *
 * @group EKF2
 * @unit m/s
 */
PARAM_DEFINE_FLOAT(EKF2_SEL_IMU_VEL, 2.0);

/**
 * Airspeed fusion threshold
 *
 * Airspeed data is fused for wind estimation if above this threshold. Set to 0 to disable airspeed fusion. For reliable wind estimation both sideslip (see EKF2_FUSE_BETA) and airspeed fusion should be enabled. Only applies to fixed-wing vehicles (or VTOLs in fixed-wing mode).
 *
 * @group EKF2
 * @decimal 1
 * @min 0.0
 * @unit m/s
 */
PARAM_DEFINE_FLOAT(EKF2_ARSP_THR, 0.0);

/**
 * Airspeed measurement delay relative to IMU measurements
 *
 * 
 *
 * @group EKF2
 * @decimal 1
 * @min 0
 * @max 300
 * @unit ms
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(EKF2_ASP_DELAY, 100);

/**
 * Gate size for TAS fusion
 *
 * Sets the number of standard deviations used by the innovation consistency test.
 *
 * @group EKF2
 * @decimal 1
 * @min 1.0
 * @unit SD
 */
PARAM_DEFINE_FLOAT(EKF2_TAS_GATE, 5.0);

/**
 * Measurement noise for airspeed fusion
 *
 * 
 *
 * @group EKF2
 * @decimal 1
 * @min 0.5
 * @max 5.0
 * @unit m/s
 */
PARAM_DEFINE_FLOAT(EKF2_EAS_NOISE, 1.4);

/**
 * Aux global position (AGP) sensor aiding
 *
 * Set bits in the following positions to enable: 0 : Horizontal position fusion 1 : Vertical position fusion
 *
 * @group EKF2
 * @bit 0 Horizontal position
 * @bit 1 Vertical position
 * @min 0
 * @max 3
 * @min 0
 * @max 3
 */
PARAM_DEFINE_INT32(EKF2_AGP_CTRL, 0);

/**
 * Aux global position estimator delay relative to IMU measurements
 *
 * 
 *
 * @group EKF2
 * @decimal 1
 * @min 0
 * @max 300
 * @unit ms
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(EKF2_AGP_DELAY, 0);

/**
 * Measurement noise for aux global position measurements
 *
 * Used to lower bound or replace the uncertainty included in the message
 *
 * @group EKF2
 * @decimal 2
 * @min 0.01
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_AGP_NOISE, 0.9);

/**
 * Gate size for aux global position fusion
 *
 * Sets the number of standard deviations used by the innovation consistency test.
 *
 * @group EKF2
 * @decimal 1
 * @min 1.0
 * @unit SD
 */
PARAM_DEFINE_FLOAT(EKF2_AGP_GATE, 3.0);

/**
 * Auxiliary Velocity Estimate delay relative to IMU measurements
 *
 * 
 *
 * @group EKF2
 * @decimal 1
 * @min 0
 * @max 300
 * @unit ms
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(EKF2_AVEL_DELAY, 5);

/**
 * Barometric sensor height aiding
 *
 * If this parameter is enabled then the estimator will make use of the barometric height measurements to estimate its height in addition to other height sources (if activated).
 *
 * @group EKF2
 * @boolean
 */
PARAM_DEFINE_INT32(EKF2_BARO_CTRL, 1);

/**
 * Barometer measurement delay relative to IMU measurements
 *
 * 
 *
 * @group EKF2
 * @decimal 1
 * @min 0
 * @max 300
 * @unit ms
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(EKF2_BARO_DELAY, 0);

/**
 * Gate size for barometric and GPS height fusion
 *
 * Sets the number of standard deviations used by the innovation consistency test.
 *
 * @group EKF2
 * @decimal 1
 * @min 1.0
 * @unit SD
 */
PARAM_DEFINE_FLOAT(EKF2_BARO_GATE, 5.0);

/**
 * Measurement noise for barometric altitude
 *
 * 
 *
 * @group EKF2
 * @decimal 2
 * @min 0.01
 * @max 15.0
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_BARO_NOISE, 3.5);

/**
 * Baro deadzone range for height fusion
 *
 * Sets the value of deadzone applied to negative baro innovations. Deadzone is enabled when EKF2_GND_EFF_DZ > 0.
 *
 * @group EKF2
 * @decimal 1
 * @min 0.0
 * @max 10.0
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_GND_EFF_DZ, 4.0);

/**
 * Height above ground level for ground effect zone
 *
 * Sets the maximum distance to the ground level where negative baro innovations are expected.
 *
 * @group EKF2
 * @decimal 1
 * @min 0.0
 * @max 5.0
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_GND_MAX_HGT, 0.5);

/**
 * Static pressure position error coefficient for the positive X axis
 *
 * This is the ratio of static pressure error to dynamic pressure generated by a positive wind relative velocity along the X body axis. If the baro height estimate rises during forward flight, then this will be a negative number.
 *
 * @group EKF2
 * @decimal 2
 * @min -0.5
 * @max 0.5
 */
PARAM_DEFINE_FLOAT(EKF2_PCOEF_XP, 0.0);

/**
 * Static pressure position error coefficient for the negative X axis
 *
 * This is the ratio of static pressure error to dynamic pressure generated by a negative wind relative velocity along the X body axis. If the baro height estimate rises during backwards flight, then this will be a negative number.
 *
 * @group EKF2
 * @decimal 2
 * @min -0.5
 * @max 0.5
 */
PARAM_DEFINE_FLOAT(EKF2_PCOEF_XN, 0.0);

/**
 * Pressure position error coefficient for the positive Y axis
 *
 * This is the ratio of static pressure error to dynamic pressure generated by a wind relative velocity along the positive Y (RH) body axis. If the baro height estimate rises during sideways flight to the right, then this will be a negative number.
 *
 * @group EKF2
 * @decimal 2
 * @min -0.5
 * @max 0.5
 */
PARAM_DEFINE_FLOAT(EKF2_PCOEF_YP, 0.0);

/**
 * Pressure position error coefficient for the negative Y axis
 *
 * This is the ratio of static pressure error to dynamic pressure generated by a wind relative velocity along the negative Y (LH) body axis. If the baro height estimate rises during sideways flight to the left, then this will be a negative number.
 *
 * @group EKF2
 * @decimal 2
 * @min -0.5
 * @max 0.5
 */
PARAM_DEFINE_FLOAT(EKF2_PCOEF_YN, 0.0);

/**
 * Static pressure position error coefficient for the Z axis
 *
 * This is the ratio of static pressure error to dynamic pressure generated by a wind relative velocity along the Z body axis.
 *
 * @group EKF2
 * @decimal 2
 * @min -0.5
 * @max 0.5
 */
PARAM_DEFINE_FLOAT(EKF2_PCOEF_Z, 0.0);

/**
 * Maximum airspeed used for baro static pressure compensation
 *
 * 
 *
 * @group EKF2
 * @decimal 1
 * @min 5.0
 * @max 50.0
 * @unit m/s
 */
PARAM_DEFINE_FLOAT(EKF2_ASPD_MAX, 20.0);

/**
 * Multirotor wind estimation selection
 *
 * Activate wind speed estimation using specific-force measurements and a drag model defined by EKF2_BCOEF_[XY] and EKF2_MCOEF. Only use on vehicles that have their thrust aligned with the Z axis and no thrust in the XY plane.
 *
 * @group EKF2
 * @boolean
 */
PARAM_DEFINE_INT32(EKF2_DRAG_CTRL, 0);

/**
 * Specific drag force observation noise variance
 *
 * Used by the multi-rotor specific drag force model. Increasing this makes the multi-rotor wind estimates adjust more slowly.
 *
 * @group EKF2
 * @decimal 2
 * @min 0.5
 * @max 10.0
 * @unit (m/s^2)^2
 */
PARAM_DEFINE_FLOAT(EKF2_DRAG_NOISE, 2.5);

/**
 * X-axis ballistic coefficient used for multi-rotor wind estimation
 *
 * This parameter controls the prediction of drag produced by bluff body drag along the forward/reverse axis when flying a multi-copter which enables estimation of wind drift when enabled by the EKF2_DRAG_CTRL parameter. The drag produced by this effect scales with speed squared. The predicted drag from the rotors is specified separately by the EKF2_MCOEF parameter. Set this parameter to zero to turn off the bluff body drag model for this axis.
 *
 * @group EKF2
 * @decimal 1
 * @min 0.0
 * @max 200.0
 * @unit kg/m^2
 */
PARAM_DEFINE_FLOAT(EKF2_BCOEF_X, 100.0);

/**
 * Y-axis ballistic coefficient used for multi-rotor wind estimation
 *
 * This parameter controls the prediction of drag produced by bluff body drag along the right/left axis when flying a multi-copter, which enables estimation of wind drift when enabled by the EKF2_DRAG_CTRL parameter. The drag produced by this effect scales with speed squared. The predicted drag from the rotors is specified separately by the EKF2_MCOEF parameter. Set this parameter to zero to turn off the bluff body drag model for this axis.
 *
 * @group EKF2
 * @decimal 1
 * @min 0.0
 * @max 200.0
 * @unit kg/m^2
 */
PARAM_DEFINE_FLOAT(EKF2_BCOEF_Y, 100.0);

/**
 * Propeller momentum drag coefficient for multi-rotor wind estimation
 *
 * This parameter controls the prediction of drag produced by the propellers when flying a multi-copter, which enables estimation of wind drift when enabled by the EKF2_DRAG_CTRL parameter. The drag produced by this effect scales with speed not speed squared and is produced because some of the air velocity normal to the propeller axis of rotation is lost when passing through the rotor disc. This  changes the momentum of the flow which creates a drag reaction force. When comparing un-ducted propellers of the same diameter, the effect is roughly proportional to the area of the propeller blades when viewed side on and changes with propeller selection. Momentum drag is significantly higher for ducted rotors. To account for the drag produced by the body which scales with speed squared, see documentation for the EKF2_BCOEF_X and EKF2_BCOEF_Y parameters. Set this parameter to zero to turn off the momentum drag model for both axis.
 *
 * @group EKF2
 * @decimal 2
 * @min 0
 * @max 1.0
 * @unit 1/s
 */
PARAM_DEFINE_FLOAT(EKF2_MCOEF, 0.15);

/**
 * External vision (EV) sensor aiding
 *
 * Set bits in the following positions to enable: 0 : Horizontal position fusion 1 : Vertical position fusion 2 : 3D velocity fusion 3 : Yaw
 *
 * @group EKF2
 * @bit 0 Horizontal position
 * @bit 1 Vertical position
 * @bit 2 3D velocity
 * @bit 3 Yaw
 * @min 0
 * @max 15
 * @min 0
 * @max 15
 */
PARAM_DEFINE_INT32(EKF2_EV_CTRL, 0);

/**
 * Vision Position Estimator delay relative to IMU measurements
 *
 * 
 *
 * @group EKF2
 * @decimal 1
 * @min 0
 * @max 300
 * @unit ms
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(EKF2_EV_DELAY, 0);

/**
 * External vision (EV) noise mode
 *
 * If set to 0 (default) the measurement noise is taken from the vision message and the EV noise parameters are used as a lower bound. If set to 1 the observation noise is set from the parameters directly,
 *
 * @group EKF2
 * @value 0 EV reported variance (parameter lower bound)
 * @value 1 EV noise parameters
 */
PARAM_DEFINE_INT32(EKF2_EV_NOISE_MD, 0);

/**
 * External vision (EV) minimum quality (optional)
 *
 * External vision will only be started and fused if the quality metric is above this threshold. The quality metric is a completely optional field provided by some VIO systems.
 *
 * @group EKF2
 * @decimal 1
 * @min 0
 * @max 100
 */
PARAM_DEFINE_INT32(EKF2_EV_QMIN, 0);

/**
 * Measurement noise for vision angle measurements
 *
 * Used to lower bound or replace the uncertainty included in the message
 *
 * @group EKF2
 * @decimal 2
 * @min 0.05
 * @unit rad
 */
PARAM_DEFINE_FLOAT(EKF2_EVA_NOISE, 0.1);

/**
 * Gate size for vision position fusion
 *
 * Sets the number of standard deviations used by the innovation consistency test.
 *
 * @group EKF2
 * @decimal 1
 * @min 1.0
 * @unit SD
 */
PARAM_DEFINE_FLOAT(EKF2_EVP_GATE, 5.0);

/**
 * Measurement noise for vision position measurements
 *
 * Used to lower bound or replace the uncertainty included in the message
 *
 * @group EKF2
 * @decimal 2
 * @min 0.01
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_EVP_NOISE, 0.1);

/**
 * Gate size for vision velocity estimate fusion
 *
 * Sets the number of standard deviations used by the innovation consistency test.
 *
 * @group EKF2
 * @decimal 1
 * @min 1.0
 * @unit SD
 */
PARAM_DEFINE_FLOAT(EKF2_EVV_GATE, 3.0);

/**
 * Measurement noise for vision velocity measurements
 *
 * Used to lower bound or replace the uncertainty included in the message
 *
 * @group EKF2
 * @decimal 2
 * @min 0.01
 * @unit m/s
 */
PARAM_DEFINE_FLOAT(EKF2_EVV_NOISE, 0.1);

/**
 * X position of VI sensor focal point in body frame
 *
 * Forward axis with origin relative to vehicle centre of gravity
 *
 * @group EKF2
 * @decimal 3
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_EV_POS_X, 0.0);

/**
 * Y position of VI sensor focal point in body frame
 *
 * Forward axis with origin relative to vehicle centre of gravity
 *
 * @group EKF2
 * @decimal 3
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_EV_POS_Y, 0.0);

/**
 * Z position of VI sensor focal point in body frame
 *
 * Forward axis with origin relative to vehicle centre of gravity
 *
 * @group EKF2
 * @decimal 3
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_EV_POS_Z, 0.0);

/**
 * GNSS sensor aiding
 *
 * Set bits in the following positions to enable: 0 : Longitude and latitude fusion 1 : Altitude fusion 2 : 3D velocity fusion 3 : Dual antenna heading fusion
 *
 * @group EKF2
 * @bit 0 Lon/lat
 * @bit 1 Altitude
 * @bit 2 3D velocity
 * @bit 3 Dual antenna heading
 * @min 0
 * @max 15
 * @min 0
 * @max 15
 */
PARAM_DEFINE_INT32(EKF2_GPS_CTRL, 7);

/**
 * GPS measurement delay relative to IMU measurements
 *
 * 
 *
 * @group EKF2
 * @decimal 1
 * @min 0
 * @max 300
 * @unit ms
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(EKF2_GPS_DELAY, 110);

/**
 * Measurement noise for GNSS position
 *
 * 
 *
 * @group EKF2
 * @decimal 2
 * @min 0.01
 * @max 10.0
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_GPS_P_NOISE, 0.5);

/**
 * Gate size for GNSS position fusion
 *
 * Sets the number of standard deviations used by the innovation consistency test.
 *
 * @group EKF2
 * @decimal 1
 * @min 1.0
 * @unit SD
 */
PARAM_DEFINE_FLOAT(EKF2_GPS_P_GATE, 5.0);

/**
 * Heading/Yaw offset for dual antenna GPS
 *
 * 
 *
 * @group EKF2
 * @decimal 1
 * @min 0.0
 * @max 360.0
 * @unit deg
 */
PARAM_DEFINE_FLOAT(EKF2_GPS_YAW_OFF, 0.0);

/**
 * Gate size for GNSS velocity fusion
 *
 * Sets the number of standard deviations used by the innovation consistency test.
 *
 * @group EKF2
 * @decimal 1
 * @min 1.0
 * @unit SD
 */
PARAM_DEFINE_FLOAT(EKF2_GPS_V_GATE, 5.0);

/**
 * Measurement noise for GNSS velocity
 *
 * 
 *
 * @group EKF2
 * @decimal 2
 * @min 0.01
 * @max 5.0
 * @unit m/s
 */
PARAM_DEFINE_FLOAT(EKF2_GPS_V_NOISE, 0.3);

/**
 * X position of GPS antenna in body frame
 *
 * Forward axis with origin relative to vehicle centre of gravity
 *
 * @group EKF2
 * @decimal 3
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_GPS_POS_X, 0.0);

/**
 * Y position of GPS antenna in body frame
 *
 * Forward axis with origin relative to vehicle centre of gravity
 *
 * @group EKF2
 * @decimal 3
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_GPS_POS_Y, 0.0);

/**
 * Z position of GPS antenna in body frame
 *
 * Forward axis with origin relative to vehicle centre of gravity
 *
 * @group EKF2
 * @decimal 3
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_GPS_POS_Z, 0.0);

/**
 * Integer bitmask controlling GPS checks
 *
 * Each threshold value is defined by the parameter indicated next to the check. Drift and offset checks only run when the vehicle is on ground and stationary.
 *
 * @group EKF2
 * @bit 0 Sat count (EKF2_REQ_NSATS)
 * @bit 1 PDOP (EKF2_REQ_PDOP)
 * @bit 2 EPH (EKF2_REQ_EPH)
 * @bit 3 EPV (EKF2_REQ_EPV)
 * @bit 4 Speed accuracy (EKF2_REQ_SACC)
 * @bit 5 Horizontal position drift (EKF2_REQ_HDRIFT)
 * @bit 6 Vertical position drift (EKF2_REQ_VDRIFT)
 * @bit 7 Horizontal speed offset (EKF2_REQ_HDRIFT)
 * @bit 8 Vertical speed offset (EKF2_REQ_VDRIFT)
 * @bit 9 Spoofing
 * @min 0
 * @max 1023
 * @min 0
 * @max 1023
 */
PARAM_DEFINE_INT32(EKF2_GPS_CHECK, 1023);

/**
 * Required EPH to use GPS
 *
 * 
 *
 * @group EKF2
 * @decimal 1
 * @min 2
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_REQ_EPH, 3.0);

/**
 * Required EPV to use GPS
 *
 * 
 *
 * @group EKF2
 * @decimal 1
 * @min 2
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_REQ_EPV, 5.0);

/**
 * Required speed accuracy to use GPS
 *
 * 
 *
 * @group EKF2
 * @decimal 2
 * @min 0.5
 * @max 5.0
 * @unit m/s
 */
PARAM_DEFINE_FLOAT(EKF2_REQ_SACC, 0.5);

/**
 * Required satellite count to use GPS
 *
 * 
 *
 * @group EKF2
 * @min 4
 * @max 12
 */
PARAM_DEFINE_INT32(EKF2_REQ_NSATS, 6);

/**
 * Maximum PDOP to use GPS
 *
 * 
 *
 * @group EKF2
 * @decimal 1
 * @min 1.5
 * @max 5.0
 */
PARAM_DEFINE_FLOAT(EKF2_REQ_PDOP, 2.5);

/**
 * Maximum horizontal drift speed to use GPS
 *
 * 
 *
 * @group EKF2
 * @decimal 2
 * @min 0.1
 * @max 1.0
 * @unit m/s
 */
PARAM_DEFINE_FLOAT(EKF2_REQ_HDRIFT, 0.1);

/**
 * Maximum vertical drift speed to use GPS
 *
 * 
 *
 * @group EKF2
 * @decimal 2
 * @min 0.1
 * @max 1.5
 * @unit m/s
 */
PARAM_DEFINE_FLOAT(EKF2_REQ_VDRIFT, 0.2);

/**
 * Required GPS health time on startup
 *
 * Minimum continuous period without GPS failure required to mark a healthy GPS status. It can be reduced to speed up initialization, but it's recommended to keep this unchanged for a vehicle.
 *
 * @group EKF2
 * @decimal 1
 * @min 0.1
 * @unit s
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(EKF2_REQ_GPS_H, 10.0);

/**
 * Default value of true airspeed used in EKF-GSF AHRS calculation
 *
 * If no airspeed measurements are available, the EKF-GSF AHRS calculation will assume this value of true airspeed when compensating for centripetal acceleration during turns. Set to zero to disable centripetal acceleration compensation during fixed wing flight modes.
 *
 * @group EKF2
 * @decimal 1
 * @min 0.0
 * @max 100.0
 * @unit m/s
 */
PARAM_DEFINE_FLOAT(EKF2_GSF_TAS, 15.0);

/**
 * Accelerometer measurement noise for gravity based observations
 *
 * 
 *
 * @group EKF2
 * @decimal 2
 * @min 0.1
 * @max 10.0
 * @unit g0
 */
PARAM_DEFINE_FLOAT(EKF2_GRAV_NOISE, 1.0);

/**
 * Type of magnetometer fusion
 *
 * Integer controlling the type of magnetometer fusion used - magnetic heading or 3-component vector. The fusion of magnetometer data as a three component vector enables vehicle body fixed hard iron errors to be learned, but requires a stable earth field. If set to 'Automatic' magnetic heading fusion is used when on-ground and 3-axis magnetic field fusion in-flight. If set to 'Magnetic heading' magnetic heading fusion is used at all times. If set to 'None' the magnetometer will not be used under any circumstance. If no external source of yaw is available, it is possible to use post-takeoff horizontal movement combined with GNSS velocity measurements to align the yaw angle. If set to 'Init' the magnetometer is only used to initalize the heading.
 *
 * @group EKF2
 * @value 0 Automatic
 * @value 1 Magnetic heading
 * @value 5 None
 * @value 6 Init
 * @reboot_required True
 */
PARAM_DEFINE_INT32(EKF2_MAG_TYPE, 0);

/**
 * Magnetometer measurement delay relative to IMU measurements
 *
 * 
 *
 * @group EKF2
 * @decimal 1
 * @min 0
 * @max 300
 * @unit ms
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(EKF2_MAG_DELAY, 0);

/**
 * Gate size for magnetometer XYZ component fusion
 *
 * Sets the number of standard deviations used by the innovation consistency test.
 *
 * @group EKF2
 * @decimal 1
 * @min 1.0
 * @unit SD
 */
PARAM_DEFINE_FLOAT(EKF2_MAG_GATE, 3.0);

/**
 * Measurement noise for magnetometer 3-axis fusion
 *
 * 
 *
 * @group EKF2
 * @decimal 3
 * @min 0.001
 * @max 1.0
 * @unit gauss
 */
PARAM_DEFINE_FLOAT(EKF2_MAG_NOISE, 0.05);

/**
 * Process noise for body magnetic field prediction
 *
 * 
 *
 * @group EKF2
 * @decimal 6
 * @min 0.0
 * @max 0.1
 * @unit gauss/s
 */
PARAM_DEFINE_FLOAT(EKF2_MAG_B_NOISE, 0.0001);

/**
 * Process noise for earth magnetic field prediction
 *
 * 
 *
 * @group EKF2
 * @decimal 6
 * @min 0.0
 * @max 0.1
 * @unit gauss/s
 */
PARAM_DEFINE_FLOAT(EKF2_MAG_E_NOISE, 0.001);

/**
 * Integer bitmask controlling handling of magnetic declination
 *
 * Set bits in the following positions to enable functions. 0 : Set to true to use the declination from the geo_lookup library when the GPS position becomes available, set to false to always use the EKF2_MAG_DECL value. 1 : Set to true to save the EKF2_MAG_DECL parameter to the value returned by the EKF when the vehicle disarms.
 *
 * @group EKF2
 * @bit 0 use geo_lookup declination
 * @bit 1 save EKF2_MAG_DECL on disarm
 * @min 0
 * @max 3
 * @min 0
 * @max 3
 * @reboot_required True
 */
PARAM_DEFINE_INT32(EKF2_DECL_TYPE, 3);

/**
 * Horizontal acceleration threshold used for heading observability check
 *
 * The heading is assumed to be observable when the body acceleration is greater than this parameter when a global position/velocity aiding source is active.
 *
 * @group EKF2
 * @decimal 2
 * @min 0.0
 * @max 5.0
 * @unit m/s^2
 */
PARAM_DEFINE_FLOAT(EKF2_MAG_ACCLIM, 0.5);

/**
 * Magnetic field strength test selection
 *
 * Bitmask to set which check is used to decide whether the magnetometer data is valid. If GNSS data is received, the magnetic field is compared to a World Magnetic Model (WMM), otherwise an average value is used. This check is useful to reject occasional hard iron disturbance. Set bits to 1 to enable checks. Checks enabled by the following bit positions 0 : Magnetic field strength. Set tolerance using EKF2_MAG_CHK_STR 1 : Magnetic field inclination. Set tolerance using EKF2_MAG_CHK_INC 2 : Wait for GNSS to find the theoretical strength and inclination using the WMM
 *
 * @group EKF2
 * @bit 0 Strength (EKF2_MAG_CHK_STR)
 * @bit 1 Inclination (EKF2_MAG_CHK_INC)
 * @bit 2 Wait for WMM
 * @min 0
 * @max 7
 * @min 0
 * @max 7
 */
PARAM_DEFINE_INT32(EKF2_MAG_CHECK, 1);

/**
 * Magnetic field strength check tolerance
 *
 * Maximum allowed deviation from the expected magnetic field strength to pass the check.
 *
 * @group EKF2
 * @decimal 2
 * @min 0.0
 * @max 1.0
 * @unit gauss
 */
PARAM_DEFINE_FLOAT(EKF2_MAG_CHK_STR, 0.2);

/**
 * Magnetic field inclination check tolerance
 *
 * Maximum allowed deviation from the expected magnetic field inclination to pass the check.
 *
 * @group EKF2
 * @decimal 1
 * @min 0.0
 * @max 90.0
 * @unit deg
 */
PARAM_DEFINE_FLOAT(EKF2_MAG_CHK_INC, 20.0);

/**
 * Enable synthetic magnetometer Z component measurement
 *
 * Use for vehicles where the measured body Z magnetic field is subject to strong magnetic interference. For magnetic heading fusion the magnetometer Z measurement will be replaced by a synthetic value calculated using the knowledge of the 3D magnetic field vector at the location of the drone. Therefore, this parameter will only have an effect if the global position of the drone is known. For 3D mag fusion the magnetometer Z measurement will simply be ignored instead of fusing the synthetic value.
 *
 * @group EKF2
 * @boolean
 */
PARAM_DEFINE_INT32(EKF2_SYNT_MAG_Z, 0);

/**
 * Optical flow aiding
 *
 * Enable optical flow fusion.
 *
 * @group EKF2
 * @boolean
 */
PARAM_DEFINE_INT32(EKF2_OF_CTRL, 1);

/**
 * Optical flow measurement delay relative to IMU measurements
 *
 * Assumes measurement is timestamped at trailing edge of integration period
 *
 * @group EKF2
 * @decimal 1
 * @min 0
 * @max 300
 * @unit ms
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(EKF2_OF_DELAY, 20);

/**
 * Gate size for optical flow fusion
 *
 * Sets the number of standard deviations used by the innovation consistency test.
 *
 * @group EKF2
 * @decimal 1
 * @min 1.0
 * @unit SD
 */
PARAM_DEFINE_FLOAT(EKF2_OF_GATE, 3.0);

/**
 * Optical flow minimum noise
 *
 * Measurement noise for the optical flow sensor when it's reported quality metric is at the maximum
 *
 * @group EKF2
 * @decimal 2
 * @min 0.05
 * @unit rad/s
 */
PARAM_DEFINE_FLOAT(EKF2_OF_N_MIN, 0.15);

/**
 * Optical flow maximum noise
 *
 * Measurement noise for the optical flow sensor when it's reported quality metric is at the minimum
 *
 * @group EKF2
 * @decimal 2
 * @min 0.05
 * @unit rad/s
 */
PARAM_DEFINE_FLOAT(EKF2_OF_N_MAX, 0.5);

/**
 * In air optical flow minimum quality
 *
 * Optical Flow data will only be used in air if the sensor reports a quality metric >= EKF2_OF_QMIN
 *
 * @group EKF2
 * @min 0
 * @max 255
 */
PARAM_DEFINE_INT32(EKF2_OF_QMIN, 1);

/**
 * On ground optical flow minimum quality
 *
 * Optical Flow data will only be used on the ground if the sensor reports a quality metric >= EKF2_OF_QMIN_GND
 *
 * @group EKF2
 * @min 0
 * @max 255
 */
PARAM_DEFINE_INT32(EKF2_OF_QMIN_GND, 0);

/**
 * Optical flow angular rate compensation source
 *
 * Auto: use gyro from optical flow message if available, internal gyro otherwise. Internal: always use internal gyro
 *
 * @group EKF2
 * @value 0 Auto
 * @value 1 Internal
 */
PARAM_DEFINE_INT32(EKF2_OF_GYR_SRC, 0);

/**
 * X position of optical flow focal point in body frame
 *
 * Forward axis with origin relative to vehicle centre of gravity
 *
 * @group EKF2
 * @decimal 3
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_OF_POS_X, 0.0);

/**
 * Y position of optical flow focal point in body frame
 *
 * Forward axis with origin relative to vehicle centre of gravity
 *
 * @group EKF2
 * @decimal 3
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_OF_POS_Y, 0.0);

/**
 * Z position of optical flow focal point in body frame
 *
 * Forward axis with origin relative to vehicle centre of gravity
 *
 * @group EKF2
 * @decimal 3
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_OF_POS_Z, 0.0);

/**
 * Range sensor height aiding
 *
 * WARNING: Range finder measurements are less reliable and can experience unexpected errors. For these reasons, if accurate control of height relative to ground is required, it is recommended to use the MPC_ALT_MODE parameter instead, unless baro errors are severe enough to cause problems with landing and takeoff. If this parameter is enabled then the estimator will make use of the range finder measurements to estimate its height in addition to other height sources (if activated). Range sensor aiding can be enabled (i.e.: always use) or set in "conditional" mode. Conditional mode: This enables the range finder to be used during low speed (< EKF2_RNG_A_VMAX) and low altitude (< EKF2_RNG_A_HMAX) operation, eg takeoff and landing, where baro interference from rotor wash is excessive and can corrupt EKF state estimates. It is intended to be used where a vertical takeoff and landing is performed, and horizontal flight does not occur until above EKF2_RNG_A_HMAX.
 *
 * @group EKF2
 * @value 0 Disable range fusion
 * @value 1 Enabled (conditional mode)
 * @value 2 Enabled
 */
PARAM_DEFINE_INT32(EKF2_RNG_CTRL, 1);

/**
 * Range finder measurement delay relative to IMU measurements
 *
 * 
 *
 * @group EKF2
 * @decimal 1
 * @min 0
 * @max 300
 * @unit ms
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(EKF2_RNG_DELAY, 5);

/**
 * Gate size for range finder fusion
 *
 * Sets the number of standard deviations used by the innovation consistency test.
 *
 * @group EKF2
 * @decimal 1
 * @min 1.0
 * @unit SD
 */
PARAM_DEFINE_FLOAT(EKF2_RNG_GATE, 5.0);

/**
 * Measurement noise for range finder fusion
 *
 * 
 *
 * @group EKF2
 * @decimal 2
 * @min 0.01
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_RNG_NOISE, 0.1);

/**
 * Range sensor pitch offset
 *
 * 
 *
 * @group EKF2
 * @decimal 3
 * @min -0.75
 * @max 0.75
 * @unit rad
 */
PARAM_DEFINE_FLOAT(EKF2_RNG_PITCH, 0.0);

/**
 * Maximum horizontal velocity allowed for conditional range aid mode
 *
 * If the vehicle horizontal speed exceeds this value then the estimator will not fuse range measurements to estimate its height. This only applies when conditional range aid mode is activated (EKF2_RNG_CTRL = 1).
 *
 * @group EKF2
 * @min 0.1
 * @max 2
 * @unit m/s
 */
PARAM_DEFINE_FLOAT(EKF2_RNG_A_VMAX, 1.0);

/**
 * Maximum height above ground allowed for conditional range aid mode
 *
 * If the vehicle absolute altitude exceeds this value then the estimator will not fuse range measurements to estimate its height. This only applies when conditional range aid mode is activated (EKF2_RNG_CTRL = 1).
 *
 * @group EKF2
 * @min 1.0
 * @max 10.0
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_RNG_A_HMAX, 5.0);

/**
 * Gate size used for innovation consistency checks for range aid fusion
 *
 * A lower value means HAGL needs to be more stable in order to use range finder for height estimation in range aid mode
 *
 * @group EKF2
 * @min 0.1
 * @max 5.0
 * @unit SD
 */
PARAM_DEFINE_FLOAT(EKF2_RNG_A_IGATE, 1.0);

/**
 * Minumum range validity period
 *
 * Minimum duration during which the reported range finder signal quality needs to be non-zero in order to be declared valid (s)
 *
 * @group EKF2
 * @min 0.1
 * @max 5
 * @unit s
 */
PARAM_DEFINE_FLOAT(EKF2_RNG_QLTY_T, 1.0);

/**
 * Gate size used for range finder kinematic consistency check
 *
 * To be used, the time derivative of the distance sensor measurements projected on the vertical axis needs to be statistically consistent with the estimated vertical velocity of the drone. Decrease this value to make the filter more robust against range finder faulty data (stuck, reflections, ...). Note: tune the range finder noise parameters (EKF2_RNG_NOISE and EKF2_RNG_SFE) before tuning this gate.
 *
 * @group EKF2
 * @min 0.1
 * @max 5.0
 * @unit SD
 */
PARAM_DEFINE_FLOAT(EKF2_RNG_K_GATE, 1.0);

/**
 * Range finder range dependent noise scaler
 *
 * Specifies the increase in range finder noise with range.
 *
 * @group EKF2
 * @min 0.0
 * @max 0.2
 * @unit m/m
 */
PARAM_DEFINE_FLOAT(EKF2_RNG_SFE, 0.05);

/**
 * X position of range finder origin in body frame
 *
 * Forward axis with origin relative to vehicle centre of gravity
 *
 * @group EKF2
 * @decimal 3
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_RNG_POS_X, 0.0);

/**
 * Y position of range finder origin in body frame
 *
 * Forward axis with origin relative to vehicle centre of gravity
 *
 * @group EKF2
 * @decimal 3
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_RNG_POS_Y, 0.0);

/**
 * Z position of range finder origin in body frame
 *
 * Forward axis with origin relative to vehicle centre of gravity
 *
 * @group EKF2
 * @decimal 3
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_RNG_POS_Z, 0.0);

/**
 * Maximum distance at which the range finder could detect fog (m)
 *
 * Limit for fog detection. If the range finder measures a distance greater than this value, the measurement is considered to not be blocked by fog or rain. If there's a jump from larger than RNG_FOG to smaller than EKF2_RNG_FOG, the measurement may gets rejected. 0 - disabled
 *
 * @group EKF2
 * @decimal 1
 * @min 0.0
 * @max 20.0
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_RNG_FOG, 3.0);

/**
 * Enable synthetic sideslip fusion
 *
 * For reliable wind estimation both sideslip and airspeed fusion (see EKF2_ARSP_THR) should be enabled. Only applies to vehicles in fixed-wing mode or with airspeed fusion active. Note: side slip fusion is currently not supported for tailsitters.
 *
 * @group EKF2
 * @boolean
 */
PARAM_DEFINE_INT32(EKF2_FUSE_BETA, 0);

/**
 * Gate size for synthetic sideslip fusion
 *
 * Sets the number of standard deviations used by the innovation consistency test.
 *
 * @group EKF2
 * @decimal 1
 * @min 1.0
 * @unit SD
 */
PARAM_DEFINE_FLOAT(EKF2_BETA_GATE, 5.0);

/**
 * Noise for synthetic sideslip fusion
 *
 * 
 *
 * @group EKF2
 * @decimal 2
 * @min 0.1
 * @max 1.0
 * @unit m/s
 */
PARAM_DEFINE_FLOAT(EKF2_BETA_NOISE, 0.3);

/**
 * Terrain altitude process noise
 *
 * 
 *
 * @group EKF2
 * @decimal 1
 * @min 0.5
 * @unit m/s
 */
PARAM_DEFINE_FLOAT(EKF2_TERR_NOISE, 5.0);

/**
 * Magnitude of terrain gradient
 *
 * 
 *
 * @group EKF2
 * @decimal 2
 * @min 0.0
 * @unit m/m
 */
PARAM_DEFINE_FLOAT(EKF2_TERR_GRAD, 0.5);

/**
 * Expected range finder reading when on ground
 *
 * If the vehicle is on ground, is not moving as determined by the motion test and the range finder is returning invalid or no data, then an assumed range value of EKF2_MIN_RNG will be used by the terrain estimator so that a terrain height estimate is available at the start of flight in situations where the range finder may be inside its minimum measurements distance when on ground.
 *
 * @group EKF2
 * @decimal 2
 * @min 0.01
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_MIN_RNG, 0.1);

/**
 * Process noise spectral density for wind velocity prediction
 *
 * When unaided, the wind estimate uncertainty (1-sigma, in m/s) increases by this amount every second.
 *
 * @group EKF2
 * @decimal 3
 * @min 0.0
 * @max 1.0
 * @unit m/s^2/sqrt(Hz)
 */
PARAM_DEFINE_FLOAT(EKF2_WIND_NSD, 0.05);

/**
 * MAVLink Mode for instance 0
 *
 * The MAVLink Mode defines the set of streamed messages (for example the
 * vehicle's attitude) and their sending rates.
 * 
 *
 * @group MAVLink
 * @value 0 Normal
 * @value 1 Custom
 * @value 2 Onboard
 * @value 3 OSD
 * @value 4 Magic
 * @value 5 Config
 * @value 7 Minimal
 * @value 8 External Vision
 * @value 10 Gimbal
 * @value 11 Onboard Low Bandwidth
 * @value 12 uAvionix
 * @value 13 Low Bandwidth
 * @reboot_required True
 */
PARAM_DEFINE_INT32(MAV_0_MODE, 0);

/**
 * MAVLink Mode for instance 1
 *
 * The MAVLink Mode defines the set of streamed messages (for example the
 * vehicle's attitude) and their sending rates.
 * 
 *
 * @group MAVLink
 * @value 0 Normal
 * @value 1 Custom
 * @value 2 Onboard
 * @value 3 OSD
 * @value 4 Magic
 * @value 5 Config
 * @value 7 Minimal
 * @value 8 External Vision
 * @value 10 Gimbal
 * @value 11 Onboard Low Bandwidth
 * @value 12 uAvionix
 * @value 13 Low Bandwidth
 * @reboot_required True
 */
PARAM_DEFINE_INT32(MAV_1_MODE, 2);

/**
 * MAVLink Mode for instance 2
 *
 * The MAVLink Mode defines the set of streamed messages (for example the
 * vehicle's attitude) and their sending rates.
 * 
 *
 * @group MAVLink
 * @value 0 Normal
 * @value 1 Custom
 * @value 2 Onboard
 * @value 3 OSD
 * @value 4 Magic
 * @value 5 Config
 * @value 7 Minimal
 * @value 8 External Vision
 * @value 10 Gimbal
 * @value 11 Onboard Low Bandwidth
 * @value 12 uAvionix
 * @value 13 Low Bandwidth
 * @reboot_required True
 */
PARAM_DEFINE_INT32(MAV_2_MODE, 0);

/**
 * Maximum MAVLink sending rate for instance 0
 *
 * Configure the maximum sending rate for the MAVLink streams in Bytes/sec.
 * If the configured streams exceed the maximum rate, the sending rate of
 * each stream is automatically decreased.
 * 
 * If this is set to 0 a value of half of the theoretical maximum bandwidth is used.
 * This corresponds to baudrate/20 Bytes/s (baudrate/10 = maximum data rate on
 * 8N1-configured links).
 * 
 *
 * @group MAVLink
 * @min 0
 * @unit B/s
 * @reboot_required True
 */
PARAM_DEFINE_INT32(MAV_0_RATE, 1200);

/**
 * Maximum MAVLink sending rate for instance 1
 *
 * Configure the maximum sending rate for the MAVLink streams in Bytes/sec.
 * If the configured streams exceed the maximum rate, the sending rate of
 * each stream is automatically decreased.
 * 
 * If this is set to 0 a value of half of the theoretical maximum bandwidth is used.
 * This corresponds to baudrate/20 Bytes/s (baudrate/10 = maximum data rate on
 * 8N1-configured links).
 * 
 *
 * @group MAVLink
 * @min 0
 * @unit B/s
 * @reboot_required True
 */
PARAM_DEFINE_INT32(MAV_1_RATE, 0);

/**
 * Maximum MAVLink sending rate for instance 2
 *
 * Configure the maximum sending rate for the MAVLink streams in Bytes/sec.
 * If the configured streams exceed the maximum rate, the sending rate of
 * each stream is automatically decreased.
 * 
 * If this is set to 0 a value of half of the theoretical maximum bandwidth is used.
 * This corresponds to baudrate/20 Bytes/s (baudrate/10 = maximum data rate on
 * 8N1-configured links).
 * 
 *
 * @group MAVLink
 * @min 0
 * @unit B/s
 * @reboot_required True
 */
PARAM_DEFINE_INT32(MAV_2_RATE, 0);

/**
 * Enable MAVLink Message forwarding for instance 0
 *
 * If enabled, forward incoming MAVLink messages to other MAVLink ports if the
 * message is either broadcast or the target is not the autopilot.
 * 
 * This allows for example a GCS to talk to a camera that is connected to the
 * autopilot via MAVLink (on a different link than the GCS).
 * 
 *
 * @group MAVLink
 * @boolean
 * @reboot_required True
 */
PARAM_DEFINE_INT32(MAV_0_FORWARD, 1);

/**
 * Enable MAVLink Message forwarding for instance 1
 *
 * If enabled, forward incoming MAVLink messages to other MAVLink ports if the
 * message is either broadcast or the target is not the autopilot.
 * 
 * This allows for example a GCS to talk to a camera that is connected to the
 * autopilot via MAVLink (on a different link than the GCS).
 * 
 *
 * @group MAVLink
 * @boolean
 * @reboot_required True
 */
PARAM_DEFINE_INT32(MAV_1_FORWARD, 0);

/**
 * Enable MAVLink Message forwarding for instance 2
 *
 * If enabled, forward incoming MAVLink messages to other MAVLink ports if the
 * message is either broadcast or the target is not the autopilot.
 * 
 * This allows for example a GCS to talk to a camera that is connected to the
 * autopilot via MAVLink (on a different link than the GCS).
 * 
 *
 * @group MAVLink
 * @boolean
 * @reboot_required True
 */
PARAM_DEFINE_INT32(MAV_2_FORWARD, 0);

/**
 * Enable software throttling of mavlink on instance 0
 *
 * If enabled, MAVLink messages will be throttled according to
 * `txbuf` field reported by radio_status.
 * 
 * Requires a radio to send the mavlink message RADIO_STATUS.
 * 
 *
 * @group MAVLink
 * @boolean
 * @reboot_required True
 */
PARAM_DEFINE_INT32(MAV_0_RADIO_CTL, 1);

/**
 * Enable software throttling of mavlink on instance 1
 *
 * If enabled, MAVLink messages will be throttled according to
 * `txbuf` field reported by radio_status.
 * 
 * Requires a radio to send the mavlink message RADIO_STATUS.
 * 
 *
 * @group MAVLink
 * @boolean
 * @reboot_required True
 */
PARAM_DEFINE_INT32(MAV_1_RADIO_CTL, 1);

/**
 * Enable software throttling of mavlink on instance 2
 *
 * If enabled, MAVLink messages will be throttled according to
 * `txbuf` field reported by radio_status.
 * 
 * Requires a radio to send the mavlink message RADIO_STATUS.
 * 
 *
 * @group MAVLink
 * @boolean
 * @reboot_required True
 */
PARAM_DEFINE_INT32(MAV_2_RADIO_CTL, 1);

/**
 * Enable serial flow control for instance 0
 *
 * This is used to force flow control on or off for the the mavlink
 * instance. By default it is auto detected. Use when auto detection fails.
 * 
 *
 * @group MAVLink
 * @value 0 Force off
 * @value 1 Force on
 * @value 2 Auto-detected
 * @reboot_required True
 */
PARAM_DEFINE_INT32(MAV_0_FLOW_CTRL, 2);

/**
 * Enable serial flow control for instance 1
 *
 * This is used to force flow control on or off for the the mavlink
 * instance. By default it is auto detected. Use when auto detection fails.
 * 
 *
 * @group MAVLink
 * @value 0 Force off
 * @value 1 Force on
 * @value 2 Auto-detected
 * @reboot_required True
 */
PARAM_DEFINE_INT32(MAV_1_FLOW_CTRL, 2);

/**
 * Enable serial flow control for instance 2
 *
 * This is used to force flow control on or off for the the mavlink
 * instance. By default it is auto detected. Use when auto detection fails.
 * 
 *
 * @group MAVLink
 * @value 0 Force off
 * @value 1 Force on
 * @value 2 Auto-detected
 * @reboot_required True
 */
PARAM_DEFINE_INT32(MAV_2_FLOW_CTRL, 2);

/**
 * Configures the frequency of HIGH_LATENCY2 stream for instance 0
 *
 * Positive real value that configures the transmission frequency of the
 * HIGH_LATENCY2 stream for instance 0, configured in iridium mode.
 * This parameter has no effect if the instance mode is different from iridium.
 * 
 *
 * @group MAVLink
 * @decimal 3
 * @increment 0.001
 * @min 0.0
 * @max 50.0
 * @unit Hz
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(MAV_0_HL_FREQ, 0.015);

/**
 * Configures the frequency of HIGH_LATENCY2 stream for instance 1
 *
 * Positive real value that configures the transmission frequency of the
 * HIGH_LATENCY2 stream for instance 1, configured in iridium mode.
 * This parameter has no effect if the instance mode is different from iridium.
 * 
 *
 * @group MAVLink
 * @decimal 3
 * @increment 0.001
 * @min 0.0
 * @max 50.0
 * @unit Hz
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(MAV_1_HL_FREQ, 0.015);

/**
 * Configures the frequency of HIGH_LATENCY2 stream for instance 2
 *
 * Positive real value that configures the transmission frequency of the
 * HIGH_LATENCY2 stream for instance 2, configured in iridium mode.
 * This parameter has no effect if the instance mode is different from iridium.
 * 
 *
 * @group MAVLink
 * @decimal 3
 * @increment 0.001
 * @min 0.0
 * @max 50.0
 * @unit Hz
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(MAV_2_HL_FREQ, 0.015);

/**
 * Accelerometer 0 calibration device ID
 *
 * Device ID of the accelerometer this calibration applies to.
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 */
PARAM_DEFINE_INT32(CAL_ACC0_ID, 0);

/**
 * Accelerometer 1 calibration device ID
 *
 * Device ID of the accelerometer this calibration applies to.
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 */
PARAM_DEFINE_INT32(CAL_ACC1_ID, 0);

/**
 * Accelerometer 2 calibration device ID
 *
 * Device ID of the accelerometer this calibration applies to.
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 */
PARAM_DEFINE_INT32(CAL_ACC2_ID, 0);

/**
 * Accelerometer 3 calibration device ID
 *
 * Device ID of the accelerometer this calibration applies to.
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 */
PARAM_DEFINE_INT32(CAL_ACC3_ID, 0);

/**
 * Accelerometer 0 priority
 *
 * 
 *
 * @group Sensor Calibration
 * @value -1 Uninitialized
 * @value 0 Disabled
 * @value 1 Min
 * @value 25 Low
 * @value 50 Medium (Default)
 * @value 75 High
 * @value 100 Max
 * @decimal 3
 * @category System
 */
PARAM_DEFINE_INT32(CAL_ACC0_PRIO, -1);

/**
 * Accelerometer 1 priority
 *
 * 
 *
 * @group Sensor Calibration
 * @value -1 Uninitialized
 * @value 0 Disabled
 * @value 1 Min
 * @value 25 Low
 * @value 50 Medium (Default)
 * @value 75 High
 * @value 100 Max
 * @decimal 3
 * @category System
 */
PARAM_DEFINE_INT32(CAL_ACC1_PRIO, -1);

/**
 * Accelerometer 2 priority
 *
 * 
 *
 * @group Sensor Calibration
 * @value -1 Uninitialized
 * @value 0 Disabled
 * @value 1 Min
 * @value 25 Low
 * @value 50 Medium (Default)
 * @value 75 High
 * @value 100 Max
 * @decimal 3
 * @category System
 */
PARAM_DEFINE_INT32(CAL_ACC2_PRIO, -1);

/**
 * Accelerometer 3 priority
 *
 * 
 *
 * @group Sensor Calibration
 * @value -1 Uninitialized
 * @value 0 Disabled
 * @value 1 Min
 * @value 25 Low
 * @value 50 Medium (Default)
 * @value 75 High
 * @value 100 Max
 * @decimal 3
 * @category System
 */
PARAM_DEFINE_INT32(CAL_ACC3_PRIO, -1);

/**
 * Accelerometer 0 rotation relative to airframe
 *
 * An internal sensor will force a value of -1, so a GCS should only attempt to configure the rotation if the value is greater than or equal to zero.
 * 
 *
 * @group Sensor Calibration
 * @value -1 Internal
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 * @value 8 Roll 180°
 * @value 9 Roll 180°, Yaw 45°
 * @value 10 Roll 180°, Yaw 90°
 * @value 11 Roll 180°, Yaw 135°
 * @value 12 Pitch 180°
 * @value 13 Roll 180°, Yaw 225°
 * @value 14 Roll 180°, Yaw 270°
 * @value 15 Roll 180°, Yaw 315°
 * @value 16 Roll 90°
 * @value 17 Roll 90°, Yaw 45°
 * @value 18 Roll 90°, Yaw 90°
 * @value 19 Roll 90°, Yaw 135°
 * @value 20 Roll 270°
 * @value 21 Roll 270°, Yaw 45°
 * @value 22 Roll 270°, Yaw 90°
 * @value 23 Roll 270°, Yaw 135°
 * @value 24 Pitch 90°
 * @value 25 Pitch 270°
 * @value 26 Pitch 180°, Yaw 90°
 * @value 27 Pitch 180°, Yaw 270°
 * @value 28 Roll 90°, Pitch 90°
 * @value 29 Roll 180°, Pitch 90°
 * @value 30 Roll 270°, Pitch 90°
 * @value 31 Roll 90°, Pitch 180°
 * @value 32 Roll 270°, Pitch 180°
 * @value 33 Roll 90°, Pitch 270°
 * @value 34 Roll 180°, Pitch 270°
 * @value 35 Roll 270°, Pitch 270°
 * @value 36 Roll 90°, Pitch 180°, Yaw 90°
 * @value 37 Roll 90°, Yaw 270°
 * @value 38 Roll 90°, Pitch 68°, Yaw 293°
 * @value 39 Pitch 315°
 * @value 40 Roll 90°, Pitch 315°
 * @category System
 * @min -1
 * @max 40
 */
PARAM_DEFINE_INT32(CAL_ACC0_ROT, -1);

/**
 * Accelerometer 1 rotation relative to airframe
 *
 * An internal sensor will force a value of -1, so a GCS should only attempt to configure the rotation if the value is greater than or equal to zero.
 * 
 *
 * @group Sensor Calibration
 * @value -1 Internal
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 * @value 8 Roll 180°
 * @value 9 Roll 180°, Yaw 45°
 * @value 10 Roll 180°, Yaw 90°
 * @value 11 Roll 180°, Yaw 135°
 * @value 12 Pitch 180°
 * @value 13 Roll 180°, Yaw 225°
 * @value 14 Roll 180°, Yaw 270°
 * @value 15 Roll 180°, Yaw 315°
 * @value 16 Roll 90°
 * @value 17 Roll 90°, Yaw 45°
 * @value 18 Roll 90°, Yaw 90°
 * @value 19 Roll 90°, Yaw 135°
 * @value 20 Roll 270°
 * @value 21 Roll 270°, Yaw 45°
 * @value 22 Roll 270°, Yaw 90°
 * @value 23 Roll 270°, Yaw 135°
 * @value 24 Pitch 90°
 * @value 25 Pitch 270°
 * @value 26 Pitch 180°, Yaw 90°
 * @value 27 Pitch 180°, Yaw 270°
 * @value 28 Roll 90°, Pitch 90°
 * @value 29 Roll 180°, Pitch 90°
 * @value 30 Roll 270°, Pitch 90°
 * @value 31 Roll 90°, Pitch 180°
 * @value 32 Roll 270°, Pitch 180°
 * @value 33 Roll 90°, Pitch 270°
 * @value 34 Roll 180°, Pitch 270°
 * @value 35 Roll 270°, Pitch 270°
 * @value 36 Roll 90°, Pitch 180°, Yaw 90°
 * @value 37 Roll 90°, Yaw 270°
 * @value 38 Roll 90°, Pitch 68°, Yaw 293°
 * @value 39 Pitch 315°
 * @value 40 Roll 90°, Pitch 315°
 * @category System
 * @min -1
 * @max 40
 */
PARAM_DEFINE_INT32(CAL_ACC1_ROT, -1);

/**
 * Accelerometer 2 rotation relative to airframe
 *
 * An internal sensor will force a value of -1, so a GCS should only attempt to configure the rotation if the value is greater than or equal to zero.
 * 
 *
 * @group Sensor Calibration
 * @value -1 Internal
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 * @value 8 Roll 180°
 * @value 9 Roll 180°, Yaw 45°
 * @value 10 Roll 180°, Yaw 90°
 * @value 11 Roll 180°, Yaw 135°
 * @value 12 Pitch 180°
 * @value 13 Roll 180°, Yaw 225°
 * @value 14 Roll 180°, Yaw 270°
 * @value 15 Roll 180°, Yaw 315°
 * @value 16 Roll 90°
 * @value 17 Roll 90°, Yaw 45°
 * @value 18 Roll 90°, Yaw 90°
 * @value 19 Roll 90°, Yaw 135°
 * @value 20 Roll 270°
 * @value 21 Roll 270°, Yaw 45°
 * @value 22 Roll 270°, Yaw 90°
 * @value 23 Roll 270°, Yaw 135°
 * @value 24 Pitch 90°
 * @value 25 Pitch 270°
 * @value 26 Pitch 180°, Yaw 90°
 * @value 27 Pitch 180°, Yaw 270°
 * @value 28 Roll 90°, Pitch 90°
 * @value 29 Roll 180°, Pitch 90°
 * @value 30 Roll 270°, Pitch 90°
 * @value 31 Roll 90°, Pitch 180°
 * @value 32 Roll 270°, Pitch 180°
 * @value 33 Roll 90°, Pitch 270°
 * @value 34 Roll 180°, Pitch 270°
 * @value 35 Roll 270°, Pitch 270°
 * @value 36 Roll 90°, Pitch 180°, Yaw 90°
 * @value 37 Roll 90°, Yaw 270°
 * @value 38 Roll 90°, Pitch 68°, Yaw 293°
 * @value 39 Pitch 315°
 * @value 40 Roll 90°, Pitch 315°
 * @category System
 * @min -1
 * @max 40
 */
PARAM_DEFINE_INT32(CAL_ACC2_ROT, -1);

/**
 * Accelerometer 3 rotation relative to airframe
 *
 * An internal sensor will force a value of -1, so a GCS should only attempt to configure the rotation if the value is greater than or equal to zero.
 * 
 *
 * @group Sensor Calibration
 * @value -1 Internal
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 * @value 8 Roll 180°
 * @value 9 Roll 180°, Yaw 45°
 * @value 10 Roll 180°, Yaw 90°
 * @value 11 Roll 180°, Yaw 135°
 * @value 12 Pitch 180°
 * @value 13 Roll 180°, Yaw 225°
 * @value 14 Roll 180°, Yaw 270°
 * @value 15 Roll 180°, Yaw 315°
 * @value 16 Roll 90°
 * @value 17 Roll 90°, Yaw 45°
 * @value 18 Roll 90°, Yaw 90°
 * @value 19 Roll 90°, Yaw 135°
 * @value 20 Roll 270°
 * @value 21 Roll 270°, Yaw 45°
 * @value 22 Roll 270°, Yaw 90°
 * @value 23 Roll 270°, Yaw 135°
 * @value 24 Pitch 90°
 * @value 25 Pitch 270°
 * @value 26 Pitch 180°, Yaw 90°
 * @value 27 Pitch 180°, Yaw 270°
 * @value 28 Roll 90°, Pitch 90°
 * @value 29 Roll 180°, Pitch 90°
 * @value 30 Roll 270°, Pitch 90°
 * @value 31 Roll 90°, Pitch 180°
 * @value 32 Roll 270°, Pitch 180°
 * @value 33 Roll 90°, Pitch 270°
 * @value 34 Roll 180°, Pitch 270°
 * @value 35 Roll 270°, Pitch 270°
 * @value 36 Roll 90°, Pitch 180°, Yaw 90°
 * @value 37 Roll 90°, Yaw 270°
 * @value 38 Roll 90°, Pitch 68°, Yaw 293°
 * @value 39 Pitch 315°
 * @value 40 Roll 90°, Pitch 315°
 * @category System
 * @min -1
 * @max 40
 */
PARAM_DEFINE_INT32(CAL_ACC3_ROT, -1);

/**
 * Accelerometer 0 X-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit m/s^2
 */
PARAM_DEFINE_FLOAT(CAL_ACC0_XOFF, 0.0);

/**
 * Accelerometer 1 X-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit m/s^2
 */
PARAM_DEFINE_FLOAT(CAL_ACC1_XOFF, 0.0);

/**
 * Accelerometer 2 X-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit m/s^2
 */
PARAM_DEFINE_FLOAT(CAL_ACC2_XOFF, 0.0);

/**
 * Accelerometer 3 X-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit m/s^2
 */
PARAM_DEFINE_FLOAT(CAL_ACC3_XOFF, 0.0);

/**
 * Accelerometer 0 Y-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit m/s^2
 */
PARAM_DEFINE_FLOAT(CAL_ACC0_YOFF, 0.0);

/**
 * Accelerometer 1 Y-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit m/s^2
 */
PARAM_DEFINE_FLOAT(CAL_ACC1_YOFF, 0.0);

/**
 * Accelerometer 2 Y-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit m/s^2
 */
PARAM_DEFINE_FLOAT(CAL_ACC2_YOFF, 0.0);

/**
 * Accelerometer 3 Y-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit m/s^2
 */
PARAM_DEFINE_FLOAT(CAL_ACC3_YOFF, 0.0);

/**
 * Accelerometer 0 Z-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit m/s^2
 */
PARAM_DEFINE_FLOAT(CAL_ACC0_ZOFF, 0.0);

/**
 * Accelerometer 1 Z-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit m/s^2
 */
PARAM_DEFINE_FLOAT(CAL_ACC1_ZOFF, 0.0);

/**
 * Accelerometer 2 Z-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit m/s^2
 */
PARAM_DEFINE_FLOAT(CAL_ACC2_ZOFF, 0.0);

/**
 * Accelerometer 3 Z-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit m/s^2
 */
PARAM_DEFINE_FLOAT(CAL_ACC3_ZOFF, 0.0);

/**
 * Accelerometer 0 X-axis scaling factor
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @min 0.1
 * @max 3.0
 */
PARAM_DEFINE_FLOAT(CAL_ACC0_XSCALE, 1.0);

/**
 * Accelerometer 1 X-axis scaling factor
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @min 0.1
 * @max 3.0
 */
PARAM_DEFINE_FLOAT(CAL_ACC1_XSCALE, 1.0);

/**
 * Accelerometer 2 X-axis scaling factor
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @min 0.1
 * @max 3.0
 */
PARAM_DEFINE_FLOAT(CAL_ACC2_XSCALE, 1.0);

/**
 * Accelerometer 3 X-axis scaling factor
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @min 0.1
 * @max 3.0
 */
PARAM_DEFINE_FLOAT(CAL_ACC3_XSCALE, 1.0);

/**
 * Accelerometer 0 Y-axis scaling factor
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @min 0.1
 * @max 3.0
 */
PARAM_DEFINE_FLOAT(CAL_ACC0_YSCALE, 1.0);

/**
 * Accelerometer 1 Y-axis scaling factor
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @min 0.1
 * @max 3.0
 */
PARAM_DEFINE_FLOAT(CAL_ACC1_YSCALE, 1.0);

/**
 * Accelerometer 2 Y-axis scaling factor
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @min 0.1
 * @max 3.0
 */
PARAM_DEFINE_FLOAT(CAL_ACC2_YSCALE, 1.0);

/**
 * Accelerometer 3 Y-axis scaling factor
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @min 0.1
 * @max 3.0
 */
PARAM_DEFINE_FLOAT(CAL_ACC3_YSCALE, 1.0);

/**
 * Accelerometer 0 Z-axis scaling factor
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @min 0.1
 * @max 3.0
 */
PARAM_DEFINE_FLOAT(CAL_ACC0_ZSCALE, 1.0);

/**
 * Accelerometer 1 Z-axis scaling factor
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @min 0.1
 * @max 3.0
 */
PARAM_DEFINE_FLOAT(CAL_ACC1_ZSCALE, 1.0);

/**
 * Accelerometer 2 Z-axis scaling factor
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @min 0.1
 * @max 3.0
 */
PARAM_DEFINE_FLOAT(CAL_ACC2_ZSCALE, 1.0);

/**
 * Accelerometer 3 Z-axis scaling factor
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @min 0.1
 * @max 3.0
 */
PARAM_DEFINE_FLOAT(CAL_ACC3_ZSCALE, 1.0);

/**
 * Barometer 0 calibration device ID
 *
 * Device ID of the barometer this calibration applies to.
 *
 * @group Sensor Calibration
 * @category System
 */
PARAM_DEFINE_INT32(CAL_BARO0_ID, 0);

/**
 * Barometer 1 calibration device ID
 *
 * Device ID of the barometer this calibration applies to.
 *
 * @group Sensor Calibration
 * @category System
 */
PARAM_DEFINE_INT32(CAL_BARO1_ID, 0);

/**
 * Barometer 2 calibration device ID
 *
 * Device ID of the barometer this calibration applies to.
 *
 * @group Sensor Calibration
 * @category System
 */
PARAM_DEFINE_INT32(CAL_BARO2_ID, 0);

/**
 * Barometer 3 calibration device ID
 *
 * Device ID of the barometer this calibration applies to.
 *
 * @group Sensor Calibration
 * @category System
 */
PARAM_DEFINE_INT32(CAL_BARO3_ID, 0);

/**
 * Barometer 0 priority
 *
 * 
 *
 * @group Sensor Calibration
 * @value -1 Uninitialized
 * @value 0 Disabled
 * @value 1 Min
 * @value 25 Low
 * @value 50 Medium (Default)
 * @value 75 High
 * @value 100 Max
 * @category System
 */
PARAM_DEFINE_INT32(CAL_BARO0_PRIO, -1);

/**
 * Barometer 1 priority
 *
 * 
 *
 * @group Sensor Calibration
 * @value -1 Uninitialized
 * @value 0 Disabled
 * @value 1 Min
 * @value 25 Low
 * @value 50 Medium (Default)
 * @value 75 High
 * @value 100 Max
 * @category System
 */
PARAM_DEFINE_INT32(CAL_BARO1_PRIO, -1);

/**
 * Barometer 2 priority
 *
 * 
 *
 * @group Sensor Calibration
 * @value -1 Uninitialized
 * @value 0 Disabled
 * @value 1 Min
 * @value 25 Low
 * @value 50 Medium (Default)
 * @value 75 High
 * @value 100 Max
 * @category System
 */
PARAM_DEFINE_INT32(CAL_BARO2_PRIO, -1);

/**
 * Barometer 3 priority
 *
 * 
 *
 * @group Sensor Calibration
 * @value -1 Uninitialized
 * @value 0 Disabled
 * @value 1 Min
 * @value 25 Low
 * @value 50 Medium (Default)
 * @value 75 High
 * @value 100 Max
 * @category System
 */
PARAM_DEFINE_INT32(CAL_BARO3_PRIO, -1);

/**
 * Barometer 0 offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 */
PARAM_DEFINE_FLOAT(CAL_BARO0_OFF, 0.0);

/**
 * Barometer 1 offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 */
PARAM_DEFINE_FLOAT(CAL_BARO1_OFF, 0.0);

/**
 * Barometer 2 offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 */
PARAM_DEFINE_FLOAT(CAL_BARO2_OFF, 0.0);

/**
 * Barometer 3 offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 */
PARAM_DEFINE_FLOAT(CAL_BARO3_OFF, 0.0);

/**
 * Gyroscope 0 calibration device ID
 *
 * Device ID of the gyroscope this calibration applies to.
 *
 * @group Sensor Calibration
 * @category System
 */
PARAM_DEFINE_INT32(CAL_GYRO0_ID, 0);

/**
 * Gyroscope 1 calibration device ID
 *
 * Device ID of the gyroscope this calibration applies to.
 *
 * @group Sensor Calibration
 * @category System
 */
PARAM_DEFINE_INT32(CAL_GYRO1_ID, 0);

/**
 * Gyroscope 2 calibration device ID
 *
 * Device ID of the gyroscope this calibration applies to.
 *
 * @group Sensor Calibration
 * @category System
 */
PARAM_DEFINE_INT32(CAL_GYRO2_ID, 0);

/**
 * Gyroscope 3 calibration device ID
 *
 * Device ID of the gyroscope this calibration applies to.
 *
 * @group Sensor Calibration
 * @category System
 */
PARAM_DEFINE_INT32(CAL_GYRO3_ID, 0);

/**
 * Gyroscope 0 priority
 *
 * 
 *
 * @group Sensor Calibration
 * @value -1 Uninitialized
 * @value 0 Disabled
 * @value 1 Min
 * @value 25 Low
 * @value 50 Medium (Default)
 * @value 75 High
 * @value 100 Max
 * @category System
 */
PARAM_DEFINE_INT32(CAL_GYRO0_PRIO, -1);

/**
 * Gyroscope 1 priority
 *
 * 
 *
 * @group Sensor Calibration
 * @value -1 Uninitialized
 * @value 0 Disabled
 * @value 1 Min
 * @value 25 Low
 * @value 50 Medium (Default)
 * @value 75 High
 * @value 100 Max
 * @category System
 */
PARAM_DEFINE_INT32(CAL_GYRO1_PRIO, -1);

/**
 * Gyroscope 2 priority
 *
 * 
 *
 * @group Sensor Calibration
 * @value -1 Uninitialized
 * @value 0 Disabled
 * @value 1 Min
 * @value 25 Low
 * @value 50 Medium (Default)
 * @value 75 High
 * @value 100 Max
 * @category System
 */
PARAM_DEFINE_INT32(CAL_GYRO2_PRIO, -1);

/**
 * Gyroscope 3 priority
 *
 * 
 *
 * @group Sensor Calibration
 * @value -1 Uninitialized
 * @value 0 Disabled
 * @value 1 Min
 * @value 25 Low
 * @value 50 Medium (Default)
 * @value 75 High
 * @value 100 Max
 * @category System
 */
PARAM_DEFINE_INT32(CAL_GYRO3_PRIO, -1);

/**
 * Gyroscope 0 rotation relative to airframe
 *
 * An internal sensor will force a value of -1, so a GCS should only attempt to configure the rotation if the value is greater than or equal to zero.
 * 
 *
 * @group Sensor Calibration
 * @value -1 Internal
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 * @value 8 Roll 180°
 * @value 9 Roll 180°, Yaw 45°
 * @value 10 Roll 180°, Yaw 90°
 * @value 11 Roll 180°, Yaw 135°
 * @value 12 Pitch 180°
 * @value 13 Roll 180°, Yaw 225°
 * @value 14 Roll 180°, Yaw 270°
 * @value 15 Roll 180°, Yaw 315°
 * @value 16 Roll 90°
 * @value 17 Roll 90°, Yaw 45°
 * @value 18 Roll 90°, Yaw 90°
 * @value 19 Roll 90°, Yaw 135°
 * @value 20 Roll 270°
 * @value 21 Roll 270°, Yaw 45°
 * @value 22 Roll 270°, Yaw 90°
 * @value 23 Roll 270°, Yaw 135°
 * @value 24 Pitch 90°
 * @value 25 Pitch 270°
 * @value 26 Pitch 180°, Yaw 90°
 * @value 27 Pitch 180°, Yaw 270°
 * @value 28 Roll 90°, Pitch 90°
 * @value 29 Roll 180°, Pitch 90°
 * @value 30 Roll 270°, Pitch 90°
 * @value 31 Roll 90°, Pitch 180°
 * @value 32 Roll 270°, Pitch 180°
 * @value 33 Roll 90°, Pitch 270°
 * @value 34 Roll 180°, Pitch 270°
 * @value 35 Roll 270°, Pitch 270°
 * @value 36 Roll 90°, Pitch 180°, Yaw 90°
 * @value 37 Roll 90°, Yaw 270°
 * @value 38 Roll 90°, Pitch 68°, Yaw 293°
 * @value 39 Pitch 315°
 * @value 40 Roll 90°, Pitch 315°
 * @category System
 * @min -1
 * @max 40
 */
PARAM_DEFINE_INT32(CAL_GYRO0_ROT, -1);

/**
 * Gyroscope 1 rotation relative to airframe
 *
 * An internal sensor will force a value of -1, so a GCS should only attempt to configure the rotation if the value is greater than or equal to zero.
 * 
 *
 * @group Sensor Calibration
 * @value -1 Internal
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 * @value 8 Roll 180°
 * @value 9 Roll 180°, Yaw 45°
 * @value 10 Roll 180°, Yaw 90°
 * @value 11 Roll 180°, Yaw 135°
 * @value 12 Pitch 180°
 * @value 13 Roll 180°, Yaw 225°
 * @value 14 Roll 180°, Yaw 270°
 * @value 15 Roll 180°, Yaw 315°
 * @value 16 Roll 90°
 * @value 17 Roll 90°, Yaw 45°
 * @value 18 Roll 90°, Yaw 90°
 * @value 19 Roll 90°, Yaw 135°
 * @value 20 Roll 270°
 * @value 21 Roll 270°, Yaw 45°
 * @value 22 Roll 270°, Yaw 90°
 * @value 23 Roll 270°, Yaw 135°
 * @value 24 Pitch 90°
 * @value 25 Pitch 270°
 * @value 26 Pitch 180°, Yaw 90°
 * @value 27 Pitch 180°, Yaw 270°
 * @value 28 Roll 90°, Pitch 90°
 * @value 29 Roll 180°, Pitch 90°
 * @value 30 Roll 270°, Pitch 90°
 * @value 31 Roll 90°, Pitch 180°
 * @value 32 Roll 270°, Pitch 180°
 * @value 33 Roll 90°, Pitch 270°
 * @value 34 Roll 180°, Pitch 270°
 * @value 35 Roll 270°, Pitch 270°
 * @value 36 Roll 90°, Pitch 180°, Yaw 90°
 * @value 37 Roll 90°, Yaw 270°
 * @value 38 Roll 90°, Pitch 68°, Yaw 293°
 * @value 39 Pitch 315°
 * @value 40 Roll 90°, Pitch 315°
 * @category System
 * @min -1
 * @max 40
 */
PARAM_DEFINE_INT32(CAL_GYRO1_ROT, -1);

/**
 * Gyroscope 2 rotation relative to airframe
 *
 * An internal sensor will force a value of -1, so a GCS should only attempt to configure the rotation if the value is greater than or equal to zero.
 * 
 *
 * @group Sensor Calibration
 * @value -1 Internal
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 * @value 8 Roll 180°
 * @value 9 Roll 180°, Yaw 45°
 * @value 10 Roll 180°, Yaw 90°
 * @value 11 Roll 180°, Yaw 135°
 * @value 12 Pitch 180°
 * @value 13 Roll 180°, Yaw 225°
 * @value 14 Roll 180°, Yaw 270°
 * @value 15 Roll 180°, Yaw 315°
 * @value 16 Roll 90°
 * @value 17 Roll 90°, Yaw 45°
 * @value 18 Roll 90°, Yaw 90°
 * @value 19 Roll 90°, Yaw 135°
 * @value 20 Roll 270°
 * @value 21 Roll 270°, Yaw 45°
 * @value 22 Roll 270°, Yaw 90°
 * @value 23 Roll 270°, Yaw 135°
 * @value 24 Pitch 90°
 * @value 25 Pitch 270°
 * @value 26 Pitch 180°, Yaw 90°
 * @value 27 Pitch 180°, Yaw 270°
 * @value 28 Roll 90°, Pitch 90°
 * @value 29 Roll 180°, Pitch 90°
 * @value 30 Roll 270°, Pitch 90°
 * @value 31 Roll 90°, Pitch 180°
 * @value 32 Roll 270°, Pitch 180°
 * @value 33 Roll 90°, Pitch 270°
 * @value 34 Roll 180°, Pitch 270°
 * @value 35 Roll 270°, Pitch 270°
 * @value 36 Roll 90°, Pitch 180°, Yaw 90°
 * @value 37 Roll 90°, Yaw 270°
 * @value 38 Roll 90°, Pitch 68°, Yaw 293°
 * @value 39 Pitch 315°
 * @value 40 Roll 90°, Pitch 315°
 * @category System
 * @min -1
 * @max 40
 */
PARAM_DEFINE_INT32(CAL_GYRO2_ROT, -1);

/**
 * Gyroscope 3 rotation relative to airframe
 *
 * An internal sensor will force a value of -1, so a GCS should only attempt to configure the rotation if the value is greater than or equal to zero.
 * 
 *
 * @group Sensor Calibration
 * @value -1 Internal
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 * @value 8 Roll 180°
 * @value 9 Roll 180°, Yaw 45°
 * @value 10 Roll 180°, Yaw 90°
 * @value 11 Roll 180°, Yaw 135°
 * @value 12 Pitch 180°
 * @value 13 Roll 180°, Yaw 225°
 * @value 14 Roll 180°, Yaw 270°
 * @value 15 Roll 180°, Yaw 315°
 * @value 16 Roll 90°
 * @value 17 Roll 90°, Yaw 45°
 * @value 18 Roll 90°, Yaw 90°
 * @value 19 Roll 90°, Yaw 135°
 * @value 20 Roll 270°
 * @value 21 Roll 270°, Yaw 45°
 * @value 22 Roll 270°, Yaw 90°
 * @value 23 Roll 270°, Yaw 135°
 * @value 24 Pitch 90°
 * @value 25 Pitch 270°
 * @value 26 Pitch 180°, Yaw 90°
 * @value 27 Pitch 180°, Yaw 270°
 * @value 28 Roll 90°, Pitch 90°
 * @value 29 Roll 180°, Pitch 90°
 * @value 30 Roll 270°, Pitch 90°
 * @value 31 Roll 90°, Pitch 180°
 * @value 32 Roll 270°, Pitch 180°
 * @value 33 Roll 90°, Pitch 270°
 * @value 34 Roll 180°, Pitch 270°
 * @value 35 Roll 270°, Pitch 270°
 * @value 36 Roll 90°, Pitch 180°, Yaw 90°
 * @value 37 Roll 90°, Yaw 270°
 * @value 38 Roll 90°, Pitch 68°, Yaw 293°
 * @value 39 Pitch 315°
 * @value 40 Roll 90°, Pitch 315°
 * @category System
 * @min -1
 * @max 40
 */
PARAM_DEFINE_INT32(CAL_GYRO3_ROT, -1);

/**
 * Gyroscope 0 X-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit rad/s
 */
PARAM_DEFINE_FLOAT(CAL_GYRO0_XOFF, 0.0);

/**
 * Gyroscope 1 X-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit rad/s
 */
PARAM_DEFINE_FLOAT(CAL_GYRO1_XOFF, 0.0);

/**
 * Gyroscope 2 X-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit rad/s
 */
PARAM_DEFINE_FLOAT(CAL_GYRO2_XOFF, 0.0);

/**
 * Gyroscope 3 X-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit rad/s
 */
PARAM_DEFINE_FLOAT(CAL_GYRO3_XOFF, 0.0);

/**
 * Gyroscope 0 Y-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit rad/s
 */
PARAM_DEFINE_FLOAT(CAL_GYRO0_YOFF, 0.0);

/**
 * Gyroscope 1 Y-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit rad/s
 */
PARAM_DEFINE_FLOAT(CAL_GYRO1_YOFF, 0.0);

/**
 * Gyroscope 2 Y-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit rad/s
 */
PARAM_DEFINE_FLOAT(CAL_GYRO2_YOFF, 0.0);

/**
 * Gyroscope 3 Y-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit rad/s
 */
PARAM_DEFINE_FLOAT(CAL_GYRO3_YOFF, 0.0);

/**
 * Gyroscope 0 Z-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit rad/s
 */
PARAM_DEFINE_FLOAT(CAL_GYRO0_ZOFF, 0.0);

/**
 * Gyroscope 1 Z-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit rad/s
 */
PARAM_DEFINE_FLOAT(CAL_GYRO1_ZOFF, 0.0);

/**
 * Gyroscope 2 Z-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit rad/s
 */
PARAM_DEFINE_FLOAT(CAL_GYRO2_ZOFF, 0.0);

/**
 * Gyroscope 3 Z-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit rad/s
 */
PARAM_DEFINE_FLOAT(CAL_GYRO3_ZOFF, 0.0);

/**
 * Magnetometer 0 calibration device ID
 *
 * Device ID of the magnetometer this calibration applies to.
 *
 * @group Sensor Calibration
 * @category System
 */
PARAM_DEFINE_INT32(CAL_MAG0_ID, 0);

/**
 * Magnetometer 1 calibration device ID
 *
 * Device ID of the magnetometer this calibration applies to.
 *
 * @group Sensor Calibration
 * @category System
 */
PARAM_DEFINE_INT32(CAL_MAG1_ID, 0);

/**
 * Magnetometer 2 calibration device ID
 *
 * Device ID of the magnetometer this calibration applies to.
 *
 * @group Sensor Calibration
 * @category System
 */
PARAM_DEFINE_INT32(CAL_MAG2_ID, 0);

/**
 * Magnetometer 3 calibration device ID
 *
 * Device ID of the magnetometer this calibration applies to.
 *
 * @group Sensor Calibration
 * @category System
 */
PARAM_DEFINE_INT32(CAL_MAG3_ID, 0);

/**
 * Magnetometer 0 priority
 *
 * 
 *
 * @group Sensor Calibration
 * @value -1 Uninitialized
 * @value 0 Disabled
 * @value 1 Min
 * @value 25 Low
 * @value 50 Medium (Default)
 * @value 75 High
 * @value 100 Max
 * @category System
 */
PARAM_DEFINE_INT32(CAL_MAG0_PRIO, -1);

/**
 * Magnetometer 1 priority
 *
 * 
 *
 * @group Sensor Calibration
 * @value -1 Uninitialized
 * @value 0 Disabled
 * @value 1 Min
 * @value 25 Low
 * @value 50 Medium (Default)
 * @value 75 High
 * @value 100 Max
 * @category System
 */
PARAM_DEFINE_INT32(CAL_MAG1_PRIO, -1);

/**
 * Magnetometer 2 priority
 *
 * 
 *
 * @group Sensor Calibration
 * @value -1 Uninitialized
 * @value 0 Disabled
 * @value 1 Min
 * @value 25 Low
 * @value 50 Medium (Default)
 * @value 75 High
 * @value 100 Max
 * @category System
 */
PARAM_DEFINE_INT32(CAL_MAG2_PRIO, -1);

/**
 * Magnetometer 3 priority
 *
 * 
 *
 * @group Sensor Calibration
 * @value -1 Uninitialized
 * @value 0 Disabled
 * @value 1 Min
 * @value 25 Low
 * @value 50 Medium (Default)
 * @value 75 High
 * @value 100 Max
 * @category System
 */
PARAM_DEFINE_INT32(CAL_MAG3_PRIO, -1);

/**
 * Magnetometer 0 rotation relative to airframe
 *
 * An internal sensor will force a value of -1, so a GCS should only attempt to configure the rotation if the value is greater than or equal to zero.
 * Set to "Custom Euler Angle" to define the rotation using CAL_MAG0_ROLL, CAL_MAG0_PITCH and CAL_MAG0_YAW.
 * 
 *
 * @group Sensor Calibration
 * @value -1 Internal
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 * @value 8 Roll 180°
 * @value 9 Roll 180°, Yaw 45°
 * @value 10 Roll 180°, Yaw 90°
 * @value 11 Roll 180°, Yaw 135°
 * @value 12 Pitch 180°
 * @value 13 Roll 180°, Yaw 225°
 * @value 14 Roll 180°, Yaw 270°
 * @value 15 Roll 180°, Yaw 315°
 * @value 16 Roll 90°
 * @value 17 Roll 90°, Yaw 45°
 * @value 18 Roll 90°, Yaw 90°
 * @value 19 Roll 90°, Yaw 135°
 * @value 20 Roll 270°
 * @value 21 Roll 270°, Yaw 45°
 * @value 22 Roll 270°, Yaw 90°
 * @value 23 Roll 270°, Yaw 135°
 * @value 24 Pitch 90°
 * @value 25 Pitch 270°
 * @value 26 Pitch 180°, Yaw 90°
 * @value 27 Pitch 180°, Yaw 270°
 * @value 28 Roll 90°, Pitch 90°
 * @value 29 Roll 180°, Pitch 90°
 * @value 30 Roll 270°, Pitch 90°
 * @value 31 Roll 90°, Pitch 180°
 * @value 32 Roll 270°, Pitch 180°
 * @value 33 Roll 90°, Pitch 270°
 * @value 34 Roll 180°, Pitch 270°
 * @value 35 Roll 270°, Pitch 270°
 * @value 36 Roll 90°, Pitch 180°, Yaw 90°
 * @value 37 Roll 90°, Yaw 270°
 * @value 38 Roll 90°, Pitch 68°, Yaw 293°
 * @value 39 Pitch 315°
 * @value 40 Roll 90°, Pitch 315°
 * @value 100 Custom Euler Angle
 * @category System
 * @min -1
 * @max 100
 */
PARAM_DEFINE_INT32(CAL_MAG0_ROT, -1);

/**
 * Magnetometer 1 rotation relative to airframe
 *
 * An internal sensor will force a value of -1, so a GCS should only attempt to configure the rotation if the value is greater than or equal to zero.
 * Set to "Custom Euler Angle" to define the rotation using CAL_MAG1_ROLL, CAL_MAG1_PITCH and CAL_MAG1_YAW.
 * 
 *
 * @group Sensor Calibration
 * @value -1 Internal
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 * @value 8 Roll 180°
 * @value 9 Roll 180°, Yaw 45°
 * @value 10 Roll 180°, Yaw 90°
 * @value 11 Roll 180°, Yaw 135°
 * @value 12 Pitch 180°
 * @value 13 Roll 180°, Yaw 225°
 * @value 14 Roll 180°, Yaw 270°
 * @value 15 Roll 180°, Yaw 315°
 * @value 16 Roll 90°
 * @value 17 Roll 90°, Yaw 45°
 * @value 18 Roll 90°, Yaw 90°
 * @value 19 Roll 90°, Yaw 135°
 * @value 20 Roll 270°
 * @value 21 Roll 270°, Yaw 45°
 * @value 22 Roll 270°, Yaw 90°
 * @value 23 Roll 270°, Yaw 135°
 * @value 24 Pitch 90°
 * @value 25 Pitch 270°
 * @value 26 Pitch 180°, Yaw 90°
 * @value 27 Pitch 180°, Yaw 270°
 * @value 28 Roll 90°, Pitch 90°
 * @value 29 Roll 180°, Pitch 90°
 * @value 30 Roll 270°, Pitch 90°
 * @value 31 Roll 90°, Pitch 180°
 * @value 32 Roll 270°, Pitch 180°
 * @value 33 Roll 90°, Pitch 270°
 * @value 34 Roll 180°, Pitch 270°
 * @value 35 Roll 270°, Pitch 270°
 * @value 36 Roll 90°, Pitch 180°, Yaw 90°
 * @value 37 Roll 90°, Yaw 270°
 * @value 38 Roll 90°, Pitch 68°, Yaw 293°
 * @value 39 Pitch 315°
 * @value 40 Roll 90°, Pitch 315°
 * @value 100 Custom Euler Angle
 * @category System
 * @min -1
 * @max 100
 */
PARAM_DEFINE_INT32(CAL_MAG1_ROT, -1);

/**
 * Magnetometer 2 rotation relative to airframe
 *
 * An internal sensor will force a value of -1, so a GCS should only attempt to configure the rotation if the value is greater than or equal to zero.
 * Set to "Custom Euler Angle" to define the rotation using CAL_MAG2_ROLL, CAL_MAG2_PITCH and CAL_MAG2_YAW.
 * 
 *
 * @group Sensor Calibration
 * @value -1 Internal
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 * @value 8 Roll 180°
 * @value 9 Roll 180°, Yaw 45°
 * @value 10 Roll 180°, Yaw 90°
 * @value 11 Roll 180°, Yaw 135°
 * @value 12 Pitch 180°
 * @value 13 Roll 180°, Yaw 225°
 * @value 14 Roll 180°, Yaw 270°
 * @value 15 Roll 180°, Yaw 315°
 * @value 16 Roll 90°
 * @value 17 Roll 90°, Yaw 45°
 * @value 18 Roll 90°, Yaw 90°
 * @value 19 Roll 90°, Yaw 135°
 * @value 20 Roll 270°
 * @value 21 Roll 270°, Yaw 45°
 * @value 22 Roll 270°, Yaw 90°
 * @value 23 Roll 270°, Yaw 135°
 * @value 24 Pitch 90°
 * @value 25 Pitch 270°
 * @value 26 Pitch 180°, Yaw 90°
 * @value 27 Pitch 180°, Yaw 270°
 * @value 28 Roll 90°, Pitch 90°
 * @value 29 Roll 180°, Pitch 90°
 * @value 30 Roll 270°, Pitch 90°
 * @value 31 Roll 90°, Pitch 180°
 * @value 32 Roll 270°, Pitch 180°
 * @value 33 Roll 90°, Pitch 270°
 * @value 34 Roll 180°, Pitch 270°
 * @value 35 Roll 270°, Pitch 270°
 * @value 36 Roll 90°, Pitch 180°, Yaw 90°
 * @value 37 Roll 90°, Yaw 270°
 * @value 38 Roll 90°, Pitch 68°, Yaw 293°
 * @value 39 Pitch 315°
 * @value 40 Roll 90°, Pitch 315°
 * @value 100 Custom Euler Angle
 * @category System
 * @min -1
 * @max 100
 */
PARAM_DEFINE_INT32(CAL_MAG2_ROT, -1);

/**
 * Magnetometer 3 rotation relative to airframe
 *
 * An internal sensor will force a value of -1, so a GCS should only attempt to configure the rotation if the value is greater than or equal to zero.
 * Set to "Custom Euler Angle" to define the rotation using CAL_MAG3_ROLL, CAL_MAG3_PITCH and CAL_MAG3_YAW.
 * 
 *
 * @group Sensor Calibration
 * @value -1 Internal
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 * @value 8 Roll 180°
 * @value 9 Roll 180°, Yaw 45°
 * @value 10 Roll 180°, Yaw 90°
 * @value 11 Roll 180°, Yaw 135°
 * @value 12 Pitch 180°
 * @value 13 Roll 180°, Yaw 225°
 * @value 14 Roll 180°, Yaw 270°
 * @value 15 Roll 180°, Yaw 315°
 * @value 16 Roll 90°
 * @value 17 Roll 90°, Yaw 45°
 * @value 18 Roll 90°, Yaw 90°
 * @value 19 Roll 90°, Yaw 135°
 * @value 20 Roll 270°
 * @value 21 Roll 270°, Yaw 45°
 * @value 22 Roll 270°, Yaw 90°
 * @value 23 Roll 270°, Yaw 135°
 * @value 24 Pitch 90°
 * @value 25 Pitch 270°
 * @value 26 Pitch 180°, Yaw 90°
 * @value 27 Pitch 180°, Yaw 270°
 * @value 28 Roll 90°, Pitch 90°
 * @value 29 Roll 180°, Pitch 90°
 * @value 30 Roll 270°, Pitch 90°
 * @value 31 Roll 90°, Pitch 180°
 * @value 32 Roll 270°, Pitch 180°
 * @value 33 Roll 90°, Pitch 270°
 * @value 34 Roll 180°, Pitch 270°
 * @value 35 Roll 270°, Pitch 270°
 * @value 36 Roll 90°, Pitch 180°, Yaw 90°
 * @value 37 Roll 90°, Yaw 270°
 * @value 38 Roll 90°, Pitch 68°, Yaw 293°
 * @value 39 Pitch 315°
 * @value 40 Roll 90°, Pitch 315°
 * @value 100 Custom Euler Angle
 * @category System
 * @min -1
 * @max 100
 */
PARAM_DEFINE_INT32(CAL_MAG3_ROT, -1);

/**
 * Magnetometer 0 Custom Euler Roll Angle
 *
 * Setting this parameter changes CAL_MAG0_ROT to "Custom Euler Angle"
 *
 * @group Sensor Calibration
 * @category System
 * @min -180
 * @max 180
 * @unit deg
 */
PARAM_DEFINE_FLOAT(CAL_MAG0_ROLL, 0.0);

/**
 * Magnetometer 1 Custom Euler Roll Angle
 *
 * Setting this parameter changes CAL_MAG1_ROT to "Custom Euler Angle"
 *
 * @group Sensor Calibration
 * @category System
 * @min -180
 * @max 180
 * @unit deg
 */
PARAM_DEFINE_FLOAT(CAL_MAG1_ROLL, 0.0);

/**
 * Magnetometer 2 Custom Euler Roll Angle
 *
 * Setting this parameter changes CAL_MAG2_ROT to "Custom Euler Angle"
 *
 * @group Sensor Calibration
 * @category System
 * @min -180
 * @max 180
 * @unit deg
 */
PARAM_DEFINE_FLOAT(CAL_MAG2_ROLL, 0.0);

/**
 * Magnetometer 3 Custom Euler Roll Angle
 *
 * Setting this parameter changes CAL_MAG3_ROT to "Custom Euler Angle"
 *
 * @group Sensor Calibration
 * @category System
 * @min -180
 * @max 180
 * @unit deg
 */
PARAM_DEFINE_FLOAT(CAL_MAG3_ROLL, 0.0);

/**
 * Magnetometer 0 Custom Euler Pitch Angle
 *
 * Setting this parameter changes CAL_MAG0_ROT to "Custom Euler Angle"
 *
 * @group Sensor Calibration
 * @category System
 * @min -180
 * @max 180
 * @unit deg
 */
PARAM_DEFINE_FLOAT(CAL_MAG0_PITCH, 0.0);

/**
 * Magnetometer 1 Custom Euler Pitch Angle
 *
 * Setting this parameter changes CAL_MAG1_ROT to "Custom Euler Angle"
 *
 * @group Sensor Calibration
 * @category System
 * @min -180
 * @max 180
 * @unit deg
 */
PARAM_DEFINE_FLOAT(CAL_MAG1_PITCH, 0.0);

/**
 * Magnetometer 2 Custom Euler Pitch Angle
 *
 * Setting this parameter changes CAL_MAG2_ROT to "Custom Euler Angle"
 *
 * @group Sensor Calibration
 * @category System
 * @min -180
 * @max 180
 * @unit deg
 */
PARAM_DEFINE_FLOAT(CAL_MAG2_PITCH, 0.0);

/**
 * Magnetometer 3 Custom Euler Pitch Angle
 *
 * Setting this parameter changes CAL_MAG3_ROT to "Custom Euler Angle"
 *
 * @group Sensor Calibration
 * @category System
 * @min -180
 * @max 180
 * @unit deg
 */
PARAM_DEFINE_FLOAT(CAL_MAG3_PITCH, 0.0);

/**
 * Magnetometer 0 Custom Euler Yaw Angle
 *
 * Setting this parameter changes CAL_MAG0_ROT to "Custom Euler Angle"
 *
 * @group Sensor Calibration
 * @category System
 * @min -180
 * @max 180
 * @unit deg
 */
PARAM_DEFINE_FLOAT(CAL_MAG0_YAW, 0.0);

/**
 * Magnetometer 1 Custom Euler Yaw Angle
 *
 * Setting this parameter changes CAL_MAG1_ROT to "Custom Euler Angle"
 *
 * @group Sensor Calibration
 * @category System
 * @min -180
 * @max 180
 * @unit deg
 */
PARAM_DEFINE_FLOAT(CAL_MAG1_YAW, 0.0);

/**
 * Magnetometer 2 Custom Euler Yaw Angle
 *
 * Setting this parameter changes CAL_MAG2_ROT to "Custom Euler Angle"
 *
 * @group Sensor Calibration
 * @category System
 * @min -180
 * @max 180
 * @unit deg
 */
PARAM_DEFINE_FLOAT(CAL_MAG2_YAW, 0.0);

/**
 * Magnetometer 3 Custom Euler Yaw Angle
 *
 * Setting this parameter changes CAL_MAG3_ROT to "Custom Euler Angle"
 *
 * @group Sensor Calibration
 * @category System
 * @min -180
 * @max 180
 * @unit deg
 */
PARAM_DEFINE_FLOAT(CAL_MAG3_YAW, 0.0);

/**
 * Magnetometer 0 X-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit gauss
 */
PARAM_DEFINE_FLOAT(CAL_MAG0_XOFF, 0.0);

/**
 * Magnetometer 1 X-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit gauss
 */
PARAM_DEFINE_FLOAT(CAL_MAG1_XOFF, 0.0);

/**
 * Magnetometer 2 X-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit gauss
 */
PARAM_DEFINE_FLOAT(CAL_MAG2_XOFF, 0.0);

/**
 * Magnetometer 3 X-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit gauss
 */
PARAM_DEFINE_FLOAT(CAL_MAG3_XOFF, 0.0);

/**
 * Magnetometer 0 Y-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit gauss
 */
PARAM_DEFINE_FLOAT(CAL_MAG0_YOFF, 0.0);

/**
 * Magnetometer 1 Y-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit gauss
 */
PARAM_DEFINE_FLOAT(CAL_MAG1_YOFF, 0.0);

/**
 * Magnetometer 2 Y-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit gauss
 */
PARAM_DEFINE_FLOAT(CAL_MAG2_YOFF, 0.0);

/**
 * Magnetometer 3 Y-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit gauss
 */
PARAM_DEFINE_FLOAT(CAL_MAG3_YOFF, 0.0);

/**
 * Magnetometer 0 Z-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit gauss
 */
PARAM_DEFINE_FLOAT(CAL_MAG0_ZOFF, 0.0);

/**
 * Magnetometer 1 Z-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit gauss
 */
PARAM_DEFINE_FLOAT(CAL_MAG1_ZOFF, 0.0);

/**
 * Magnetometer 2 Z-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit gauss
 */
PARAM_DEFINE_FLOAT(CAL_MAG2_ZOFF, 0.0);

/**
 * Magnetometer 3 Z-axis offset
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @unit gauss
 */
PARAM_DEFINE_FLOAT(CAL_MAG3_ZOFF, 0.0);

/**
 * Magnetometer 0 X-axis scaling factor
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @min 0.1
 * @max 3.0
 */
PARAM_DEFINE_FLOAT(CAL_MAG0_XSCALE, 1.0);

/**
 * Magnetometer 1 X-axis scaling factor
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @min 0.1
 * @max 3.0
 */
PARAM_DEFINE_FLOAT(CAL_MAG1_XSCALE, 1.0);

/**
 * Magnetometer 2 X-axis scaling factor
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @min 0.1
 * @max 3.0
 */
PARAM_DEFINE_FLOAT(CAL_MAG2_XSCALE, 1.0);

/**
 * Magnetometer 3 X-axis scaling factor
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @min 0.1
 * @max 3.0
 */
PARAM_DEFINE_FLOAT(CAL_MAG3_XSCALE, 1.0);

/**
 * Magnetometer 0 Y-axis scaling factor
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @min 0.1
 * @max 3.0
 */
PARAM_DEFINE_FLOAT(CAL_MAG0_YSCALE, 1.0);

/**
 * Magnetometer 1 Y-axis scaling factor
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @min 0.1
 * @max 3.0
 */
PARAM_DEFINE_FLOAT(CAL_MAG1_YSCALE, 1.0);

/**
 * Magnetometer 2 Y-axis scaling factor
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @min 0.1
 * @max 3.0
 */
PARAM_DEFINE_FLOAT(CAL_MAG2_YSCALE, 1.0);

/**
 * Magnetometer 3 Y-axis scaling factor
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @min 0.1
 * @max 3.0
 */
PARAM_DEFINE_FLOAT(CAL_MAG3_YSCALE, 1.0);

/**
 * Magnetometer 0 Z-axis scaling factor
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @min 0.1
 * @max 3.0
 */
PARAM_DEFINE_FLOAT(CAL_MAG0_ZSCALE, 1.0);

/**
 * Magnetometer 1 Z-axis scaling factor
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @min 0.1
 * @max 3.0
 */
PARAM_DEFINE_FLOAT(CAL_MAG1_ZSCALE, 1.0);

/**
 * Magnetometer 2 Z-axis scaling factor
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @min 0.1
 * @max 3.0
 */
PARAM_DEFINE_FLOAT(CAL_MAG2_ZSCALE, 1.0);

/**
 * Magnetometer 3 Z-axis scaling factor
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 * @min 0.1
 * @max 3.0
 */
PARAM_DEFINE_FLOAT(CAL_MAG3_ZSCALE, 1.0);

/**
 * Magnetometer 0 X-axis off diagonal scale factor
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 */
PARAM_DEFINE_FLOAT(CAL_MAG0_XODIAG, 0.0);

/**
 * Magnetometer 1 X-axis off diagonal scale factor
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 */
PARAM_DEFINE_FLOAT(CAL_MAG1_XODIAG, 0.0);

/**
 * Magnetometer 2 X-axis off diagonal scale factor
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 */
PARAM_DEFINE_FLOAT(CAL_MAG2_XODIAG, 0.0);

/**
 * Magnetometer 3 X-axis off diagonal scale factor
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 */
PARAM_DEFINE_FLOAT(CAL_MAG3_XODIAG, 0.0);

/**
 * Magnetometer 0 Y-axis off diagonal scale factor
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 */
PARAM_DEFINE_FLOAT(CAL_MAG0_YODIAG, 0.0);

/**
 * Magnetometer 1 Y-axis off diagonal scale factor
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 */
PARAM_DEFINE_FLOAT(CAL_MAG1_YODIAG, 0.0);

/**
 * Magnetometer 2 Y-axis off diagonal scale factor
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 */
PARAM_DEFINE_FLOAT(CAL_MAG2_YODIAG, 0.0);

/**
 * Magnetometer 3 Y-axis off diagonal scale factor
 *
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 */
PARAM_DEFINE_FLOAT(CAL_MAG3_YODIAG, 0.0);

/**
 * Magnetometer 0 Z-axis off diagonal scale factor
 *
 * 
 *
 * @group Sensor Calibration
 * @category System
 * @volatile True
 */
PARAM_DEFINE_FLOAT(CAL_MAG0_ZODIAG, 0.0);

/**
 * Magnetometer 1 Z-axis off diagonal scale factor
 *
 * 
 *
 * @group Sensor Calibration
 * @category System
 * @volatile True
 */
PARAM_DEFINE_FLOAT(CAL_MAG1_ZODIAG, 0.0);

/**
 * Magnetometer 2 Z-axis off diagonal scale factor
 *
 * 
 *
 * @group Sensor Calibration
 * @category System
 * @volatile True
 */
PARAM_DEFINE_FLOAT(CAL_MAG2_ZODIAG, 0.0);

/**
 * Magnetometer 3 Z-axis off diagonal scale factor
 *
 * 
 *
 * @group Sensor Calibration
 * @category System
 * @volatile True
 */
PARAM_DEFINE_FLOAT(CAL_MAG3_ZODIAG, 0.0);

/**
 * Magnetometer 0 X Axis throttle compensation
 *
 * Coefficient describing linear relationship between
 * X component of magnetometer in body frame axis
 * and either current or throttle depending on value of CAL_MAG_COMP_TYP.
 * Unit for throttle-based compensation is [G] and
 * for current-based compensation [G/kA]
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 */
PARAM_DEFINE_FLOAT(CAL_MAG0_XCOMP, 0.0);

/**
 * Magnetometer 1 X Axis throttle compensation
 *
 * Coefficient describing linear relationship between
 * X component of magnetometer in body frame axis
 * and either current or throttle depending on value of CAL_MAG_COMP_TYP.
 * Unit for throttle-based compensation is [G] and
 * for current-based compensation [G/kA]
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 */
PARAM_DEFINE_FLOAT(CAL_MAG1_XCOMP, 0.0);

/**
 * Magnetometer 2 X Axis throttle compensation
 *
 * Coefficient describing linear relationship between
 * X component of magnetometer in body frame axis
 * and either current or throttle depending on value of CAL_MAG_COMP_TYP.
 * Unit for throttle-based compensation is [G] and
 * for current-based compensation [G/kA]
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 */
PARAM_DEFINE_FLOAT(CAL_MAG2_XCOMP, 0.0);

/**
 * Magnetometer 3 X Axis throttle compensation
 *
 * Coefficient describing linear relationship between
 * X component of magnetometer in body frame axis
 * and either current or throttle depending on value of CAL_MAG_COMP_TYP.
 * Unit for throttle-based compensation is [G] and
 * for current-based compensation [G/kA]
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 */
PARAM_DEFINE_FLOAT(CAL_MAG3_XCOMP, 0.0);

/**
 * Magnetometer 0 Y Axis throttle compensation
 *
 * Coefficient describing linear relationship between
 * Y component of magnetometer in body frame axis
 * and either current or throttle depending on value of CAL_MAG_COMP_TYP.
 * Unit for throttle-based compensation is [G] and
 * for current-based compensation [G/kA]
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 */
PARAM_DEFINE_FLOAT(CAL_MAG0_YCOMP, 0.0);

/**
 * Magnetometer 1 Y Axis throttle compensation
 *
 * Coefficient describing linear relationship between
 * Y component of magnetometer in body frame axis
 * and either current or throttle depending on value of CAL_MAG_COMP_TYP.
 * Unit for throttle-based compensation is [G] and
 * for current-based compensation [G/kA]
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 */
PARAM_DEFINE_FLOAT(CAL_MAG1_YCOMP, 0.0);

/**
 * Magnetometer 2 Y Axis throttle compensation
 *
 * Coefficient describing linear relationship between
 * Y component of magnetometer in body frame axis
 * and either current or throttle depending on value of CAL_MAG_COMP_TYP.
 * Unit for throttle-based compensation is [G] and
 * for current-based compensation [G/kA]
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 */
PARAM_DEFINE_FLOAT(CAL_MAG2_YCOMP, 0.0);

/**
 * Magnetometer 3 Y Axis throttle compensation
 *
 * Coefficient describing linear relationship between
 * Y component of magnetometer in body frame axis
 * and either current or throttle depending on value of CAL_MAG_COMP_TYP.
 * Unit for throttle-based compensation is [G] and
 * for current-based compensation [G/kA]
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 */
PARAM_DEFINE_FLOAT(CAL_MAG3_YCOMP, 0.0);

/**
 * Magnetometer 0 Z Axis throttle compensation
 *
 * Coefficient describing linear relationship between
 * Z component of magnetometer in body frame axis
 * and either current or throttle depending on value of CAL_MAG_COMP_TYP.
 * Unit for throttle-based compensation is [G] and
 * for current-based compensation [G/kA]
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 */
PARAM_DEFINE_FLOAT(CAL_MAG0_ZCOMP, 0.0);

/**
 * Magnetometer 1 Z Axis throttle compensation
 *
 * Coefficient describing linear relationship between
 * Z component of magnetometer in body frame axis
 * and either current or throttle depending on value of CAL_MAG_COMP_TYP.
 * Unit for throttle-based compensation is [G] and
 * for current-based compensation [G/kA]
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 */
PARAM_DEFINE_FLOAT(CAL_MAG1_ZCOMP, 0.0);

/**
 * Magnetometer 2 Z Axis throttle compensation
 *
 * Coefficient describing linear relationship between
 * Z component of magnetometer in body frame axis
 * and either current or throttle depending on value of CAL_MAG_COMP_TYP.
 * Unit for throttle-based compensation is [G] and
 * for current-based compensation [G/kA]
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 */
PARAM_DEFINE_FLOAT(CAL_MAG2_ZCOMP, 0.0);

/**
 * Magnetometer 3 Z Axis throttle compensation
 *
 * Coefficient describing linear relationship between
 * Z component of magnetometer in body frame axis
 * and either current or throttle depending on value of CAL_MAG_COMP_TYP.
 * Unit for throttle-based compensation is [G] and
 * for current-based compensation [G/kA]
 * 
 *
 * @group Sensor Calibration
 * @decimal 3
 * @category System
 * @volatile True
 */
PARAM_DEFINE_FLOAT(CAL_MAG3_ZCOMP, 0.0);

/**
 * uXRCE-DDS domain ID
 *
 * uXRCE-DDS domain ID
 *
 * @group UXRCE-DDS Client
 * @category System
 * @reboot_required True
 */
PARAM_DEFINE_INT32(UXRCE_DDS_DOM_ID, 0);

/**
 * uXRCE-DDS session key
 *
 * uXRCE-DDS key, must be different from zero.
 * In a single agent - multi client configuration, each client
 * must have a unique session key.
 * 
 *
 * @group UXRCE-DDS Client
 * @category System
 * @reboot_required True
 */
PARAM_DEFINE_INT32(UXRCE_DDS_KEY, 1);

/**
 * uXRCE-DDS participant configuration
 *
 * Set the participant configuration on the Agent's system.
 * 0: Use the default configuration.
 * 1: Restrict messages to localhost
 *    (use in combination with ROS_LOCALHOST_ONLY=1).
 * 2: Use a custom participant with the profile name "px4_participant".
 * 
 *
 * @group UXRCE-DDS Client
 * @value 0 Default
 * @value 1 Localhost-only
 * @value 2 Custom participant
 * @category System
 * @min 0
 * @max 2
 * @reboot_required True
 */
PARAM_DEFINE_INT32(UXRCE_DDS_PTCFG, 0);

/**
 * Enable uXRCE-DDS timestamp synchronization
 *
 * When enabled, uxrce_dds_client will synchronize the timestamps of the incoming and outgoing messages measuring the offset between the Agent OS time and the PX4 time.
 *
 * @group UXRCE-DDS Client
 * @boolean
 * @category System
 * @reboot_required True
 */
PARAM_DEFINE_INT32(UXRCE_DDS_SYNCT, 1);

/**
 * Enable uXRCE-DDS system clock synchronization
 *
 * When enabled along with UXRCE_DDS_SYNCT, uxrce_dds_client will set the system clock using the agents UTC timestamp.
 *
 * @group UXRCE-DDS Client
 * @boolean
 * @category System
 * @reboot_required True
 */
PARAM_DEFINE_INT32(UXRCE_DDS_SYNCC, 0);

/**
 * TX rate timeout configuration
 *
 * Specifies after how many seconds without sending data the DDS connection is reestablished.
 * A value less than one disables the TX rate timeout.
 * 
 *
 * @group UXRCE-DDS Client
 * @category System
 * @unit s
 * @reboot_required True
 */
PARAM_DEFINE_INT32(UXRCE_DDS_TX_TO, 3);

/**
 * RX rate timeout configuration
 *
 * Specifies after how many seconds without receiving data the DDS connection is reestablished.
 * A value less than one disables the RX rate timeout.
 * 
 *
 * @group UXRCE-DDS Client
 * @category System
 * @unit s
 * @reboot_required True
 */
PARAM_DEFINE_INT32(UXRCE_DDS_RX_TO, -1);

/**
 * The KP parameter for the position control in the horizontal direction
 *
 * 
 *
 * @group TRIF
 * @min 0.0
 */
PARAM_DEFINE_FLOAT(TRIF_POS_KP_XY, 5.0);

/**
 * The KP parameter for the position control in the vertical direction
 *
 * 
 *
 * @group TRIF
 * @min 0.0
 */
PARAM_DEFINE_FLOAT(TRIF_POS_KP_Z, 3);

/**
 * The KP parameter for the velocity control in the horizontal direction
 *
 * 
 *
 * @group TRIF
 * @min 0.0
 */
PARAM_DEFINE_FLOAT(TRIF_VEL_KP_XY, 3.0);

/**
 * The KP parameter for the velocity control in the vertical direction
 *
 * 
 *
 * @group TRIF
 * @min 0.0
 */
PARAM_DEFINE_FLOAT(TRIF_VEL_KP_Z, 2);

/**
 * The KI parameter for the velocity control in the horizontal direction
 *
 * 
 *
 * @group TRIF
 * @min 0.0
 */
PARAM_DEFINE_FLOAT(TRIF_VEL_KI_XY, 1.5);

/**
 * The KI parameter for the velocity control in the vertical direction
 *
 * 
 *
 * @group TRIF
 * @min 0.0
 */
PARAM_DEFINE_FLOAT(TRIF_VEL_KI_Z, 1.5);

/**
 * The executor of the acceleration command
 *
 * Configure the executor of the acceleration command. Can be selected between Simple and Complete.
 * When configured to Simple, the controller will execute the acceleration command by publishing
 * a message to the trajectory_setpoint topic, which contains only the acceleration command, and
 * the following control is executed by the original PX4 controller. In contrast, when configured
 * to Complete, the controller will do the hover thrust estimation and attitude calculation, and
 * then publish the attitude command to the attitude_setpoint topic, the following control is
 * executed by the original PX4 controller.
 * 
 *
 * @group TRIF
 * @value 0 Simple
 * @value 1 Complete
 */
PARAM_DEFINE_INT32(TRIF_ACC_EXE, 0);
