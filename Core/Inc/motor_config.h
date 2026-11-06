#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#define SPEED_AVERAGE_LENGTH 10U
#define CONTROL_PERIOD_MS 10U
#define P_FINE_BAND_RPM 2.0f
#define SPEED_REPORT_PERIOD_MS 1000U
// Counts per output-shaft revolution in TI12 mode: verify on your motor.
#define ENCODER_COUNTS_PER_REV 1320.0f
// Moving average over 10 speed samples: 100 ms window at a 10 ms period.
// Larger windows smooth more but delay the speed feedback.
// At 1320 counts/rev, one count over this window represents about 0.455 RPM.
#if SPEED_AVERAGE_LENGTH < 1U || SPEED_AVERAGE_LENGTH > 200U
#error "SPEED_AVERAGE_LENGTH must be between 1 and 200"
#endif
// Positive RPM means vehicle-forward for both mirrored wheels.
#define LEFT_ENCODER_DIRECTION   1
#define RIGHT_ENCODER_DIRECTION -1
#define LEFT_DRIVE_DIRECTION     1
#define RIGHT_DRIVE_DIRECTION   -1
#define TARGET_STOP_EPSILON_RPM 0.01f

#endif
