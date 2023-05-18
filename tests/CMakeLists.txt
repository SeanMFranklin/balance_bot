
#mbot_dmp_test
add_executable(mbot_dmp_test
  mbot_dmp_test.c
)
target_link_libraries(mbot_dmp_test
  pico_stdlib
  pico_multicore
  hardware_pwm
  hardware_sync
  hardware_i2c
  hardware_flash
  rclib
  comms
)
pico_enable_stdio_usb(mbot_dmp_test 1)
pico_enable_stdio_uart(mbot_dmp_test 0)
pico_add_extra_outputs(mbot_dmp_test)

#mbot_calibrate_accel
add_executable(mbot_calibrate_accel
  mbot_calibrate_accel.c
)
target_link_libraries(mbot_calibrate_accel
  pico_stdlib
  pico_multicore
  hardware_pwm
  hardware_sync
  hardware_i2c
  hardware_flash
  rclib
  comms
)
pico_enable_stdio_usb(mbot_calibrate_accel 1)
pico_enable_stdio_uart(mbot_calibrate_accel 0)
pico_add_extra_outputs(mbot_calibrate_accel)

#mbot_calibrate_gyro
add_executable(mbot_calibrate_gyro
  mbot_calibrate_gyro.c
)
target_link_libraries(mbot_calibrate_gyro
  pico_stdlib
  pico_multicore
  hardware_pwm
  hardware_sync
  hardware_i2c
  hardware_flash
  rclib
  comms
)
pico_enable_stdio_usb(mbot_calibrate_gyro 1)
pico_enable_stdio_uart(mbot_calibrate_gyro 0)
pico_add_extra_outputs(mbot_calibrate_gyro)

#MATRIX-TESTS
add_executable(rc_matrix_test
rc_matrix_test.c
)
target_link_libraries(rc_matrix_test
  pico_stdlib
  rclib
)
pico_enable_stdio_usb(rc_matrix_test 1)
pico_add_extra_outputs(rc_matrix_test)


#FRAM-TEST
add_executable(mbot_fram_test
  mbot_fram_test.c
)
target_link_libraries(mbot_fram_test
  pico_stdlib
  hardware_i2c
  rclib
)
pico_enable_stdio_usb(mbot_fram_test 1)
pico_enable_stdio_uart(mbot_fram_test 0)
pico_add_extra_outputs(mbot_fram_test)

#MOTOR-TEST
add_executable(mbot_motor_test
  mbot_motor_test.c
)
target_link_libraries(mbot_motor_test
  pico_stdlib
  pico_multicore
  hardware_pio
  hardware_pwm
  rclib
)
pico_enable_stdio_usb(mbot_motor_test 1)
pico_enable_stdio_uart(mbot_motor_test 0)
pico_add_extra_outputs(mbot_motor_test)

#MOTOR-ANALYZE
add_executable(mbot_motor_analyze
  mbot_motor_analyze.c
)
target_link_libraries(mbot_motor_analyze
  pico_stdlib
  pico_multicore
  hardware_pio
  hardware_pwm
  hardware_adc
  rclib
)
pico_enable_stdio_usb(mbot_motor_analyze 1)
pico_enable_stdio_uart(mbot_motor_analyze 0)
pico_add_extra_outputs(mbot_motor_analyze)

#I control tests
add_executable(mbot_current_control_test
  mbot_current_control_test.c
)
target_link_libraries(mbot_current_control_test
  pico_stdlib
  pico_multicore
  hardware_pio
  hardware_pwm
  hardware_adc
  rclib
)
pico_enable_stdio_usb(mbot_current_control_test 1)
pico_enable_stdio_uart(mbot_current_control_test 0)
pico_add_extra_outputs(mbot_current_control_test)


#Encoder tests
add_executable(mbot_encoder_test
  mbot_encoder_test.c
)
target_link_libraries(mbot_encoder_test
  pico_stdlib
  pico_multicore
  hardware_pio
  hardware_pwm
  rclib
)
pico_enable_stdio_usb(mbot_encoder_test 1)
pico_enable_stdio_uart(mbot_encoder_test 0)
pico_add_extra_outputs(mbot_encoder_test)
