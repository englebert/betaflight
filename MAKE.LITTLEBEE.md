i am having JHEMCU GF30F722-ICM F722 Baro OSD 5V 10V Dual BEC Flight Controller.

I need to recompile the betaflight latest version on it. 

At the same time, I had added a few things to this board as the current sensors are using INA226  with modified shunt to 0.001 ohm. The  battery voltage should get from INA226.

The FC will be stepped voltage to the flight controller board at stable 12V as the main battery system only running on 2S battery. So this will prevent brown out during battery voltage below 6.5V. I had run some test when the board hits 6.5V and below it will brown out.

Show me the parameters to compile the firmware.

below are the existing config:
diff all
# version
# Betaflight / STM32F7X2 (S7X2) 4.5.1 Dec  2 2024 / 17:43:58 (77d01ba3b) MSP API: 1.46
# config rev: 283a21f
# start the command batch
batch start
# reset configuration to default settings
defaults nosave
board_name JHEF7DUAL
manufacturer_id JHEF
mcu_id 002c00453335510b36333431
signature 
# feature
feature GPS
feature TELEMETRY
feature LED_STRIP
feature OSD
# serial
serial 0 2 115200 115200 0 115200
serial 1 64 115200 57600 0 115200
# led
led 0 8,6::CB:8
led 1 9,6::CB:8
led 2 10,6::CB:8
led 3 11,6::CB:8
# master
set gyro_to_use = SECOND
set dyn_notch_count = 1
set dyn_notch_q = 500
set acc_calibration = -20,50,36,1
set dshot_bidir = ON
set ibata_scale = 264
set ibata_offset = 424
set small_angle = 180
set pid_process_denom = 2
set osd_vbat_pos = 2349
set osd_crosshairs_pos = 2286
set osd_ah_sbar_pos = 2287
set osd_ah_pos = 2159
set osd_current_pos = 2433
set osd_warnings_pos = 14730
set osd_avg_cell_voltage_pos = 2413
set osd_displayport_device = AUTO
set osd_canvas_width = 30
set osd_canvas_height = 13
set vcd_video_system = AUTO
profile 0
profile 1
profile 2
profile 3
# restore original profile selection
profile 0
rateprofile 0
rateprofile 1
rateprofile 2
rateprofile 3
# restore original rateprofile selection
rateprofile 0
# save configuration
save
#

Lets build the INA226 into the betaflight for this JHEMCU GF30F722-ICM F722 board.
