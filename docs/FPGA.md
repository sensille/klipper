CMD_GET_VERSION:          0 <>
  -- issued with fpga_setup
RSP_GET_VERSION:          0 <version>
CMD_SYNC_TIME:            1 <ref-time low> <ref-time high>
  -- issued with fpga_setup
CMD_GET_TIME:             2 <>
  -- fpga_get_time(oid)
RSP_GET_TIME:             1 <time low> <time high>
CMD_CONFIG_PWM:           3 <channel> <cycle-ticks> <value (0-cycle-ticks) >
                      <default (0/1)> <max_duration>
CMD_SCHEDULE_PWM:         4 <channel> <clock> <on-ticks>
  -- where on-ticks = number of ticks signal is low (not high)
CMD_CONFIG_STEPPER:       5 <channel> <dedge>
CMD_QUEUE_STEP:           6 <channel> <interval> <count> <add>
CMD_SET_NEXT_STEP_DIR:    7 <channel> <dir>
CMD_RESET_STEP_CLOCK:     8 <channel> <time>
    Actual stepping starts 2 cycles after the given time
CMD_STEPPER_GET_POS:      9 <channel>
RSP_STEPPER_GET_POS:      2 <position>
CMD_ENDSTOP_SET_STEPPER: 10 <endstop-channel> <stepper-channel>
CMD_ENDSTOP_QUERY:       11 <channel>
RSP_ENDSTOP_STATE:        3 <homing> <pin_value>
CMD_ENDSTOP_HOME:        12 <endstop-channel> <time> <sample_count> <pin_value>
CMD_TMCUART_WRITE:       13 <channel> <slave> <register> <data>
CMD_TMCUART_READ:        14 <channel> <slave> <register>
RSP_TMCUART_READ:         4 <status> <register>
CMD_SET_DIGITAL_OUT:     15 <channel> <value>
CMD_CONFIG_DIGITAL_OUT:  16 <channel> <value> <default_value> <max_duration>
CMD_SCHEDULE_DIGITAL_OUT:17 <channel> <clock> <value>
CMD_UPDATE_DIGITAL_OUT:  18 <channel> <value>
  -- identical to CMD_SET_DIGITAL_OUT
