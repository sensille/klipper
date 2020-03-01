CMD_GET_VERSION:    0 <>
  -- issued with fpga_setup
RSP_GET_VERSION:    0 <version>
CMD_SYNC_TIME:      1 <ref-time low> <ref-time high>
  -- issued with fpga_setup
CMD_GET_TIME:       2 <>
  -- fpga_get_time(oid)
RSP_GET_TIME:       1 <time low> <time high>
CMD_CONFIG_PWM:     3 <channel> <cycle-ticks> <value (0-cycle-ticks) >
                      <default (0/1)> <max_duration>
  -- fpga_config_pwm(oid, oid_fpga, channel, cycle-ticks, value, default,
                      max_duration)
CMD_SCHEDULE_PWM:   4 <channel> <new duty>
    where duty = number of ticks signal is low (not high)

cmdtab[CMD_CONFIG_STEPPER] = { UNIT_STEPPER, ARGS_, 1'b0, 1'b0 };
in: <channel> <min_stop_interval>
always uses dedge

        cmdtab[CMD_QUEUE_STEP] = { UNIT_STEPPER, ARGS_, 1'b0, 1'b0 };
in: <channel> <interval> <count> <add>

        cmdtab[CMD_SET_NEXT_STEP_DIR] = { UNIT_STEPPER, ARGS_, 1'b0, 1'b0 };
in: <channel> <dir>

        cmdtab[CMD_RESET_STEP_CLOCK] = { UNIT_STEPPER, ARGS_, 1'b0, 1'b0 };
in: <channel> <time>
Actual stepping starts 2 cycles after the given time

        cmdtab[CMD_STEPPER_GET_POS] = { UNIT_STEPPER, ARGS_, 1'b0, 1'b0 };
in: <channel>
RSP_STEPPER_GET_POS
out: <position>

        cmdtab[CMD_ENDSTOP_SET_STEPPER] = { UNIT_STEPPER, ARGS_, 1'b0, 1'b0 };
in: <endstop-channel> <stepper-channel>

        cmdtab[CMD_ENDSTOP_QUERY] = { UNIT_STEPPER, ARGS_, 1'b0, 1'b0 };
in: <channel>
RSP_ENDSTOP_QUERY
out: <homing> <pin_value>

        cmdtab[CMD_ENDSTOP_HOME] = { UNIT_STEPPER, ARGS_, 1'b0, 1'b0 };
in: <endstop-channel> <time> <sample_count> <pin_value>

