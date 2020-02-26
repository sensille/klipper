        cmdtab[CMD_GET_VERSION] = { UNIT_SYSTEM, ARGS_0, 1'b0, 1'b1 };
in: nothing

RSP_GET_VERSION
out: version

        cmdtab[CMD_SYNC_TIME] = { UNIT_SYSTEM, ARGS_2, 1'b0, 1'b0 };
in: <ref-time low> <ref-time high>
        cmdtab[CMD_GET_TIME] = { UNIT_SYSTEM, ARGS_0, 1'b0, 1'b1 };
in: nothing
RSP_GET_TIME
out: <time low> <time high>

        cmdtab[CMD_CONFIG_PWM] = { UNIT_PWM, ARGS_5, 1'b0, 1'b0 };
in: <channel> <cycle-ticks> <value (0-cycle-ticks) > <default (0/1)> <max_duration>

        cmdtab[CMD_SCHEDULE_PWM] = { UNIT_PWM, ARGS_3, 1'b0, 1'b0 };
in: <channel> <new duty>
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

