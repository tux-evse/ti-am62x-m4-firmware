// FIXME (aw): copyright?
#include <stdlib.h>

#include "FreeRTOS.h"

#include <kernel/dpl/DebugP.h>

#include <iec61851_fsm.hpp>

#include "bsp_drivers/rpmsg.hpp"
#include "bsp_drivers/task_config.h"

#include "bsp_drivers/charger_lock.hpp"
#include "bsp_drivers/control_pilot.hpp"
#include "bsp_drivers/power_switch.hpp"
#include "bsp_drivers/rcd.hpp"

/* TMA : Declaration Variable global */
static constexpr uint32_t CHORE_INTERVAL_MS = 1000;


void handle_incoming_message(const HighToLow& in, iec61851::FSM& fsm) {
    if (in.which_message == HighToLow_set_pwm_tag){
        auto& set_pwm = in.message.set_pwm;
        switch (set_pwm.state) {
        case PWMState_F:
            fsm.set_pwm_f();
            DebugP_log("MODE PWMState_F \r\n");
            break;
        case PWMState_OFF:
            fsm.set_pwm_off();
            DebugP_log("MODE PWMState_OFF \r\n");
            break;
        case PWMState_ON:
            if(FsmDcAppyFlag == 1){
                fsm.set_pwm_on(FsmDcAppy);
                DebugP_log(" Case 1 PwmFlag (linux) %d and  STATE B Duty 5% to = %f \r\n", FsmDcAppyFlag, FsmDcAppy);
                //DebugP_log("MODE PWM ON: DC = %d , %f\r\n";set_pwm.duty_cycle,set_pwm.duty_cycle);
            }else{
                fsm.set_pwm_on(set_pwm.duty_cycle);
                DebugP_log(" Case 1 Pwm 5% (by linux) \r\n");
            }
            break;
        default:
            // NOT ALLOWED
            break;
        DebugP_log("PWM STATE : %d, %f / PWM DC : %d, %f \r\n",set_pwm.state,set_pwm.state,set_pwm.duty_cycle,set_pwm.duty_cycle);
        }
    } else if (in.which_message == HighToLow_allow_power_on_tag) {
        fsm.allow_power_on(in.message.allow_power_on);
    } else if (in.which_message == HighToLow_enable_tag) {
        fsm.enable();
    } else if (in.which_message == HighToLow_disable_tag) {
        fsm.disable();
    } else if (in.which_message == HighToLow_heartbeat_tag) {
        DebugP_log("Received a heartbeat from the CPU\r\n");
    } else if (in.which_message == GetSLAC_state_tag) {
        DebugP_log("Received a SLAC Status from the CPU\r\n");
        auto& get_slac = in.message.get_slac;
        switch (get_slac.state) {
        case SLACState_PROGRESS :
            DebugP_log("SLAC STATE = SLACState_PROGRESS \r\n");
            FsmGetSlacStatus = 1;
            break;
        case SLACState_NOK :
            DebugP_log("SLAC STATE = SLACState_NOK \r\n");
            FsmGetSlacStatus = 2;
            break;
        case SLACState_OK :
            DebugP_log("SLAC STATE = SLACState_OK \r\n");
            FsmGetSlacStatus = 3;
            break;
        default:
            // NOT ALLOWED
            break;
        }//END SWITCH
    } //END_IF_GetSLAC
} //END_FUNCTION

// TMA : added debug Log
void push_event(const iec61851::Event& event, RPMsg& link) {
    auto convert = [](const iec61851::Event& event) {
        using ET = iec61851::Event;
        switch (event) {
        case ET::CarPluggedIn:
            DebugP_log("IEC61851Event_CAR_PLUGGED_IN \r\n");
            return IEC61851Event_CAR_PLUGGED_IN;
        case ET::CarRequestedPower:
            DebugP_log("IEC61851Event_CAR_REQUESTED_POWER \r\n");
            return IEC61851Event_CAR_REQUESTED_POWER;
        case ET::CarRequestedStopPower:
            DebugP_log("IEC61851Event_CAR_REQUESTED_STOP_POWER \r\n");
            return IEC61851Event_CAR_REQUESTED_STOP_POWER;
        case ET::CarUnplugged:
            DebugP_log("IEC61851Event_CAR_UNPLUGGED \r\n");
            return IEC61851Event_CAR_UNPLUGGED;
        case ET::EF_To_BCD:
            DebugP_log("IEC61851Event_EF_TO_BCD \r\n");
            return IEC61851Event_EF_TO_BCD;
        case ET::Error_DF:
            DebugP_log("IEC61851Event_ERROR_DF \r\n");
            return IEC61851Event_ERROR_DF;
        case ET::Error_E:
            DebugP_log("IEC61851Event_ERROR_DF \r\n");
            return IEC61851Event_ERROR_E;
        case ET::Error_OverCurrent:
            DebugP_log("IEC61851Event_ERROR_OVER_CURRENT \r\n");
            return IEC61851Event_ERROR_OVER_CURRENT;
        case ET::Error_RCD:
            DebugP_log("IEC61851Event_ERROR_RCD \r\n");
            return IEC61851Event_ERROR_RCD;
        case ET::Error_Relais:
            DebugP_log("IEC61851Event_ERROR_RELAIS \r\n");
            return IEC61851Event_ERROR_RELAIS;
        case ET::Error_VentilationNotAvailable:
            DebugP_log("IEC61851Event_ERROR_VENTILATION_NOT_AVAILABLE \r\n");
            return IEC61851Event_ERROR_VENTILATION_NOT_AVAILABLE;
        case ET::EvseReplugFinished:
            DebugP_log("IEC61851Event_EVSE_REPLUG_FINISHED\r\n");
            return IEC61851Event_EVSE_REPLUG_FINISHED;
        case ET::EvseReplugStarted:
            DebugP_log("IEC61851Event_EVSE_REPLUG_STARTED \r\n");
            return IEC61851Event_EVSE_REPLUG_STARTED;
        case ET::BCD_To_EF:
            DebugP_log("IEC61851Event_BCD_TO_EF \r\n");
            return IEC61851Event_BCD_TO_EF;
        case ET::PermanentFault:
            DebugP_log("IEC61851Event_PERMANENT_FAULT \r\n");
            return IEC61851Event_PERMANENT_FAULT;
        case ET::PowerOff:
            DebugP_log("IEC61851Event_POWER_OFF \r\n");
            return IEC61851Event_POWER_OFF;
        case ET::PowerOn:
            DebugP_log("IEC61851Event_POWER_ON\r\n");
            return IEC61851Event_POWER_ON;
        /* TMA : PP cases updated in active mode STATE B/C */
        case ET::PpImaxNC:
            DebugP_log("IEC61851Event_PP_IMAX_NC \r\n");
            return IEC61851Event_PP_IMAX_NC;
        case ET::PpImax13A:
            DebugP_log("IEC61851Event_PP_IMAX_13A \r\n");
            return IEC61851Event_PP_IMAX_13A;
        case ET::PpImax20A:
            DebugP_log("IEC61851Event_PP_IMAX_20A\r\n");
            return IEC61851Event_PP_IMAX_20A;
        case ET::PpImax32A:
            DebugP_log("IEC61851Event_PP_IMAX_32A \r\n");
            return IEC61851Event_PP_IMAX_32A;
        case ET::PpImax64A:
            DebugP_log("IEC61851Event_PP_IMAX_64A \r\n");
            return IEC61851Event_PP_IMAX_64A;
        default:
            DebugP_log("IEC61851Event_PERMANENT_FAULT \r\n");
            return IEC61851Event_PERMANENT_FAULT;
        }
    };

    LowToHigh out;
    out.which_message = LowToHigh_event_tag;
    out.message.event = convert(event);
    link.send_msg(out);
}

void main_task(void* args) {
    DebugP_log("Hello from ti am62x charger firmware!\r\n");

    //
    // initialize necessary classes
    //

    // RPMsg link to high level
    // first wait for RPMsg linux connection
    DebugP_assert(RPMsg::wait_for_linux(10 * 1000));
    RPMsg rpmsg_link;
    DebugP_log("RPMsg link init done ...\r\n");

    // power switch
    PowerSwitch power_switch;

    // adc sampler
    Sampler sampler;

    // control pilot (creates pwm, and needs reference to sampler, in order to trigger adc readout)
    ControlPilot control_pilot(sampler);
    //set_pwm_on
    // rcd
    RCD rcd;

    // charger lock
    ChargerLock charger_lock;

    // clock
    iec61851::IClock clock{static_cast<uint32_t>(ClockP_usecToTicks(1000)), ClockP_getTicks}; //FCAM update

    // hal interface, needed by iec61851 finite state machine
    iec61851::HAL hal{
        rcd, charger_lock, power_switch, control_pilot, clock,
    };

#if 1
    // finite state machine, remote controlled from EVerest
    iec61851::FSM fsm{hal, [&rpmsg_link](const iec61851::Event& ev) { push_event(ev, rpmsg_link); }};
#else
    // This is a very simple charging logic that does not implement all cases
    // of IEC61851-1. It can be used to do basic charging on the microcontroller
    // only. You should not use this, use the remote control version in combination
    // with EVerest above.
    iec61851::FSM fsm{hal, [&hal](const iec61851::Event& ev) {
        float constexpr max_current = 16*0.6/100.;

        if (ev == iec61851::Event::CarPluggedIn) hal.cp.set_pwm(max_current);
        else if (ev == iec61851::Event::CarRequestedPower) hal.power_switch.on(true);
        else if (ev == iec61851::Event::CarRequestedStopPower) hal.power_switch.off();
        else if (ev == iec61851::Event::CarUnplugged) hal.cp.set_pwm(1.);
    }};
    fsm.enable();
#endif

    // start necessary tasks
    TaskHandle_t sampling_task_handle;
    auto status = xTaskCreate(Sampler::sampling_task_trampoline, "sampling_task", SAMPLING_TASK_STACK_SIZE, &sampler,
                              SAMPLING_TASK_PRIORITY, &sampling_task_handle);
    DebugP_assert(status == pdPASS);

    uint32_t last_chore_ts = ClockP_getTicks();
    uint32_t last_heartbeat_ts = ClockP_getTicks();

    const uint32_t chore_interval_ticks = ClockP_usecToTicks(CHORE_INTERVAL_MS * 1000);
    const uint32_t heartbeat_interval_ticks = ClockP_usecToTicks(CHORE_INTERVAL_MS * 3000);

    while (true) {
        // main loop

        constexpr uint16_t FSM_WAIT_INTERVAL_MS = 500;// TMA : Timeout 50 to 500
        HighToLow input;

        // 1. wait on message with specific timeout
        const auto status = rpmsg_link.get_msg(input, FSM_WAIT_INTERVAL_MS);

        // 2. if message, handle it
        if (status == RPMsgRcvStatus::TIMEOUT) {
            // no new input message
        } else if (status == RPMsgRcvStatus::CORRUPT_MESSAGE) {
            // FIXME (aw): proper error handling?
        } else if (status == RPMsgRcvStatus::OK) {
            // got new input
            handle_incoming_message(input, fsm);
        }

        // 3. run state machine update
        fsm.run();

        // 4. do chore
        uint32_t current_ts = ClockP_getTicks();
        if (chore_interval_ticks < (current_ts - last_chore_ts)) {
            auto cp_signal = sampler.get_latest_cp_signal();
           DebugP_log("CP: valid: %d , hi : %f  ,low : %f \r\n", cp_signal.valid, cp_signal.high, cp_signal.low);
            /*DebugP_log("FCAM Debug: hi_trg: %d, low_trg: %d, adcs: %d, sigs: %d, msgs: %d\r\n", ovfls, matchs, adcs, sigs,
                       messages_received);*/
// FCAM: PP signal Added
            auto pp_signal = sampler.get_latest_pp_signal();
            DebugP_log("PP: valid: %d , hi : %f  ,low : %f \r\n", pp_signal.valid ,  pp_signal.high ,  pp_signal.low );
            //DebugP_log("PP: hi: %f, PP: dec: %d, valid: 1\r\n", pp_signal.high, pp_signal.high, 1/*pp_signal.valid*/);

            last_chore_ts = current_ts;
        }

        if (heartbeat_interval_ticks < (current_ts - last_heartbeat_ts)) {
           last_heartbeat_ts = current_ts;
           McuHeartbeat heartbeat= McuHeartbeat_init_default;
           LowToHigh out {
             .which_message = LowToHigh_heartbeat_tag,
             .message.heartbeat = heartbeat,
           };
           if (rpmsg_link.send_msg(out /* Fulup timeout ???? */)) {
               DebugP_log("Success Send timeout \r\n");
           } else {
               DebugP_log("Fail Send timeout \r\n");
           }
        }
    }

    vTaskDelete(NULL);
}
