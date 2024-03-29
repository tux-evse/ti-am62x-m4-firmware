// based on work from cornelius
#include "iec61851_fsm.hpp"
//#include "bsp_drivers/pb_codec/high_to_low.pb.h"

#include <math.h>
#include <string.h>
#include <kernel/dpl/DebugP.h>

/* TMA *******************************************
 *  ******** declaration global variables ********
 *  ********************************************** */
// TMA Patch: flag to replace information come from Linux
uint8_t FsmDcAppyFlag = 0;
uint8_t FsmSetSlacStatus = 0; // set 5% by default
extern float set_pwm_DC_given;

// TMA Declaration Var to get DC in function of PP
float FsmDcAppy = 0;
float FsmDcPP = 0;
//float STATE_B_DC;
//float STATE_C_DC;
uint8_t locSlacStatus = 4; //SLACState

namespace iec61851 {
//Global Variable :
PPState ppcurr_State;
PPState memo_ppcurr_State = (PPState)0;  // to check if pp state changes
uint8_t SlacFlagStatus;


//Defined Declaration :
/* acquisition in STATE B */
#define FSM_PP_NC_THRESHOLD  0.5  //TMA : 0.495000
#define FSM_PP_13A_THRESHOLD 0.36  //TMA : 0.388000
#define FSM_PP_20A_THRESHOLD 0.28  //TMA : 0.331050
#define FSM_PP_32A_THRESHOLD 0.23  //TMA : 0.248000
#define FSM_PP_63A_THRESHOLD 0.15  //TMA : 0.213000

/* **************************************************************************
                      IEC61851 DECLARATION FUNCTIONS
**************************************************************************** */
// checks if voltage is within center+-interval
static bool is_voltage_in_range(float voltage, float center) {
    const float interval = 1.1;

    return ((voltage > center - interval) && (voltage < center + interval));
}

// checks if current is within ranges
static bool is_current_in_range(float current, float threshold) {

    return (current > threshold);
}

Countdown::Countdown(IClock& clock_, uint32_t period_in_ms) :
    clock(clock_), period_in_ticks(period_in_ms * clock_.ticks_per_ms){};

void Countdown::start() {
    running = true;
    ticks_at_start = clock.get_current_ticks();
}

void Countdown::stop() {
    running = false;
}

bool Countdown::elapsed() const {
    if (!running) {
        // not started
        return false;
    }
    uint32_t ticks_now = clock.get_current_ticks();
    return (ticks_now - ticks_at_start >= period_in_ticks);
}

FSM::FSM(HAL& hal_, EventCallback event_cb) :
    hal(hal_),
    push_event(event_cb),
    pwm_stop_countdown(hal_.clock, COUNTDOWN_PWM_STOP_IN_MS),
    lock_check_countdown(hal_.clock, COUNTDOWN_LOCK_CHECK_IN_MS),
    overcurrent_countdown(hal_.clock, COUNTDOWN_OVERCURRENT_IN_MS) {
    disable();
    hal.rcd.enable(); // Default is RCD is enabled
}

FSM::~FSM() {
    set_pwm_f();
}

void FSM::run() {

    // check if RCD fired in the meantime (actual emergency switch off happens
    // in interrupt)


    if (hal.rcd.got_fired()) {
        hal.rcd.reset(); // Note this does NOT reset the fault flag in the
                         // powerSwitch, just the flag that we do not keep on
                         // sending Error_RCD events!
        push_event(Event::Error_RCD);
    }

    // check if we need to stop LOCK motor after one second
    if (lock_check_countdown.elapsed()) {
        hal.lock.unlock();
        lock_check_countdown.stop();
    }

    // FIXME (aw): where do we get the overcurrent values from?
#if 0
    // check for (slow) over current situation:
    // e.g. car may not follow PWM current limit within N seconds
    checkOverCurrent(events);
#endif

    // update currentState from Car reading if signal is stable
    if (read_cp_state(cur_state)) {
        switch (cur_state) {
        case CPState::Disabled:
            // simply wait until someone enables us...
            power_on_allowed = false;
            power_off();
            break;

        case CPState::A:
            use_three_phases_latch = use_three_phases;
            set_pwm_off();
            simplified_mode = false;
            ventilated_charging_active = false;

            // Table A.6: Sequence 2.1 Unplug at state Bx (or any other
            // state) Table A.6: Sequence 2.2 Unplug at state Cx, Dx
            if (prev_state != CPState::A && prev_state != CPState::Disabled && prev_state != CPState::F) {
                push_event(Event::CarRequestedStopPower);
                power_off();
                push_event(Event::CarUnplugged);

                // If car was unplugged, reset RCD flag.
                if (rcd_reclosing_allowed) {
                    hal.power_switch.reset_emergency_switch();
                    hal.rcd.reset();
                }
            }
            break;

        case CPState::B:
            // Table A.6: Sequence 7 EV stops charging
            // Table A.6: Sequence 8.2 EV supply equipment
            // responds to EV opens S2 (w/o PWM)


            if (prev_state != CPState::A && prev_state != CPState::B) {
                push_event(Event::CarRequestedStopPower);
                // Need to switch off according to Table A.6 Sequence 8.1
                // within
                power_off();
            }

            ventilated_charging_active = false;

            // Table A.6: Sequence 1.1 Plug-in
            if (prev_state == CPState::A || prev_state == CPState::Disabled) {
                push_event(Event::CarPluggedIn);
                simplified_mode = false;
            }

            if (FsmSetSlacStatus == 3 ){// SLAC NOK, timeout so apply PP on CP
                    // TMA : B state get PP Current state
                    read_pp_state(ppcurr_State);

                    if (memo_ppcurr_State != ppcurr_State){
                    memo_ppcurr_State = ppcurr_State;
                        if(ppcurr_State == (PPState)0){
                            push_event(Event::PpImaxNC);
                            SlacFlagStatus = 0; // Set 5% of duty cycle (default value)
                        }else if(ppcurr_State == (PPState)1){
                            push_event(Event::PpImax13A);
                        }else if(ppcurr_State == (PPState)2){
                            push_event(Event::PpImax20A);
                        }else if(ppcurr_State == (PPState)3){
                            push_event(Event::PpImax32A);
                        }else if(ppcurr_State == (PPState)4){
                            push_event(Event::PpImax64A);
                        }else
                        {
                            //Do Nothing
                        }
                       // TMA  calculation DC = f(Imax)
                        FsmDcPP = calcul_dutyCycle(ppcurr_State);
                        // Flag  = 1 to applied the new DC
                        FsmDcAppyFlag = 1;
                    }

            }else{ // SLAC ongoing or OK (1 or 2)
                FsmDcAppyFlag = 2;
            }

            // CP PWM application
            if(FsmDcAppyFlag == 1){ //apply PWM duty cycle  based on PP
                if((set_pwm_DC_given< FsmDcPP) && (set_pwm_DC_given>0.06)){
                    FsmDcAppy = set_pwm_DC_given; // use linux pwm dc if it's lower than PP calculated one, ignore 5% as corner case
                    //DebugP_log("replace FsmDcAppy by linux cmd\r\n");
                }else{
                    FsmDcAppy = FsmDcPP;
                    //DebugP_log("use FsmDcAppy original \r\n");
                }

                set_pwm_on(FsmDcAppy);
                DebugP_log("flag = %d, Duty = %f by PP & Linux cmd min \r\n", FsmDcAppyFlag, FsmDcAppy);
            }else{ // apply DC given by linux, FsmDcAppyFlag == 0 (default), or == 2 (apply given DC)
                set_pwm_on(set_pwm_DC_given);
                DebugP_log("flag = %d, duty = %f (by Linux) \r\n", FsmDcAppyFlag, set_pwm_DC_given);
            }


            if (prev_state == CPState::E || prev_state == CPState::F) {
                push_event(Event::EF_To_BCD);
            }

            if (!cur_pwm_running) { // B1
            } else {                // B2
            }

            break;

        case CPState::C:
            DebugP_log("STATE C, previous: %d current : %d  \r\n",prev_state, ppcurr_State);

            // Table A.6: Sequence 1.2 Plug-in
            if (prev_state == CPState::A) {
                push_event(Event::CarPluggedIn);
                simplified_mode = true;
            }
            if (prev_state == CPState::B) {
                // TJZH 17012024: stop to get PP Current state in C state
                read_pp_state(ppcurr_State);
                DebugP_log("STATE B to C, PPState : %d  \r\n",ppcurr_State);
                if(ppcurr_State == (PPState)0){
                      push_event(Event::PpImaxNC);
                  }else if(ppcurr_State == (PPState)1){
                      push_event(Event::PpImax13A);
                  }else if(ppcurr_State == (PPState)2){
                      push_event(Event::PpImax20A);
                  }else if(ppcurr_State == (PPState)3){
                      push_event(Event::PpImax32A);
                  }else if(ppcurr_State == (PPState)4){
                      push_event(Event::PpImax64A);
                  };
                //
                //  else {
                //      // Do Nothing
                //  }
                 // TMA  calculation DC = f(Imax)
                 //FsmDcAppy = calcul_dutyCycle(ppcurr_State);
                push_event(Event::CarRequestedPower);
            }

            if (prev_state == CPState::E || prev_state == CPState::F) {
                push_event(Event::EF_To_BCD);
            }

            if (!cur_pwm_running) {
                // C1
                // Table A.6 Sequence 10.2: EV does not stop drawing power
                // even if PWM stops. Stop within 6 seconds (E.g. Kona1!)
                if (prev_pwm_running) {
                    pwm_stop_countdown.start();
                }
                if (pwm_stop_countdown.elapsed()) {
                    // force power off under load
                    power_off();
                }
            }

            // C2
            if (power_on_allowed) {
                // Table A.6: Sequence 4 EV ready to charge.
                // Must enable power within 3 seconds.
                //DebugP_log("EVSE decides to close relay \r\n");
                power_on();

                // Simulate Request power Event here for simplified mode
                // to ensure that this mode behaves similar for higher
                // layers. Note this does not work with 5% mode
                // correctly, but simplified mode does not support HLC
                // anyway.
                if (!prev_pwm_running && simplified_mode)
                    push_event(Event::CarRequestedPower);
            }

            // Charging Station decides to stop
            if (!power_on_allowed){
                power_off();
                //DebugP_log("EVSE decides to open relay \r\n");
            }


            // CP PWM application
            if(FsmDcAppyFlag == 1){ //apply PWM duty cycle  based on PP
                if((set_pwm_DC_given< FsmDcPP) && (set_pwm_DC_given>0.06)){
                    FsmDcAppy = set_pwm_DC_given; // use linux pwm dc if it's lower than PP calculated one, ignore 5% as corner case
                    DebugP_log("replace FsmDcAppy by linux cmd\r\n");
                }else{
                    FsmDcAppy = FsmDcPP;
                    DebugP_log("use FsmDcAppy original \r\n");
                }

                set_pwm_on(FsmDcAppy);
                DebugP_log("flag = %d, Duty = %f by PP & linux cmd min \r\n", FsmDcAppyFlag, FsmDcAppy);
            }else{ // apply DC given by linux, FsmDcAppyFlag == 0 (default), or == 2 (apply given DC)
                set_pwm_on(set_pwm_DC_given);
                //DebugP_log("flag = %d, duty = %f (by linux) \r\n", FsmDcAppyFlag, set_pwm_DC_given);
            }


            break;

        case CPState::D:
            // Table A.6: Sequence 1.2 Plug-in (w/ventilation)
            if (prev_state == CPState::A) {
                push_event(Event::CarPluggedIn);
                push_event(Event::CarRequestedPower);
                simplified_mode = true;
            }

            if (prev_state == CPState::B) {
                push_event(Event::CarRequestedPower);
            }

            if (prev_state == CPState::E || prev_state == CPState::F) {
                push_event(Event::EF_To_BCD);
            }

            if (!cur_pwm_running) {
                // Table A.6 Sequence 10.2: EV does not stop drawing power
                // even if PWM stops. Stop within 6 seconds (E.g. Kona1!)
                if (prev_pwm_running) {
                    pwm_stop_countdown.start();
                }
                if (pwm_stop_countdown.elapsed()) {
                    // force power off under load
                    power_off();
                }
            } else {
                if (power_on_allowed && !hal.power_switch.is_on()) {
                    // Table A.6: Sequence 4 EV ready to charge.
                    // Must enable power within 3 seconds.
                    if (!has_ventilation) {
                        push_event(Event::Error_VentilationNotAvailable);
                      DebugP_log("CP STATE POWER to D : ERROR NOT AVAILABLE \r\n");// TMA added
                    } else {
                        power_on();
                    }
                }
                if (prev_state == CPState::C) {
                    // Car switches to ventilation while charging.
                    /*if (!has_ventilation)
                        push_event(Event::Error_VentilationNotAvailable);*/
                    if (!has_ventilation){
                        push_event(Event::Error_VentilationNotAvailable);
                        DebugP_log("CP STATE C to D : ERROR NOT AVAILABLE \r\n"); // TMA added
                        }
                }
            }
            break;

        case CPState::E:
            if (prev_state != cur_state)
                push_event(Event::Error_E);
            if (prev_state == CPState::B || prev_state == CPState::C || prev_state == CPState::D) {
                push_event(Event::BCD_To_EF);
            }
            power_off();
            set_pwm_off();
            break;
        case CPState::F:
            power_off();
            if (prev_state == CPState::B || prev_state == CPState::C || prev_state == CPState::D) {
                push_event(Event::BCD_To_EF);
            }
            break;
        case CPState::DF:
            if (prev_state != cur_state)
                push_event(Event::Error_DF);
            if (prev_state == CPState::B || prev_state == CPState::C || prev_state == CPState::D) {
                push_event(Event::BCD_To_EF);
            }
            power_off();
            break;
        }

        prev_state = cur_state;
        prev_pwm_running = cur_pwm_running;
    }
}

void FSM::set_pwm_off() {
    hal.cp.set_pwm(1.);
    pwm_duty_cycle = 1.;
    cur_pwm_running = false;
    power_on_allowed = false;
}

void FSM::set_pwm_on(float duty_cycle) {
    hal.cp.set_pwm(duty_cycle);
    pwm_duty_cycle = duty_cycle;
    cur_pwm_running = true;
    //DebugP_log("apply PWM DC: %f \r\n",pwm_duty_cycle);
}

// NOTE: F can be exited by set_pwm_off or set_pwm_on only
void FSM::set_pwm_f() {
    // EVSE error - constant -12V signal on CP
    hal.cp.set_pwm(0.);
    pwm_duty_cycle = 0.;
    cur_state = CPState::F;
    cur_pwm_running = false;
    power_on_allowed = false;
}

void FSM::enable() {
    cur_state = CPState::A;
    set_pwm_off();
    hal.cp.enable();
}

void FSM::disable() {
    cur_state = CPState::Disabled;
    set_pwm_off();
    hal.cp.disable();
}

// Translate ADC readings for lo and hi part of PWM to IEC61851 states.
// returns false if signal is unstable/invalid and cp argument was not
// updated.
// FIXME (aw): return as struct, with valid info
bool FSM::read_cp_state(CPState& state) {
    // FIXME (aw): dropped simulation here
    const auto cp = hal.cp.get_cp_signal();
    if (!cp.valid) {
        return false;
    }

    // FIXME (aw): the read function should not decide, whether it should set the var or not
    if (state == CPState::Disabled) {
        return true; // stay in Disabled independent of measurement
    }

    // sth is wrong with negative signal
    if (cur_pwm_running && !is_voltage_in_range(cp.low, -12.)) {
        // CP-PE short or signal somehow gone
        if (is_voltage_in_range(cp.low, 0.) && is_voltage_in_range(cp.high, 0.))
            state = CPState::E;
        // Diode fault
        else if (is_voltage_in_range(cp.high + cp.low, 0.)) {
            state = CPState::DF;
        } else {
            return false;
        }
    } else if (is_voltage_in_range(cp.high, 12.)) {
        // +12V State A IDLE (open circuit)
        state = CPState::A;
    } else if (is_voltage_in_range(cp.high, 9.)) {
        state = CPState::B;
    } else if (is_voltage_in_range(cp.high, 6.)) {
        state = CPState::C;
    } else if (is_voltage_in_range(cp.high, 3.)) {
        state = CPState::D;
    } else if (is_voltage_in_range(cp.high, -12.)) {
        state = CPState::F;
    } else {
        return false;
    }
    return true;
}

// TMA Translate ADC readings for lo and hi part of PWM to IEC61851 states.
// returns false if signal is unstable/invalid and cp argument was not
// updated.
// FIXME (aw): return as struct, with valid info
bool FSM::read_pp_state(PPState& state) {
   const auto pp = hal.cp.get_pp_signal();
   if (!pp.valid) {
       return false;
   }
   if(pp.valid == true)
   {
       /* TMA : check if CHARGING NC range */
       if (is_current_in_range(pp.high,FSM_PP_NC_THRESHOLD)) {
           state = PPState::NOCONNECTED;
           // TMA : DC 5% applied
       }else if (is_current_in_range(pp.high,FSM_PP_13A_THRESHOLD)) {
           state = PPState::CHARGING13;
       }else if (is_current_in_range(pp.high,FSM_PP_20A_THRESHOLD)) {
           state = PPState::CHARGING20;
       }else if (is_current_in_range(pp.high,FSM_PP_32A_THRESHOLD)) {
           state = PPState::CHARGING32;
       }else if (is_current_in_range(pp.high,FSM_PP_63A_THRESHOLD)) {
           state = PPState::CHARGING63;
       }else{
           state = PPState::OUTOFRANGE;/* OUT OF RANGE */
       }
       return true;
   } else {
    return false;
   }
}

/* ****************************************************************
// TMA STATE B : calculate PWM for STATE B/C.
// FUNCTION : float dutyCycle(PPState& state)
// INPUT : PPstate[0;4] => corresponding to Iav = Ipp max [0;63]A
// OUTPUT : Duty Cycle = [0.05;0.95]%
 **************************************************************** */
float FSM::calcul_dutyCycle(PPState& state) {
    float state_DC = 1.0;
    if (state == PPState::NOCONNECTED)
    {
        state_DC = 0.05;                 // DC = 5%
    }else if (state == PPState::CHARGING13)
    {
        state_DC = ((13.0/0.6)/100.0);    // DC = 21%
    }else if (state == PPState::CHARGING20)
    {
        state_DC = ((20.0/0.6)/100.0);    // DC = 33%
    }else if (state == PPState::CHARGING32)
    {
        state_DC = ((32.0/0.6)/100.0);    // DC = 53%
    }else if (state == PPState::CHARGING63)
    {
        state_DC = ((63.0/2.4+64)/100.0); // DC = 90%
    }
    else
    {
        state_DC = 0.05;// DC = 5%
    }
    return state_DC;
}

void FSM::set_three_phases(bool enable) {
    use_three_phases = enable;
}

// FIXME implement me on new HW
bool FSM::set_phase_switch_while_charging(bool enable) {
    return true;
}

void FSM::set_ventilation(bool enable) {
    has_ventilation = enable;
}

bool FSM::power_on() {
    if (hal.power_switch.is_on()) {
        return true;
    }

    hal.rcd.disable();
    auto success = hal.power_switch.on(use_three_phases_latch);

    push_event((success) ? Event::PowerOn : Event::Error_Relais);

    // FIXME (aw): we should only enable, if it was enabled before! This could be done, if we can check if the rcd was
    // enabled, before we disable it above
    hal.rcd.enable();

    // lock connector here
    hal.lock.lock();
    lock_check_countdown.start();

    return success;
}

bool FSM::power_off() {

    if (!hal.power_switch.is_on()) {
        return true;
    }
    // disable RCD
    hal.rcd.disable();
    // actually switch off relais
    auto success = hal.power_switch.off();

    push_event((success) ? Event::PowerOff : Event::Error_Relais);

    // unlock connector lock
    hal.lock.unlock();
    lock_check_countdown.start();

    return success;
}

void FSM::set_country_code(const char* iso3166_alpha2_code) {
    if (strcmp(iso3166_alpha2_code, "CH") == 0 || strcmp(iso3166_alpha2_code, "DK") == 0 ||
        strcmp(iso3166_alpha2_code, "GB") == 0 || strcmp(iso3166_alpha2_code, "FR") == 0) {
        // Recosing of RCD is not allowed in these countries after a
        // failure.
        rcd_reclosing_allowed = false;
    } else {
        rcd_reclosing_allowed = true;
    }
}

bool FSM::is_ventilated_charging_active() {
    return ventilated_charging_active;
}

bool FSM::uses_three_phases() {
    return use_three_phases;
}

CPState FSM::get_current_cp_state() {
    return cur_state;
}
/*
SLACState FSM::slacflagStatus() {
    return slacPwmActivation;
}*/

const char* event_to_string(Event e) {
    switch (e) {
    case Event::CarPluggedIn:
        return "CarPluggedIn";
    case Event::CarRequestedPower:
        return "CarRequestedPower";
    case Event::PowerOn:
        return "PowerOn";
    case Event::PowerOff:
        return "PowerOff";
    case Event::CarRequestedStopPower:
        return "CarRequestedStopPower";
    case Event::CarUnplugged:
        return "CarUnplugged";
    case Event::Error_E:
        return "Error_E";
    case Event::Error_DF:
        return "Error_DF";
    case Event::Error_Relais:
        return "Error_Relais";
    case Event::Error_RCD:
        return "Error_RCD";
    case Event::Error_VentilationNotAvailable:
        return "Error_VentilationNotAvailable";
    case Event::Error_OverCurrent:
        return "Error_OverCurrent";
    case Event::EF_To_BCD:
        return "EF_To_BCD";
    case Event::BCD_To_EF:
        return "BCD_To_EF";
    case Event::PermanentFault:
        return "PermanentFault";
    case Event::EvseReplugStarted:
        return "EvseReplugStarted";
    case Event::EvseReplugFinished:
        return "EvseReplugFinished";
    default:
        break;
    }
    return "";
}

// EXT
#if 0
const char* state_to_string(FSM::CPState) {
    switch (currentState) {
    case CPState::Disabled:
        return "DS";
    case CPState::A:
        if (pwmRunning)
            return "A2";
        else
            return "A1";
        break;
    case CPState::B:
        if (pwmRunning)
            return "B2";
        else
            return "B1";
        break;
    case CPState::C:
        if (pwmRunning)
            return "C2";
        else
            return "C1";
        break;
    case CPState::D:
        if (pwmRunning)
            return "D2";
        else
            return "D1";
        break;
    case CPState::E:
        return "E ";
        break;
    case CPState::F:
        return "F ";
        break;
    case CPState::DF:
        return "DF";
        break;
    }
    return "UN";
}
#endif

void FSM::allow_power_on(bool enable) {
    power_on_allowed = enable;
}

} // namespace iec61851
