// based on work from cornelius

#ifndef LIB_IEC61851_FSM_HPP
#define LIB_IEC61851_FSM_HPP

#include <functional>

#include "iec61851_hal.hpp"
//#include "high_to_low.pb.h"

// TMA
extern uint8_t FsmDcAppyFlag;
// 0 : default value, apply duty based on PP 
// 1 : apply duty based on linux given (5%)
// 2 : apply duty based on PP
extern uint8_t FsmSetSlacStatus;
extern float FsmDcAppy;
//extern float STATE_B_DC;
//extern float STATE_C_DC;

namespace iec61851 {

// FIXME (aw): events should be disentangled
enum class Event {
    CarPluggedIn,
    CarRequestedPower,
    PowerOn,
    PowerOff,
    CarRequestedStopPower,
    CarUnplugged,
    Error_E,
    Error_DF,
    Error_Relais,
    Error_RCD,
    Error_VentilationNotAvailable,
    Error_OverCurrent,
    EF_To_BCD,
    BCD_To_EF,
    PermanentFault,
    EvseReplugStarted,
    EvseReplugFinished,
    PpImaxNC,
    PpImax13A,
    PpImax20A,
    PpImax32A,
    PpImax64A
};

enum class CPState {
    Disabled,
    A,
    B,
    C,
    D,
    E,
    F,
    DF
};

/* TMA : Class PPstate */
enum class PPState {
    NOCONNECTED,
    CHARGING13,
    CHARGING20,
    CHARGING32,
    CHARGING63,
    OUTOFRANGE,
    DF
};

/* TMA : Class SLAC FLAG */
/*
enum class SLACState {
    UNKNOW,
    SLAC_ON,
    SLAC_OFF,
    DF
};*/


class Countdown {
public:
    Countdown(IClock& clock_, uint32_t period_in_ms);
    void start();
    void stop();
    bool elapsed() const;

private:
    IClock& clock;
    uint32_t period_in_ticks {0};
    uint32_t ticks_at_start {0};
    bool running {false};
};

class FSM {
public:
    using EventCallback = std::function<void(const Event&)>;
    FSM(HAL&, EventCallback);
    ~FSM();

    // FIXME (aw): control pilot should not manage the power!  It should only manage pwm
    void set_three_phases(bool enable);
    bool uses_three_phases();

    void set_ventilation(bool enable);
    bool uses_ventilation();

    // reqd for local regulations in IEC norm
    void set_country_code(const char* iso3166_alpha2_code);

    void enable();
    void disable();

    // trigger replug sequence while charging to switch number of phases
    bool set_phase_switch_while_charging(bool enable);

    bool is_ventilated_charging_active();
    bool is_power_on();

    // TMA : get the CP / PP values  (Voltage / current)
    CPState get_current_cp_state();
    PPState get_current_pp_state();

    // TMA : definition of SLAC Status
    //SLACState slacflagStatus();
    void set_pwm_on(float dc);
    void set_pwm_off();
    void set_pwm_f();
    void allow_power_on(bool enable);

    void replug(unsigned int t);

    void run();

    void set_over_current(bool enable, uint32_t timeout);

private:
    HAL& hal;
    EventCallback push_event;
    Countdown pwm_stop_countdown;
    Countdown lock_check_countdown;
    Countdown overcurrent_countdown;

    static const uint32_t COUNTDOWN_PWM_STOP_IN_MS = 6000;
    static const uint32_t COUNTDOWN_LOCK_CHECK_IN_MS = 1000;
    static const uint32_t COUNTDOWN_OVERCURRENT_IN_MS = 7000;

    // TMA : added
    bool read_cp_state(CPState& cp);
    bool read_pp_state(PPState& pp);

    float calcul_dutyCycle(PPState& pp);

    bool power_on();
    bool power_off();
#if 0
    void checkLock();
#endif

#if 0
    void checkOverCurrent(std::queue<FSM::Event>& events);

    bool overcurrent;
    uint32_t ocTimeoutTick;
    uint32_t ocTimeout;
#endif

#if 0
    void startTimer(uint32_t msecs);
    bool timerElapsed();
    uint32_t timerCountdown;
    uint32_t timerTick;
#endif


#if 0
    uint32_t lockSwOffTick;
#endif
    CPState cur_state{CPState::Disabled};
    CPState prev_state{CPState::Disabled};

    bool power_on_allowed{false};
    bool rcd_reclosing_allowed{false};
    bool use_three_phases{true};
    bool use_three_phases_latch{true};
    bool has_ventilation{false};
    bool ventilated_charging_active{true};
    bool simplified_mode;
    //
    //SLACState slacPwmActivation;

    float pwm_duty_cycle{0};
    bool cur_pwm_running{false};
    bool prev_pwm_running{false};
};

const char* event_to_string(Event);
const char* state_to_string(CPState);

} // namespace iec61851

#endif // LIB_IEC61851_FSM_HPP
