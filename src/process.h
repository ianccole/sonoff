#ifndef process_h
#define process_h

#include <Homie.h>
#include <PID.h>
#include <Timeprop.h>

#define PID_SETPOINT                  19.5      // Setpoint value. This is the process value that the process is
                                                // aiming for.
                                                // May be adjusted via MQTT using cmnd pid_sp

#define PID_PROPBAND                  5         // Proportional band in process units (eg degrees). This controls
                                                // the gain of the loop and is the range of process value over which
                                                // the power output will go from 0 to full power. The units are that
                                                // of the process and setpoint, so for example in a heating
                                                // application it might be set to 1.5 degrees.
                                                // May be adjusted via MQTT using cmnd pid_pb

#define PID_INTEGRAL_TIME             1800      // Integral time seconds. This is a setting for the integral time,
                                                // in seconds. It represents the time constant of the integration
                                                // effect. The larger the value the slower the integral effect will be.
                                                // Obviously the slower the process is the larger this should be. For
                                                // example for a domestic room heated by convection radiators a setting
                                                // of one hour might be appropriate (in seconds). To disable the
                                                // integral effect set this to a large number.
                                                // May be adjusted via MQTT using cmnd pid_ti

#define PID_DERIVATIVE_TIME           15        // Derivative time seconds. This is a setting for the derivative time,
                                                // in seconds. It represents the time constant of the derivative effect.
                                                // The larger the value the greater will be the derivative effect.
                                                // Typically this will be set to somewhat less than 25% of the integral
                                                // setting, once the integral has been adjusted to the optimum value. To
                                                // disable the derivative effect set this to 0. When initially tuning a
                                                // loop it is often sensible to start with derivative zero and wind it in
                                                // once other parameters have been setup.
                                                // May be adjusted via MQTT using cmnd pid_td

#define PID_INITIAL_INT               0.5       // Initial integral value (0:1). This is an initial value which is used
                                                // to preset the integrated error value when the flow is deployed in
                                                // order to assist in homing in on the setpoint the first time. It should
                                                // be set to an estimate of what the power requirement might be in order
                                                // to maintain the process at the setpoint. For example for a domestic
                                                // room heating application it might be set to 0.2 indicating that 20% of
                                                // the available power might be required to maintain the setpoint. The
                                                // value is of no consequence apart from device restart.

#define PID_MAX_INTERVAL              300       // This is the maximum time in seconds that is expected between samples.
                                                // It is provided to cope with unusual situations such as a faulty sensor
                                                // that might prevent the node from being supplied with a process value.
                                                // If no new process value is received for this time then the power is set
                                                // to the value defined for PID_MANUAL_POWER.
                                                // May be adjusted via MQTT using cmnd pid_max_interval

#define PID_DERIV_SMOOTH_FACTOR       3         // In situations where the process sensor has limited resolution (such as
                                                // the DS18B20), the use of deriviative can be problematic as when the
                                                // process is changing only slowly the steps in the value cause spikes in
                                                // the derivative. To reduce the effect of these this parameter can be
                                                // set to apply a filter to the derivative term. I have found that with
                                                // the DS18B20 that a value of 3 here can be beneficial, providing
                                                // effectively a low pass filter on the derivative at 1/3 of the derivative
                                                // time. This feature may also be useful if the process value is particularly
                                                // noisy. The smaller the value the greater the filtering effect but the
                                                // more it will reduce the effectiveness of the derivative. A value of zero
                                                // disables this feature.
                                                // May be adjusted via MQTT using cmnd pid_d_smooth

#define PID_AUTO                      1         // Auto mode 1 or 0 (for manual). This can be used to enable or disable
                                                // the control (1=enable, auto mode, 0=disabled, manual mode). When in
                                                // manual mode the output is set the value definded for PID_MANUAL_POWER
                                                // May be adjusted via MQTT using cmnd pid_auto

#define PID_MANUAL_POWER              0         // Power output when in manual mode or fallback mode if too long elapses
                                                // between process values
                                                // May be adjusted via MQTT using cmnd pid_manual_power

#define PID_UPDATE_SECS               0         // How often to run the pid algorithm (integer secs) or 0 to run the algorithm
                                                // each time a new pv value is received, for most applictions specify 0.
                                                // Otherwise set this to a time
                                                // that is short compared to the response of the process.  For example,
                                                // something like 15 seconds may well be appropriate for a domestic room
                                                // heating application.
                                                // May be adjusted via MQTT using cmnd pid_update_secs

#define PID_USE_TIMPROP               1         // To use an internal relay for a time proportioned output to drive the
                                                // process, set this to indicate which timeprop output to use. For a device
                                                // with just one relay then this will be 1.
                                                // It is then also necessary to define USE_TIMEPROP and set the output up as
                                                // explained in xdrv_91_timeprop.ino
                                                // To disable this feature leave this undefined (undefined, not defined to nothing).

#define TIMEPROP_CYCLETIME           60         // cycle time seconds
#define TIMEPROP_DEADTIME            0          // actuator action time seconds
#define TIMEPROP_OPINVERT            false      // whether to invert the output
#define TIMEPROP_FALLBACK_POWER      0          // falls back to this if too long betwen power updates
#define TIMEPROP_MAX_UPDATE_INTERVAL 120        // max no secs that are allowed between power updates (0 to disable)

typedef std::function<bool(bool on)> switchHandler;

class Process {
public:
    Process(
        PID & pid,
        Timeprop & tp,
        HomieNode & node);

    void initPID(
        double setpoint, 
        double prop_band, 
        double t_integral, 
        double t_derivative,
        double integral_default, 
        int max_interval, 
        double smooth_factor, 
        unsigned char mode_auto, 
        double manual_op
        );

    void initTP(
        int cycleTime, 
        int deadTime, 
        bool invert, 
        float fallbackPower, 
        int maxUpdateInterval,
        unsigned long nowSecs
    );

    void setHandler(const switchHandler& handler) { _handler = handler; }
    void everySecond(unsigned long nowSecs);

private:
    void runPID();

    PID&            m_pid;
    Timeprop&       m_tp;
    int             m_state;

    HomieNode&      m_node;

    switchHandler   _handler;
};

#endif // process_h
