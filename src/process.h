#ifndef process_h
#define process_h

#include <Homie.h>
#include <PID.h>
#include <Timeprop.h>

class Process {
public:
    Process(
        PID & pid,
        Timeprop & tp,
        HomieNode & node);

    void init_pid(
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

    void init_tp(
        int cycleTime, 
        int deadTime, 
        unsigned char invert, 
        float fallbackPower, 
        int maxUpdateInterval,
        unsigned long nowSecs
    );

private:
    PID&        m_pid;
    Timeprop&   m_tp;
    HomieNode&  m_node;

};

#endif // process_h
