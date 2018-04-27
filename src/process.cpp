

#include "process.h"

Process::Process(
    PID & pid,
    Timeprop & tp,
    HomieNode & node) : m_pid(pid), m_tp(tp), m_node(node), m_state(0), _handler(nullptr)
{
}

bool propertyInputHandler(HomieRange range, String value) {
  return true;
}

void Process::initPID(
    double setpoint, 
    double prop_band, 
    double t_integral, 
    double t_derivative,
    double integral_default, 
    int max_interval, 
    double smooth_factor, 
    unsigned char mode_auto, 
    double manual_op
    )
{
    m_node.advertise("setpoint").settable(propertyInputHandler);
	m_node.setProperty("setpoint").send(String(setpoint));

    m_node.advertise("propband").settable(propertyInputHandler);
	m_node.setProperty("propband").send(String(prop_band));
  
    m_node.advertise("integraltime").settable(propertyInputHandler);
	m_node.setProperty("integraltime").send(String(t_integral));
  
    m_node.advertise("derivativetime").settable(propertyInputHandler);
	m_node.setProperty("derivativetime").send(String(t_derivative));
  
    m_node.advertise("initialintegral").settable(propertyInputHandler);
	m_node.setProperty("initialintegral").send(String(integral_default));

    m_node.advertise("maxinterval").settable(propertyInputHandler);
	m_node.setProperty("maxinterval").send(String(max_interval));
  
    m_node.advertise("derivativesmooth").settable(propertyInputHandler);
	m_node.setProperty("derivativesmooth").send(String(smooth_factor));
  
    m_node.advertise("auto").settable(propertyInputHandler);
	m_node.setProperty("auto").send(String(mode_auto));

    m_node.advertise("manualpower").settable(propertyInputHandler);
	m_node.setProperty("manualpower").send(String(manual_op));

    m_node.advertise("updatesecs").settable(propertyInputHandler);
	m_node.setProperty("updatesecs").send(String(1));
    
    m_pid.initialise( setpoint, prop_band, t_integral, t_derivative, 
        integral_default, max_interval, smooth_factor, mode_auto, manual_op );  
}

void Process::initTP(
    int cycleTime, 
    int deadTime, 
    bool invert, 
    float fallbackPower, 
    int maxUpdateInterval,
    unsigned long nowSecs
)
{
    m_node.advertise("cycleTime").settable(propertyInputHandler);
	m_node.setProperty("cycleTime").send(String(cycleTime));

    m_node.advertise("deadTime").settable(propertyInputHandler);
	m_node.setProperty("deadTime").send(String(deadTime));

    m_node.advertise("invert").settable(propertyInputHandler);
	m_node.setProperty("invert").send(String(invert));

    m_node.advertise("fallbackPower").settable(propertyInputHandler);
	m_node.setProperty("fallbackPower").send(String(fallbackPower));

    m_node.advertise("maxUpdateInterval").settable(propertyInputHandler);
	m_node.setProperty("maxUpdateInterval").send(String(maxUpdateInterval));

    m_tp.initialise(cycleTime, deadTime, invert, fallbackPower, maxUpdateInterval, nowSecs);
}

void Process::everySecond(unsigned long nowSecs)
{
    static int sec_counter = 0;

    int newState = m_tp.tick(nowSecs);

    if (m_state != newState) {

        // set relay state here....
        m_state = newState;
    }

    // void Timeprop_Every_Second() {
    // for (int i=0; i<TIMEPROP_NUM_OUTPUTS; i++) {
    //     int newState = timeprops[i].tick(utc_time);
    //     if (newState != bitRead(currentRelayStates, relayNos[i]-1)){
    //     ExecuteCommandPower(relayNos[i], newState);
    //     }
    // }    


}
