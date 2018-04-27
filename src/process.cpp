

#include "process.h"

Process::Process(
    PID & pid,
    Timeprop & tp,
    HomieNode & node) 
    : 
    _pid(pid), 
    _tp(tp), 
    _node(node), 
    _state(0), 
    _handler(nullptr),
    _max_interval(PID_MAX_INTERVAL),
    _update_seconds(PID_UPDATE_SECS),
    _last_pv_update_secs(0),
    _run_pid_now(false) {
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
    int update_seconds,
    double smooth_factor, 
    unsigned char mode_auto, 
    double manual_op
    )
{
    _node.advertise("setpoint").settable(propertyInputHandler);
	_node.setProperty("setpoint").send(String(setpoint));

    _node.advertise("propband").settable(propertyInputHandler);
	_node.setProperty("propband").send(String(prop_band));
  
    _node.advertise("integraltime").settable(propertyInputHandler);
	_node.setProperty("integraltime").send(String(t_integral));
  
    _node.advertise("derivativetime").settable(propertyInputHandler);
	_node.setProperty("derivativetime").send(String(t_derivative));
  
    _node.advertise("initialintegral").settable(propertyInputHandler);
	_node.setProperty("initialintegral").send(String(integral_default));

    _node.advertise("maxinterval").settable(propertyInputHandler);
	_node.setProperty("maxinterval").send(String(max_interval));

    _node.advertise("updateSeconds").settable(propertyInputHandler);
	_node.setProperty("updateSeconds").send(String(update_seconds));
  
    _node.advertise("derivativesmooth").settable(propertyInputHandler);
	_node.setProperty("derivativesmooth").send(String(smooth_factor));
  
    _node.advertise("auto").settable(propertyInputHandler);
	_node.setProperty("auto").send(String(mode_auto));

    _node.advertise("manualpower").settable(propertyInputHandler);
	_node.setProperty("manualpower").send(String(manual_op));

    _node.advertise("updatesecs").settable(propertyInputHandler);
	_node.setProperty("updatesecs").send(String(1));
    
    _pid.initialise( setpoint, prop_band, t_integral, t_derivative, 
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
    _node.advertise("cycleTime").settable(propertyInputHandler);
	_node.setProperty("cycleTime").send(String(cycleTime));

    _node.advertise("deadTime").settable(propertyInputHandler);
	_node.setProperty("deadTime").send(String(deadTime));

    _node.advertise("invert").settable(propertyInputHandler);
	_node.setProperty("invert").send(String(invert));

    _node.advertise("fallbackPower").settable(propertyInputHandler);
	_node.setProperty("fallbackPower").send(String(fallbackPower));

    _node.advertise("maxUpdateInterval").settable(propertyInputHandler);
	_node.setProperty("maxUpdateInterval").send(String(maxUpdateInterval));

    _tp.initialise(cycleTime, deadTime, invert, fallbackPower, maxUpdateInterval, nowSecs);
}

void Process::newPV(float value, unsigned long seconds) {
    Homie.getLogger() << "new PV: " << value << " at " << seconds << endl;

    _last_pv_update_secs = seconds;
    _pid.setPv(value, seconds);
    if (_update_seconds == 0) {
        _run_pid_now = true;
    }
}

void Process::runPID(int seconds) {
    Homie.getLogger() << "runPID seconds: " << seconds << endl;

    double power = _pid.tick(seconds);

    Homie.getLogger() << "runPID power: " << power << endl;

//   char buf[10];
//   dtostrfd(power, 3, buf);
//   snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("{\"%s\":\"%s\"}"), "power", buf);
//   MqttPublishPrefixTopic_P(TELE, "PID", false);

// #if defined PID_USE_TIMPROP
//     // send power to appropriate timeprop output
//     Timeprop_Set_Power( PID_USE_TIMPROP-1, power );
// #endif // PID_USE_TIMPROP

    _tp.setPower(power, seconds);
}

void Process::everySecond(unsigned long nowSecs) {
    static int sec_counter = 0;

    // Homie.getLogger() << "sec_counter: " << sec_counter << endl;
    // Homie.getLogger() << "sec_counter: " << sec_counter << endl;

    if (_run_pid_now  || nowSecs - _last_pv_update_secs > _max_interval  ||  (_update_seconds != 0 && sec_counter++ % _update_seconds  ==  0)) {
        runPID(nowSecs);
        _run_pid_now = false;
    }

    int newState = _tp.tick(nowSecs);
    if (_state != newState) {
        Homie.getLogger() << "new switch " << (newState ? "on" : "off") <<  " at : " << nowSecs << endl;
        if (_handler){
            _handler(newState);    
        }

        _state = newState;
    }

    // void Timeprop_Every_Second() {
    // for (int i=0; i<TIMEPROP_NUM_OUTPUTS; i++) {
    //     int newState = timeprops[i].tick(utc_time);
    //     if (newState != bitRead(currentRelayStates, relayNos[i]-1)){
    //     ExecuteCommandPower(relayNos[i], newState);
    //     }
    // }
}
