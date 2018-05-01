#include "process.h"

Process::Process(
        int pinRelay,
        int pinLED,
        int pinButton
) 
    : HomieNode("switch", "switch"),
    _pinRelay(pinRelay),
    _pinLED(pinLED),
    _pinButton(pinButton),
    _state(0), 
    _handler(nullptr),
    _max_interval(PID_MAX_INTERVAL),
    _update_seconds(PID_UPDATE_SECS),
    _last_pv_update_secs(0),
    _run_pid_now(false) {
}

bool Process::switchOnOff(bool on) {

    digitalWrite(_pinRelay, on ? HIGH : LOW);
    digitalWrite(_pinLED, on ? LOW : HIGH);
    setProperty("on").send(on ? "true" : "false");
    Homie.getLogger() << "Switch is " << (on ? "on" : "off") << endl;

    return digitalRead(_pinRelay) == HIGH;;
}

bool Process::switchOnHandler(HomieRange range, String value) {
    if (value != "true" && value != "false") return false;

    bool on = (value == "true");

    switchOnOff(on);

    return true;
}

void Process::init() {
    setProperty("unit").send("c");
    advertise("on").settable();

    initPID( 
        PID_SETPOINT, 
        PID_PROPBAND, 
        PID_INTEGRAL_TIME, 
        PID_DERIVATIVE_TIME, 
        PID_INITIAL_INT, 
        PID_MAX_INTERVAL,
        PID_UPDATE_SECS, 
        PID_DERIV_SMOOTH_FACTOR, 
        PID_AUTO, 
        PID_MANUAL_POWER 
        );  

    initTP(
        TIMEPROP_CYCLETIME,
        TIMEPROP_DEADTIME,
        TIMEPROP_OPINVERT,
        TIMEPROP_FALLBACK_POWER,
        TIMEPROP_MAX_UPDATE_INTERVAL,
        millis() / 1000
    ); 
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
    advertise("setpoint").settable(propertyInputHandler);
	setProperty("setpoint").send(String(setpoint));

    advertise("propband").settable(propertyInputHandler);
	setProperty("propband").send(String(prop_band));
  
    advertise("integraltime").settable(propertyInputHandler);
	setProperty("integraltime").send(String(t_integral));
  
    advertise("derivativetime").settable(propertyInputHandler);
	setProperty("derivativetime").send(String(t_derivative));
  
    advertise("initialintegral").settable(propertyInputHandler);
	setProperty("initialintegral").send(String(integral_default));

    advertise("maxinterval").settable(propertyInputHandler);
	setProperty("maxinterval").send(String(max_interval));

    advertise("updateSeconds").settable(propertyInputHandler);
	setProperty("updateSeconds").send(String(update_seconds));
  
    advertise("derivativesmooth").settable(propertyInputHandler);
	setProperty("derivativesmooth").send(String(smooth_factor));
  
    advertise("auto").settable(propertyInputHandler);
	setProperty("auto").send(String(mode_auto));

    advertise("manualpower").settable(propertyInputHandler);
	setProperty("manualpower").send(String(manual_op));

    advertise("updatesecs").settable(propertyInputHandler);
	setProperty("updatesecs").send(String(1));
    
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
    advertise("cycleTime").settable(propertyInputHandler);
	setProperty("cycleTime").send(String(cycleTime));

    advertise("deadTime").settable(propertyInputHandler);
	setProperty("deadTime").send(String(deadTime));

    advertise("invert").settable(propertyInputHandler);
	setProperty("invert").send(String(invert));

    advertise("fallbackPower").settable(propertyInputHandler);
	setProperty("fallbackPower").send(String(fallbackPower));

    advertise("maxUpdateInterval").settable(propertyInputHandler);
	setProperty("maxUpdateInterval").send(String(maxUpdateInterval));

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

void Process::setup() {
    setProperty("unit").send("c");
    advertise("on").settable();


    init();
}

void Process::onReadyToOperate() {
	// LN.log("RelaisNode", LoggerNode::DEBUG, "Ready");
	// RelaisNode::updateRelais(0xFFFF);
}

void Process::toggleRelay() {
    bool on = digitalRead(_pinRelay) == HIGH;
    digitalWrite(_pinRelay, on ? LOW : HIGH);
    digitalWrite(_pinLED, on ? HIGH : LOW);
    setProperty("on").send(on ? "false" : "true");
    Homie.getLogger() << "Switch is " << (on ? "off" : "on") << endl;
}

void Process::loop() {
    static unsigned long buttonDownTime = 0;
    static byte lastButtonState = 1;
    static byte buttonPressHandled = 0;

    if (millis() % 1000 == 0) {
        everySecond(millis() / 1000);
    }

    byte buttonState = digitalRead(_pinButton);
    if ( buttonState != lastButtonState ) {
        if (buttonState == LOW) {
            buttonDownTime     = millis();
            buttonPressHandled = 0;
        }
        else {
            unsigned long dt = millis() - buttonDownTime;
            if ( dt >= 90 && dt <= 900 && buttonPressHandled == 0 ) {
                toggleRelay();
                buttonPressHandled = 1;
            }
        }
        lastButtonState = buttonState;
    }
    
}

bool Process::handleInput(const String  &property, const HomieRange& range, const String &value) {
    Homie.getLogger() 
        << "Event: property " 
        << property
        << "index " 
        << range.index
        << "value " 
        << value 
        << endl;

    if ( property.equalsIgnoreCase("on") ) {
        switchOnHandler(range,value);
    }

	// int16_t id = range.index;
	// if (id <= 0 || id > 16) {
	// 	LN.logf("RelaisNode::handleInput()", LoggerNode::ERROR,
	// 			"Receive unknown property %s with value %s.", property.c_str(),
	// 			value.c_str());
	// 	return false;
	// }
	// bool on = value.equalsIgnoreCase("ON");
	// uint16_t selected_bit = (1 << (id-1));
	// bool inverted = (invert_bitset & selected_bit) != 0;
	// LN.logf("RelaisNode::handleInput()", LoggerNode::INFO,
	// 		"Receive command to switch %d to %c%s.", id, inverted ? '~':' ', on ? "On" : "Off");

	// if (on ^ inverted ) {
	// 	relais_bitset |= selected_bit;
	// } else	{
	// 	relais_bitset &= ~selected_bit;
	// }
	// updateMaskLoop |= selected_bit;
	return true;
}