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

bool Process::procHandler(const String  &property, HomieRange range, String value) {

    if ( property.equalsIgnoreCase("setpoint") ) {
    	setProperty(property).send(value);
        _pid.setSp(value.toFloat());
    }
    if ( property.equalsIgnoreCase("propband") ) {
    	setProperty(property).send(value);
        _pid.setPb(value.toFloat());
    }
    if ( property.equalsIgnoreCase("integraltime") ) {
    	setProperty(property).send(value);
        _pid.setTi(value.toFloat());
    }
    if ( property.equalsIgnoreCase("derivativetime") ) {
    	setProperty(property).send(value);
        _pid.setTd(value.toFloat());
    }
    if ( property.equalsIgnoreCase("initialintegral") ) {
    	setProperty(property).send(value);
        _pid.setInitialInt(value.toFloat());
    }
    if ( property.equalsIgnoreCase("maxinterval") ) {
    	setProperty(property).send(value);
        _pid.setMaxInterval(value.toInt());
    }
    // if ( property.equalsIgnoreCase("updateSeconds") ) {
    //     _pid.set(value.toInt);
    // }
    if ( property.equalsIgnoreCase("derivativesmooth") ) {
    	setProperty(property).send(value);
         _pid.setDSmooth(value.toFloat());
    }
    if ( property.equalsIgnoreCase("auto") ) {
    	setProperty(property).send(value);
         _pid.setAuto(value.toInt());
    }
    if ( property.equalsIgnoreCase("manualpower") ) {
    	setProperty(property).send(value);
         _pid.setManualPower(value.toFloat());
    }
    // if ( property.equalsIgnoreCase("updatesecs") ) {
    //      _pid.set(value.toFloat);
    // }

    if ( property.equalsIgnoreCase("cycletime") ) {
    	setProperty(property).send(value);
         _tp.setCt(value.toInt());
    }
    if ( property.equalsIgnoreCase("deadtime") ) {
    	setProperty(property).send(value);
         _tp.setDt(value.toInt());
    }
    if ( property.equalsIgnoreCase("invert") ) {
    	setProperty(property).send(value);
         _tp.setInvert(value.toInt());
    }
    if ( property.equalsIgnoreCase("fallbackpower") ) {
    	setProperty(property).send(value);
         _tp.setFallback(value.toFloat());
    }
    if ( property.equalsIgnoreCase("maxupdateinterval") ) {
    	setProperty(property).send(value);
         _tp.setInterval(value.toInt());
    }

    _run_pid_now = true;

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
	setProperty("setpoint").send(String(setpoint));
	setProperty("propband").send(String(prop_band));
	setProperty("integraltime").send(String(t_integral));
	setProperty("derivativetime").send(String(t_derivative));
	setProperty("initialintegral").send(String(integral_default));
	setProperty("maxinterval").send(String(max_interval));
	setProperty("updateSeconds").send(String(update_seconds));
	setProperty("derivativesmooth").send(String(smooth_factor));
	setProperty("auto").send(String(mode_auto));
	setProperty("manualpower").send(String(manual_op));
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
	setProperty("cycletime").send(String(cycleTime));
	setProperty("deadtime").send(String(deadTime));
	setProperty("invert").send(String(invert));
	setProperty("fallbackpower").send(String(fallbackPower));
	setProperty("maxupdateinterval").send(String(maxUpdateInterval));

    _tp.initialise(cycleTime, deadTime, invert, fallbackPower, maxUpdateInterval, nowSecs);
}

void Process::newPV(float value, unsigned long seconds) {
    Homie.getLogger() << "Temperature: " << value << " °C" << endl;
    setProperty("degrees").send(String(value));

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
    setProperty("power").send(String(power));

    _tp.setPower(power, seconds);
}

void Process::everySecond(unsigned long nowSecs) {
    static int sec_counter = 0;

    if (_run_pid_now  || nowSecs - _last_pv_update_secs > _max_interval  ||  (_update_seconds != 0 && sec_counter++ % _update_seconds  ==  0)) {
        runPID(nowSecs);
        _run_pid_now = false;
    }

    int newState = _tp.tick(nowSecs);
    if (_state != newState) {
        static int change_secs = 0;
        Homie.getLogger() << "new switch " << (newState ? "on" : "off") <<  " at : " << nowSecs << endl;
        switchOnOff(newState);
        setProperty("state").send(String(newState));
        setProperty("statetime").send(String(nowSecs-change_secs));
        Homie.getLogger() << "state time: " << nowSecs-change_secs << endl;
        _state = newState;
        change_secs = nowSecs;
    }
}

void Process::setup() {
    advertise("on").settable();
    advertise("unit");
    advertise("degrees");

    advertise("setpoint").settable();
    advertise("propband").settable();
    advertise("integraltime").settable();
    advertise("derivativetime").settable();
    advertise("initialintegral").settable();
    advertise("maxinterval").settable();
    advertise("updateSeconds").settable();
    advertise("derivativesmooth").settable();
    advertise("auto").settable();
    advertise("manualpower").settable();
    advertise("updatesecs").settable();
    advertise("power");

    advertise("cycletime").settable();
    advertise("deadtime").settable();
    advertise("invert").settable();
    advertise("fallbackpower").settable();
    advertise("maxupdateinterval").settable();
    advertise("state");
    advertise("statetime");
    
    pinMode(_pinRelay, OUTPUT);
    pinMode(_pinLED, OUTPUT);
    pinMode(_pinButton, INPUT);
    digitalWrite(_pinRelay, LOW);
    digitalWrite(_pinLED, HIGH);
}

void Process::onReadyToOperate() {
    setProperty("unit").send("c");

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
        << "property: " 
        << property
        << " index: " 
        << range.index
        << " value: " 
        << value 
        << endl;

    if ( property.equalsIgnoreCase("on") ) {
        return switchOnHandler(range,value);
    }

    return procHandler(property,range,value);
 
	return true;
}