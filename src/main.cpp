/*
 *   Tested with "WiFi Smart Socket ESP8266 MQTT"
 *   and "Sonoff - WiFi Wireless Smart Switch ESP8266 MQTT"
 *
 *   The Relay could be toggeled with the physical pushbutton
*/

#include <Homie.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "process.h"
#include "PID.h"
#include "Timeprop.h"


static PID pid;
static Timeprop tp;

const int PIN_ONE_WIRE = 14;
const int PIN_RELAY = 12;
const int PIN_LED = 13;
const int PIN_BUTTON = 0;
const int PIN_REDLED = 4;

const int TEMPERATURE_INTERVAL = 60;			// seconds
unsigned long lastTemperatureSent = 0;

OneWire oneWire(PIN_ONE_WIRE);
DallasTemperature DS18B20(&oneWire);

unsigned long buttonDownTime = 0;
byte lastButtonState = 1;
byte buttonPressHandled = 0;

HomieNode switchNode("switch", "switch");
HomieNode temperatureNode("temperature", "temperature");

Process proc(pid, tp, switchNode);

bool switchOnHandler(HomieRange range, String value) {
  if (value != "true" && value != "false") return false;

  bool on = (value == "true");
  digitalWrite(PIN_RELAY, on ? HIGH : LOW);
  digitalWrite(PIN_REDLED, on ? LOW : HIGH);
  switchNode.setProperty("on").send(value);
  Homie.getLogger() << "Switch is " << (on ? "on" : "off") << endl;

  return true;
}

void toggleRelay() {
  bool on = digitalRead(PIN_RELAY) == HIGH;
  digitalWrite(PIN_RELAY, on ? LOW : HIGH);
  digitalWrite(PIN_REDLED, on ? HIGH : LOW);
  switchNode.setProperty("on").send(on ? "false" : "true");
  Homie.getLogger() << "Switch is " << (on ? "off" : "on") << endl;
}

void loopHandler() {
    if (millis() - lastTemperatureSent >= TEMPERATURE_INTERVAL * 1000UL || lastTemperatureSent == 0) {
      float temperature = 22; // Fake temperature here, for the example

      DS18B20.requestTemperatures();
      temperature = DS18B20.getTempCByIndex(0);

      Homie.getLogger() << "Temperature: " << temperature << " °C" << endl;
      switchNode.setProperty("degrees").send(String(temperature));
      lastTemperatureSent = millis();
    }
  
  byte buttonState = digitalRead(PIN_BUTTON);
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

void setupHandler()
{
	switchNode.setProperty("unit").send("c");

  proc.init_pid( 
    PID_SETPOINT, 
    PID_PROPBAND, 
    PID_INTEGRAL_TIME, 
    PID_DERIVATIVE_TIME, 
    PID_INITIAL_INT, 
    PID_MAX_INTERVAL, 
    PID_DERIV_SMOOTH_FACTOR, 
    PID_AUTO, 
    PID_MANUAL_POWER 
    );  

  proc.init_tp(
    TIMEPROP_CYCLETIME,
    TIMEPROP_DEADTIME,
    TIMEPROP_OPINVERT,
    TIMEPROP_FALLBACK_POWER,
    TIMEPROP_MAX_UPDATE_INTERVAL,
    millis() / 1000
  ); 
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println();
  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_REDLED, OUTPUT);
  pinMode(PIN_BUTTON, INPUT);
  digitalWrite(PIN_RELAY, LOW);
  digitalWrite(PIN_REDLED, HIGH);

  Homie_setFirmware("itead-sonoff-buton", "1.0.3");
  Homie.setLedPin(PIN_LED, LOW).setResetTrigger(PIN_BUTTON, LOW, 5000);

  switchNode.advertise("on").settable(switchOnHandler);

 	Homie.setSetupFunction(setupHandler);
  Homie.setLoopFunction(loopHandler);
  Homie.setup();
}

void loop() {
  Homie.loop();
}
