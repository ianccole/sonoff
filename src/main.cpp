/*
 *   Tested with "WiFi Smart Socket ESP8266 MQTT"
 *   and "Sonoff - WiFi Wireless Smart Switch ESP8266 MQTT"
 *
 *   The Relay could be toggeled with the physical pushbutton
*/

#include <Homie.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//#include <PID_v1.h>
#include "PID.h"

static PID pid;

const int PIN_ONE_WIRE = 14;
const int PIN_RELAY = 12;
const int PIN_LED = 13;
const int PIN_BUTTON = 0;

const int TEMPERATURE_INTERVAL = 60;			// seconds
unsigned long lastTemperatureSent = 0;

OneWire oneWire(PIN_ONE_WIRE);
DallasTemperature DS18B20(&oneWire);

unsigned long buttonDownTime = 0;
byte lastButtonState = 1;
byte buttonPressHandled = 0;

HomieNode switchNode("switch", "switch");
HomieNode temperatureNode("temperature", "temperature");

bool switchOnHandler(HomieRange range, String value) {
  if (value != "true" && value != "false") return false;

  bool on = (value == "true");
  digitalWrite(PIN_RELAY, on ? HIGH : LOW);
  switchNode.setProperty("on").send(value);
  Homie.getLogger() << "Switch is " << (on ? "on" : "off") << endl;

  return true;
}

void toggleRelay() {
  bool on = digitalRead(PIN_RELAY) == HIGH;
  digitalWrite(PIN_RELAY, on ? LOW : HIGH);
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

bool propertyInputHandler(HomieRange range, String value) {

  Homie.getLogger() << "new value is " << value << endl;
  return true;
}

void setupHandler()
{
	switchNode.setProperty("unit").send("c");

  switchNode.advertise("setpoint").settable(propertyInputHandler);
	switchNode.setProperty("setpoint").send("19.5");

  switchNode.advertise("propband").settable(propertyInputHandler);
	switchNode.setProperty("propband").send("5");
  
  switchNode.advertise("integraltime").settable(propertyInputHandler);
	switchNode.setProperty("integraltime").send("1800");
  
  switchNode.advertise("derivativetime").settable(propertyInputHandler);
	switchNode.setProperty("derivativetime").send("15");
  
  switchNode.advertise("initialintegral").settable(propertyInputHandler);
	switchNode.setProperty("initialintegral").send("0.5");

  switchNode.advertise("maxinterval").settable(propertyInputHandler);
	switchNode.setProperty("maxinterval").send("300");
  
  switchNode.advertise("derivativesmooth").settable(propertyInputHandler);
	switchNode.setProperty("derivativesmooth").send("3");
  
  switchNode.advertise("auto").settable(propertyInputHandler);
	switchNode.setProperty("auto").send("1");

  switchNode.advertise("timprop").settable(propertyInputHandler);
	switchNode.setProperty("timprop").send("1");
  
  switchNode.advertise("manualpower").settable(propertyInputHandler);
	switchNode.setProperty("manualpower").send("0");

  switchNode.advertise("updatesecs").settable(propertyInputHandler);
	switchNode.setProperty("updatesecs").send("1");
  
  pid.initialise( 19.5, 5, 1800, 15, 0.5, 300, 3, 1, 0 );  
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println();
  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_BUTTON, INPUT);
  digitalWrite(PIN_RELAY, LOW);

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
