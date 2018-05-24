/*
 *   Tested with "WiFi Smart Socket ESP8266 MQTT"
 *   and "Sonoff - WiFi Wireless Smart Switch ESP8266 MQTT"
 *
 *   The Relay could be toggeled with the physical pushbutton
*/

#include <Homie.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DHT.h"

#include "process.h"
#include "PID.h"
#include "Timeprop.h"

#define DHTPIN 14 
#define DHTTYPE DHT21

const int PIN_ONE_WIRE = 14;
const int PIN_RELAY = 12;
const int PIN_LED = 13;
const int PIN_BUTTON = 0;
const int PIN_REDLED = 4;

const int TEMPERATURE_INTERVAL = 60;			// seconds
unsigned long lastTemperatureSent = 0;

DHT dht(DHTPIN, DHTTYPE);

// OneWire oneWire(PIN_ONE_WIRE);
// DallasTemperature DS18B20(&oneWire);

unsigned long buttonDownTime = 0;
byte lastButtonState = 1;
byte buttonPressHandled = 0;

// HomieNode switchNode("switch", "switch");
HomieNode temperatureNode("temperature", "temperature");
HomieNode humidityNode("humidity", "humidity");

static Process proc(PIN_RELAY,PIN_REDLED,PIN_BUTTON);

// bool switchOnOff(bool on) {

//     digitalWrite(PIN_RELAY, on ? HIGH : LOW);
//     digitalWrite(PIN_REDLED, on ? LOW : HIGH);
//     switchNode.setProperty("on").send(on ? "true" : "false");
//     Homie.getLogger() << "Switch is " << (on ? "on" : "off") << endl;

//     return digitalRead(PIN_RELAY) == HIGH;;
// }

// bool switchOnHandler(HomieRange range, String value) {
//     if (value != "true" && value != "false") return false;

//     bool on = (value == "true");

//     switchOnOff(on);

//     return true;
// }

// void toggleRelay() {
//     bool on = digitalRead(PIN_RELAY) == HIGH;
//     digitalWrite(PIN_RELAY, on ? LOW : HIGH);
//     digitalWrite(PIN_REDLED, on ? HIGH : LOW);
//     switchNode.setProperty("on").send(on ? "false" : "true");
//     Homie.getLogger() << "Switch is " << (on ? "off" : "on") << endl;
// }

void loopHandler() {

    if (millis() - lastTemperatureSent >= TEMPERATURE_INTERVAL * 1000UL || lastTemperatureSent == 0) {

        // DS18B20.requestTemperatures();
        // float temperature = DS18B20.getTempCByIndex(0);

        float humidity = dht.readHumidity();
        float temperature = dht.readTemperature(true);
        temperatureNode.setProperty("F").send(String(temperature));
        humidityNode.setProperty("F").send(String(humidity));

        lastTemperatureSent = millis();
        proc.newIntTemp(temperature);
    }
}

void setupHandler() {
    // proc.setup();
    // proc.setHandler(switchOnOff);

    temperatureNode.setProperty("unit").setRetained(true).send("F");
    humidityNode.setProperty("unit").setRetained(true).send("percent");  
}

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println();

    Homie_setFirmware("itead-sonoff-buton", "1.0.3");
    Homie.setLedPin(PIN_LED, LOW).setResetTrigger(PIN_BUTTON, LOW, 5000);

    // switchNode.advertise("on").settable(switchOnHandler);

    Homie.setSetupFunction(setupHandler);
    Homie.setLoopFunction(loopHandler);

    temperatureNode.advertise("unit");
    temperatureNode.advertise("temperature");
    humidityNode.advertise("unit");
    humidityNode.advertise("percent");

    Homie.setup();
}

void loop() {
    Homie.loop();
}
