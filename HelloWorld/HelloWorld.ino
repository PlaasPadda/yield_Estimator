/* The true ESP32 chip ID is essentially its MAC address.
This sketch provides an alternate chip ID that matches
the output of the ESP.getChipId() function on ESP8266
(i.e. a 32-bit integer matching the last 3 bytes of
the MAC address. This is less unique than the
MAC address chip ID, but is helpful when you need
an identifier that can be no more than a 32-bit integer
(like for switch...case).

created 2020-06-07 by cweinhofer
with help from Cicicok */

float steering;
uint16_t power; // use int16_t if signed

void setup() {
    Serial.begin(115200);
}

void loop() {
    if (Serial.available() >= 6) { // 4 bytes float + 2 bytes short
        Serial.readBytes((char*)&steering, sizeof(steering));
        Serial.readBytes((char*)&power, sizeof(power));

        Serial.print("Steering: ");
        Serial.print(steering, 1); // print 3 decimal places
        Serial.print("Power: ");
        Serial.println(power);
    }

}
