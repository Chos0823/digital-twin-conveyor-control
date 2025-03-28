#include <ArduinoJson.h>

#define PIN_ENA   8
#define PIN_DIR   9
#define PIN_PUL   10

typedef enum _CONVEYOR_STATE {INIT, READY, RUN, DISCONNECT} CONVEYOR_STATE;

unsigned long step_count = 0;
CONVEYOR_STATE state = INIT;

const unsigned long STEPS_PER_100MM = 918;  // 100mm 이동 시 918 스텝
const unsigned long PULSE_DELAY = 500;  // 펄스 간 딜레이 (마이크로초)

void setup() {
    Serial.begin(115200);
    Serial.println("INIT");

    pinMode(PIN_ENA, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_PUL, OUTPUT);

    digitalWrite(PIN_ENA, LOW);
    digitalWrite(PIN_DIR, LOW);

    state = READY;
    Serial.println("READY");
}

void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        processCommand(input);
    }

    if (state == RUN) {
        runMotor();
    }
}

void processCommand(String input) {
    StaticJsonDocument<128> doc;
    DeserializationError error = deserializeJson(doc, input);

    if (error) {
        Serial.println("DISCONNECT");
        return;
    }

    String control = doc["control"];
    
    if (control == "go") {
        int distance = doc["distance.mm"];
        step_count = (distance / 100.0) * STEPS_PER_100MM;
        state = RUN;
        Serial.println("RUN");
    } 
    else if (control == "stop") {
        step_count = 0;
        state = READY;
        Serial.println("READY");
    }
}

void runMotor() {
    static unsigned long last_pulse_time = 0;
    static unsigned long steps_moved = 0;

    if (steps_moved < step_count) {
        unsigned long current_time = millis();
        if (current_time - last_pulse_time >= PULSE_DELAY) {
            digitalWrite(PIN_PUL, HIGH);
            delayMicroseconds(10);
            digitalWrite(PIN_PUL, LOW);
            
            last_pulse_time = current_time;
            steps_moved++;
        }
    } else {
        state = READY;
        steps_moved = 0;
        Serial.println("READY");
    }
}