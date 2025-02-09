#include <Arduino.h>
#include <ESP32Servo.h>
#include "encoder.hpp"
#include "adc.hpp"

TLE5012 tle;

void debugBlink() {
    const int LED_PIN = 2;
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    delay(20);
    digitalWrite(LED_PIN, LOW);
}

Servo servo;
float offset;

void setup() {
    debugBlink();
    Serial.begin(2000000);
    Serial.println("start");
    tle.init();
    adc_init();
    uint16_t tmp;

    // uint8_t data[4] = {0x50, 0x81, 0x08, 0x04};
    // uint8_t crc = tle.getCRC(data, 4);
    // Serial.printf("CRC RESULT: %#02x\n", crc);
    // while (true) {};

    // tmp = tle.readOneRegister(REG_AVAL);
    // Serial.printf("angle: %#04x\n", tmp);
    // tmp = tle.readOneRegister(REG_ASPD);
    // Serial.printf("speed: %#04x\n", tmp);

    // tmp = tle.readOneRegister(REG_MOD_1);
    // Serial.printf("read before: %#04x\n", tmp);

    // bool slow_upd_rate = true;
    // bool half_precision = false;
    // bool prediction = false;

    // Serial.setTimeout(1000);
    // slow_upd_rate = Serial.parseInt();
    // half_precision = Serial.parseInt();
    // prediction = Serial.parseInt();

    // if (prediction || half_precision) {
    //     Serial.println("disable autocal");
    //     tle.writeOneRegister(TLE5012::REG_MOD_2, 1, 0, 0b00); // disable autocal (AUTOCAL)
    // }
    // if (half_precision) {
    //     Serial.println("set range factor 0.5");
    //     tle.writeOneRegister(TLE5012::REG_MOD_2, 14, 4, 0x040); // set angle range to factor 0.5
    // }
    // if (slow_upd_rate) {
    //     Serial.println("set slower update rate");
    //     tle.writeOneRegister(TLE5012::REG_MOD_1, 15, 14, 0b11); // set update rate (FIR_MD)
    // }
    // if (prediction) {
    //     Serial.println("enable prediction");
    //     tle.writeOneRegister(TLE5012::REG_MOD_2, 2, 2, 1); // enable prediction
    // }
    // if (prediction && !half_precision) {
    //     Serial.println("enable autocal");
    //     tle.writeOneRegister(TLE5012::REG_MOD_2, 1, 0, 0b01); // enable autocal (AUTOCAL)
    // }

    // tmp = tle.readOneRegister(TLE5012::REG_STAT);
    // Serial.printf("STAT REG: %#04x\n", tmp);
    // Serial.printf("- RD_ST: %#01x\n", tle.getbits(tmp, 15, 15));
    // Serial.printf("- S_NR: %#01x\n", tle.getbits(tmp, 14, 13));
    // Serial.printf("- NO_GMR_A: %#01x\n", tle.getbits(tmp, 12, 12));
    // Serial.printf("- NO_GMR_XY: %#01x\n", tle.getbits(tmp, 11, 11));
    // Serial.printf("- S_ROM: %#01x\n", tle.getbits(tmp, 10, 10));
    // Serial.printf("- S_ADCT: %#01x\n", tle.getbits(tmp, 9, 9));
    // Serial.printf("- S_MAGOL: %#01x\n", tle.getbits(tmp, 7, 7));
    // Serial.printf("- S_XYOL: %#01x\n", tle.getbits(tmp, 6, 6));
    // Serial.printf("- S_OV: %#01x\n", tle.getbits(tmp, 5, 5));
    // Serial.printf("- S_DSPU: %#01x\n", tle.getbits(tmp, 4, 4));
    // Serial.printf("- S_FUSE: %#01x\n", tle.getbits(tmp, 3, 3));
    // Serial.printf("- S_VR: %#01x\n", tle.getbits(tmp, 2, 2));
    // Serial.printf("- S_WD: %#01x\n", tle.getbits(tmp, 1, 1));
    // Serial.printf("- S_RST: %#01x\n", tle.getbits(tmp, 0, 0));

    // tle.writeOneRegister(REG_MOD_2, 1, 0, 0b10); // enable autocal (AUTOCAL)

    // tmp = tle.readOneRegister(REG_MOD_1);
    // Serial.printf("read after: %#04x\n", tmp);

    // tle.writeOneRegister(REG_ACSTAT, 10, 10, 1);

    // tle.readOneRegister(0x2);
    Serial.println("end");
    
    // int iter = 100;
    // for (int i = 0; i < iter; ++i) {
    //     tle.update();
    //     offset += tle.angle;
    // }
    // offset /= (float) iter;

    const int ADC_PIN = 34;
    adcAttachPin(ADC_PIN);
    analogSetAttenuation(ADC_11db);

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    float SERVO_RANGE = 180.0;
    servo.setPeriodHertz(200);
    servo.attach(32, 500, 2500);
    servo.write(0);
    delay(1500);

    tle.update();
    float encZero = tle.getAngle() / PI * 180.0;
    float adcZero = analogReadMilliVolts(ADC_PIN) / 1000.0f;

    for (uint16_t ang = 0; ang <= SERVO_RANGE; ang += 10) {
        uint16_t angUs = static_cast<uint16_t>(500.0 + ang / SERVO_RANGE * 2000.0);
        // servo.write(ang);
        servo.writeMicroseconds(angUs);
        // delay(300);
        for (int i = 0; i < 100; i++) {
            tle.update();
            float enc = 1.0 * (tle.getAngle() / PI * 180.0 - encZero);
            float adc = -1.0 * (analogReadMilliVolts(ADC_PIN) / 1000.0f - adcZero);
            // float adcv = adc * 2.450 / 4095.0;
            Serial.printf(
                "ang: %d, adc: %0.3f, enc: %0.1f\n",
                ang, adc, enc
            );
            delay(1);
        }
        // Serial.printf("\n");
    }

    Serial.println("stop");

    // servo.write(90);
    // delay(1000);
    // tmp = analogRead(ADC_PIN);
    // Serial.printf("adc: %d, volt: %0.3f\n", tmp, ((float) tmp) / 4096.0);

    // servo.write(180);
    // delay(1000);
    // tmp = analogRead(ADC_PIN);
    // Serial.printf("adc: %d, volt: %0.3f\n", tmp, ((float) tmp) / 4096.0);
}

unsigned long start = millis();

struct msg {
    float angle;
    float velocity;
    uint32_t time;
};

void loop() {
    // unsigned long now = millis();
    // uint16_t tmp;
    // // tmp = tle.readOneRegister(TLE5012::REG_STAT);
    // // Serial.printf("stat: %#04x\n", tmp);
    // tmp = tle.readOneRegister(TLE5012::REG_AVAL);
    // Serial.printf("%#04x ", tmp);
    // tmp = tle.readOneRegister(TLE5012::REG_ASPD);
    // Serial.printf("%#04x ", tmp);
    // tmp = now;
    // Serial.printf("%#04x\n", tmp);

    // if (millis() - start > 5000) {
    //     while (true) {}
    // }

    tle.update();
    // msg m = {tle.getAngle() - offset, tle.getVelocity(), millis()};
    // Serial.printf("a: %+0.3f, v: %+0.3f\n", tle.getAngle(), tle.getVelocity());
    // // Serial.printf("%+0.3f, %+0.3f, %d\n", tle.angle - offset, tle.velocity, (uint16_t) (tle.prev_time / 1000));
    // Serial.write(reinterpret_cast<char*>(&m), sizeof(m));
    // Serial.print("\n\n");
    // // delay(5);
    delayMicroseconds(2500);
}
