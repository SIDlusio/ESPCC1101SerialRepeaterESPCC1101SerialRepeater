#include "ELECHOUSE_CC1101_SRC_DRV.h"
#include <WiFi.h>

#if defined(ESP8266)
    #define RECEIVE_ATTR ICACHE_RAM_ATTR
#elif defined(ESP32)
    #define RECEIVE_ATTR IRAM_ATTR
#else
    #define RECEIVE_ATTR
#endif

#define OWL 2000 // MAXRATE

// Initial default values OOK/ASK
float freq = 433.92;
int mod = 2; // modulation 
float rxbw = 58; // bandwidth 
float dRate = 5; // data rate 
float dev = 0; // deviation 

int rxPin = 4; // RX Pin (CC1101:8)
int txPin = 2; // TX Pin (CC1101:3)

int sCount; 
unsigned long samples[OWL]; 
int eTol = 200; 
unsigned long sSmooth[OWL]; 
unsigned long sSecondSmooth[OWL]; 
long tPush[OWL];
static unsigned long lTime = 0; // 
const int minSample = 10;
int sCountSmooth = 0; 
int sSecondCountSmooth = 0; 

bool rxConfig = false; // RX configuration

bool checkReceived(void) {
    delay(100);
    if (sCount >= minSample && micros() - lTime > 100000) {
        detachInterrupt(rxPin);
        return true;
    }
    return false;
}

void RECEIVE_ATTR rxHandler() {
    const long time = micros();
    const unsigned int duration = time - lTime;

    if (duration > 100000) {
        sCount = 0;
    }

    if (duration >= 100) {
        samples[sCount++] = duration;
    }

    if (sCount >= OWL) {
        detachInterrupt(rxPin);
        checkReceived();
    }
    if (sCount == 1 && digitalRead(rxPin) != HIGH) {
        sCount = 0;
    }

    lTime = time;
}

void analyzeSignal() {
    #define RABBIT 10

    int signalDelay[RABBIT] = {0};
    long signalTimings[RABBIT * 2]; 
    int signalTimingsCount[RABBIT] = {0}; 
    long signalTimingsSum[RABBIT] = {0};

    for (int i = 0; i < RABBIT; i++) {
        signalTimings[i * 2] = 100000;
        signalTimings[i * 2 + 1] = 0;
    }

    for (int p = 0; p < RABBIT; p++) {
        for (int i = 1; i < sCount; i++) {
            if (samples[i] < signalTimings[p * 2] && (p == 0 || samples[i] > signalTimings[(p - 1) * 2 + 1])) {
                signalTimings[p * 2] = samples[i];
            }
        }

        for (int i = 1; i < sCount; i++) {
            if (samples[i] >= signalTimings[p * 2] && samples[i] <= signalTimings[p * 2] + eTol) {
                signalTimings[p * 2 + 1] = samples[i];
                signalTimingsCount[p]++;
                signalTimingsSum[p] += samples[i];
            }
        }
    }

    int sAnalyzed = RABBIT;
    for (int i = 0; i < RABBIT; i++) {
        if (signalTimingsCount[i] == 0) {
            sAnalyzed = i;
            break;
        }
    }

    // Manual sorting
    for (int s = 1; s < sAnalyzed; s++) {
        for (int i = 0; i < sAnalyzed - s; i++) {
            if (signalTimingsCount[i] < signalTimingsCount[i + 1]) {
                // Swap all related elements
                std::swap(signalTimings[i * 2], signalTimings[(i + 1) * 2]);
                std::swap(signalTimings[i * 2 + 1], signalTimings[(i + 1) * 2 + 1]);
                std::swap(signalTimingsSum[i], signalTimingsSum[i + 1]);
                std::swap(signalTimingsCount[i], signalTimingsCount[i + 1]);
            }
        }
    }

    for (int i = 0; i < sAnalyzed; i++) {
        signalDelay[i] = signalTimingsSum[i] / signalTimingsCount[i];
    }

    if (samples[1] == signalTimings[0] && samples[1] < signalDelay[0]) {
        samples[1] = signalDelay[0];
    }

    bool lastBin = 0;
    for (int i = 1; i < sCount; i++) {
        int calc = std::round((float)samples[i] / signalDelay[0]);
        if (calc > 0) {
            lastBin = !lastBin;
            if (lastBin == 0 && calc > 8) {
                Serial.printf(" [Pause: %lu samples]", samples[i]);
            } else {
                for (int b = 0; b < calc; b++) {
                    Serial.print(lastBin);
                }
            }
        }
    }
    Serial.println();
    Serial.printf("Samples/Symbol: %d\n\n", signalDelay[0]);

    sCountSmooth = 0;
    for (int i = 1; i < sCount; i++) {
        int calc = std::round((float)samples[i] / signalDelay[0]);
        if (calc > 0) {
            sSmooth[sCountSmooth++] = calc * signalDelay[0];
        }
    }

    Serial.println("Rawdata corrected:");
    Serial.printf("Count=%d\n", sCountSmooth + 1);
    for (int i = 0; i < sCountSmooth; i++) {
        Serial.printf("%lu,", sSmooth[i]);
        tPush[i] = sSmooth[i];
    }
    Serial.println("\n");
}

void enableReceive() {
    pinMode(rxPin, INPUT);
    rxPin = digitalPinToInterrupt(rxPin);
    ELECHOUSE_cc1101.SetRx();
    sCount = 0;
    attachInterrupt(rxPin, rxHandler, CHANGE);
    sCount = 0;
}

void printReceived() {
    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.setModul(1);
    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setSyncMode(0);
    ELECHOUSE_cc1101.setPktFormat(3);
    ELECHOUSE_cc1101.setModulation(mod);
    ELECHOUSE_cc1101.setRxBW(rxbw);
    ELECHOUSE_cc1101.setMHZ(freq);
    ELECHOUSE_cc1101.setDeviation(dev);
    ELECHOUSE_cc1101.setDRate(dRate);

    if (mod == 2) {
        ELECHOUSE_cc1101.setDcFilterOff(0);
    }

    if (mod == 0) {
        ELECHOUSE_cc1101.setDcFilterOff(1);
    }

    Serial.print("Count=");
    Serial.println(sCount);

    for (int i = 1; i < sCount; i++) {
        Serial.print(samples[i]);
        Serial.print(",");
    }
    Serial.println();
    Serial.println();
}

void setup() {
    Serial.begin(38400);
    WiFi.mode(WIFI_STA);
    Serial.print(",");
    ELECHOUSE_cc1101.addSpiPin(14, 12, 13, 5, 1); // GraDient Project V0.1 Pins
}

void applyConfig() {
    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.setModul(1);
    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setSyncMode(0);
    ELECHOUSE_cc1101.setPktFormat(3);
    ELECHOUSE_cc1101.setModulation(mod);
    ELECHOUSE_cc1101.setRxBW(rxbw);
    ELECHOUSE_cc1101.setMHZ(freq);
    ELECHOUSE_cc1101.setDeviation(dev);
    ELECHOUSE_cc1101.setDRate(dRate);

    if (mod == 2) {
        ELECHOUSE_cc1101.setDcFilterOff(0);
    }

    if (mod == 0) {
        ELECHOUSE_cc1101.setDcFilterOff(1);
    }
    enableReceive();
}

void replaySignal() {
    ELECHOUSE_cc1101.setSidle();

    pinMode(txPin, OUTPUT);
    ELECHOUSE_cc1101.setModul(1);
    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setModulation(mod);
    ELECHOUSE_cc1101.setMHZ(freq);
    ELECHOUSE_cc1101.setDeviation(dev);
    ELECHOUSE_cc1101.SetTx();
    delay(100);

    for (int i = 0; i < 2000; i += 2) {
        digitalWrite(txPin, HIGH);
        delayMicroseconds(sSmooth[i]);
        digitalWrite(txPin, LOW);
        delayMicroseconds(sSmooth[i + 1]);
    }
    delay(1000);
    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.setModul(1);
    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setSyncMode(0);
    ELECHOUSE_cc1101.setPktFormat(3);
    ELECHOUSE_cc1101.setModulation(mod);
    ELECHOUSE_cc1101.setRxBW(rxbw);
    ELECHOUSE_cc1101.setMHZ(freq);
    ELECHOUSE_cc1101.setDeviation(dev);
    ELECHOUSE_cc1101.setDRate(dRate);

    if (mod == 2) {
        ELECHOUSE_cc1101.setDcFilterOff(0);
    }

    if (mod == 0) {
        ELECHOUSE_cc1101.setDcFilterOff(1);
    }
    enableReceive();
}

void loop() {
    if (checkReceived()) {
        Serial.print("Running");
        printReceived();
        analyzeSignal();
        enableReceive();
    }

    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command.startsWith("/set")) {
            sscanf(command.c_str(), "/set freq %f rxbw %f datarate %f deviation %f", &freq, &rxbw, &dRate, &dev);
            rxConfig = true;
            applyConfig();
            Serial.println("Configuration set.");
        } else if (command == "/replay") {
            if (rxConfig) {
                replaySignal();
            } else {
                Serial.println("/set freq xxx.xx rxbw xx.x datarate xx deviation xxx FIRST");
            }
        }
    }
}
