#ifndef TFLUNA_LIDAR_HPP
#define TFLUNA_LIDAR_HPP

#include <Arduino.h>
class TFLuna {
    private:
        HardwareSerial& serial;
        
    public:
        TFLuna(HardwareSerial& serialPort, int rxPin, int txPin, int baudrate = 115200) : serial(serialPort) {
            serial.begin(baudrate, SERIAL_8N1, rxPin, txPin);
            delay(100);
        }
        
        bool readData(uint16_t& distance, uint16_t& strength, float& temperature) {
            while (serial.available()) {
                serial.read();
            }
            
            int count = 0;
            unsigned long startTime = millis();
            
            while (count < 2) {
                if (millis() - startTime > 1000) {
                    return false;
                }
                
                if (serial.available()) {
                    uint8_t c = serial.read();
                    if (count == 0 && c == 0x59) {
                        count = 1;
                    } else if (count == 1 && c == 0x59) {
                        count = 2;
                    } else {
                        count = 0;
                    }
                }
                yield();
            }
            
            startTime = millis();
            while (serial.available() < 6) {
                if (millis() - startTime > 100) {
                    return false;
                }
                yield();
            }
            
            uint8_t frame[6];
            serial.readBytes(frame, 6);
    
            uint8_t dist_l = frame[0];
            uint8_t dist_h = frame[1];
            uint8_t strength_l = frame[2];
            uint8_t strength_h = frame[3];
            uint8_t temp_l = frame[4];
            uint8_t temp_h = frame[5];
            
            distance = (dist_h << 8) | dist_l;
            strength = (strength_h << 8) | strength_l;
            temperature = ((temp_h << 8) | temp_l) / 8.0 - 256.0;
            
            return true;
        }
    };
#endif