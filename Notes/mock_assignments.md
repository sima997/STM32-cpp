# Assignments

## Assignment 1 : Battery State Machine

"Write C++ code (C++11) for an RTOS-based system controlling a battery. The system should:"
- Read a simulated ADC value for battery voltage every 100 ms.
- If voltage < 11.5 V → set state = LOW,
- If voltage > 14.5 V → set state = HIGH,
- Otherwise state = NORMAL.
- Every 1 second, send a status message "V=xx.x, State=LOW/NORMAL/HIGH".

This tests:
- Periodic tasks, timing.
- State machine logic.
- Structuring code cleanly.
- Basic communication formatting.

```C++

#include <iostream>
#include <string>
#include <chrono>
#include <thread>

enum class BatteryState {LOW, NOMAL, HIGH};

float read_adc_simulated() {
    static float v = 12.0; //Static only for simulation
    v += 0.05f; //Slowly increasing voltage
    if (v > 15.5f) v = 11.0f; //reset
    return v;
}

BatteryState get_state(float voltage) {
    if(voltage < 11.5f) return BatteryState::LOW;
    if(voltage > 14.5f) return BatteryState::HIGH;
    return BatteryState::NORMAL;
}

std::string to_string(BatteryState s) {
    switch(s) {
        case BatteryState::LOW: return "LOW";
        case BatteryState::NORMAL return "NORMAL";
        case BatteryState::HIGH: return "HIGH";
    }
    return "UNKNOWN";
}

int main(void) {
    BatteryState state = BatteryState::Normal;
    int counter = 0;

    while(1) {
        float v = read_adc_simulated();
        state = get_state();

        if (counter % 10 == 0) { //every 1 second (10x100ms)
            std::cout << "Voltage=" << V
                    << "V, State =" << to_string(state)
                    <<std::endl;
        }

        std::this_thread::sleep_for(std::chrono::miliseconds(100));
        counter++;
    }
    return 0;
}
```

## Assignment 2 : Communication Frame
Implement a simple protocol frame:
- Format: [HEADER][PAYLOAD][CHECKSUM]
- HEADER = 0xAA
- PAYLOAD = 2 bytes (16-bit value)
- CHECKSUM = sum of all bytes mod 256

Write functions:
1. `pack_frame(uint16_t value, uint8_t* buffer)`
2. `bool unpack_frame(uint8_t* buffer, uint16_t& value)`

```C++
cpp

#include <iostream>
#include <cstdint>

const uint8_t HEADER = 0xAA;

uint8_t checksum(const uint8_t *data, size_t len) {
    uint16_t sum = 0;
    for(size_t i = 0; i<len; i++) sum += data[i];
    return sum % 0xFF;
}

void pack_frame(uint16_t value, uint8_t* buffer) {
    buffer[0] = HEADER;
    buffer[1] = value & 0xFF;
    buffer[2] = (value >> 8) & 0xFF;
    buffer[3] = checksum(buffer, 3);
}

bool unpack_frame(uint8_t* buffer, uint16_t& value) { //uint16_t& value is a reference to a variable value. No need to dereference like a pointer
    if(buffer[0] != HEADER) return false;
    if(checksum(buffer,3) != buffer[3]) return false;
    value = buffer[1] | (buffer[2] << 8);
    return true;
}

int main(void) {
    uint8_t frame[4];
    pack_frame(0x2568, frame);

    uint16_t parsed;
    if( unpack_frame(frame, parsed)) {
        std::cout << "Parsed value = " << parsed << std::endl;
    }else {
        std::cout << "Frame error" << std::endl;
    }
    
    return 0;
}
```


## Assignment 3 : RTOS-Style Task Scheduling (Simulated)
**Task:**
Simulate a system with 2 tasks:
- Task 1: reads sensor every 200 ms.
- Task 2: sends status every 1 second.

```C++
#include <iostream>
#include <chrono>
#include <thread>

void task_sensor() {
    static int value = 0;
    value++;
    std::cout << "[Sensor] Value=" << value << std::endl;
}

void task_status() {
    std::cout << "[Status] System running OK" << std::endl;
}

int main() {
    int tick = 0;
    while(true) {
        if (tick % 2 == 0) task_sesor(); //every 200ms
        if (tick % 10 == 0) task_status(); //every 1s

        std::this_thread::sleep_for(std::chrono::miliseconds(100));
        tick++;
    }
    return 0;
}