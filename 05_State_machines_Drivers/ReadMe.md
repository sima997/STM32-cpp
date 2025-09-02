# State machines & Drivers
## Goals
- Learn how to design structured logic for embedded systems using **C++ state machines, OOP patterns**, and **drivers**. You will wrap a sensor into a finite state machine (FSM) and control a LED to visualize states.

## Theory 
### 1. State design pattern
- Concept: Each "state" is an object that handles behaviour for that state
- Benefits:
  - Clear separation of behavior per state
  - Easy to extend with new states without modifying eqisting logic
  - Decouples the context (main FSM) from state-specific behavior
  
**C++ example:**
```cpp
class State {
public:
  virtual void handle() = 0;
  virtual ~State() = default
};

class InitState : public State {
public:
    void handle() override {
      //Initialize sensor
    }

};

class ReadState : public State {
public:
    void handle() override {
      //Read sensor
    }
}

class ProcessState : public State {
public:
    void handle() override {
      //Process data
    }
};
```

### 2. Enum class + Switch vs. OOP
- Simple FSM: `enum class SensorState { INIT, READ, PROCESS, IDLE }`
- Switch-based FSM is lightweight but less scalable
- OOP states are more modular and maintainable

```cpp
enum class SensorState { INIT, READ, PROCESS, IDLE };

void fsmLoop(SensorState &state) {
    switch(state) {
      case SensorState::INIT: 
        initSensor(); 
        state = SensorState::READ; 
        break;
      case SensorState::READ:
        readSensor();
        state = SensorState::PROCESS;
        break;
      case SensorState::PROCESS:
        processData();
        state = SensorState::IDLE;
        break;
      case SensorState::IDLE:
        idleBehavior();
        state = SensorSensor::READ;
        break;
    }
}
```

### 3. Finite State Machines for sensors/communication
- FSM is a **core pattern in embedded systems:**
  - Makes scheduling deterministic
  - Avoids spaghetti code
  - Can handle timing, errors, and retries elegantly
- Use timers or tick counters to switch states periodically.

## Practice Project – I2C/SPI sensor driver
### Step 1: Wrap a sensor in a driver
- Interface your sensor (I2C/SPI) into a **driver class**
- Implement concrete sensor class:
### Step 2: Define FSM states
- Define enum or class-based states (normal and OOP)

### Step 3: Implement FSM
- Switch-based version and OOP version

### Step 4: Visualize with LED
- Each FSM state corresponds to a **different LED pattern:**
  - `INIT` -> LED on 500ms
  - `READ` -> LED blink 2x
  - `PROCESS` -> LED fast blink
  - `IDLE` -> LED off or slow blink
- Use your GPIO template from previous days to toggle LEDs

### Step 5. Optional Enhancements
- Add **error handling states** if sensor fails.
- Use **timer interrupts** to trigger FSM updates instead of busy loops
- Make FSM **non-blocking**