# Important C/C++ algorithms for embedded systems

## Low-leve/Bitwise Algorithms
### 1. Bit masking & shifting
- **Why:** Hardware registers (GPIO, timers, Utart, etc.) are usually memory-mapped. Each bit has a meaning (enable, disable, flag, etc.)
- **Example (C)**
    ```C
    #define LED_PIN     (1<<5)      //Bit 5
    PORTA |= LED_PIN;               //Set bit 5
    PORTA &= ~LED_PIN;              //Clear bit 5
    PORTA ^= LED_PIN;               //Toggles bit 5
    bool on = (PORTA & LED_PIN);    //Test if bit 5 is set
    ```
- **Tip:** Using shifts (<<,>>) and masks avoids expensive multiplications/divisions on small MCUs.

### 2. Lookup tables
- **Why:** Computing things like `sin()`, `exp()` or even `sqrt()` can be very slow without FPU. Precomputing results in an array and doing a table lookup is much faster.
- **Example (sine wave DAC)**
    ```C
    const uint8_t sine_table[256] = {/*Precomputed values*/};

    uint8_t val = sine_table[phase++]; //Fast waveform generation
    DAC_OUTPUT = val;
    ```
- **Tradeoff:** Faster runtime at the cost of Flash/RAM usage

### 3. CRC (Cyclic Redundancy Check)
- **Why:** Ensure data integrity in communication (UART, CAN, Ethernet) or storage (Flash, EEPROM). More robust than simple checksums
- **Common:** CRC8 (small packets), CRC16 (Modbus, USB), CRC32 (Ethernet)
- **Example (CRC-16, simple implementation):**
    ```C
    uint16_t crc16(const uint8_t *data, size_t len) {
      uint16_t crc = 0xFFFF;  //CRC starts with seed value (Modbus = 0xFFFF)
      const uint16_t polynomial = 0xA001;
      for(size_t i = 0; i < len; i++) {
          crc ^= data[i];
          for(int j = 0; j < 8; j++) {   //Process each bit
              crc = (crc & 1) ? (crc >> 1) ^ polynomial : (crc >> 1);
          }
      }
      return crc;
  }
  ```


### 4. Checksums
- **Why** Lightweight error detection when CRC is "too heavy." Common in bootloaders, small protocols, or log records
- **Types:** 
  - **XOR checksum:** Fats, but weak
  - **Fletcher's checksum:** Stronger, still efficient
  - **Alder-32:** A compromise between Fletcher and CRC
- **Example (XOR checksum)**
    ```C
    uint8_t checksum_xor(const uint8_t *data, size_t len) {
        uint8_t chk = 0;
        for(size_t i = 0; i < len; i++) chk ^= data[i];
        return chk;
    }
    ```


## Timing & scheduling
### 1. Software timers (non-blocking delays)
- **Problem:** `delay_ms(1000)` wastes CPU cycles, blocking everything else.
- **Solution:** Use a **tick counter** (from SysTick, hardware timer or periodic interrupt) to implement non-blocking delays
- **Example (C,pseudo)**
    ```C
    volatile uint32_t sys_ticks = 0;

    void SysTick_Handler(void) { //Fires every 1ms
        sys_ticks++;
    }

    bool timer_expired(uint32_t start, uint32_t period_ms) {
        return (sys_ticks - srart) >= period_ms;
    }

    int main(void) {
        uint32_t led_timer = sys_ticks;
        while(1) {
            if(timer_expired(led_timer, 500)) {
                toggle_led();
                led_timer = sys_ticks;
            }
            //Other tasks run here ...
        }
    }
    ```

### 2. Task scheduling (without RTOS)
If you don't have FreeRTOS/Zephyt/etc., you can still "schedule" tasks in bare metal C
- **Round-robin scheduling:**
  - Just call each task in sequence, like:
    ```C
    while(1) {
        task1();
        task2();
        task3();
    }
    ```
  - *Pros:* Simple, predictive; *Cons:* Tasks must return quickly, no long blocking
- **Cooperative scheduling:**
  - Each task decides when to yield
    ```C
    while(1) {
        if (task1_ready()) task1();
        if (task2_ready()) task2();
    }
    ```
  - *Pros:* Lets you run  only what's needed; *Cons:* Misbehaving tasks can starve others
- **Preemptive scheduling (basic)**
  - Use a timer interrupt to "slice" time and force task switches (like a tiny RTOS).
  - *Pros:* More responsive; *Cons:* Much more complex ( context saving **needed**)

### 3. Real-time scheduling theory
Usen when tasks have strict timing requirements
- **Rate Monotonic Scheduling (RMS):**
  - Static priorities: shorter period = higher priority
  - Example: a task that runs every 10ms gets higher priority than one running every 100ms
  - Optimal for fixed-priority scheduling, but only works if CPU utilization <~69%
- **Earliest Deadline First (EDF):**
  - Dynamic priorities: the task with the closes deadline runs first
  - More efficient (can reach 100% utilization)
  - Hard to implement on bare-metal system (usually in RTOS)

## Signal Processing/Data Handling
### 1. Finite Impulse Response (FIR) Filter
- **What it is:** A digital filter where the output depends on a finite number of past input samples
- **Formula:**
    $$
    y[n] = \sum_{k=0}^M b_k \cdot x[n-k]
    $$
- **Embedded use:** Smoothing sensor signal (temperature, accelerometers, audio)
- *Pros:* Always stable, linear phase; *Cons:* Can be heavy (many multiplications)

**C example (FIR with 4 coefficients)**
```C
#define TAPS    4
float coeffs[TAPS] = {0.25, 0.25, 0.25, 0.25}; //Simple averaging FIR
float buffer[TAPS] = {};
int idx = 0;

float fir_filter(float input) {
    buffer[idx] = input
    float output = 0;
    int j = idx;
    for(int i = 0; i < TAPS; i++) {
        output += coeffs[i] * buffer[j];
        j = (j - 1 + TAPS) % TAPS;
    }
    idx = (idx + 1) % TAPS;
    return output;
}
```

### 2. Infinite Impulse Response (IIR) Filter
- **What it is:** Uses both current and past outputs
- **Formula (1st-order low-pass):**
    $$
    y[n] = \alpha \cdot x[n] + (1 - \alpha) \cdot y[n-1]
    $$
- **Embedded use:** Cheaper alternative to FIR, used in smoothing noisy sensor
- *Pros:* Efficient, fewer coefficients; *Cons:* Can become unstable if not tuned carefully

**C example (simple IIR low-pass filter):**
```C
float iir_lowpass(float input, float prev_output, float alpha) {
    return alpha * input + (1-alpha)*prev_output;
}
```
### 3. Moving Average Filter
- **What it is:** Simplest FIR of last N samples
- **Embedded use:** Noisy reduction for ADC values, sensors (e.g., temperature, battery voltage)
- *Pros:* Super easy to implement, no multiplications; *Cons:* Poor frequency response compared to proper FIR

**C example (moving average over 8 samples)**
```C
#define N   8
uint16_t buffer[N] = {0}
int pos = 0;
uint32_t sum = 0;

uint16_t moving_average(uint16_t input) {
    sum -= buffer[pos];
    buffer[pos] = input;
    sum += input;
    pos = (pos + 1) % N;
    return (uint16_t)(sum / N);
}
```

### 4. PID Algorithm
- **What is is:** Control algorithm that combines **Proportional, Integral and Derivative** terms
- **Formula:**
    $$
    u[t] = K_p e(t) + K_i \int e(t) dt + K_d \frac{de(t)}{dt}
    $$
- **Embedded use:** Motor speed control, temperature regulation, robotics
- *Pros:* Well understood, works in many control loops; *Cons:* Needs tuning (Kp,Ki,Kd)

**C example (discrete PID)**
```C
typedef struct {
    float Kp, Ki, Kd;
    float prev_error;
    float integral;
} PID_t;

float pid_compute(PID_t *pid, float setpoint, float measured, float dt) {
    float error = setpoint - measured;
    pid->integral += error * dt;
    float derivative = (error - pid->prev_error) / dt;
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    pid->prev_error = error;
    return output;
}
```

### 5. CORDIC Algorithm
- **What it is:** Iterative algorithm for computing **trigonometric functions (sin, cos, atan2)**
- **Embedded use:** When MCU has no FPU (common in Cortex-M0, AVR)
- *Pros:* No floating point needed, predictable; *Cons:* Slower accuracy growth vs lookup tables or hardware FPU

**Typical use cases**
- Angle calculation from accelerometer/gyro (atan2)
- Fast trigonometry for motor control (field-oriented control, space vector PWM)

## Search & Sorting 
### 1. Binary Search
- **When:** You have a sorted array (e.g., calibration tables, lookup values).
- **Why:** Reduces search time from O(n) -> O(log n)
- **Embedded use cases:**
  - Searching a **calibration table** (e.g., voltage -> temperature)
  - Command lookups in a sorted string array
  - Range searching in LUTs

**C example:**
```C
int binary_search(const int arr[], int size, int key) {
    int low = 0, high = size - 1;
    while(low <= high) {
        int mid = (low + high) / 2;
        if (arr[mid] == key) return mid; //found
        else if(arr[mid] < key) low = mid + 1;
        else high = mid - 1
    }
    return -1; //not found
}
```

### 2. Insertion Sort
- **When:** Sorting **small arrays** (like < 50 elements)
- **Why:** Super simple, predictable, no recursion, no dynamic memory
- **Embedded use cases:**
  - Sorting sensor readings for **median filters**
  - Maintaining small ordered lists (like a few active timers)
  - Debugg logging (sorf entities before transmission)

**C example:**
```C
void insertion_sort(int arr[], int n) {
    for(int i = 1; i < n; i++) {
        int key = arr[i];
        int j = i - 1;
        while(j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
}
```
### 3. Heap/Priority Queue
- **When:** You need to always fetch the **smallest/largest element quickly**
- **Why:** Insert O(log n), remove-min O(log n)
- **Embedded use cases**
  - **Task scheduling** (e.g., always run the task with the nearest deadline)
  - **Event queues** (next UART timeout, next sensor read)
  - Buffers where priority matters (e.g., CAN messages)

**Minimal C heap (max-heap for ints)**
```C
#define MAX_HEAP    32
int heap[MAX_HEAP];
int heap_size = 0;

void heap_insert(int val) {
    int i = heap_size++;
    heap[i] = val;
    while(i > 0 && heap[(i-1)/2] < heap(i)) {
        int tmp = heap[i];
        heap[i] = heap[(i-1)/2];
        heap[(i-1)/2] = tmp;
        i = (i-1)/2;
    }
}

int heap_extract_max(void) {
    int root = heap[0];
    heap[0] = heap[--heap_size];
    int i = 0;
    while(1) {
        int left = 2*i + 1;
        int right = 2*i +2,
        int largest = i;
        if(left < heap_size && heap[left] > heap[largest]) largest = left;
        if(right < heap_size && heap[right] > heap[largest]) largest = right;
        if(largest = i) break;
        int tmp = heap[i];
        heap[i] = heap[largest];
        heap[largest] = tmp;
        i = largest; 
    }
    return root;
}
```
## Compression & Encoding
### 1. Run-Length Encoding (RLE)
- **Idea:** Replace repeated values with count + value
  
  Example 
  ```
  Raw:      AAAABBBCCDAA
  RLE:      4A3B2C1D2A
  ```
- **Why good for embedded**
  - Extremely simple (just counters and values)
  - Works well on data with lots of repetetion (sensor logs, images like bitmap)
  - Very small code size
- **Why not:**
  - Inefficient for "noisy" data (can even expand instead of compress)

**C sketch:**
```C
size_t rle_encode(const uint8_t *in, size_t len, uint8_t *out) {
    size_t out_len = 0;
    for (size_t i = 0; i < len;) {
        uint8_t val = in[i];
        size_t run = 1;
        while(i+run < len && in[i + run] == val && run < 255) run++;
        out[out_len++] = run;
        out[out_len++] = val;
        i += run
        
    }
    return out_len;
}
```

### 2. Huffman Coding
- **Idea:** Assign shorter bit-codes to frequent values, longer codes to rare values.
- **Why good:**
  - Very efficient compression, especially on predictable data (text, sensor ranges)
- **Why tricky in embedded:**
  - Needs a frequent table and bit-level packing -> more RAM/Flash overhead
  - Often better to use **static Huffman tables** (precomputed) instead of generating on the device
- **Use cases:**
  - Firmware update compression
  - Telemetry where bandwidth is expensive

### 3. Base64/Hex Encoding
- **Not compression, but encoding** (makes binary data text-friendly)
- **Hex (Base 16):**
  - 1 byte -> 2 ASCII chars.
  - Easy to debug, but double size
- **Base64:**
  - 3 bytes -> 4 ASCII chars
  - Expands data by ~33%, but much denser than hex
- **Why used in embedded:**
  - Sending binary data over text-only channels (UART, HTTP, MQTT, JSON)
  - Debug logs with binary payload
- **Tradeoff:** Not space-efficient, but universally supported.

**Tiny hex encode (C)**
```C
void hex_encode(const uint8_t *in, size_t len) {
    const char hex[] = "0123456789ABCDEF";
    for(size_t i = 0; i < len; i++) {
        out[2*i] = hex[in[i] >> 4];
        out[2*i + 1] = hex[in[i] & 0x0F];
    }
    out[2*len] = '\0';
}
```

## Numerical Methods
### 1. Fixed-point arithmetics
- **Why:** Many small MCUs (AVR, Coretex-M0/M3, PIC) either don't have floating-point hardware or have very slow FP operations. Fixed point math uses integers to represent fractions
- **How:**
  - Pick a scale (e.g., 1000 ->three decimal places)
  - Store real number as integers (int16_t, int32_t)
  - Do math using integers, then rescale

**Example:**
```C
#define SCALE   1000        //3 decimal places

int32_t a = 1234;           //1.234
int32_t b = 2500;           //2.500

int32_t mul = (a * b) / SCALE;  //~3.085
int32_t div = (a * SCALE) / b;  //~0.494
```

### 2. Integer square root algorithms
- **Why:** Functions like `sqrtf()` are heavy on MCUs without FPU. Integer-only square root is much faster if you don't need floating-point accuracy.
- **Common algorithms:**
  - **Binary search** (simple, O(log n))
  - **Newton-Raphson** iteration (fast convergence)
  - **Bit-shift method** (classic "integer sqrt" implementation)

**Example (fast integer sqrt):**
```C
uint16_t isqrt(uint32_t n) {
    uint32_t x = n;
    uint32_t res = 0;
    uint32_t bit = 1UL << 30; //Highest power of 4 <= 2^32

    while(bit > x) bit >>= 2;

    while(bit != 0) {
        if(x >= res + bit) {
            x -= res + bit
            res = (res >> 1) + bit;
        }else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return (uint16_t)res;
}
```

### 3. Kalman filter
- **What:** An optimal estimator that fuses multiple noisy sensor inputs into one "best guess" of system state
- **Why:** Sensors are noisy (accelerometers drift, gyros bias, GPS jitter). Kalman filtering combines them inteligently
- **How it works (high level):**
  1. Predict state using a model
  2. Update with measurement
  3. Weigh prediction vs measurement using covarience (noise)
- **Embedded example use cases:**
  - IMUs: fuse accelerometer + gyroscope -> stable operation
  - Sensor fusion: GPS + wheel encoder -> better speed/position
  - Robotics: tracking state (x, y, velocity)
- **Tradeoffs:**
  - Very accurate but math-heavy (matrices, floating-point)
  - On small MCUs, people often use **simplified Kalman filters** or **complementary filters** instead

## Communication & Protocol Support
### 1. Ring Buffer/Circular Buffer
- **Why:** In UART, SPI, I2C, CAN, etc.,data comes in **asynchronously**. You can't always process it immediately - so you need a buffer to store bytes until CPU has time.
- **How it works:** A fixed-size array + two pointers (`head`, `tail`)
  - Producer (ISR) writes into `head`
  - Consumer (main loop/task) reads from `tail`
  - Wrap around at the end -> "circular"

**C example (byte ring buffer")**
```C
#define BUF_SIZE    64

typedef struct {
    uint8_t buf[BUF_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} ringbuf_t;

void rb_init(ringbuf_t *rb) {
    rb->head = rb->tail = 0;
}

//Empty: head == tail → nothing between them.
bool rb_is_empty(ringbuf_t *rb) {
    return rb->head == rb->tail;
}

//Full: advancing head would land on tail → no room left.
bool rb_full(ringbuf_t *rb) {
    return ((rb->head +1) % BUF_SIZE == rb->tail);
}

bool rb_put(ringbuf_t *rb, uint8_t data) {
    if(rb_is__full(rb)) return false;
    rb->buf[rb->head] = data;
    rb->head = (rb->head + 1) % BUF_SIZE;
    return true;
}

bool rb_get(ringbuf_t *rb, uint8_t *data) {
    if(rb_full(rb)) return false;
    *data = rb->buf[rb->tail];
    rb->tail = (rb->tail + 1) % BUF_SIZE;
    return true;
}
```

- **Use case:** UART RX ISR calls `rb_put()`, main loop calls `rb_get()` to process data safely 

### 2. Finite State Machines (FSMs)
- **Why:** Communication protocols (UART, Modbus, CAN, custom serial) often need parsing **byte-by-byte**. FSM makes parsing clean, predictable and easy to extend
- **How it works:** Define states (IDLE, HEADER, PAYLOAD, CHECKSUM, ...). Each byte moves you between states

**Example: simple packet parser FSM**
```C
typedef enum { ST_IDLE, ST_HEADER, ST_LEN, ST_PAYLOAD, ST_CHECKSUM} state_t;

state_t state = ST_IDLE;
uint8_t buffer[64];
uint8_t index = 0, length = 0, checksum = 0;

void fsm_parse_byte(uint8_t b) {
    switch(state) {
        case ST_IDLE:
            if(b == 0xAA) { //Sync byte
                state = ST_HEADER;
                checksum = b;
            }
            break;
        case ST_HEADER:
            length = b;
            checksum ^= b;
            index = 0;
            state = ST_PAYLOAD;
            break;
        case ST_PAYLOAD:
            buffer[index++] = b;
            checksum ^= b;
            if(index >= length) state = ST_CHECKSUM;
            break;
        case ST_CHECKSUM:
            if(checksum == b) {
                //Valid packet
                handle_packet(buffer, length);
            }
            state = ST_IDLE;
            break;
    }
}
```

- **Use case:** Implementing your own UART protocol, parsing Modbus, framing CAN payloads

### 3. Parsing Algorithms (framing, checksums, escaping)
When you move bytes between devices, you need rules for:
1. **Framing** -> How to know where a packet starts/ends
   - Common methods:
     - Start + length + checksum (like above)
     - Special delimiter bytes (`0x7E` in HDLC,SLIP)
2. **Checksums/CRC** -> Detect errors
   - Fletcher, Adler, CRC16/32
3. **Escaping** -> What if your delimiter appears in the data?
   - Use **byte-stuffing:** replace `0x7E` with `0x7D 0x5E`
   - Example: SLIP, PPP protocols

**Example: byte-stuffing (escaping `0x7E`)**
```C
#define FRAME_END   0x7E
#define FRAME_ESC   0x7D
#define FRAME_XOR   0x20

size_t stuff_bytes(const uint8_t *in, size_t len, uint8_t *out) {
    size_t j = 0;
    for(size_t i = 0; i < len; i++) {
        if(in[i] == FRAME_END || in[i] == FRAME_ESC) {
            out[j++] = FRAME_ESC;
            out[j++] = in[i] ^ FRAME_XOR;
        }else {
            out[j++] = in[i];
        }
    }
    out[j++] = FRAME_END; //terminator
    return j;
}
```
- **Use case:** If your protocol uses `0x7E` as "end of frame, you must escape it if it appears in payload


## Memory & Storage
### 1. Flash Wear Leveling Algorithms
- **The problem:** 
  - Flash and EEPROM cells can only be erased/written a limited number of times (r.g., **10k-100k cycles pre sector**)
  - If you always write to the same address, it will wear out much faster than the rest of memory
- **The solution -> Wear Leveling**
  
    Spread writes across different blocks/pages to maximize device lifetime
    - **Dynamic wear leveling:** Always write to a new location, keep a mapping (like mini file system)
    - **Static wear leveling:** Occasionally move rarely-updated data too, so all sectors wear evenly
- **Typical Implementation:**
  - Maintain a **log-structured journal** in Flash
  - Each new write is appended to a new sector, and metadata tracks the "last valid copy"
  - When Flash fills up, perform **garbage collection** to reclaim space

Example: Filesystems like **LittleFS**, and **FAT with FTL (Flash Transition Layer)** use this


### 2. Memory Allocation Strategies
On bare-metal, `malloc()` and `free()` are often avoided (fragmentation + non-deterministic timing). Instead, embedded uses **specialized allocators:**
1. **Fixed Block Allocator**
   - Pre-allocate memory into fixed-size blocks
   - Each allocation grabs a block, each free returns it
   - **Deterministic**, no fragmentation, very fast
   - Used in RTOSes (e.g., FreeRTOS `pvPortMalloc()` variants)
2. **Buddy Allocator:**
   - Memory is split into powers-of-two blocks
   - If a block is freed, it may "merge" with its buddy into a larger block
   - Good balance between flexibility and fragmentation
3. **No allocator (static allocation only)**
   - Allocate everything at compile time
   - Common in safety critical/low RAM systems

**Rule of thumb:**
- Small MCUs (<64KB RAM) -> avoid malloc, use static or fixed-block allocators
- Bigger embedded Linux systems -> buddy allocator or slab allocator (like the kernel)

### 3. EEPROM Emulation in FLash
- **The Problem:**
  - Many MCUs (like STM32, NXP, ESP32) have Flashm but **no true EEPROM**
  - Flash writes can only change bits form `1->0`. To reset to `1`, you must erase and entire page (slow, limited cycles)
- **The solution -> Emualte EEPROM**

    Implement a small software layer that makes Flash behave like EEPROM
  -  Reeserve one or more Flash pages
  -  Sypre variables in a **log structure** instead of rewriting in place
  -  Use wear leveling to spread updates
  -  On boot, scan the log to recover the "last values"

**Typical approach (simplified):**
1. Reserve 2 pages of Flash
2. Write key-value pairs sequentially until page is full
3. When full, copy the latest values to the other page, erase the first
4. Alternate between pages

Many MCU vendors (e.g., **STM HAL** has `EE_INIT()`) provide EEPROM emulation libraries using this exact method