// ==========================================
// 6-Slot RPM Counter (Period Method)
// ==========================================

const byte SENSOR_PIN = 2;  // Must be 2 or 3 for Arduino Uno/Nano interrupts
const float SLOTS_DISK = 6;        // Number of holes in the disk
const float IR_SENSOR_CONST = 3;
const float SLOTS = SLOTS_DISK * IR_SENSOR_CONST; // Effective number of slots

// Volatile variables are modified inside the ISR
volatile unsigned long last_pulse_time = 0;
volatile unsigned long pulse_interval = 0;
volatile bool new_pulse_available = false;

// Configuration for Smoothing (Moving Average)
const int NUM_SAMPLES = 20; // Higher = smoother, Lower = faster response
float rpm_history[NUM_SAMPLES];
int rpm_index = 0;
float total_rpm = 0;

void setup() {
  Serial.begin(9600);
  pinMode(SENSOR_PIN, INPUT_PULLUP); // Use INPUT if you have external resistors
  
  // Attach interrupt to rising edge (when the slot opens)
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), sensor_isr, RISING);
  
  // Initialize the history array to 0
  for (int i = 0; i < NUM_SAMPLES; i++) {
    rpm_history[i] = 0;
  }
}

void loop() {
  unsigned long local_interval = 0;
  bool data_ready = false;

  // --- CRITICAL SECTION START ---
  // We must disable interrupts briefly to read multi-byte variables safely
  noInterrupts();
  if (new_pulse_available) {
    local_interval = pulse_interval;
    new_pulse_available = false;
    data_ready = true;
  }
  interrupts();
  // --- CRITICAL SECTION END ---

  // 1. Calculate RPM if we have new data
  if (data_ready && local_interval > 0) {
    
    // THE MATH:
    // RPM = 60 seconds / Time for 1 revolution
    // Time for 1 rev = (pulse_interval * SLOTS) microseconds
    // RPM = 60,000,000 / (pulse_interval * 6)
    
    float instantaneous_rpm = 60000000.0 / (local_interval * SLOTS);

    // 2. Apply Moving Average Filter
    // Subtract the oldest reading from the total
    total_rpm = total_rpm - rpm_history[rpm_index];
    // Add the new reading
    rpm_history[rpm_index] = instantaneous_rpm;
    // Add the new reading to the total
    total_rpm = total_rpm + instantaneous_rpm;
    // Advance index
    rpm_index = (rpm_index + 1) % NUM_SAMPLES;

    // Calculate average
    float smooth_rpm = total_rpm / NUM_SAMPLES;
    float final = smooth_rpm + 5;

    // Print Results
    Serial.print("Raw: ");
    Serial.print(instantaneous_rpm);
    Serial.print(" | Smooth: ");
    Serial.println(final);
  }

  // 3. Timeout Logic (Detect Stopped Motor)
  // If no pulse for 1 second (1,000,000 micros), force RPM to 0
  unsigned long current_time = micros();
  // We read last_pulse_time atomically just to be safe
  unsigned long last_time_safe;
  noInterrupts();
  last_time_safe = last_pulse_time;
  interrupts();

  if ((current_time - last_time_safe) > 1000000) {
     // Reset averages to zero so it doesn't "fade" down, but drops instantly
     Serial.println("RPM: 0 (Stopped)");
     total_rpm = 0;
     for (int i = 0; i < NUM_SAMPLES; i++) rpm_history[i] = 0;
     delay(100); // Small delay to prevent serial flooding when stopped
  }
}

// --- Interrupt Service Routine ---
void sensor_isr() {
  unsigned long now = micros();
  
  // Simple Debounce: Ignore pulses closer than 100 microseconds
  // (Adjust this based on your max expected RPM)
  if (now - last_pulse_time > 100) {
    pulse_interval = now - last_pulse_time;
    last_pulse_time = now;
    new_pulse_available = true;
  }
}