#include "src/memllib/examples/InterfaceRL.hpp"

#include <cstddef>
#include <functional>

char *rl_interface_mem[sizeof(InterfaceRL)];
InterfaceRL *interface;

constexpr size_t kNInputs = 1;
constexpr size_t kNOutputs = 1;

bool test_succeeded = false;


/**
 * @brief Calculates the loss between predicted and target values.
 *
 * Computes the absolute error (AE) between predicted and target (both in [0, 1]).
 * If AE is below a threshold (0.1), sets positive to true and strength scales from 1 (perfect match) to 0 (at threshold).
 * If AE is above or equal to the threshold, sets positive to false and strength scales from 0 (at threshold) to 1 (maximum error).
 *
 * @param predicted The predicted value (float, range [0, 1]).
 * @param target The target value (float, range [0, 1]).
 * @param[out] positive True if AE < threshold, false otherwise.
 * @param[out] strength Scaled error strength (float, range [0, 1]).
 */
void loss_fn(float predicted, float target,
    bool &positive, float &strength) {

    static constexpr float threshold = 0.1f;
    float abs_error = fabsf(predicted - target);

    if (abs_error < threshold) {
        positive = true;
        strength = 1.0f - (abs_error / threshold); // AE=0 -> 1, AE=threshold -> 0
    } else {
        positive = false;
        strength = (abs_error - threshold) / (1.0f - threshold); // AE=threshold -> 0, AE=1 -> 1
    }
}


/**
 * @brief Parabolic target function.
 *
 * Returns a value on a parabola such that:
 *   x = 0   -> y = 1
 *   x = 0.5 -> y = 0
 *   x = 1   -> y = 1
 *
 * @param x Input value (float, range [0, 1]).
 * @return Parabola value (float).
 */
float target_fn(float x) {
    // Parabola: y = 4x^2 - 4x + 1
    return 4.0f * x * x - 4.0f * x + 1.0f;
}


void setup() {

    // Set up serial communication for debugging
    Serial.begin();
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB
    }
    Serial.println("Starting RL Interface Test...");
    interface = new (rl_interface_mem) InterfaceRL();
    interface->setup(kNInputs, kNOutputs);

    static constexpr size_t kTestIterations = 10000;

}

void loop() {
    static size_t milliseconds_passed = 0;
    static size_t milliseconds_passed_10ms = 0;

    size_t current_millis = millis();
    if (current_millis - milliseconds_passed >= 1000) {
        milliseconds_passed = current_millis;
        if (test_succeeded) {
            digitalWrite(33, HIGH); // Turn the LED on (HIGH is the voltage level)
        }
    } else if (current_millis - milliseconds_passed_10ms >= 10) {
        milliseconds_passed_10ms = current_millis;
        digitalWrite(33, LOW); // Turn the LED off (LOW is the voltage level)
    }
}
