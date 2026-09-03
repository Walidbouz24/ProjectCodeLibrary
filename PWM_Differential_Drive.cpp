#include "PWM_Differential_Drive.h"
#include "mbed.h"

/**
 * @brief Constructor: Initialize motor pins and PWM configuration
 * 
 * Sets up PWM period once during initialization (efficiency improvement).
 * Motor direction control pins are set to safe default state.
 */
RobotCar::RobotCar(PinName rwPin, PinName lwPin, PinName in1Pin, PinName in2Pin, PinName in3Pin, PinName in4Pin)
    : rightWheel(rwPin), leftWheel(lwPin), in1(in1Pin), in2(in2Pin), in3(in3Pin), in4(in4Pin), timePeriod(0.001f) {
    // Set PWM period once during initialization (1kHz frequency)
    rightWheel.period(timePeriod);
    leftWheel.period(timePeriod);
    
    // Initialize motor pins to safe state (all motors off)
    in1.write(0);
    in2.write(0);
    in3.write(0);
    in4.write(0);
    
    rightWheel.write(0.0f);
    leftWheel.write(0.0f);
}

/**
 * @brief Clamp duty cycle to valid PWM range [0.0, 1.0]
 */
float RobotCar::clamp_duty_cycle(float duty_cycle) const {
    if (duty_cycle < MIN_DUTY_CYCLE) return MIN_DUTY_CYCLE;
    if (duty_cycle > MAX_DUTY_CYCLE) return MAX_DUTY_CYCLE;
    return duty_cycle;
}

/**
 * @brief Move robot forward with both wheels at same speed
 * 
 * Motor configuration:
 * - Right wheel: in1=0, in2=1 (forward)
 * - Left wheel: in3=0, in4=1 (forward)
 */
void RobotCar::forward(float duty_cycle) {
    duty_cycle = clamp_duty_cycle(duty_cycle);
    
    // Set motor direction pins
    in1.write(0);
    in2.write(1);  // Right wheel forward
    in3.write(0);
    in4.write(1);  // Left wheel forward
    
    // Apply PWM speed (period already set in constructor)
    rightWheel.write(duty_cycle);
    leftWheel.write(duty_cycle);
}

/**
 * @brief Move robot backward with both wheels at same speed
 * 
 * Motor configuration:
 * - Right wheel: in1=1, in2=0 (backward)
 * - Left wheel: in3=1, in4=0 (backward)
 */
void RobotCar::back(float duty_cycle) {
    duty_cycle = clamp_duty_cycle(duty_cycle);
    
    // Set motor direction pins
    in1.write(1);
    in2.write(0);  // Right wheel backward
    in3.write(1);
    in4.write(0);  // Left wheel backward
    
    // Apply PWM speed
    rightWheel.write(duty_cycle);
    leftWheel.write(duty_cycle);
}

/**
 * @brief Move robot backward with curve (right wheel moves, left wheel stationary)
 * 
 * Motor configuration:
 * - Right wheel: in1=1, in2=0 (backward) with full speed
 * - Left wheel: in3=1, in4=0 (backward) with zero speed
 */
void RobotCar::back_curve(float duty_cycle) {
    duty_cycle = clamp_duty_cycle(duty_cycle);
    
    // Set motor direction pins
    in1.write(1);
    in2.write(0);  // Right wheel backward
    in3.write(1);
    in4.write(0);  // Left wheel backward
    
    // Apply PWM speed: right wheel moves, left wheel stops
    rightWheel.write(duty_cycle);
    leftWheel.write(0.0f);
}

/**
 * @brief Rotate robot on spot clockwise (right turn)
 * 
 * Motor configuration:
 * - Right wheel: in1=1, in2=0 (backward)
 * - Left wheel: in3=0, in4=1 (forward)
 * Both wheels same speed = rotation on spot
 */
void RobotCar::on_spot_right(float duty_cycle) {
    duty_cycle = clamp_duty_cycle(duty_cycle);
    
    // Set motor direction pins for opposite rotation
    in1.write(1);
    in2.write(0);  // Right wheel backward
    in3.write(0);
    in4.write(1);  // Left wheel forward
    
    // Apply PWM speed
    rightWheel.write(duty_cycle);
    leftWheel.write(duty_cycle);
}

/**
 * @brief Rotate robot on spot counter-clockwise (left turn)
 * 
 * Motor configuration:
 * - Right wheel: in1=0, in2=1 (forward)
 * - Left wheel: in3=1, in4=0 (backward)
 * Both wheels same speed = rotation on spot
 */
void RobotCar::on_spot_left(float duty_cycle) {
    duty_cycle = clamp_duty_cycle(duty_cycle);
    
    // Set motor direction pins for opposite rotation
    in1.write(0);
    in2.write(1);  // Right wheel forward
    in3.write(1);
    in4.write(0);  // Left wheel backward
    
    // Apply PWM speed
    rightWheel.write(duty_cycle);
    leftWheel.write(duty_cycle);
}

/**
 * @brief Curve right while moving forward
 * 
 * Motor configuration:
 * - Right wheel: in1=0, in2=1 (forward) with reduced speed
 * - Left wheel: in3=0, in4=1 (forward) with full speed
 * Speed difference creates rightward curve
 */
void RobotCar::curve_right(float duty_cycle) {
    duty_cycle = clamp_duty_cycle(duty_cycle);
    
    // Set motor direction pins
    in1.write(0);
    in2.write(1);  // Right wheel forward
    in3.write(0);
    in4.write(1);  // Left wheel forward
    
    // Apply PWM speed with reduced right wheel speed for curve
    float reduced_speed = clamp_duty_cycle(duty_cycle - CURVE_SPEED_REDUCTION);
    rightWheel.write(reduced_speed);
    leftWheel.write(duty_cycle);
}

/**
 * @brief Curve left while moving forward
 * 
 * Motor configuration:
 * - Right wheel: in1=0, in2=1 (forward) with full speed
 * - Left wheel: in3=0, in4=1 (forward) with reduced speed
 * Speed difference creates leftward curve
 */
void RobotCar::curve_left(float duty_cycle) {
    duty_cycle = clamp_duty_cycle(duty_cycle);
    
    // Set motor direction pins
    in1.write(0);
    in2.write(1);  // Right wheel forward
    in3.write(0);
    in4.write(1);  // Left wheel forward
    
    // Apply PWM speed with reduced left wheel speed for curve
    float reduced_speed = clamp_duty_cycle(duty_cycle - CURVE_SPEED_REDUCTION);
    rightWheel.write(duty_cycle);
    leftWheel.write(reduced_speed);
}

/**
 * @brief Stop all motors immediately
 * 
 * Sets all motor control pins to 0 and PWM duty cycle to 0
 */
void RobotCar::stop() {
    // Disable all motor direction control
    in1.write(0);
    in2.write(0);
    in3.write(0);
    in4.write(0);
    
    // Stop PWM signals
    rightWheel.write(0.0f);
    leftWheel.write(0.0f);
}
