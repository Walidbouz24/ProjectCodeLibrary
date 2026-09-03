#ifndef ROBOTCAR_H
#define ROBOTCAR_H

#include "mbed.h"

/**
 * @class RobotCar
 * @brief PWM Differential Drive Controller for two-wheeled robot
 * 
 * Provides methods to control forward, backward, turning, and rotational motion
 * with improved input validation and efficiency.
 */
class RobotCar {
    public:
        /**
         * @brief Constructor for RobotCar
         * @param rwPin PWM pin for right wheel
         * @param lwPin PWM pin for left wheel
         * @param in1Pin Motor driver input pin 1 (right wheel forward/backward)
         * @param in2Pin Motor driver input pin 2 (right wheel forward/backward)
         * @param in3Pin Motor driver input pin 3 (left wheel forward/backward)
         * @param in4Pin Motor driver input pin 4 (right wheel forward/backward)
         */
        RobotCar(PinName rwPin, PinName lwPin, PinName in1Pin, PinName in2Pin, PinName in3Pin, PinName in4Pin);

        /**
         * @brief Move robot forward
         * @param duty_cycle PWM duty cycle (0.0 - 1.0)
         */
        void forward(float duty_cycle);

        /**
         * @brief Move robot backward
         * @param duty_cycle PWM duty cycle (0.0 - 1.0)
         */
        void back(float duty_cycle);

        /**
         * @brief Move robot backward while curving (right wheel moves, left wheel stops)
         * @param duty_cycle PWM duty cycle (0.0 - 1.0)
         */
        void back_curve(float duty_cycle);

        /**
         * @brief Rotate robot on spot (clockwise)
         * @param duty_cycle PWM duty cycle (0.0 - 1.0)
         */
        void on_spot_right(float duty_cycle);

        /**
         * @brief Rotate robot on spot (counter-clockwise)
         * @param duty_cycle PWM duty cycle (0.0 - 1.0)
         */
        void on_spot_left(float duty_cycle);

        /**
         * @brief Curve right while moving forward
         * @param duty_cycle PWM duty cycle (0.0 - 1.0)
         */
        void curve_right(float duty_cycle);

        /**
         * @brief Curve left while moving forward
         * @param duty_cycle PWM duty cycle (0.0 - 1.0)
         */
        void curve_left(float duty_cycle);

        /**
         * @brief Stop all motors immediately
         */
        void stop();

    private:
        PwmOut rightWheel;
        PwmOut leftWheel;
        DigitalOut in1;
        DigitalOut in2;
        DigitalOut in3;
        DigitalOut in4;
        
        float timePeriod;
        static constexpr float CURVE_SPEED_REDUCTION = 0.20f;  // Speed reduction for curves
        static constexpr float MAX_DUTY_CYCLE = 1.0f;
        static constexpr float MIN_DUTY_CYCLE = 0.0f;

        /**
         * @brief Clamp duty cycle value to valid range [0.0, 1.0]
         * @param duty_cycle Input duty cycle value
         * @return Clamped duty cycle value
         */
        float clamp_duty_cycle(float duty_cycle) const;
};

#endif
