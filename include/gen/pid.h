#pragma once

namespace gen {
class PID {
    public:
        /**
         * @brief Construct a new PID object
         *
         * @param kP proportional gain
         * @param kI integral gain
         * @param kD derivative gain
         * @param iMax maximum error magnitude for integral accumulation
         * @param sign whether to reset integral when sign of error flips
         */
        PID(float kP, float kI, float kD, float iMax = 0, bool sign = false);

        /**
         * @brief Update the PID
         *
         * @param error target minus position - AKA error
         * @return float output
         *
         */
        float update(float error);

        /**
         * @brief reset integral, derivative, and prevTime
         *
         */
        void reset();
    protected:
        // gains
        const float kP;
        const float kI;
        const float kD;

        // optimizations
        const float iMax;
        const bool sign;

        float integral = 0;
        float prevError = 0;
};
} // namespace gen