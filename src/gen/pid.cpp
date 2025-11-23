#include "gen/pid.h"
#include "gen/misc.h"

namespace gen {
PID::PID(float kP, float kI, float kD, float iMax, bool sign)
    : kP(kP),
      kI(kI),
      kD(kD),
      iMax(iMax),
      sign(sign) {}

float PID::update(const float error) {
    integral += error;
    if (getSign(error) != getSign((prevError)) && sign) integral = 0;
    if (fabs(error) > iMax && iMax != 0) integral = 0;
    const float derivative = error - prevError;
    prevError = error;
    return error * kP + integral * kI + derivative * kD;
}

void PID::reset() {
    integral = 0;
    prevError = 0;
}
} // namespace gen