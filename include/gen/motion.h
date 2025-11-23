// #pragma once

// #include "gen/odom.h"
// #include "pros/motors.hpp"

// namespace gen {

// /**
//  * Simple chassis wrapper that keeps track of the drive hardware details and
//  * offers a couple of helper drive methods.
//  */
// class Chassis {
//   public:
//     /**
//      * @param leftMotors reference to the left motor group
//      * @param rightMotors reference to the right motor group
//      * @param gearRatio motor rotations per wheel rotation (e.g. 3.0 for 3:1)
//      * @param motorRpm free speed of the motor in RPM
//      * @param wheelDiameter drive wheel diameter (same unit used by callers)
//      */
//     Chassis(pros::MotorGroup& leftMotors,
//             pros::MotorGroup& rightMotors,
//             double gearRatio,
//             double motorRpm,
//             double wheelDiameter);

//     /**
//      * Direct tank control using raw power [-127, 127].
//      */
//     void tank(int leftPower, int rightPower);

//     /**
//      * Arcade control using forward/turn inputs [-127, 127].
//      */
//     void arcade(int forward, int turn);

//     /**
//      * Apply braking on both sides.
//      */
//     void stop();

//     double getGearRatio() const;
//     double getMotorRpm() const;
//     double getWheelDiameter() const;
//     double getWheelCircumference() const;

//     /**
//      * Convert a motor RPM value into linear speed at the wheel surface.
//      */
//     double rpmToLinearSpeed(double motorRpm) const;

//   private:
//     pros::MotorGroup& left;
//     pros::MotorGroup& right;
//     double gearRatio;
//     double motorRpm;
//     double wheelDiameter;
// };

// }  // namespace gen
