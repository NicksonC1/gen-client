// #include "gen/motion.h"

// #include <algorithm>

// namespace {
// constexpr double kPi = 3.14159265358979323846;

// int clampPower(const int value) {
//   return std::clamp(value, -127, 127);
// }
// }  // namespace

// namespace gen {

// Chassis::Chassis(pros::MotorGroup& leftMotors,
//                  pros::MotorGroup& rightMotors,
//                  const double gearRatio,
//                  const double motorRpm,
//                  const double wheelDiameter)
//     : left(leftMotors),
//       right(rightMotors),
//       gearRatio(gearRatio),
//       motorRpm(motorRpm),
//       wheelDiameter(wheelDiameter) {}

// void Chassis::tank(const int leftPower, const int rightPower) {
//   left.move(clampPower(leftPower));
//   right.move(clampPower(rightPower));
// }

// void Chassis::arcade(const int forward, const int turn) {
//   const int leftPower = clampPower(forward + turn);
//   const int rightPower = clampPower(forward - turn);
//   tank(leftPower, rightPower);
// }

// void Chassis::stop() {
//   left.brake();
//   right.brake();
// }

// double Chassis::getGearRatio() const {
//   return gearRatio;
// }

// double Chassis::getMotorRpm() const {
//   return motorRpm;
// }

// double Chassis::getWheelDiameter() const {
//   return wheelDiameter;
// }

// double Chassis::getWheelCircumference() const {
//   return wheelDiameter * kPi;
// }

// double Chassis::rpmToLinearSpeed(const double rpmValue) const {
//   if (gearRatio == 0) return 0;
//   const double wheelRpm = rpmValue / gearRatio;
//   return wheelRpm * getWheelCircumference() / 60.0;
// }

// }  // namespace gen
