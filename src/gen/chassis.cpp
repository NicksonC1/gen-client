// #include "gen/chassis.h"

// #include <algorithm>
// #include <cstdint>

// namespace gen {

// Chassis::Chassis(pros::MotorGroup& left,
//                  pros::MotorGroup& right,
//                  const Drivetrain& drivetrain,
//                  Tuning lateral,
//                  Tuning angular)
//     : left_(left),
//       right_(right),
//       drivetrain_(drivetrain),
//       lateral_(lateral),
//       angular_(angular) {}

// Pose Chassis::getPose(const bool radians) const { return gen::getPose(radians); }

// void Chassis::tank(const int left_power, const int right_power) {
//   const int left_cmd = static_cast<int>(std::clamp(left_power, -127, 127));
//   const int right_cmd = static_cast<int>(std::clamp(right_power, -127, 127));
//   left_.move(left_cmd);
//   right_.move(right_cmd);
// }

// void Chassis::arcade(const int forward, const int turn) { tank(forward + turn, forward - turn); }

// void Chassis::stop() {
//   left_.move(0);
//   right_.move(0);
// }

// }  // namespace gen
