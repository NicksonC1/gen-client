// #include "colorsort.h"

// namespace gen{
// class Color {
//     public:
//     enum class colorVals { NONE, BLUE, RED };
//     colorVals state = colorVals::NONE;
//     bool isDone = false, isC = false, extend_once = false;
//     double rLow = 5.0, rHigh = 38.0, bLow = 190.0, bHigh = 220.0, minProx = 95; 
//     inline bool isRed(double h, double low, double max) { return h > low && h < max; }
//     inline bool isBlue(double h, double low, double max) { return h > low && h < max; }
//     inline bool withinProx(int input, double max) { return (input > max); }
//     colorVals colorConvertor(colorVals input) { return (input == colorVals::BLUE) ? colorVals::RED : colorVals::BLUE; }
//     void colorSort(colorVals input) {
//         colorVals lastColor = colorVals::NONE;
//         if(TaskHandler::colorSort){
//             if(input == colorVals::RED && isRed(Sensor::o_colorSort.get_hue(),rLow,rHigh) && withinProx(Sensor::o_colorSort.get_proximity(),minProx)){
//                 // Piston::miniHood.set_value(true);
//                 pros::delay(200);
//                 extend_once = true;
//             }
//             else if(input == colorVals::BLUE && isBlue(Sensor::o_colorSort.get_hue(),bLow,bHigh) && withinProx(Sensor::o_colorSort.get_proximity(),minProx)){
//                 // Piston::miniHood.set_value(true);
//                 pros::delay(200);
//                 extend_once = true;
//             }
//             // else { Piston::miniHood.set_value(false); }
//             extend_once = false;
//         }
//     }
// };
// } // namespace gen