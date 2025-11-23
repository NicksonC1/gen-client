#pragma once

#include <string>
#include <utility>
#include <vector>

#include "gen/selector.h"

// Auton stubs and routine list: adjust names/implementations and entries as needed.
namespace Auton {

inline void main() {}
inline void leftB() {}
inline void leftR() {}
inline void rightB() {}
inline void rightR() {}
inline void soloB() {}
inline void soloR() {}
inline void skills() {}

inline std::vector<gen::selector::AutonRoutine> routines = {
    {"Default Auton", main},
    {"Blue Left Qual", leftB},
    {"Red Left Qual", leftR},
    {"Blue Right Qual", rightB},
    {"Red Right Qual", rightR},
    {"Blue Solo Qual", soloB},
    {"Red Solo Qual", soloR},
    {"Skills", skills},
};

}  // namespace Auton
