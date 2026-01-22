#pragma once

#include "string"

namespace vpie {
enum class EnemyColor {
    RED = 0,
    BLUE = 1,
    WHITE = 2,
};

enum class ArmorType { SMALL, LARGE, INVALID };

inline std::string armorTypeToString(const ArmorType& type) {
    switch (type) {
        case ArmorType::SMALL:
            return "small";
        case ArmorType::LARGE:
            return "large";
        default:
            return "invalid";
    }
}

inline std::string Color2String(EnemyColor c){
    switch (c) {
        case EnemyColor::RED:
            return "R";
        case EnemyColor::BLUE:
            return "B";
        default:
            return "W";
    }
}

}  // namespace vpie