//
// Created by wpie on 23-11-21.
//

#pragma once
namespace RM {  // only include rm official concepts
    enum class BulletType {
        D_42MM [[maybe_unused]], D_17MM [[maybe_unused]]
    };

    enum class Color {
        NONE [[maybe_unused]],
        BLUE,
        RED,
        PURPLE [[maybe_unused]],
    };
    enum class ArmorId {
        SENTRY,
        ONE,
        TWO,
        THREE,
        FOUR,
        FIVE,
        OUTPOST,
        BASE,
        INVALID
    };
    enum class ArmorSize {
        SMALL, LARGE, INVALID
    };
    enum class TargetArmorNum {
        UNCERTAIN, ONE, TWO, THREE, FOUR
    };
    enum class TargetClassify {
        BASE_DART,
        BASE_BOTTOM,
        BASE_TOP,
        OUTPOST_DART,
        OUTPOST_SPIN,

        HERO_1,
        ENGINEER_2,
        STANDARD_3,
        STANDARD_4,
        STANDARD_5,
        SENTRY,

        POWER_RUNE
    };
}  // namespace RM

