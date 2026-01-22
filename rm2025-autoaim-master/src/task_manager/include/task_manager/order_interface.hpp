#pragma once
#include <interfaces/msg/detail/order__struct.hpp>
#include <map>
namespace TM {

// struct IOrder {
//     virtual ~IOrder() = default;
// };
// struct OColor : IOrder {
//     enum class Color {
//         NONE [[maybe_unused]],
//         BLUE,
//         RED,
//         PURPLE [[maybe_unused]],
//     };
//     Color color_order{};
// };
// struct OArmorId : IOrder {
//     enum class ArmorId { SENTRY, ONE, TWO, THREE, FOUR, FIVE, OUTPOST, BASE, INVALID };
//     ArmorId armorid_order{};
// };
// struct OTargetArmorNum : IOrder {
//     enum class TargetArmorNum { UNCERTAIN, ONE, TWO, THREE, FOUR };
//     TargetArmorNum targetarmornum_order{};
// };
// struct OTargetHP : IOrder {
//     enum class TargetClassify {
//         BASE,
//         OUTPOST,

//         HERO_1,
//         ENGINEER_2,
//         STANDARD_3,
//         STANDARD_4,
//         STANDARD_5,
//         SENTRY,
//     };
//     TargetClassify targetclassify_order{};
//     std::map<TargetClassify, int> target_hp{
//         {TargetClassify::BASE, -1},       {TargetClassify::OUTPOST, -1},    {TargetClassify::HERO_1, -1},
//         {TargetClassify::ENGINEER_2, -1}, {TargetClassify::STANDARD_3, -1}, {TargetClassify::STANDARD_4, -1},
//         {TargetClassify::STANDARD_5, -1}, {TargetClassify::SENTRY, -1},
//     };
// };

// struct OSwitchMode : IOrder {
//     enum class SwitchMode { NONE, LEFT, RIGHT, RESET };
//     SwitchMode switchmode_order{};
// };
// struct OTargetType : IOrder {
//     enum class TargetType { BUILDING, ROBOT, POWER_RUNE, BUILDING_AND_ROBOT };
//     TargetType targetype_order{};
// };
// struct OTargetPriorityRule : IOrder {
//     enum class TargetPriorityRule { ONLY_HERO, AUTO, MANUAL };
//     TargetPriorityRule targetpriorityrule_order{};
// };
// struct OSentryBehaviorMode : IOrder {
//     enum class SentryBehaviorMode { NOT_SENTRY, ATTACK, DEFEND };
//     SentryBehaviorMode sentrybehaviormode_order{};
// };
// struct OSpecialOrder : IOrder {
//     enum class SpecialOrder { NONE, USE_SENTRY_BACKUP_MODULE, USE_SECONDARY_CAMERA };
//     SpecialOrder specialorder_order{};
// };
enum class CameraMode {
    CAMERA_SLEEP = interfaces::msg::Order::CAMERA_SLEEP,
    SYNC_STAMP = interfaces::msg::Order::SYNC_STAMP,
    WORK = interfaces::msg::Order::WORK,
};
enum class SnycMode {
    SYNC_SLEEP = interfaces::msg::Order::SYNC_SLEEP,
    SYNC_START = interfaces::msg::Order::SYNC_START,
    SYNC_WORKING = interfaces::msg::Order::SYNC_WORKING,
};
}  // namespace TM

namespace RM {  // only include rm official concepts
enum class BulletType { D_42MM [[maybe_unused]], D_17MM [[maybe_unused]] };

enum class Color {
    NONE [[maybe_unused]] = interfaces::msg::Order::COLOR_NONE,
    BLUE = interfaces::msg::Order::BLUE,
    RED = interfaces::msg::Order::RED,
    PURPLE [[maybe_unused]] = interfaces::msg::Order::PURPLE,
};

enum class TargetClassify {
    BASE_DART,  // base
    BASE_BOTTOM,
    BASE_TOP,
    OUTPOST_DART,  // outpost
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

namespace MCU {
enum class SwitchMode { 
    NONE = interfaces::msg::Order::NONE, 
    LEFT = interfaces::msg::Order::LEFT, 
    RIGHT = interfaces::msg::Order::RIGHT, 
    RESET = interfaces::msg::Order::RESET
};
enum class TargetType { 
    BUILDING = interfaces::msg::Order::BUILDING, 
    ROBOT = interfaces::msg::Order::ROBOT,
    POWER_RUNE  = interfaces::msg::Order::POWER_RUNE, 
    BUILDING_AND_ROBOT = interfaces::msg::Order::BUILDING_AND_ROBOT 
} ;
enum class TargetPriorityRule { 
    ONLY_HERO = interfaces::msg::Order::ONLY_HERO, 
    AUTO = interfaces::msg::Order::AUTO, 
    MANUAL = interfaces::msg::Order::MANUAL
} ;
enum class SentryBehaviorMode { 
    NOT_SENTRY = interfaces::msg::Order::NOT_SENTRY, 
    ATTACK = interfaces::msg::Order::ATTACK, 
    DEFEND = interfaces::msg::Order::DEFEND 
} ;
enum class SpecialOrder { 
    NONE  = interfaces::msg::Order::SPECIAL_ORDER_NONE, 
    USE_SENTRY_BACKUP_MODULE = interfaces::msg::Order::USE_SENTRY_BACKUP_MODULE, 
    USE_SECONDARY_CAMERA= interfaces::msg::Order::USE_SECONDARY_CAMERA };
}  // namespace MCU