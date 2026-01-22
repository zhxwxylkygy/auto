#pragma once
#include <filesystem>
#include<iostream>
#include <algorithm>
#include <map>
#include <type_traits>
#include "fstream"
#include "optional"
#include <ranges>
#include "string"
#include "string_view"

namespace rbmp {

static const std::string kMapPath = "/root/master/program/RM2025-autoaim/src/utils/robot_mapper/robot_map.conf";
//static const std::string kMapPath = "./src/utils/robot_mapper/robot_map.conf";
static const std::string kDelimiter = ":";

namespace enum_reflect {
#if defined(__clang__)
#define PRETTY_FUNCTION_NAME __PRETTY_FUNCTION__
#define OFFSET 2
#elif defined(__GNUC__)
#define PRETTY_FUNCTION_NAME __PRETTY_FUNCTION__
#define OFFSET 51
#elif defined(_MSC_VER)
#define PRETTY_FUNCTION_NAME __FUNCSIG__
#define OFFSET 17
#endif

// bool is_valid() [E = Color, V = Color::GREEN]
// bool is_valid() [E = Color, V = 1]
template <typename E, E V>
constexpr std::string_view get_enum_value_name() {
    std::string_view name{PRETTY_FUNCTION_NAME, sizeof(PRETTY_FUNCTION_NAME) - OFFSET};
    for (std::size_t i = name.size(); i > 0; --i) {
        if (!((name[i - 1] >= '0' && name[i - 1] <= '9') || (name[i - 1] >= 'a' && name[i - 1] <= 'z') ||
              (name[i - 1] >= 'A' && name[i - 1] <= 'Z') || (name[i - 1] == '_'))) {
            name.remove_prefix(i);
            break;
        }
    }
    if (name.size() > 0 && ((name.front() >= 'a' && name.front() <= 'z') ||
                            (name.front() >= 'A' && name.front() <= 'Z') || (name.front() == '_'))) {
        return name;
    }
    return {};  // Invalid name.
}

// bool is_valid() [E = Color, V = Color::GREEN]
// bool is_valid() [E = Color, V = 1]
template <typename E, E V>
constexpr bool is_valid() {
    return get_enum_value_name<E, V>().size() != 0;
}

template <int... Is>
constexpr auto make_integer_list_wrapper(std::integer_sequence<int, Is...>) {
    constexpr int half_size = sizeof...(Is) / 2;
    return std::integer_sequence<int, (Is - half_size)...>();
}

constexpr auto test_integer_sequence_v = make_integer_list_wrapper(std::make_integer_sequence<int, 256>());

template <typename E, int... Is>
constexpr size_t get_enum_size(std::integer_sequence<int, Is...>) {
    constexpr std::array<bool, sizeof...(Is)> valid{is_valid<E, static_cast<E>(Is)>()...};
    constexpr std::size_t count = [](decltype((valid)) valid_) constexpr noexcept -> std::size_t {
        auto count_ = std::size_t{0};
        for (std::size_t i_ = 0; i_ < valid_.size(); ++i_) {
            if (valid_[i_]) {
                ++count_;
            }
        }
        return count_;
    }(valid);
    return count;
}

template <typename E>
constexpr std::size_t enum_size_v = get_enum_size<E>(test_integer_sequence_v);

template <typename E, int... Is>
constexpr auto get_all_valid_values(std::integer_sequence<int, Is...>) {
    constexpr std::array<bool, sizeof...(Is)> valid{is_valid<E, static_cast<E>(Is)>()...};
    constexpr std::array<int, sizeof...(Is)> integer_value{Is...};
    std::array<int, enum_size_v<E>> values{};
    for (std::size_t i = 0, v = 0; i < sizeof...(Is); ++i) {
        if (valid[i]) {
            values[v++] = integer_value[i];
        }
    }
    return values;
}

template <typename E, int... Is>
constexpr auto get_all_valid_names(std::integer_sequence<int, Is...>) {
    constexpr std::array<std::string_view, sizeof...(Is)> names{get_enum_value_name<E, static_cast<E>(Is)>()...};
    std::array<std::string_view, enum_size_v<E>> valid_names{};
    for (std::size_t i = 0, v = 0; i < names.size(); ++i) {
        if (names[i].size() != 0) {
            valid_names[v++] = names[i];
        }
    }
    return valid_names;
}

template <typename E>
constexpr auto enum_names_v = get_all_valid_names<E>(test_integer_sequence_v);

template <typename E>
constexpr auto enum_values_v = get_all_valid_values<E>(test_integer_sequence_v);

constexpr auto static_throw(int n) -> void {
    n <= 0 ? throw std::runtime_error("should not reach here. Invalid value.") : 0;
}

template <typename E>
constexpr E string2enum(const std::string_view str) {
    constexpr auto valid_names = enum_names_v<E>;
    constexpr auto valid_values = enum_values_v<E>;
    constexpr auto enum_size = enum_size_v<E>;
    for (size_t i = 0; i < enum_size; ++i) {
        if (str == valid_names[i]) {
            return static_cast<E>(valid_values[i]);
        }
    }
    static_throw(-1);
    return E{};
}

template <typename E>
constexpr std::string_view enum2string(E V) {
    constexpr auto valid_names = enum_names_v<E>;
    constexpr auto valid_values = enum_values_v<E>;
    constexpr auto enum_size = enum_size_v<E>;
    for (size_t i = 0; i < enum_size; ++i) {
        if (static_cast<int>(V) == valid_values[i]) {
            return valid_names[i];
        }
    }
    static_throw(-1);
    return "";
}
}  // namespace enum_reflect

static void Trim(std::string& sv) {
    sv.erase(std::remove_if(sv.begin(), sv.end(), ::isspace), sv.end());
}

static std::string ExecCmd(std::string_view cmd) {
    FILE* pipe = popen(cmd.data(), "r");
    if (!pipe) {
        return "";
    }
    size_t ret = 0;
    constexpr size_t bufferLen = 16;
    char buf[bufferLen + 1] = {0};
    std::string result{};
    while ((ret = fread(buf, sizeof(char), bufferLen, pipe)) == bufferLen) {
        result.append(buf);
    }
    if (ferror(pipe) != 0) {
        return "";
    }
    if (feof(pipe) != 0) {
        result.append(buf, ret);
    }
    Trim(result);
    return result;
}

static std::map<std::string, std::string> GenerateMap() {
    std::map<std::string, std::string> map{};
    std::ifstream fs{kMapPath};
    auto p = std::filesystem::current_path();
    std::string line{};
    while (fs.good()) {
        std::getline(fs, line);
        auto tokens = line | std::ranges::views::split(kDelimiter) | std::views::transform([](auto&& token) {
                          return std::string_view(&*token.begin(), std::ranges::distance(token));
                      });

        auto it = std::ranges::begin(tokens);
        auto type = std::string{*it};
        Trim(type);

        std::ranges::advance(it, 1);
        auto sn = std::string{*it};
        Trim(sn);

        map[sn] = type;
    }
    return map;
}

template <typename E, typename = typename std::enable_if<std::is_enum_v<E>, void>::type>
std::optional<E> GetMyRobotType() {
    static auto map = GenerateMap();
    static auto sn = ExecCmd("dmidecode -s system-serial-number");
    if (map.find(sn) == map.end()) {
        std::cout << "sn = " << sn << std::endl;
        return std::nullopt;
    }
    return enum_reflect::string2enum<E>(map[sn]);
}

}  // namespace rbmp