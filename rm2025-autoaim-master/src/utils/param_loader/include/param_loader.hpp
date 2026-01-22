#pragma once

#include <exception>
#include <filesystem>
#include <iostream>
#include "voml/voml.h"
#include "robot_mapper.hpp"
#include "toml/types.hpp"


class ParamLoader {
   public:
    static ParamLoader& GetInstance() {
        static ParamLoader instance;
        return instance;
    }

    

    ParamLoader& operator=(const ParamLoader&) = delete;

    

    template <typename T, typename Key1, typename Key2, typename ... Keys>
    T GetParam(Key1&& k1, Key2&& k2, Keys&& ... keys) {

        return toml::find<T>(data, k1, k2, keys...);
    }
    template <typename T>
    T GetParam(const toml::key ky) {

        return toml::find<T>(data,ky);
    }

   private:
    ParamLoader(){
        Init();
    }
    void Init() {
        auto robot_map = rbmp::GetMyRobotType<RobotMap>();
        if (!robot_map.has_value()) {
            std::cerr << "toml selector: no robot type" << std::endl;
            std::terminate();
        }
        auto path = std::filesystem::current_path();
        //std::cout << "path now :" << path << std::endl;
        switch (robot_map.value()) {
            case RobotMap::RM2025_SENTRY_UPPER:
                data = toml::parse("./param/rm2025_sentry_up_head.toml");
                break;
            case RobotMap::RM2025_STANDARD_A1:
                data = toml::parse("./param/rm2025_standard_a1.toml");
                break;
            case RobotMap::RM2024_STANDARD_A3:
                data = toml::parse("./param/rm2024_standard_a3.toml");
                break;
            case RobotMap::RM2024_STANDARD_A1:
                data = toml::parse("./param/rm2024_standard_a1.toml");
                break;
            case RobotMap::RM2024_STANDARD_A2:
                data = toml::parse("./param/rm2024_standard_a2.toml");
                break;
            case RobotMap::RM2024_B_STANDARD:
                std::terminate();
                break;
            case RobotMap::RM2024_HERO_A1:
                data = toml::parse("./param/rm2024_hero.toml");
                break;
            case RobotMap::RM2024_SENTRY_ALPHA:
                data = toml::parse("./param/rm2024_sentry_alpha.toml");
                break;
            case RobotMap::RM2024_HERO_A2:
                data = toml::parse("./param/rm2024_hero_a2.toml");
                break;
            case RobotMap::RM2023_B_STANDARD:
                std::terminate();
                break;
            case RobotMap::RM2023_SENTRY:
                std::terminate();
                break;
            case RobotMap::RM2024_SENTRY_NAVIGATION:
                std::terminate();
                break;
            case RobotMap::RM2023_STANDARD:
                data = toml::parse("./param/rm2023_standard.toml");
                break;
            case RobotMap::RM2024_AERIAL:
                data = toml::parse("./param/rm2024_aerial.toml");
                break;
            default:
                std::cerr << "toml selector: undefined robot type" << std::endl;
                std::terminate();
        }
    }
    enum class RobotMap {

        RM2024_STANDARD_A1,
        RM2024_STANDARD_A2,
        RM2024_STANDARD_A3,
        RM2024_B_STANDARD,
        RM2024_HERO_A1,
        RM2024_SENTRY_ALPHA,
        RM2024_HERO_A2,
        RM2024_SENTRY_NAVIGATION,
        RM2023_B_STANDARD,
        RM2023_SENTRY,
        RM2023_STANDARD,
        RM2024_AERIAL, 
        RM2025_STANDARD_A1, 
        RM2025_SENTRY_UPPER, 
    };
    
    toml::basic_value<toml::discard_comments, std::unordered_map, std::vector> data{};
};