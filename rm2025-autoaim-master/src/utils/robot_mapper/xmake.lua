set_project("robot_mapper")
set_version("1.0.0")

target("robot_mapper")
    set_kind("static")
    add_files("./robot_mapper.hpp",{rule = "c++"})