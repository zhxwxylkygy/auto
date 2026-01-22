//
// Created by XianY on 2023/3/16.
//

#ifndef ULTRA_VISION_FRAMEWORK_PACKER_H
#define ULTRA_VISION_FRAMEWORK_PACKER_H

#define START_PROTOCOL _Pragma("pack(1)")
#define END_PROTOCOL _Pragma("pack()")

#define AS_PROTOCOL [[gnu::packed]]
#define PROTOCOL struct AS_PROTOCOL

#endif //ULTRA_VISION_FRAMEWORK_PACKER_H
