# 定义 SwitchMode 枚举
int32 switch_mode  # "NONE", "LEFT", "RIGHT", "RESET"
int32 NONE = 0
int32 LEFT = 1
int32 RIGHT = 2
int32 RESET = 3

# 定义 TargetType 枚举
int32 target_type  # "BUILDING", "ROBOT", "POWER_RUNE", "BUILDING_AND_ROBOT"
int32 BUILDING = 0
int32 ROBOT = 1
int32 POWER_RUNE = 2
int32 BUILDING_AND_ROBOT = 3

# 定义 TargetPriorityRule 枚举
int32 target_priority_rule  # "ONLY_HERO", "AUTO", "MANUAL"
int32 ONLY_HERO = 0
int32 AUTO = 1
int32 MANUAL = 2

# 定义 SentryBehaviorMode 枚举
int32 sentry_behavior_mode  # "NOT_SENTRY", "ATTACK", "DEFEND"
int32 NOT_SENTRY = 0
int32 ATTACK = 0
int32 DEFEND = 0

# 定义 SpecialOrder 枚举
int32 special_order  # "NONE", "USE_SENTRY_BACKUP_MODULE", "USE_SECONDARY_CAMERA"
int32 SPECIAL_ORDER_NONE = 0
int32 USE_SENTRY_BACKUP_MODULE = 0
int32 USE_SECONDARY_CAMERA = 0

# EnemyHP 结构体成员
int32 hero_1
int32 engineer_2
int32 standard_3
int32 standard_4
int32 standard_5
int32 sentry
int32 out_pose
int32 base

# 颜色
int32 color
int32 COLOR_NONE = 0
int32 BLUE = 1
int32 RED = 2
int32 PURPLE = 3

#相机触发模式
int32 camera_mode
int32 CAMERA_SLEEP = 0
int32 SYNC_STAMP = 1
int32 WORK = 2

#同步节点模式
int32 sync_mode
int32 SYNC_SLEEP = 0
int32 SYNC_START = 1
int32 SYNC_WAIT = 2
int32 SYNC_WORKING = 3

#是否完成同步
bool is_sync_ok
# 剩余时间
int32 remaining_time_s

# 系统时间点（毫秒）
uint64 system_time_point_ms
