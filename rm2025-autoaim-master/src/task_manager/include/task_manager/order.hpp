#include <interfaces/msg/order.hpp>
#include <iostream>
#include <memory>
#include <mutex>
#include <task_manager/order_interface.hpp>
#include <vector>
namespace TM {
class Order {
   public:
    static std::shared_ptr<Order> GetInstance() {
        // if (order_target_ == nullptr) {
        //     order_target_ = std::shared_ptr<Order>(new Order);
        // }
        static std::shared_ptr<Order> o = std::shared_ptr<Order>(new Order);
        return o;
    }

    [[nodiscard]] RM::Color FetchColorOrder() const noexcept{
        return therad_server_.state_.color;
    }
    [[nodiscard]]bool GetSyncStatus(){
        return therad_server_.state_.is_sync_ok;
    }
    void UpdateOrders(const interfaces::msg::Order::SharedPtr order_msg) {
        therad_server_.state_.switchmode = static_cast<MCU::SwitchMode>(order_msg->switch_mode);
        therad_server_.state_.targetype = static_cast<MCU::TargetType>(order_msg->target_type);
        therad_server_.state_.targetpriorityrule = static_cast<MCU::TargetPriorityRule>(order_msg->target_priority_rule);
        therad_server_.state_.sentrybehaviormode = static_cast<MCU::SentryBehaviorMode>(order_msg->sentry_behavior_mode);
        therad_server_.state_.specialorder = static_cast<MCU::SpecialOrder>(order_msg->special_order);
        therad_server_.state_.color = static_cast<RM::Color>(order_msg->color);
        therad_server_.state_.camera_mode = static_cast<CameraMode>(order_msg->camera_mode);
        therad_server_.state_.sync_mode = static_cast<SnycMode>(order_msg->sync_mode);
        therad_server_.state_.target_hp[RM::TargetClassify::BASE_DART] = order_msg->base;
        therad_server_.state_.target_hp[RM::TargetClassify::OUTPOST_DART] = order_msg->out_pose;
        therad_server_.state_.target_hp[RM::TargetClassify::HERO_1] = order_msg->hero_1;
        therad_server_.state_.target_hp[RM::TargetClassify::ENGINEER_2] = order_msg->engineer_2;
        therad_server_.state_.target_hp[RM::TargetClassify::STANDARD_3] = order_msg->standard_3;
        therad_server_.state_.target_hp[RM::TargetClassify::STANDARD_4] = order_msg->standard_4;
        therad_server_.state_.target_hp[RM::TargetClassify::STANDARD_5] = order_msg->standard_5;
        therad_server_.state_.target_hp[RM::TargetClassify::SENTRY] = order_msg->sentry;
    }
    void UpdateTime(const interfaces::msg::Order::SharedPtr order_msg) {
        therad_server_.times_.remain_time_s = order_msg->remaining_time_s;
        therad_server_.times_.system_time_point_ms = order_msg->system_time_point_ms;
    }

    [[nodiscard]] bool IsStateChange() {
        static auto state_last = therad_server_.state_;
        if (state_last != therad_server_.state_) {
            state_last = therad_server_.state_;
            return true;
        }
        return false;
    }

    void SetSyncAlreadyState() { therad_server_.state_.is_sync_ok = true; }

    [[nodiscard]] interfaces::msg::Order GetOrderMessage() {
        interfaces::msg::Order order_msg;
        order_msg.special_order = static_cast<int>(therad_server_.state_.specialorder);
        order_msg.camera_mode = static_cast<int>(therad_server_.state_.camera_mode);
        order_msg.color = static_cast<int>(therad_server_.state_.color);
        order_msg.sync_mode = static_cast<int>(therad_server_.state_.sync_mode);
        order_msg.sentry_behavior_mode = static_cast<int>(therad_server_.state_.sentrybehaviormode);
        order_msg.switch_mode = static_cast<int>(therad_server_.state_.switchmode);
        order_msg.target_type = static_cast<int>(therad_server_.state_.targetype);
        order_msg.target_priority_rule = static_cast<int>(therad_server_.state_.targetpriorityrule);
        order_msg.base = therad_server_.state_.target_hp[RM::TargetClassify::BASE_DART];
        order_msg.out_pose = therad_server_.state_.target_hp[RM::TargetClassify::OUTPOST_DART];
        order_msg.hero_1 = therad_server_.state_.target_hp[RM::TargetClassify::HERO_1];
        order_msg.engineer_2 = therad_server_.state_.target_hp[RM::TargetClassify::ENGINEER_2];
        order_msg.standard_3 = therad_server_.state_.target_hp[RM::TargetClassify::STANDARD_3];
        order_msg.standard_4 = therad_server_.state_.target_hp[RM::TargetClassify::STANDARD_4];
        order_msg.standard_5 = therad_server_.state_.target_hp[RM::TargetClassify::STANDARD_5];
        order_msg.sentry = therad_server_.state_.target_hp[RM::TargetClassify::SENTRY];
        order_msg.camera_mode = order_msg.is_sync_ok = therad_server_.state_.is_sync_ok;
        order_msg.remaining_time_s = therad_server_.times_.remain_time_s ;
        order_msg.system_time_point_ms = therad_server_.times_.system_time_point_ms;
        return order_msg;
    }

    // int CreateOrderMessage(std::string type_name) {}

   private:
    Order() = default;
    // static std::shared_ptr<Order> order_target_;

    struct State {
        bool is_sync_ok = false;
        MCU::SwitchMode switchmode;
        MCU::TargetType targetype;
        MCU::TargetPriorityRule targetpriorityrule;
        MCU::SentryBehaviorMode sentrybehaviormode;
        MCU::SpecialOrder specialorder;
        RM::Color color;
        CameraMode camera_mode;
        SnycMode sync_mode;

        std::map<RM::TargetClassify, int> target_hp{
            {RM::TargetClassify::BASE_DART, -1},  {RM::TargetClassify::OUTPOST_DART, -1},
            {RM::TargetClassify::HERO_1, -1},     {RM::TargetClassify::ENGINEER_2, -1},
            {RM::TargetClassify::STANDARD_3, -1}, {RM::TargetClassify::STANDARD_4, -1},
            {RM::TargetClassify::STANDARD_5, -1}, {RM::TargetClassify::SENTRY, -1},
        };

        bool operator==(const State& other) const {
            // Compare primitive members
            if (switchmode != other.switchmode || targetype != other.targetype ||
                targetpriorityrule != other.targetpriorityrule || sentrybehaviormode != other.sentrybehaviormode ||
                specialorder != other.specialorder || color != other.color || camera_mode != other.camera_mode ||
                sync_mode != other.sync_mode) {
                return false;
            }

            // Compare map content
            if (target_hp != other.target_hp) {
                return false;
            }

            return true;
        }
    };
    
    struct remain_time {
        int remain_time_s;
        uint64_t system_time_point_ms;
    };
    

    struct ThredServer{
        State state_{};
        remain_time times_{};
        std::mutex m_;
    };

    ThredServer therad_server_;
    // std::vector<IOrder> orders{};
    // OColor o_color;
    // OSwitchMode o_switchmode;
    // OTargetType o_targetype;
    // OTargetPriorityRule o_targetpriorityrule;
    // OSentryBehaviorMode o_sentrybehaviormode;
    // OSpecialOrder o_specialorder;
};
// std::shared_ptr<Order> Order::order_target_ = nullptr;
// IOrder& operator=(OrderType type) {
//         order_value = static_cast<int>(type);  // 将 enum 转换为 int
//         return *this;
//     }
}  // namespace TM
