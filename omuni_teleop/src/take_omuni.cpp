#include"take_omuni.hpp"

#define TX_DEVICE_ID 1
#define RX_DEVICE_ID 1

#define PUBLISH_RATE_MS 50  // publish周期(ms), 短くしすぎるとマイコンが処理しきれなくなるので注意

#define TARGET_NODE 0

class TakeOmuni : public rclcpp::Node {
public:
    TakeOmuni(uint8_t tx_device_id , uint8_t rx_device_id)
        : Node("take_omuni"), 
          tx_device_id_(tx_device_id),
          rx_device_id_(rx_device_id) {
    }

private:
    uint8_t tx_device_id_;
    uint8_t rx_device_id_;