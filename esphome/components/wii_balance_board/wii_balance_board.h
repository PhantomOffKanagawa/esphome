#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/components/button/button.h"
#include "esphome/components/sensor/sensor.h"
#include "wii.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

#include "task_queue.h"
#include <unordered_map>
#include <string>

namespace esphome {
namespace wii_balance_board {

struct Sample {
  float samples[64] = {NAN};
  size_t sample_count{0};
  uint8_t battery{0};
  uint8_t temperature{0};
  uint8_t referenceTemperature{0};
  float measurement{NAN};
  bool metaPublished{false};
};

class WiiBalanceBoard : public Component {
 public:
  WiiBalanceBoard();

  void setup() override;
  void loop() override;
  void dump_config() override;
  void sync(bool enable);

  void set_temperature_sensor(sensor::Sensor *temperature_sensor);
  void set_reference_temperature_sensor(sensor::Sensor *reference_temperature_sensor);
  void set_battery_level(sensor::Sensor *battery_level);
  void set_weight(sensor::Sensor *weight);
  void set_syncing(binary_sensor::BinarySensor *syncing);
  void set_stddev(float stddev);
  void set_led_pin(int led_pin);

  // Balance mode: keep the connection up and stream per-corner loads and derived
  // posture metrics until the user steps off. Scale mode (default): take one stable
  // weight reading and disconnect.
  void set_balance_mode(bool enable);
  bool balance_mode() const { return balance_mode_; }
  bool connected() const { return !sampleMap.empty(); }
  void disconnect_all();
  void set_balance_update_interval(uint32_t ms) { balance_update_interval_ = ms; }
  void set_off_board_timeout(uint32_t ms) { off_board_timeout_ = ms; }

  void set_top_left(sensor::Sensor *s) { top_left_ = s; }
  void set_top_right(sensor::Sensor *s) { top_right_ = s; }
  void set_bottom_left(sensor::Sensor *s) { bottom_left_ = s; }
  void set_bottom_right(sensor::Sensor *s) { bottom_right_ = s; }
  void set_live_weight(sensor::Sensor *s) { live_weight_ = s; }
  void set_left_percent(sensor::Sensor *s) { left_percent_ = s; }
  void set_front_percent(sensor::Sensor *s) { front_percent_ = s; }
  void set_cop_x(sensor::Sensor *s) { cop_x_ = s; }
  void set_cop_y(sensor::Sensor *s) { cop_y_ = s; }
  void set_sway(sensor::Sensor *s) { sway_ = s; }
  void set_on_board(binary_sensor::BinarySensor *s) { on_board_ = s; }

  // High-rate UDP stream of the raw balance data (JSON, one datagram per interval)
  // for a local visualizer, bypassing Home Assistant.
  void set_udp_stream(const std::string &host, uint16_t port, uint32_t interval_ms);

 protected:
  void board_connected(uint16_t handle);
  void board_disconnected(uint16_t handle);
  void board_sample(uint16_t handle, uint8_t battery, uint8_t reference_temp, uint8_t temperature, float topRightLoad,
                    float bottomRightLoad, float topLeftLoad, float bottomLeftLoad);

  detail::Bluetooth bluetooth;
  detail::Wii wii;
  std::unordered_map<uint16_t, Sample> sampleMap;
  detail::TaskQueue queue;

  void balance_sample(uint16_t handle, float tl, float tr, float bl, float br, float total);
  void publish_balance();

  float std_dev_;
  HighFrequencyLoopRequester high_freq_;
  int led_pin_;

  bool balance_mode_{false};
  uint32_t balance_update_interval_{250};
  uint32_t off_board_timeout_{15000};
  uint32_t last_publish_ms_{0};
  uint32_t last_on_board_ms_{0};
  bool on_board_state_{false};

  std::string udp_host_;
  uint16_t udp_port_{0};
  uint32_t udp_interval_{50};
  uint32_t last_udp_ms_{0};
  int udp_sock_{-1};
  void udp_send();
  // Accumulators for the current publish window
  struct BalanceWindow {
    double tl{0}, tr{0}, bl{0}, br{0}, total{0};
    double cx{0}, cy{0}, cx2{0}, cy2{0};
    uint32_t n{0};
    void reset() { *this = BalanceWindow{}; }
  } window_, udp_window_;

  sensor::Sensor *top_left_{nullptr};
  sensor::Sensor *top_right_{nullptr};
  sensor::Sensor *bottom_left_{nullptr};
  sensor::Sensor *bottom_right_{nullptr};
  sensor::Sensor *live_weight_{nullptr};
  sensor::Sensor *left_percent_{nullptr};
  sensor::Sensor *front_percent_{nullptr};
  sensor::Sensor *cop_x_{nullptr};
  sensor::Sensor *cop_y_{nullptr};
  sensor::Sensor *sway_{nullptr};
  binary_sensor::BinarySensor *on_board_{nullptr};

  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *reference_temperature_sensor_{nullptr};
  sensor::Sensor *battery_level_{nullptr};
  sensor::Sensor *weight_{nullptr};
  binary_sensor::BinarySensor *syncing_{nullptr};
};

}  // namespace wii_balance_board
}  // namespace esphome
