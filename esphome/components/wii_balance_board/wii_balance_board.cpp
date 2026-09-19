#include "esphome/core/log.h"
#include "wii_balance_board.h"

#include "esphome/core/application.h"

#include <numeric>
#include <lwip/sockets.h>
#include <lwip/inet.h>
#include <errno.h>
#include <algorithm>
#include <cmath>
#include "utils.h"

namespace esphome {
namespace wii_balance_board {

static const char *TAG = "wii_balance_board.component";

uint8_t interpret_battery_level(uint8_t batteryLevel) {
  if (batteryLevel >= 0x8d) {
    return 100;
  } else if (batteryLevel >= 0x7d) {
    return 75;
  } else if (batteryLevel >= 0x78) {
    return 50;
  } else if (batteryLevel >= 0x6A) {
    return 25;
  } else {
    return 0;
  }
}

WiiBalanceBoard::WiiBalanceBoard() : wii(&bluetooth), std_dev_(0.4) {}

void WiiBalanceBoard::board_connected(uint16_t handle) {
  ESP_LOGI(TAG, "Connected board, scheduling disconnect in max 60 seconds");
  // wiimote->set_led(handle, 1);

  if (sampleMap.count(handle) > 0) {
    ESP_LOGE(TAG, "Same handle connected twice, ignoring connection.");
  } else {
    // Queue sampling timeout
    sampleMap.emplace(handle, Sample());
    high_freq_.start();

    if (balance_mode_) {
      ESP_LOGI(TAG, "Balance mode: streaming until off-board for %u ms", (unsigned) off_board_timeout_);
      last_on_board_ms_ = millis();
      last_publish_ms_ = millis();
      window_.reset();
    } else {
      // Schedule timeout disconnect
      queue.add(handle, millis() + 60000, [this](int handle) {
        ESP_LOGI(TAG, "Scheduled disconnect.");
        wii.disconnect(handle, 0x0011);
        wii.disconnect(handle, 0x0013);
      });
    }
  }
}

void WiiBalanceBoard::set_balance_mode(bool enable) {
  if (enable == balance_mode_) {
    return;
  }
  balance_mode_ = enable;
  ESP_LOGI(TAG, "Balance mode %s", enable ? "ON" : "OFF");
  for (auto &kv : sampleMap) {
    uint16_t handle = kv.first;
    if (enable) {
      queue.cancel(handle);
      kv.second.measurement = NAN;
      last_on_board_ms_ = millis();
      last_publish_ms_ = millis();
      window_.reset();
    } else {
      kv.second.measurement = NAN;
      kv.second.sample_count = 0;
      for (auto &v : kv.second.samples) {
        v = NAN;
      }
      queue.add(handle, millis() + 60000, [this](int handle) {
        ESP_LOGI(TAG, "Scheduled disconnect.");
        wii.disconnect(handle, 0x0011);
        wii.disconnect(handle, 0x0013);
      });
    }
  }
  if (!enable && on_board_ != nullptr) {
    on_board_->publish_state(false);
  }
}

void WiiBalanceBoard::disconnect_all() {
  for (auto &kv : sampleMap) {
    ESP_LOGI(TAG, "Disconnecting board %u", kv.first);
    queue.cancel(kv.first);
    wii.disconnect(kv.first, 0x0011);
    wii.disconnect(kv.first, 0x0013);
  }
}

// Board is 43.3 x 23.8 cm; load cells sit close to the corners. Center of pressure
// is expressed in cm from the board center, +x = right, +y = front (toes).
static constexpr float BOARD_HALF_WIDTH_CM = 21.5f;
static constexpr float BOARD_HALF_DEPTH_CM = 11.9f;

void WiiBalanceBoard::balance_sample(uint16_t handle, float tl, float tr, float bl, float br, float total) {
  uint32_t now = millis();
  bool on = total > 10.0f;
  if (on) {
    last_on_board_ms_ = now;
  }
  if (on != on_board_state_) {
    on_board_state_ = on;
    if (on_board_ != nullptr) {
      on_board_->publish_state(on);
    }
  }

  if (on) {
    float left = tl + bl, right = tr + br, front = tl + tr, back = bl + br;
    float cx = (right - left) / total * BOARD_HALF_WIDTH_CM;
    float cy = (front - back) / total * BOARD_HALF_DEPTH_CM;
    window_.tl += tl;
    window_.tr += tr;
    window_.bl += bl;
    window_.br += br;
    window_.total += total;
    window_.cx += cx;
    window_.cy += cy;
    window_.cx2 += cx * cx;
    window_.cy2 += cy * cy;
    window_.n++;

    udp_window_.tl += tl;
    udp_window_.tr += tr;
    udp_window_.bl += bl;
    udp_window_.br += br;
    udp_window_.total += total;
    udp_window_.cx += cx;
    udp_window_.cy += cy;
    udp_window_.n++;
  }

  if (udp_port_ != 0 && now - last_udp_ms_ >= udp_interval_) {
    last_udp_ms_ = now;
    udp_send();
  }

  if (now - last_publish_ms_ >= balance_update_interval_) {
    last_publish_ms_ = now;
    publish_balance();
  }

  if (!on && now - last_on_board_ms_ > off_board_timeout_) {
    ESP_LOGI(TAG, "Off board for %u ms, disconnecting", (unsigned) off_board_timeout_);
    last_on_board_ms_ = now;  // avoid re-triggering while the disconnect is in flight
    wii.disconnect(handle, 0x0011);
    wii.disconnect(handle, 0x0013);
  }
}

void WiiBalanceBoard::set_udp_stream(const std::string &host, uint16_t port, uint32_t interval_ms) {
  udp_host_ = host;
  udp_port_ = port;
  udp_interval_ = interval_ms;
}

void WiiBalanceBoard::udp_send() {
  if (udp_host_.empty() || udp_port_ == 0) {
    return;
  }
  if (udp_sock_ < 0) {
    // Created lazily: the network stack is not initialised yet during setup().
    udp_sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_sock_ < 0) {
      ESP_LOGW(TAG, "Could not create UDP stream socket");
      udp_port_ = 0;  // don't retry every sample
      return;
    }
  }
  char buf[224];
  bool on = udp_window_.n > 0;
  double n = on ? udp_window_.n : 1;
  int len = snprintf(buf, sizeof(buf),
                     "{\"t\":%u,\"on\":%d,\"w\":%.2f,\"tl\":%.2f,\"tr\":%.2f,\"bl\":%.2f,\"br\":%.2f,\"x\":%.2f,\"y\":%.2f}",
                     (unsigned) millis(), on ? 1 : 0, udp_window_.total / n, udp_window_.tl / n, udp_window_.tr / n,
                     udp_window_.bl / n, udp_window_.br / n, udp_window_.cx / n, udp_window_.cy / n);
  udp_window_.reset();
  if (len <= 0) {
    return;
  }
  struct sockaddr_in dest {};
  dest.sin_family = AF_INET;
  dest.sin_port = htons(udp_port_);
  dest.sin_addr.s_addr = inet_addr(udp_host_.c_str());
  int sent = sendto(udp_sock_, buf, len, 0, (struct sockaddr *) &dest, sizeof(dest));
  static uint32_t sent_count = 0;
  if (sent < 0) {
    if ((sent_count++ % 250) == 0) {
      ESP_LOGW(TAG, "UDP sendto failed: errno %d", errno);
    }
  } else if ((sent_count++ % 250) == 0) {
    ESP_LOGD(TAG, "UDP stream ok (%u packets sent, last %d bytes -> %s:%u)", (unsigned) sent_count, sent,
             udp_host_.c_str(), udp_port_);
  }
}

void WiiBalanceBoard::publish_balance() {
  auto pub = [](sensor::Sensor *s, float v) {
    if (s != nullptr) {
      s->publish_state(v);
    }
  };
  if (window_.n == 0) {
    // Nobody on the board: report zeros so the UI can show an empty board.
    pub(top_left_, 0);
    pub(top_right_, 0);
    pub(bottom_left_, 0);
    pub(bottom_right_, 0);
    pub(live_weight_, 0);
    return;
  }
  double n = window_.n;
  float tl = window_.tl / n, tr = window_.tr / n, bl = window_.bl / n, br = window_.br / n;
  float total = window_.total / n;
  float mcx = window_.cx / n, mcy = window_.cy / n;
  float vx = window_.cx2 / n - mcx * mcx, vy = window_.cy2 / n - mcy * mcy;
  float sway = std::sqrt(std::max(0.0f, vx) + std::max(0.0f, vy));

  pub(top_left_, tl);
  pub(top_right_, tr);
  pub(bottom_left_, bl);
  pub(bottom_right_, br);
  pub(live_weight_, total);
  pub(left_percent_, (tl + bl) / total * 100.0f);
  pub(front_percent_, (tl + tr) / total * 100.0f);
  pub(cop_x_, mcx);
  pub(cop_y_, mcy);
  pub(sway_, sway);
  window_.reset();
}

void WiiBalanceBoard::board_disconnected(uint16_t handle) {
  ESP_LOGI(TAG, "Board disconnected, uploaded sampled data.");
  if (sampleMap.count(handle) > 0) {
    auto &sample = sampleMap[handle];
    if (sample.referenceTemperature > 0) {
      reference_temperature_sensor_->publish_state(sample.referenceTemperature);
      temperature_sensor_->publish_state(sample.temperature);
      battery_level_->publish_state(sample.battery);
    }
    if (!isnan(sample.measurement)) {
      weight_->publish_state(sample.measurement);
    }
    sampleMap.erase(handle);
  }
  if (sampleMap.empty()) {
    high_freq_.stop();
    if (on_board_state_) {
      on_board_state_ = false;
      if (on_board_ != nullptr) {
        on_board_->publish_state(false);
      }
    }
    window_.reset();
    publish_balance();
    udp_window_.reset();
    udp_send();
  }
}

void WiiBalanceBoard::board_sample(uint16_t handle, uint8_t battery, uint8_t reference_temp, uint8_t temperature,
                                   float topRightLoad, float bottomRightLoad, float topLeftLoad, float bottomLeftLoad) {
  auto it = sampleMap.find(handle);
  if (it == sampleMap.end()) {
    return;
  }
  Sample &sample = it->second;

  // Ignore zero data
  if (reference_temp == 0) {
    return;
  }

  sample.referenceTemperature = reference_temp;
  sample.battery = battery;
  sample.temperature = temperature;

  float tempFactor = (.999 * (1.0 - .0007 * (sample.temperature - sample.referenceTemperature)));
  float totalWeight = (topRightLoad + bottomRightLoad + topLeftLoad + bottomLeftLoad) / 1000;
  float adjusted = totalWeight * tempFactor;

  // Battery and temperatures come with every report; publish them once per session
  // so they are populated in balance mode too (scale mode also publishes at the end).
  if (!sample.metaPublished) {
    sample.metaPublished = true;
    if (reference_temperature_sensor_ != nullptr) reference_temperature_sensor_->publish_state(reference_temp);
    if (temperature_sensor_ != nullptr) temperature_sensor_->publish_state(temperature);
    if (battery_level_ != nullptr) battery_level_->publish_state(battery);
  }

  if (balance_mode_) {
    // Balance mode: only the live/posture sensors update. The Weight sensor is
    // reserved for scale mode so half-on / one-foot readings never pollute it.
    balance_sample(handle, topLeftLoad / 1000 * tempFactor, topRightLoad / 1000 * tempFactor,
                   bottomLeftLoad / 1000 * tempFactor, bottomRightLoad / 1000 * tempFactor, adjusted);
    return;
  }

  if (!isnan(sample.measurement)) {
    return;
  }

  // Ignore small samples (noise), in std dev calculation.
  if (adjusted < 10) {
    return;
  }

  int size = 64;
  sample.samples[sample.sample_count] = adjusted;
  sample.sample_count = (sample.sample_count + 1) % size;

  // Not enough samples yet
  if (isnan(sample.samples[size - 1])) {
    return;
  }

  // For every 16th data point, sample standard deviation.
  if (sample.sample_count % 16 == 0) {
    float mean = 0;
    for (size_t i = 0; i < size; ++i) {
      mean += sample.samples[i];
    }
    mean /= size;

    float variance = std::accumulate(sample.samples, sample.samples + size, 0.0,
                                     [&mean, &size](float accumulator, const float &val) {
                                       return accumulator + ((val - mean) * (val - mean) / (size - 1));
                                     });

    float deviation = std::sqrt(variance);

    if (mean > 10 && deviation < std_dev_) {  // Ignore all means below 10kg.
      sample.measurement = mean;

      // We have a valid sample, schedule board disconnect.
      ESP_LOGD(TAG, "Sample valid, disconnecting");
      queue.reschedule(handle, millis() + 100);
    }
  }
}

void WiiBalanceBoard::setup() {
  if (!udp_host_.empty() && udp_port_ != 0) {
    ESP_LOGI(TAG, "UDP balance stream -> %s:%u every %u ms", udp_host_.c_str(), udp_port_, (unsigned) udp_interval_);
  }
  if (led_pin_ >= 0) {
    pinMode(led_pin_, OUTPUT);
    digitalWrite(led_pin_, HIGH);
  }
  bluetooth.onReady([](auto) { ESP_LOGI(TAG, "Bluetooth initialized"); });

  wii.onEvent([this](const detail::WiiEvent &event) {
    std::visit(overloaded{
                   [this](const detail::ScanStarted &) {
                     syncing_->publish_state(true);
                     if (led_pin_ >= 0) {
                       digitalWrite(led_pin_, LOW);
                     }
                   },
                   [this](const detail::ScanStopped &) {
                     syncing_->publish_state(false);
                     if (led_pin_ >= 0) {
                       digitalWrite(led_pin_, HIGH);
                     }
                   },
                   [this](const detail::BalanceBoardConnected &board) {
                     syncing_->publish_state(false);
                     if (led_pin_ >= 0) {
                       digitalWrite(led_pin_, HIGH);
                     }
                     sync(false);
                     this->board_connected(board.handle);
                   },
                   [this](const detail::BalanceBoardDisconnected &board) { this->board_disconnected(board.handle); },
                   [this](const detail::BalanceBoardData &data) {
                     this->board_sample(data.handle, interpret_battery_level(data.batteryLevel),
                                        data.referenceTemperature, data.temperature, data.tr, data.br, data.tl,
                                        data.bl);
                   },
               },
               event);
  });
}

void WiiBalanceBoard::loop() {
  wii.step();
  queue.process(millis());
}

void WiiBalanceBoard::sync(bool enable) {
  ESP_LOGI(TAG, enable ? "Starting scan" : "Stopping scan");
  wii.sync(enable);
}

void WiiBalanceBoard::dump_config() { ESP_LOGCONFIG(TAG, "Wii Balance Board"); }

void WiiBalanceBoard::set_temperature_sensor(sensor::Sensor *temperature_sensor) {
  temperature_sensor_ = temperature_sensor;
}
void WiiBalanceBoard::set_reference_temperature_sensor(sensor::Sensor *reference_temperature_sensor) {
  reference_temperature_sensor_ = reference_temperature_sensor;
}
void WiiBalanceBoard::set_battery_level(sensor::Sensor *battery_level) { battery_level_ = battery_level; }
void WiiBalanceBoard::set_weight(sensor::Sensor *weight) { weight_ = weight; }
void WiiBalanceBoard::set_stddev(float stddev) { this->std_dev_ = stddev; }
void WiiBalanceBoard::set_led_pin(int led_pin) { this->led_pin_ = led_pin; }
void WiiBalanceBoard::set_syncing(binary_sensor::BinarySensor *syncing) { this->syncing_ = syncing; }

}  // namespace wii_balance_board
}  // namespace esphome
