#include "bluetooth.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"

#include <esp_bt.h>
#if __has_include(<esp_coexist.h>)
#include <esp_coexist.h>
#endif

#include <cstring>
#include <unordered_map>
#include <unordered_set>

#include "log.h"
#include "lowlevel_bt.h"
#include "ring_buffer.h"
#include <esp_mac.h>

#define CHECK_RESULT(x) \
  if (!x) { \
    ESP_LOGE(TAG, #x " failed!"); \
  }

static const char *TAG = "bluetooth";

static_assert(CONFIG_BT_ENABLED,
              "Bluetooth is not enabled! Please run `make menuconfig` to and enable it");
static_assert(CONFIG_BT_CLASSIC_ENABLED, "Board does not support Bluetooth BR/EDR");

namespace esphome::wii_balance_board::detail {

static uint8_t g_identifier = 1;

struct L2CapConnection {
  uint16_t handle;
  uint16_t localCid;
  uint16_t psm;
  uint16_t remoteCid;
  uint16_t mtu;

  bool localConfigured;
  bool remoteConfigured;
};

class ConnectionStore {
  std::vector<L2CapConnection> l2CapConnections;

 public:
  ConnectionStore() {}
  ConnectionStore(const ConnectionStore &) = delete;
  ConnectionStore &operator=(const ConnectionStore &) = delete;

  L2CapConnection *findLocal(uint16_t handle, uint16_t localCid) {
    auto itr = std::find_if(l2CapConnections.begin(), l2CapConnections.end(),
                            [handle, localCid](const L2CapConnection &connection) {
                              return connection.handle == handle && connection.localCid == localCid;
                            });
    return &*itr;
  }

  L2CapConnection *findPsm(uint16_t handle, uint16_t psm) {
    auto itr = std::find_if(l2CapConnections.begin(), l2CapConnections.end(),
                            [handle, psm](const L2CapConnection &connection) {
                              return connection.handle == handle && connection.psm == psm;
                            });
    return &*itr;
  }

  uint16_t nextCid(uint16_t handle) {
    uint16_t nextCid = 0x0040;
    for (const auto &connection : l2CapConnections) {
      if (handle == connection.handle) {
        nextCid = std::max(nextCid, static_cast<uint16_t>(connection.localCid + 1));
      }
    }
    return nextCid;
  }

  bool remove(L2CapConnection &connection) {
    auto itr = std::remove_if(l2CapConnections.begin(), l2CapConnections.end(),
                              [&connection](const L2CapConnection &e) { return &e == &connection; });
    if (itr == l2CapConnections.end()) {
      return false;
    }
    l2CapConnections.erase(itr, l2CapConnections.end());
    return true;
  }

  bool remove(uint16_t handle) {
    auto cnt = std::erase_if(l2CapConnections, [handle](const L2CapConnection &e) { return e.handle == handle; });
    return cnt > 0;
  }

  void emplace(L2CapConnection connection) { l2CapConnections.emplace_back(std::move(connection)); }
};

struct Bluetooth::Impl {
  Bluetooth *bluetooth;
  std::array<uint8_t, 6> macAddress;
  std::function<void(Bluetooth *)> readyListener;
  std::function<void(Bluetooth *, const HCIEvent &)> hciListener;
  std::function<bool(Bluetooth *, const HCIConnectionRequest &)> connectionRequestListener = [](auto...) {
    return false;
  };

  std::function<bool(Bluetooth *, const ACLConnectionRequest &)> aclConnectionRequestListener = [](auto...) {
    return false;
  };

  std::function<void(Bluetooth *, const ACLEvent &)> aclListener;

  RingBuffer rxBuffer;
  RingBuffer txBuffer;
  ConnectionStore connections;
  bool initialized{false};
  std::unordered_set<uint64_t> discovered;
  std::unordered_set<uint64_t> connectRequests;
  // Devices we connected to from an inquiry (Pair button + red sync button). That is
  // a fresh pairing: always PIN-authenticate so the board records us as its host.
  std::unordered_set<uint64_t> freshPairings;
  std::unordered_map<uint64_t, HCIInquiryResult> nameRequests;

  Impl(Bluetooth *bluetooth) : bluetooth(bluetooth), rxBuffer(4096), txBuffer(2048) {
    esp_read_mac(macAddress.data(), ESP_MAC_BT);
  }

  // Link key from the last successful pairing, persisted so the board can
  // reconnect on its own (power button) after a reboot.
  struct LinkKeyStore {
    uint64_t bdaddr{0};
    uint8_t key[16]{};
  } __attribute__((packed));
  LinkKeyStore linkKey;
  bool linkKeyLoaded{false};
  ESPPreferenceObject linkKeyPref;

  void loadLinkKey() {
    if (linkKeyLoaded) {
      return;
    }
    linkKeyLoaded = true;
    linkKeyPref = global_preferences->make_preference<LinkKeyStore>(fnv1_hash("wii_balance_board_link_key"));
    if (linkKeyPref.load(&linkKey) && linkKey.bdaddr != 0) {
      ESP_LOGI(TAG, "Loaded stored link key for %012llX", (unsigned long long) linkKey.bdaddr);
    } else {
      linkKey.bdaddr = 0;
    }
  }

  void step() {
    while (esp_vhci_host_check_send_available()) {
      if (auto txData = txBuffer.read(0)) {
        // ESP_LOGD(TAG, "TX>: %s", formatHex(txData.data(), txData.size()));
        esp_vhci_host_send_packet(txData.data(), txData.size());
      } else {
        break;
      }
    }

    // Drain everything the controller queued since the last loop; the board streams
    // reports at ~100 Hz and handling one packet per loop lets the buffer fill up.
    for (int drained = 0; drained < 64; drained++) {
      auto rxData = rxBuffer.read(0);
      if (!rxData) {
        break;
      }
      const char *type;
      uint8_t typeColor;
      switch (rxData[0]) {
        case 0x04:
          type = "HCI";
          typeColor = 44;
          handleHCIEvent(rxData[1], rxData.data() + 3, rxData[2]);
          break;
        case 0x02:
          type = "ACL";
          typeColor = 43;
          {
            uint16_t handle = ((rxData[2] & 0x0F) << 8) | rxData[1];
            uint8_t packetBoundaryFlag = (rxData[2] & 0x30) >> 4;  // Packet_Boundary_Flag
            uint8_t broadcastFlag = (rxData[2] & 0xC0) >> 6;       // Broadcast_Flag

            if (packetBoundaryFlag != 0b10) {
              ESP_LOGE(TAG, "unsupported packet_boundary_flag = 0b%02B", packetBoundaryFlag);
              break;
            }

            if (broadcastFlag != 0b00) {
              ESP_LOGE(TAG, "unsupported broadcast_flag 0b%02B", broadcastFlag);
              break;
            }

            uint16_t len = (rxData[6] << 8) | rxData[5];
            uint16_t channelId = (rxData[8] << 8) | rxData[7];
            handleACLEvent(rxData[9], handle, channelId, rxData.data() + 9, len);
          }
          break;
        default:
          type = "ERR";
          typeColor = 41;
          break;
      }

      // ESP_LOGD(TAG, "[%s] RX> %s", type, formatHex(rxData.data(), rxData.size()));
    }
  }

  // HCI
  void handleHCICommandComplete(uint8_t *data, size_t len) {
    if (data[1] == 0x03 && data[2] == 0x0C) {  // reset
      if (data[3] == 0x00) {
        CHECK_RESULT(enqueue_cmd_read_bd_addr(txBuffer));
      } else {
        ESP_LOGE(TAG, "Reset failed");
      }
    } else if (data[1] == 0x09 && data[2] == 0x10) {  // read_bd_addr
      if (data[3] == 0x00) {                          // OK
        char name[] = "ESP32-BT-WIIP";
        CHECK_RESULT(enqueue_cmd_write_local_name(txBuffer, (uint8_t *) name, sizeof(name)));
      } else {
        ESP_LOGE(TAG, "read_bd_addr failed.");
      }
    } else if (data[1] == 0x13 && data[2] == 0x0C) {  // write_local_name
      if (data[3] == 0x00) {                          // OK
        uint8_t cod[3] = {0x04, 0x05, 0x00};
        CHECK_RESULT(enqueue_cmd_write_class_of_device(txBuffer, cod));
      } else {
        ESP_LOGE(TAG, "write_local_name failed.");
      }
    } else if (data[1] == 0x24 && data[2] == 0x0C) {  // write_class_of_device
      if (data[3] == 0x00) {                          // OK
        CHECK_RESULT(enqueue_cmd_write_scan_enable(txBuffer, 3));
      } else {
        ESP_LOGE(TAG, "write_class_of_device failed.");
      }
    } else if (data[1] == 0x1A && data[2] == 0x0C) {  // write_scan_enable
      if (data[3] == 0x00) {                          // OK
        if (initialized) {
          return;  // Re-enable after a disconnect; nothing else to do.
        }
        CHECK_RESULT(enqueue_cmd_write_page_scan_activity(txBuffer, 0x0100, 0x0012));
      } else {
        ESP_LOGE(TAG, "write_scan_enable failed.");
      }
    } else if (data[1] == 0x1C && data[2] == 0x0C) {  // write_page_scan_activity
      if (data[3] != 0x00) {
        ESP_LOGW(TAG, "write_page_scan_activity failed (%02X), keeping defaults", data[3]);
      }
      CHECK_RESULT(enqueue_cmd_write_page_scan_type(txBuffer, 0x01));
    } else if (data[1] == 0x47 && data[2] == 0x0C) {  // write_page_scan_type
      if (data[3] != 0x00) {
        ESP_LOGW(TAG, "write_page_scan_type failed (%02X), keeping default", data[3]);
      }
      initialized = true;
      readyListener(bluetooth);
    }
  }

  void handleHCICommandStatusEvent(uint8_t *data, size_t len) {
    if (data[2] == 0x01 && data[3] == 0x04) {
      if (data[0] == 0x00) {
        hciListener(bluetooth, HCIInquiryStarted{});
      } else {
        log_e("Failed to start inquiry, error=%02X", data[0]);
      }
    }
  }

  void handleHCIInqueryResult(uint8_t *data, size_t len) {
    uint8_t num = data[0];
    for (uint8_t i = 0; i < num; ++i) {
      int pos = 1 + (6 + 1 + 2 + 3 + 2) * i;
      uint64_t bdaddr = *(const uint64_t *) (data + pos) & 0xFFFFFFFFFFFFull;
      uint32_t cod = (data[pos + 9] << 16) | (data[pos + 10] << 8) | data[pos + 11];
      if (discovered.emplace(bdaddr).second) {
        HCIInquiryResult res{
            .bdaddr = bdaddr,
            .psrm = data[pos + 6],
            .classOfDevice = cod,
            .clkOffset = static_cast<uint16_t>(((0x80 | data[pos + 12]) << 8) | (data[pos + 13])),
        };
        hciListener(bluetooth, res);
      }
    }
  }

  void handleHCIInqueryComplete(uint8_t *data, size_t len) {
    hciListener(bluetooth, HCIInquiryComplete{});
    discovered.clear();
  }

  void handleHCIDisconnect(uint8_t *data, size_t len) {
    uint8_t status = data[0];
    uint16_t handle = data[2] << 8 | data[1];
    if (status == 0x00) {
      hciListener(bluetooth, HCIDisconnected{.handle = handle, .reason = data[3]});
      connections.remove(handle);
      // Make sure we stay reachable for a board-initiated reconnect.
      ESP_LOGD(TAG, "Re-enabling inquiry+page scan");
      CHECK_RESULT(enqueue_cmd_write_scan_enable(txBuffer, 3));
    }
  }

  void handleHCIRemoteNameRequestComplete(uint8_t *data, size_t len) {
    uint8_t status = data[0];
    char *name = (char *) (data + 7);
    uint64_t bdaddr = *(const uint64_t *) (data + 1) & 0xFFFFFFFFFFFFull;
    auto &inquiry = nameRequests.at(bdaddr);
    hciListener(bluetooth, HCIRemoteName{.inquiry =
                                             HCIInquiryResult{
                                                 .bdaddr = inquiry.bdaddr,
                                                 .psrm = inquiry.psrm,
                                                 .classOfDevice = inquiry.classOfDevice,
                                                 .clkOffset = inquiry.clkOffset,
                                             },
                                         .remoteName = {name}});
    nameRequests.erase(bdaddr);
  }

  void handleHCIConnectionComplete(uint8_t *data, size_t len) {
    uint8_t status = data[0];
    uint16_t handle = data[2] << 8 | data[1];
    uint64_t bdaddr = *(const uint64_t *) (data + 3) & 0xFFFFFFFFFFFFull;
    if (status == 0x00) {
      hciListener(bluetooth, HCIConnectionEstablished{
                                 .bdaddr = bdaddr, .handle = handle, .accepted = !connectRequests.contains(bdaddr)});
    } else {
      hciListener(
          bluetooth,
          HCIConnectionFailed{
              .bdaddr = bdaddr, .handle = handle, .reason = status, .accepted = !connectRequests.contains(bdaddr)});
    }
    connectRequests.erase(bdaddr);
  }

  void handleHCIConnectionRequest(uint8_t *data, size_t len) {
    uint64_t bdaddr = *(const uint64_t *) (data) &0xFFFFFFFFFFFFull;
    uint32_t cod = (data[6] << 16) | (data[7] << 8) | data[8];
    uint8_t link_type = data[9];

    if (connectionRequestListener(bluetooth, HCIConnectionRequest{.bdaddr = bdaddr, .classOfDevice = cod})) {
      CHECK_RESULT(enqueue_cmd_accept_connection(txBuffer, bdaddr));
    } else {
      CHECK_RESULT(enqueue_cmd_reject_connection(txBuffer, bdaddr, 0x0F));
    }
  }

  void handleHCIPINRequest(uint8_t *data, size_t len) {
    uint64_t bdaddr = *(const uint64_t *) (data) &0xFFFFFFFFFFFFull;

    hciListener(bluetooth, HCIPINRequest{.bdaddr = bdaddr});
  }

  // HCI event 0x17: Link Key Request (bdaddr only). Answer with the stored key if
  // it belongs to this device, otherwise let the listener send a negative reply so
  // the controller falls back to PIN pairing.
  void handleHCILinkKeyRequest(uint8_t *data, size_t len) {
    uint64_t bdaddr = *(const uint64_t *) (data) &0xFFFFFFFFFFFFull;
    loadLinkKey();

    if (freshPairings.contains(bdaddr)) {
      ESP_LOGI(TAG, "Link key request from %012llX during fresh pairing, forcing PIN pairing",
               (unsigned long long) bdaddr);
      CHECK_RESULT(enqueue_cmd_negative_reply(txBuffer, bdaddr));
      return;
    }

    if (linkKey.bdaddr == bdaddr) {
      ESP_LOGI(TAG, "Link key request from %012llX, replying with stored key", (unsigned long long) bdaddr);
      CHECK_RESULT(enqueue_cmd_link_key_reply(txBuffer, bdaddr, linkKey.key));
      return;
    }

    ESP_LOGI(TAG, "Link key request from %012llX, no stored key", (unsigned long long) bdaddr);
    hciListener(bluetooth, HCILinkKeyRequest{
                               .bdaddr = bdaddr,
                               .keyType = 0,
                               .linkKeyData = nullptr,
                               .size = 0,
                           });
  }

  // HCI event 0x18: Link Key Notification (bdaddr 6, key 16, type 1).
  void handleHCILinkKeyNotification(uint8_t *data, size_t len) {
    if (len < 23) {
      ESP_LOGW(TAG, "Short link key notification (%u bytes)", (unsigned) len);
      return;
    }
    uint64_t bdaddr = *(const uint64_t *) (data) &0xFFFFFFFFFFFFull;
    loadLinkKey();
    linkKey.bdaddr = bdaddr;
    memcpy(linkKey.key, data + 6, 16);
    ESP_LOGI(TAG, "Stored link key for %012llX (type %u)", (unsigned long long) bdaddr, data[22]);
    linkKeyPref.save(&linkKey);
    freshPairings.erase(bdaddr);
  }

  void handleHCIEvent(uint8_t eventCode, uint8_t *data, size_t len) {
    ESP_LOGV(TAG, "HCI event 0x%02X len %u", eventCode, (unsigned) len);
    if (eventCode == 0x04 || eventCode == 0x03 || eventCode == 0x05 || eventCode == 0x17 || eventCode == 0x18) {
      ESP_LOGD(TAG, "HCI event 0x%02X (%s)", eventCode,
               eventCode == 0x04   ? "connection request"
               : eventCode == 0x03 ? "connection complete"
               : eventCode == 0x05 ? "disconnection complete"
               : eventCode == 0x17 ? "link key request"
                                   : "link key notification");
    }
    switch (eventCode) {
      case 0x0F:
        handleHCICommandStatusEvent(data, len);
        break;
      case 0x0E:
        handleHCICommandComplete(data, len);
        break;
      case 0x02:
        handleHCIInqueryResult(data, len);
        break;
      case 0x01:
        handleHCIInqueryComplete(data, len);
        break;
      case 0x05:
        handleHCIDisconnect(data, len);
        break;
      case 0x07:
        handleHCIRemoteNameRequestComplete(data, len);
        break;
      case 0x03:
        handleHCIConnectionComplete(data, len);
        break;
      case 0x06: {  // Authentication Complete: status, handle
        uint16_t handle = (data[2] << 8) | data[1];
        if (data[0] == 0x00) {
          // bluez and the Wii both encrypt the link after authenticating; the board
          // appears to only remember a host (for power-button reconnect) when the
          // link was encrypted.
          ESP_LOGI(TAG, "Authentication complete on handle %u, enabling encryption", handle);
          CHECK_RESULT(enqueue_cmd_set_conn_encryption(txBuffer, handle, true));
        } else {
          ESP_LOGW(TAG, "Authentication failed on handle %u (status %02X)", handle, data[0]);
        }
        break;
      }
      case 0x08: {  // Encryption Change: status, handle, enabled
        uint16_t handle = (data[2] << 8) | data[1];
        if (data[0] == 0x00) {
          ESP_LOGI(TAG, "Encryption %s on handle %u", data[3] ? "enabled" : "disabled", handle);
        } else {
          ESP_LOGW(TAG, "Encryption change failed on handle %u (status %02X)", handle, data[0]);
        }
        break;
      }
      case 0x04:
        handleHCIConnectionRequest(data, len);
        break;
      case 0x17:
        handleHCILinkKeyRequest(data, len);
        break;
      case 0x18:
        handleHCILinkKeyNotification(data, len);
        break;
      case 0x16:
        handleHCIPINRequest(data, len);
        break;
    }
  }

  void sendHCIReset() { CHECK_RESULT(enqueue_cmd_reset(txBuffer)); }

  void sendHCIDisconnect(uint16_t handle) { CHECK_RESULT(enqueue_cmd_disconnect(txBuffer, handle)); }

  void sendHCIScan() {
    if (!initialized) {
      ESP_LOGE(TAG, "Cannot sync, bluetooth not initialized (controller status=%d, HCI reset never completed)",
               (int) esp_bt_controller_get_status());
      return;
    }

    uint8_t timeout = 0x10;  // Sync for 20.48 seconds (0x10 * 1.28s)

    CHECK_RESULT(enqueue_cmd_inquiry(txBuffer, 0x9E8B33, timeout, 0x00));
  }

  void sendHCIScanCancel() {
    if (!initialized) {
      ESP_LOGE(TAG, "Cannot sync, bluetooth not initialized");
      return;
    }

    uint8_t timeout = 0x10;  // Sync for 20.48 seconds (0x10 * 1.28s)

    CHECK_RESULT(enqueue_cmd_inquiry_cancel(txBuffer));
  }

  void sendHCIRequestRemoteName(const HCIInquiryResult &result) {
    nameRequests.emplace(result.bdaddr, result);
    CHECK_RESULT(enqueue_cmd_remote_name_request(txBuffer, result.bdaddr, result.psrm, result.clkOffset));
  }

  void sendHCIConnect(const HCIInquiryResult &result) {
    connectRequests.emplace(result.bdaddr);
    freshPairings.emplace(result.bdaddr);
    CHECK_RESULT(enqueue_cmd_create_connection(txBuffer, result.bdaddr, 0x0008, result.psrm, result.clkOffset, 0x00));
  }

  void sendHCINegativeReply(uint64_t bdaddr) { CHECK_RESULT(enqueue_cmd_negative_reply(txBuffer, bdaddr)); }

  void sendHCIPINReply(uint64_t bdaddr, uint8_t *pinData, size_t len) {
    if (len > 16) {
      ESP_LOGE(TAG, "PIN too long, max 16 characters");
      return;
    }
    CHECK_RESULT(enqueue_cmd_pin_reply(txBuffer, bdaddr, pinData, len));
  }

  void sendHCIAuth(uint16_t handle) { CHECK_RESULT(enqueue_cmd_auth_request(txBuffer, handle)); }

  // ACL
  void handleL2ConfigurationRequest(uint16_t handle, uint8_t *data) {
    uint8_t identifier = data[1];
    uint16_t len = (data[3] << 8) | data[2];
    uint16_t destinationCid = (data[5] << 8) | data[4];
    uint16_t flags = (data[7] << 8) | data[6];

    if (flags != 0x0000) {
      ESP_LOGE(TAG, "Unsupported flags %04X", flags);
      return;
    }

    if (len != 0x08) {
      ESP_LOGE(TAG, "Unexpected configuration length %04X", len);
      return;
    }

    L2CapConnection *connection = connections.findLocal(handle, destinationCid);
    if (connection == nullptr) {
      ESP_LOGW(TAG, "Unexpected configuration requestion");
      return;
    }

    if (data[8] == 0x01 && data[9] == 0x02) {  // MTU
      uint16_t mtu = (data[11] << 8) | data[10];
      connection->mtu = mtu;
      uint8_t packetBoundaryFlag = 0b10;  // Packet_Boundary_Flag
      uint8_t broadcastFlag = 0b00;       // Broadcast_Flag
      uint16_t channelId = 0x0001;
      uint16_t sourceCid = connection->remoteCid;
      uint8_t data[] = {
          0x05,        // CONFIGURATION RESPONSE
          identifier,  // Identifier
          0x0A,
          0x00,  // Length: 0x000A
          (uint8_t) (sourceCid & 0xFF),
          (uint8_t) (sourceCid >> 8),  // Source CID
          0x00,
          0x00,  // Flags
          0x00,
          0x00,  // Res
          0x01,
          0x02,
          (uint8_t) (mtu & 0xFF),
          (uint8_t) (mtu >> 8)  // type=01 len=02 value=xx xx
      };

      uint16_t dataLen = 14;
      CHECK_RESULT(enqueue_acl_l2cap_single_packet(txBuffer, handle, packetBoundaryFlag, broadcastFlag, channelId, data,
                                                   dataLen));
      connection->remoteConfigured = true;
      if (connection->remoteConfigured && connection->localConfigured) {
        aclListener(bluetooth, ACLConnectionEstablished{
                                   .handle = handle,
                                   .sourceCid = sourceCid,
                                   .psm = connection->psm,
                               });
      }
    }
  }

  void handleL2DisconnectRequest(uint16_t handle, uint8_t *data) {
    uint8_t identifier = data[1];
    uint16_t destinationCid = (data[5] << 8) | data[4];
    uint16_t sourceCid = (data[7] << 8) | data[6];
    uint32_t key = handle << 16 | destinationCid;

    L2CapConnection *connection = connections.findLocal(handle, destinationCid);
    if (connection == nullptr) {
      // Send command reject rsp
      return;
    }
    ESP_LOGD(TAG, "Sending disconnect response");
    if (connection->remoteCid == sourceCid) {
      uint8_t response[] = {
          0x07,        // Disconnect response
          identifier,  // Identifier
          0x04,
          0x00,  // Length: 0x0004
          (uint8_t) (connection->localCid & 0xFF),
          (uint8_t) (connection->localCid >> 8),  // Destination CID
          (uint8_t) (connection->remoteCid & 0xFF),
          (uint8_t) (connection->remoteCid >> 8),  // Source CID
      };

      sendL2DataChannel(handle, 0x0001, response, 8);
      connections.remove(*connection);
    } else {
      ESP_LOGD(TAG, "Mismatch");
    }
  }

  void handleL2ConnectionResponse(uint16_t handle, uint8_t *data) {
    uint8_t identifier = data[1];
    uint16_t len = (data[3] << 8) | data[2];
    uint16_t destinationCid = (data[5] << 8) | data[4];
    uint16_t sourceCid = (data[7] << 8) | data[6];
    uint16_t result = (data[9] << 8) | data[8];
    uint16_t status = (data[11] << 8) | data[10];

    auto *connection = connections.findLocal(handle, sourceCid);
    if (connection == nullptr) {
      ESP_LOGW(TAG, "Received unexpected L2Cap Connection response, ignoring");
      return;
    }

    if (result == 0x0000) {  // Connection established, initiate configuration
      connection->remoteCid = destinationCid;
      sendL2Configure(handle, destinationCid, connection->mtu);
    } else if (result >= 0x0002) {  // Connection failed
      aclListener(bluetooth, ACLConnectionFailed{
                                 .handle = handle,
                                 .sourceCid = sourceCid,
                                 .psm = connection->psm,
                             });
      connections.remove(*connection);
    }
  }

  void handleL2ConfigurationResponse(uint16_t handle, uint8_t *data) {
    uint16_t sourceCid = (data[5] << 8) | data[4];
    auto *connection = connections.findLocal(handle, sourceCid);

    connection->localConfigured = true;
    if (connection && connection->localConfigured && connection->remoteConfigured) {
      aclListener(bluetooth, ACLConnectionEstablished{
                                 .handle = handle,
                                 .sourceCid = sourceCid,
                                 .psm = connection->psm,
                             });
    }
  }

  void handleL2DisconnectResponse(uint16_t handle, uint8_t *data) {
    uint16_t sourceCid = (data[7] << 8) | data[6];
    auto *connection = connections.findLocal(handle, sourceCid);
    if (connection) {
      aclListener(bluetooth, ACLDisconnected{
                                 .handle = handle,
                                 .psm = connection->psm,
                             });
      connections.remove(*connection);
    }
  }

  void handleL2ConnectionRequest(uint16_t handle, uint8_t *data) {
    uint16_t sourceCid = (data[7] << 8) | data[6];
    uint16_t psm = (data[5] << 8) | data[4];
    bool accepted = aclConnectionRequestListener(
        bluetooth, ACLConnectionRequest{.handle = handle, .sourceCid = sourceCid, .psm = psm});
    auto localCid = connections.nextCid(handle);
    if (accepted) {
      connections.emplace(L2CapConnection{
          .handle = handle,
          .localCid = localCid,
          .psm = psm,
          .remoteCid = sourceCid,
          .mtu = 0x00B9,
          .localConfigured = false,
          .remoteConfigured = false,
      });
    }
    uint16_t result = accepted ? 0x00 : 0x04;  // Connection refused if idx == -1.
    uint8_t response[] = {
        0x03,
        data[1],  // Request identifier
        0x08,
        0x00,
        (uint8_t) (localCid & 0xFF),
        (uint8_t) (localCid >> 8),
        (uint8_t) (sourceCid & 0xFF),
        (uint8_t) (sourceCid >> 8),
        (uint8_t) (result & 0xFF),
        (uint8_t) (result >> 8),
        0x00,
        0x00,  // No status
    };
    sendL2DataChannel(handle, 0x0001, response, 12);
    if (accepted) {  // Send config request
      sendL2Configure(handle, sourceCid, 0x00B9);
    }
  }

  void handleACLEvent(uint8_t event, uint16_t handle, uint16_t channelId, uint8_t *data, size_t len) {
    switch (event) {
      case 0x02:
        handleL2ConnectionRequest(handle, data);
        break;
      case 0x03:
        handleL2ConnectionResponse(handle, data);
        break;
      case 0x04:
        handleL2ConfigurationRequest(handle, data);
        break;
      case 0x05:
        handleL2ConfigurationResponse(handle, data);
        break;
      case 0x06:
        handleL2DisconnectRequest(handle, data);
        break;
      case 0x07:
        handleL2DisconnectResponse(handle, data);
        break;
      default:
        aclListener(bluetooth, ACLData{
                                   .handle = handle,
                                   .channelId = channelId,
                                   .data = data,
                                   .len = len,
                               });
        break;
    }
  }

  void sendL2Configure(uint16_t handle, uint16_t destinationCid, uint16_t mtu) {
    uint8_t data[] = {
        0x04,            // CONFIGURATION REQUEST
        g_identifier++,  // Identifier
        0x08,
        0x00,  // Length: 0x0008
        (uint8_t) (destinationCid & 0xFF),
        (uint8_t) (destinationCid >> 8),  // Destination CID
        0x00,
        0x00,  // Flags
        0x01,
        0x02,
        (uint8_t) (mtu & 0xFF),
        (uint8_t) (mtu >> 8)  // type=01 len=02 value=2 bytes mtu
    };

    sendL2DataChannel(handle, 0x0001, data, 12);
  }

  void sendL2Connect(uint16_t connection_handle, uint16_t psm, uint16_t mtu) {
    uint16_t localCid = connections.nextCid(connection_handle);
    uint8_t data[] = {0x02,          // CONNECTION REQUEST
                      g_identifier,  // Identifier
                      0x04,
                      0x00,  // Length:     0x0004
                      (uint8_t) (psm & 0xFF),
                      (uint8_t) (psm >> 8),
                      (uint8_t) (localCid & 0xFF),
                      (uint8_t) (localCid >> 8)};
    uint16_t data_len = 8;

    sendL2DataChannel(connection_handle, 0x0001, data, 8);

    connections.emplace(L2CapConnection{
        .handle = connection_handle,
        .localCid = localCid,
        .psm = psm,
        .remoteCid = 0,
        .mtu = mtu,
        .localConfigured = false,
        .remoteConfigured = false,
    });
    g_identifier++;
  }

  void sendL2Data(uint16_t handle, uint16_t psm, uint8_t *data, size_t len) {
    uint32_t hp = (handle << 16) | psm;
    auto *connection = connections.findPsm(handle, psm);
    if (connection == nullptr) {
      ESP_LOGE(TAG, "Cannot send L2 data, handle/psm connection not found");
      return;
    }
    sendL2DataChannel(handle, connection->remoteCid, data, len);
  }

  void sendL2Disconnect(uint16_t handle, uint16_t psm) {
    auto *connection = connections.findPsm(handle, psm);
    if (connection) {
      uint8_t data[] = {
          0x06,            // Disconnect REQUEST
          g_identifier++,  // Identifier
          0x04,
          0x00,  // Length: 0x0004
          (uint8_t) (connection->remoteCid & 0xFF),
          (uint8_t) (connection->remoteCid >> 8),
          (uint8_t) (connection->localCid & 0xFF),
          (uint8_t) (connection->localCid >> 8),
      };

      sendL2DataChannel(handle, 0x0001, data, 8);
    }
  }

  void sendL2DataChannel(uint16_t handle, uint16_t channelId, uint8_t *data, size_t len) {
    uint8_t packetBoundaryFlag = 0b10;  // Packet_Boundary_Flag
    uint8_t broadcastFlag = 0b00;       // Broadcast_Flag

    CHECK_RESULT(
        enqueue_acl_l2cap_single_packet(txBuffer, handle, packetBoundaryFlag, broadcastFlag, channelId, data, len));
  }
};

std::function<int(uint8_t *data, size_t len)> gListener;

static void sendReady() {}

static int recv(uint8_t *data, uint16_t len) { return gListener(data, len); }

static const esp_vhci_host_callback_t callback = {sendReady, recv};

// Arduino's initArduino() releases the Classic BT controller memory at boot unless
// btClassicInUse()/btInUse() (weak symbols in esp32-hal-bt.c) return true. Provide
// strong definitions so the memory is kept; otherwise esp_bt_controller_init()
// fails with ESP_ERR_INVALID_STATE.
extern "C" bool btClassicInUse(void) { return true; }
extern "C" bool btInUse(void) { return true; }

// Start the BT controller directly via ESP-IDF instead of Arduino's btStart(),
// which silently returns false when the Arduino core is built without its BT shim.
static bool start_bt_controller() {
  esp_bt_controller_status_t status = esp_bt_controller_get_status();
  if (status == ESP_BT_CONTROLLER_STATUS_ENABLED) {
    return true;
  }
  if (status == ESP_BT_CONTROLLER_STATUS_IDLE) {
    esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_bt_controller_init(&cfg);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "esp_bt_controller_init failed: %s", esp_err_to_name(err));
      return false;
    }
  }
  esp_err_t err = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_bt_controller_enable(CLASSIC_BT) failed: %s", esp_err_to_name(err));
    return false;
  }
  ESP_LOGI(TAG, "BT controller enabled (Classic BR/EDR)");
#if __has_include(<esp_coexist.h>)
  // WiFi stays connected the whole time; make sure BT page scan gets air time.
  esp_coex_preference_set(ESP_COEX_PREFER_BT);
#endif
  return true;
}

Bluetooth::Bluetooth() : m_impl(std::make_unique<Bluetooth::Impl>(this)) {
  if (!start_bt_controller()) {
    ESP_LOGE(TAG, "Failed to initialize Bluetooth");
    return;
  }

  auto *impl = m_impl.get();
  gListener = [impl](uint8_t *data, size_t len) {
    // Runs on the BT controller task. Never block forever here: if the loop task is
    // waiting in esp_vhci_host_send_packet() while this waits for buffer space, both
    // deadlock and the task watchdog fires.
    if (auto buffer = impl->rxBuffer.allocate(len, 20)) {
      memcpy(buffer.data(), data, len);
      return ESP_OK;
    }
    ESP_LOGW(TAG, "Buffer error, dropping packets.");
    return ESP_OK;
  };

  esp_vhci_host_register_callback(&callback);
  m_impl->sendHCIReset();
}

Bluetooth::~Bluetooth() { ESP_LOGD(TAG, "Shut down"); }

void Bluetooth::onReady(const std::function<void(Bluetooth *)> &listener) { m_impl->readyListener = listener; }

void Bluetooth::process() { m_impl->step(); }

// HCI
void Bluetooth::onHCIEvent(const std::function<void(Bluetooth *, const HCIEvent &)> &listener) {
  m_impl->hciListener = listener;
}

void Bluetooth::onHCIConnectionRequest(const std::function<bool(Bluetooth *, const HCIConnectionRequest &)> &listener) {
  m_impl->connectionRequestListener = listener;
}

void Bluetooth::requestRemoteName(const HCIInquiryResult &result) { m_impl->sendHCIRequestRemoteName(result); }

void Bluetooth::connect(const HCIInquiryResult &result) { m_impl->sendHCIConnect(result); }

void Bluetooth::scan(bool enable) {
  if (enable) {
    m_impl->sendHCIScan();
  } else {
    m_impl->sendHCIScanCancel();
  }
}

void Bluetooth::disconnect(uint16_t handle) { m_impl->sendHCIDisconnect(handle); }

// ACL
void Bluetooth::l2cap_connect(uint16_t handle, uint16_t psm, uint16_t mtu) { m_impl->sendL2Connect(handle, psm, mtu); }

void Bluetooth::onACLEvent(const std::function<void(Bluetooth *, const ACLEvent &)> &listener) {
  m_impl->aclListener = listener;
}

void Bluetooth::auth(uint16_t handle) { m_impl->sendHCIAuth(handle); }

void Bluetooth::negativeReply(uint64_t bdaddr) { m_impl->sendHCINegativeReply(bdaddr); }

void Bluetooth::sendPinReply(uint64_t bdaddr, uint8_t *pinData, size_t len) {
  m_impl->sendHCIPINReply(bdaddr, pinData, len);
}

void Bluetooth::onACLConnectionRequest(const std::function<bool(Bluetooth *, const ACLConnectionRequest &)> &listener) {
  m_impl->aclConnectionRequestListener = listener;
}

void Bluetooth::l2cap_disconnect(uint16_t handle, uint16_t psm) { m_impl->sendL2Disconnect(handle, psm); }

void Bluetooth::l2send_data(uint16_t handle, uint16_t psm, uint8_t *data, size_t len) {
  m_impl->sendL2Data(handle, psm, data, len);
}

std::span<uint8_t, 6> Bluetooth::macAddress() { return m_impl->macAddress; }

}  // namespace esphome::wii_balance_board::detail
