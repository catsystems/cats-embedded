/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "task.hpp"
#include "task_buzzer.hpp"
#include "task_state_est.hpp"

namespace task {

class Telemetry final : public Task<Telemetry, 1024> {
 public:
  explicit Telemetry(const StateEstimation* task_state_estimation, const Buzzer& task_buzzer)
      : m_testing_enabled{global_cats_config.enable_testing_mode},
        m_task_state_estimation{task_state_estimation},
        m_task_buzzer{task_buzzer} {}

 private:
  [[noreturn]] void Run() noexcept override;

  struct packed_tx_msg_t {
    uint8_t state : 3;
    uint16_t timestamp : 15;
    uint8_t errors : 6;
    int32_t lat : 22;
    int32_t lon : 22;
    int32_t altitude : 17;
    int16_t velocity : 10;
    uint16_t voltage : 8;
    uint16_t pyro_continuity : 2;
    bool testing_on : 1;
    // fill up to 16 bytes
    uint8_t : 0;  // sent
    uint8_t d1;   // dummy
  } __attribute__((packed));

  struct packed_rx_msg_t {
    /* Header used to check if the packet is used for arming */
    uint8_t header;
    /* Testing passcode, only if this matches with the configured passcode is the packet accepted. */
    uint32_t passcode;
    /* Event which needs to be triggered */
    uint8_t event;
    /* If this bit is set to one, the flight computer arms itself for testing; only then can events be triggered */
    bool enable_testing_telemetry;
    uint32_t dummy1;
    uint32_t dummy2;
  } __attribute__((packed));

  uint32_t m_test_phrase_crc = 0;
  /* used to notify the groundstation that the flight computer is in testing mode */
  bool m_testing_enabled;
  /* used to notify the groundstation if the testing is armed */
  bool m_testing_armed = false;
  /* used to check if the current groundstation event was already triggered and that we are waiting for the
   * groundstation to reset the event */
  bool m_event_reset = false;
  /* timeout to check if data is received from the groundstation for the testing mode */
  uint32_t m_testing_timeout = 0;
  static constexpr uint32_t RX_PACKET_HEADER{0x72}; /* Random Header */

  void PackTxMessage(uint32_t ts, gnss_data_t* gnss, packed_tx_msg_t* tx_payload,
                     estimation_output_t estimation_data) const noexcept;
  void ParseRxMessage(packed_rx_msg_t* rx_payload) noexcept;
  bool Parse(uint8_t op_code, const uint8_t* buffer, uint32_t length, gnss_data_t* gnss) noexcept;
  static void SendLinkPhrase() noexcept;
  static void SendSettings(uint8_t command, uint8_t value) noexcept;
  static void SendEnable() noexcept;
  static void SendDisable() noexcept;
  static void SendTxPayload(uint8_t* payload, uint32_t length) noexcept;
  [[nodiscard]] static bool CheckValidOpCode(uint8_t op_code) noexcept;
  static void RequestVersionNum() noexcept;

  const StateEstimation* m_task_state_estimation = nullptr;
  const Buzzer& m_task_buzzer;

  float32_t m_amplifier_temperature{0.0F};

  static constexpr float32_t k_amplifier_hot_limit{60.F};
};

}  // namespace task
