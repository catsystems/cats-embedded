/*
 * CATS Flight Software
 * Copyright (C) 2022 Control and Telemetry Systems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "task.h"

namespace task {

class Telemetry final : public Task<Telemetry, 1024> {
 public:
  friend class Task<Telemetry, 1024>;

  [[noreturn]] void Run() noexcept override;

 private:
  struct packed_tx_msg_t {
    uint8_t state : 3;
    uint8_t errors : 4;
    uint16_t timestamp : 15;
    int32_t lat : 22;
    int32_t lon : 22;
    int32_t altitude : 17;
    int16_t velocity : 10;
    uint16_t voltage : 8;
    uint16_t continuity : 3;
    // fill up to 16 bytes
    uint8_t : 0;  // sent
    uint8_t d1;   // dummy
    uint8_t d2;   // dummy
    uint8_t d3;   // dummy
  } __attribute__((packed));

  void PackTxMessage(uint32_t ts, gnss_data_t* gnss, packed_tx_msg_t* tx_payload,
                     estimation_output_t estimation_data) const noexcept;
  void ParseTxMessage(packed_tx_msg_t* rx_payload) const noexcept;
  bool Parse(uint8_t op_code, const uint8_t* buffer, uint32_t length, gnss_data_t* gnss) const noexcept;
  void SendLinkPhrase(uint8_t* phrase, uint32_t length) const noexcept;
  void SendSettings(uint8_t command, uint8_t value) const noexcept;
  void SendEnable() const noexcept;
  void SendDisable() const noexcept;
  void SendTxPayload(uint8_t* payload, uint32_t length) const noexcept;
  [[nodiscard]] bool CheckValidOpCode(uint8_t op_code) const noexcept;

  Telemetry() = default;
};
}  // namespace task
