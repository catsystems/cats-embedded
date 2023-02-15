
#include "Parser.h"
#include "FHSS/crc.h"
#include "common.h"

void Parser::parse() {

  cmd_table[opCodeIndex].cmd(&buffer[2], dataIndex);

  /* Reset the parser buffer */
  reset();
}

int32_t Parser::getOpCodeIndex(uint8_t opCode) {
  for (int32_t i = 0; i < CMD_NUMBER; i++) {
    if (opCode == cmd_table[i].identifier)
      return i;
  }
  return -1;
}

void Parser::process(uint8_t ch) {
  switch (state) {
  case STATE_OP:
    opCodeIndex = getOpCodeIndex(ch);
    if (opCodeIndex >= 0) {
      buffer[INDEX_OP] = ch;
      state = STATE_LEN;
    }
    break;
  case STATE_LEN:
    if (ch <= 16) {
      buffer[INDEX_LEN] = ch;
      if (ch > 0) {
        state = STATE_DATA;
      } else {
        state = STATE_CRC;
      }
    }
    break;
  case STATE_DATA:
    if ((buffer[INDEX_LEN] - dataIndex) > 0) {
      buffer[dataIndex + 2] = ch;
      dataIndex++;
    }
    if ((buffer[INDEX_LEN] - dataIndex) == 0) {
      state = STATE_CRC;
    }
    break;
  case STATE_CRC: {
    uint8_t crc = crc8(buffer, dataIndex + 2);
    if (crc == ch) {
      parse();
    } else {
      reset();
    }
  } break;
  default:
    break;
  }
}

void Parser::cmdDirection(uint8_t *args, uint32_t length) {
  if (length != 1)
    return;

  if (args[0] == TX || args[0] == RX) {
    Link.setDirection((transmission_direction_e)args[0]);
  }
}

void Parser::cmdPAGain(uint8_t *args, uint32_t length) {
  if (length != 1)
    return;

  if (args[0] < 50) {
    Link.setPAGain(args[0]);
  }
}

void Parser::cmdPowerLevel(uint8_t *args, uint32_t length) {
  if (length != 1)
    return;

  Link.setPowerLevel(args[0]);
}

void Parser::cmdMode(uint8_t *args, uint32_t length) {
  if (length != 1)
    return;

  Link.setMode((transmission_mode_e)args[0]);
}

void Parser::cmdModeIndex(uint8_t *args, uint32_t length) {
  // UNUSED
}

void Parser::cmdLinkPhrase(uint8_t *args, uint32_t length) {
  if (length != 4)
    return;

  if (args[0] != 0) {
	uint32_t phrasecrc = args[0] << 24;
	phrasecrc += args[1] << 16;
	phrasecrc += args[2] << 8;
	phrasecrc += args[3];
    Link.setLinkPhraseCrc(phrasecrc);
  }
}

void Parser::cmdEnable(uint8_t *args, uint32_t length) {
  if (length != 0)
    return;
  Link.enableTransmission();
}

void Parser::cmdDisable(uint8_t *args, uint32_t length) {
  if (length != 0)
    return;
  Link.disableTransmission();
}

void Parser::cmdTX(uint8_t *args, uint32_t length) {
  Link.writeBytes(args, length);
}

void Parser::cmdVersionNum(uint8_t *args, uint32_t length) {
  send_version_num = true;
}

void Parser::cmdRX(uint8_t *args, uint32_t length) {
  // UNUSED
}

void Parser::cmdInfo(uint8_t *args, uint32_t length) {
  // UNUSED
}

void Parser::cmdGNSSLoc(uint8_t *args, uint32_t length) {
  // UNUSED
}

void Parser::cmdGNSSTime(uint8_t *args, uint32_t length) {
  // UNUSED
}

void Parser::cmdGNSSInfo(uint8_t *args, uint32_t length) {
  // UNUSED
}
