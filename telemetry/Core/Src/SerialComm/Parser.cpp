
#include "Parser.h"
#include "common.h"

void Parser::parse() {

  cmd_table[opCodeIndex].cmd(buffer, bufferIndex);

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
  if (opCodeIndex < 0) {
    opCode = ch;
    opCodeIndex = getOpCodeIndex(opCode);
    if (opCodeIndex < 0) {
      reset();
    }
  } else if (validSize == false) {
    length = ch;
    if (length <= 16)
      validSize = true;
    else {
      reset();
    }
  } else if ((length - bufferIndex) > 0) {
    buffer[bufferIndex] = ch;
    bufferIndex++;
  }

  if ((length - bufferIndex) == 0 && validSize) {
    parse();
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
  if (length > 8)
    return;

  if (args[0] != 0) {
    Link.setLinkPhrase(args, length);
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
