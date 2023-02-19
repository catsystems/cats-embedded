
#include "parser.h"
#include "crc.h"
#include "console.h"

void Parser::parse() {

  (this->*commandFunction[opCodeIndex])(&buffer[2], dataIndex);

  /* Reset the parser buffer */
  reset();
}

int32_t Parser::getOpCodeIndex(uint8_t opCode) {
  for (int32_t i = 0; i < CMD_NUMBER; i++) {
    if (opCode == cmdIndex[i])
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
      console.error.println("[PARSER] CRC Failed");  
      reset();
    }
  } break;
  default:
    break;
  }
}

void Parser::cmdRX(uint8_t *args, uint32_t length) {
  data->commit(args, length);
}

void Parser::cmdInfo(uint8_t *args, uint32_t length) {
  info->commit(args, length);
}

void Parser::cmdGNSSLoc(uint8_t *args, uint32_t length) {
  if(location != NULL){
    location->commit(args, length);
  }
}

void Parser::cmdGNSSTime(uint8_t *args, uint32_t length) {
  if(time != NULL){
    time->commit(args, length);
  }
}

void Parser::cmdGNSSInfo(uint8_t *args, uint32_t length) {
  console.log.println("GNSS Info Received");
}
