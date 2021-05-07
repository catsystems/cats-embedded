///*
// * cli.c
// *
// *  Created on: 3 May 2021
// *      Author: Luca
// */
//
//#define CLI_IN_BUFFER_SIZE 256
//#define CLI_OUT_BUFFER_SIZE 256
//
//static uint32_t bufferIndex = 0;
//
//static char cliBuffer[CLI_IN_BUFFER_SIZE];
//
//static osMessageQueueId_t *cli_in;
//static osMessageQueueId_t *cli_out;
//
//
//#define ARRAYLEN(x) (sizeof(x) / sizeof((x)[0]))
//
//typedef struct {
//    const char *name;
//    const char *description;
//    const char *args;
//    cliCommandFn *cliCommand;
//} clicmd_t;
//
//#define CLI_COMMAND_DEF(name, description, args, cliCommand) \
//{ \
//    name , \
//    description , \
//    args , \
//    cliCommand \
//}
//
//static bool isEmpty(const char *string)
//{
//    return (string == NULL || *string == '\0') ? true : false;
//}
//
//const clicmd_t cmdTable[] = {
//    //CLI_COMMAND_DEF("bl", "reboot into bootloader", "[rom]", cliBootloader),
//    CLI_COMMAND_DEF("defaults", "reset to defaults and reboot", "[nosave|show]", cliDefaults),
//    CLI_COMMAND_DEF("dump", "dump configuration",
//        "[master|profile|rates|hardware|all] {defaults|bare}", cliDump),
//    CLI_COMMAND_DEF("exit", NULL, NULL, cliExit),
//    CLI_COMMAND_DEF("get", "get variable value", "[name]", cliGet),
//    CLI_COMMAND_DEF("help", "display command help", "[search string]", cliHelp),
//    CLI_COMMAND_DEF("mcu_id", "id of the microcontroller", NULL, cliMcuId),
//    CLI_COMMAND_DEF("save", "save and reboot", NULL, cliSave),
//    CLI_COMMAND_DEF("set", "change setting", "[<name>=<value>]", cliSet),
//    CLI_COMMAND_DEF("status", "show status", NULL, cliStatus),
//    CLI_COMMAND_DEF("version", "show version", NULL, cliVersion),
//};
//
//static void cliDefaults(const char *cmdName, char *cmdline){
//
//}
//
//static void cliHelp(const char *cmdName, char *cmdline)
//{
//    bool anyMatches = false;
//
//    for (uint32_t i = 0; i < ARRAYLEN(cmdTable); i++) {
//        bool printEntry = false;
//        if (isEmpty(cmdline)) {
//            printEntry = true;
//        } else {
//            if (strcasestr(cmdTable[i].name, cmdline)
//                || strcasestr(cmdTable[i].description, cmdline)
//               ) {
//                printEntry = true;
//            }
//        }
//
//        if (printEntry) {
//            anyMatches = true;
//            cliPrint(cmdTable[i].name);
//            if (cmdTable[i].description) {
//                cliPrintf(" - %s", cmdTable[i].description);
//            }
//            if (cmdTable[i].args) {
//                cliPrintf("\r\n\t%s", cmdTable[i].args);
//            }
//            cliPrintLinefeed();
//        }
//    }
//    if (!isEmpty(cmdline) && !anyMatches) {
//        cliPrintErrorLinef(cmdName, "NO MATCHES FOR '%s'", cmdline);
//    }
//}
//
//static void cliPrintInternal(const char *str)
//{
//	while (*str) {
//		bufWriterAppend(writer, *str++);
//	}
//}
//
//void cliPrint(const char *str)
//{
//    cliPrintInternal(str);
//}
//
//void cliPrintLinefeed(void)
//{
//    cliPrint("\r\n");
//}
//
//void cliPrintLine(const char *str)
//{
//    cliPrint(str);
//    cliPrintLinefeed();
//}
//
//static void cliPrintHashLine(const char *str)
//{
//    cliPrint("\r\n# ");
//    cliPrintLine(str);
//}
//
//static void cliPrintfva(const char *format, va_list va)
//{
//    if (cliWriter) {
//    	char* buffer[CLI_OUT_BUFFER_SIZE];
//    	vsnprintf(buffer, CLI_OUT_BUFFER_SIZE, format, va);
//    	cliPrintInternal(buffer);
//    }
//}
//
//static bool cliDumpPrintLinef(dumpFlags_t dumpMask, bool equalsDefault, const char *format, ...)
//{
//    if (!((dumpMask & DO_DIFF) && equalsDefault)) {
//        va_list va;
//        va_start(va, format);
//        cliPrintfva(format, va);
//        va_end(va);
//        cliPrintLinefeed();
//        return true;
//    } else {
//        return false;
//    }
//}
//
//static void cliWrite(uint8_t ch)
//{
//    if (cliWriter) {
//        bufWriterAppend(cliWriter, ch);
//    }
//}
//
//static bool cliDefaultPrintLinef(dumpFlags_t dumpMask, bool equalsDefault, const char *format, ...)
//{
//    if ((dumpMask & SHOW_DEFAULTS) && !equalsDefault) {
//        cliWrite('#');
//
//        va_list va;
//        va_start(va, format);
//        cliPrintfva(format, va);
//        va_end(va);
//        cliPrintLinefeed();
//        return true;
//    } else {
//        return false;
//    }
//}
//
//void cliPrintf(const char *format, ...)
//{
//    va_list va;
//    va_start(va, format);
//    cliPrintfva(format, va);
//    va_end(va);
//}
//
//
//void cliPrintLinef(const char *format, ...)
//{
//    va_list va;
//    va_start(va, format);
//    cliPrintfva(format, va);
//    va_end(va);
//    cliPrintLinefeed();
//}
//
//static void cliPrintErrorVa(const char *cmdName, const char *format, va_list va)
//{
//    if (cliErrorWriter) {
//        cliPrintInternal("###ERROR IN ");
//        cliPrintInternal(cmdName);
//        cliPrintInternal(": ");
//        char* buffer[CLI_OUT_BUFFER_SIZE];
//        vsnprintf(buffer, CLI_OUT_BUFFER_SIZE, format, va);
//        cliPrintInternal(buffer);
//        cliPrintInternal(": ");
//        va_end(va);
//
//        cliPrintInternal( "###");
//    }
//}
//
//static void cliPrintError(const char *cmdName, const char *format, ...)
//{
//    va_list va;
//    va_start(va, format);
//    cliPrintErrorVa(cmdName, format, va);
//
//    if (!cliWriter) {
//        // Supply our own linefeed in case we are printing inside a custom defaults operation
//        // TODO: Fix this by rewriting the entire CLI to have self contained line feeds
//        // instead of expecting the directly following command to supply the line feed.
//        cliPrintInternal("\r\n");
//    }
//}
//
//static void cliPrintErrorLinef(const char *cmdName, const char *format, ...)
//{
//    va_list va;
//    va_start(va, format);
//    cliPrintErrorVa(cmdName, format, va);
//    cliPrintInternal("\r\n");
//}
//
//static void processCharacter(const char c){
//    if (bufferIndex && (c == '\n' || c == '\r')) {
//        // enter pressed
//        cliPrintLinefeed();
//
//        // Strip comment starting with # from line
//        char *p = cliBuffer;
//        p = strchr(p, '#');
//        if (NULL != p) {
//            bufferIndex = (uint32_t)(p - cliBuffer);
//        }
//
//        // Strip trailing whitespace
//        while (bufferIndex > 0 && cliBuffer[bufferIndex - 1] == ' ') {
//            bufferIndex--;
//        }
//
//        // Process non-empty lines
//        if (bufferIndex > 0) {
//            cliBuffer[bufferIndex] = 0; // null terminate
//
//            const clicmd_t *cmd;
//            char *options;
//            for (cmd = cmdTable; cmd < cmdTable + ARRAYLEN(cmdTable); cmd++) {
//                if ((options = checkCommand(cliBuffer, cmd->name))) {
//                    break;
//                }
//            }
//            if (cmd < cmdTable + ARRAYLEN(cmdTable)) {
//                cmd->cliCommand(cmd->name, options);
//            } else {
//            	cliPrintLine("UNKNOWN COMMAND, TRY 'HELP'");
//            }
//            bufferIndex = 0;
//        }
//
//        memset(cliBuffer, 0, sizeof(cliBuffer));
//
//        // 'exit' will reset this flag, so we don't need to print prompt again
//        if (!cliMode) {
//            return;
//        }
//
//    } else if (bufferIndex < sizeof(cliBuffer) && c >= 32 && c <= 126) {
//        if (!bufferIndex && c == ' ')
//            return; // Ignore leading spaces
//        cliBuffer[bufferIndex++] = c;
//        cliWrite(c);
//    }
//}
//
//void cliProcess(void){
//    if (!cliWriter) {
//        return;
//    }
//
//    while (osMessageQueueGetCount(cli_in)) {
//        uint8_t c  = 0;
//        osMessageQueueGet(cli_in, &c, 0U, 1);
//        processCharacter(c);
//    }
//}
//
//
//void cliEnter(osMessageQueueId_t *in, osMessageQueueId_t *out){
//    cliMode = true;
//    cli_in = in;
//    cli_out = out;
//
//    cliPrintLine("\r\nEntering CLI Mode, type 'exit' to return, or 'help'");
//}
