#ifndef CLI_H_
#define CLI_H_

#include <Arduino.h>


#ifndef CLI_CMD_LIST_MAX
#define CLI_CMD_LIST_MAX      16
#endif
#ifndef CLI_CMD_NAME_MAX
#define CLI_CMD_NAME_MAX      16
#endif
#ifndef CLI_LINE_HIS_MAX
#define CLI_LINE_HIS_MAX      4
#endif
#ifndef CLI_LINE_BUF_MAX
#define CLI_LINE_BUF_MAX      64
#endif


typedef struct
{
  uint16_t   argc;
  char     **argv;

  int32_t  (*getData)(uint8_t index);
  float    (*getFloat)(uint8_t index);
  char    *(*getStr)(uint8_t index);
  bool     (*isStr)(uint8_t index, const char *p_str);
} cli_args_t;


bool     cliInit(void);
bool     cliOpen(Stream &port);
bool     cliIsBusy(void);
bool     cliMain(void);
void     cliPrintf(const char *fmt, ...);
bool     cliAdd(const char *cmd_str, void (*p_func)(cli_args_t *), const char *usage = NULL);  // usage : help 에 표시할 입력 형식
bool     cliKeepLoop(void);
void     cliPutch(uint8_t data);
Stream  *cliGetPort(void);
uint32_t cliAvailable(void);
uint8_t  cliRead(void);
uint32_t cliWrite(const uint8_t *p_data, uint32_t length);
bool     cliRunStr(const char *fmt, ...);


#endif
