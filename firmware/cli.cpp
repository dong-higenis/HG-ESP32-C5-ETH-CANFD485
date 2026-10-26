/*
 * cli.cpp
 *
 *  ESP32 (Arduino) port of baram cli module
 */


#include "cli.h"
#include <stdarg.h>
#include <string.h>
#include <strings.h>


#define CLI_KEY_BACK              0x08
#define CLI_KEY_DEL               0x7F
#define CLI_KEY_ENTER             0x0D
#define CLI_KEY_LF                0x0A
#define CLI_KEY_ESC               0x1B
#define CLI_KEY_LEFT              0x44
#define CLI_KEY_RIGHT             0x43
#define CLI_KEY_UP                0x41
#define CLI_KEY_DOWN              0x42
#define CLI_KEY_HOME              0x31
#define CLI_KEY_END               0x34

#define CLI_PROMPT_STR            "cli# "

#define CLI_ARGS_MAX              16
#define CLI_PRINT_BUF_MAX         256


enum
{
  CLI_RX_IDLE,
  CLI_RX_SP1,
  CLI_RX_SP2,
  CLI_RX_SP3,
  CLI_RX_SP4,
};


typedef struct
{
  char   cmd_str[CLI_CMD_NAME_MAX];
  const char *usage;
  void (*cmd_func)(cli_args_t *);
} cli_cmd_t;


typedef struct
{
  uint8_t buf[CLI_LINE_BUF_MAX];
  uint8_t buf_len;
  uint8_t cursor;
  uint8_t count;
} cli_line_t;


typedef struct
{
  Stream  *port;
  bool     is_open;
  bool     is_busy;
  uint8_t  state;
  char     print_buffer[CLI_PRINT_BUF_MAX];
  uint16_t argc;
  char    *argv[CLI_ARGS_MAX];

  SemaphoreHandle_t lock;

  bool        hist_line_new;
  int8_t      hist_line_i;
  uint8_t     hist_line_last;
  uint8_t     hist_line_count;

  cli_line_t  line_buf[CLI_LINE_HIS_MAX];
  cli_line_t  line;

  uint16_t    cmd_count;
  cli_cmd_t   cmd_list[CLI_CMD_LIST_MAX];
  cli_args_t  cmd_args;
} cli_t;



static cli_t cli_node;



static bool cliUpdate(cli_t *p_cli, uint8_t rx_data);
static void cliLineClean(cli_t *p_cli);
static void cliLineAdd(cli_t *p_cli);
static void cliLineChange(cli_t *p_cli, int8_t key_up);
static void cliShowPrompt(cli_t *p_cli);
static bool cliRunCmd(cli_t *p_cli);
static bool cliParseArgs(cli_t *p_cli);
static void cliOut(const char *fmt, ...);

static int32_t  cliArgsGetData(uint8_t index);
static float    cliArgsGetFloat(uint8_t index);
static char    *cliArgsGetStr(uint8_t index);
static bool     cliArgsIsStr(uint8_t index, const char *p_str);

static void cliShowList(cli_args_t *args);


bool cliInit(void)
{
  cli_node.port    = NULL;
  cli_node.is_open = false;
  cli_node.is_busy = false;
  cli_node.state   = CLI_RX_IDLE;
  cli_node.lock    = xSemaphoreCreateRecursiveMutex();

  cli_node.hist_line_i     = 0;
  cli_node.hist_line_last  = 0;
  cli_node.hist_line_count = 0;
  cli_node.hist_line_new   = false;

  cli_node.cmd_count = 0;

  cli_node.cmd_args.getData  = cliArgsGetData;
  cli_node.cmd_args.getFloat = cliArgsGetFloat;
  cli_node.cmd_args.getStr   = cliArgsGetStr;
  cli_node.cmd_args.isStr    = cliArgsIsStr;

  cliLineClean(&cli_node);


  cliAdd("help", cliShowList);

  return true;
}

bool cliOpen(Stream &port)
{
  cli_node.port    = &port;
  cli_node.is_open = true;

  return cli_node.is_open;
}

bool cliIsBusy(void)
{
  return cli_node.is_busy;
}

Stream *cliGetPort(void)
{
  return cli_node.port;
}

void cliShowPrompt(cli_t *p_cli)
{
  cliOut("\r\n");
  cliOut(CLI_PROMPT_STR);
}

bool cliMain(void)
{
  if (cli_node.is_open != true)
  {
    return false;
  }

  while (cli_node.port->available() > 0)
  {
    cliUpdate(&cli_node, (uint8_t)cli_node.port->read());
  }

  return true;
}

uint32_t cliAvailable(void)
{
  if (cli_node.is_open != true)
  {
    return 0;
  }
  return cli_node.port->available();
}

uint8_t cliRead(void)
{
  if (cli_node.is_open != true)
  {
    return 0;
  }
  return (uint8_t)cli_node.port->read();
}

uint32_t cliWrite(const uint8_t *p_data, uint32_t length)
{
  uint32_t ret;

  if (cli_node.is_open != true)
  {
    return 0;
  }

  xSemaphoreTakeRecursive(cli_node.lock, portMAX_DELAY);
  ret = cli_node.port->write(p_data, length);
  xSemaphoreGiveRecursive(cli_node.lock);

  return ret;
}

bool cliUpdate(cli_t *p_cli, uint8_t rx_data)
{
  bool ret = false;
  uint8_t tx_buf[8];
  cli_line_t *line;

  line = &p_cli->line;


  if (p_cli->state == CLI_RX_IDLE)
  {
    switch(rx_data)
    {
      // 엔터
      //
      case CLI_KEY_ENTER:
        if (line->count > 0)
        {
          cliLineAdd(p_cli);
          cliRunCmd(p_cli);
        }

        line->count = 0;
        line->cursor = 0;
        line->buf[0] = 0;
        cliShowPrompt(p_cli);
        break;

      // CR/LF 로 보내는 터미널 대응
      //
      case CLI_KEY_LF:
        break;


      case CLI_KEY_ESC:
        p_cli->state = CLI_RX_SP1;
        break;


      // DEL
      //
      case CLI_KEY_DEL:
        if (line->cursor < line->count)
        {
          uint8_t mov_len;

          mov_len = line->count - line->cursor;
          for (int i=1; i<mov_len; i++)
          {
            line->buf[line->cursor + i - 1] = line->buf[line->cursor + i];
          }

          line->count--;
          line->buf[line->count] = 0;

          cliOut("\x1B[1P");
        }
        break;


      // 백스페이스
      //
      case CLI_KEY_BACK:
        if (line->count > 0 && line->cursor > 0)
        {
          if (line->cursor == line->count)
          {
            line->count--;
            line->buf[line->count] = 0;
          }

          if (line->cursor < line->count)
          {
            uint8_t mov_len;

            mov_len = line->count - line->cursor;

            for (int i=0; i<mov_len; i++)
            {
              line->buf[line->cursor + i - 1] = line->buf[line->cursor + i];
            }

            line->count--;
            line->buf[line->count] = 0;
          }
        }

        if (line->cursor > 0)
        {
          line->cursor--;
          cliOut("\b \b\x1B[1P");
        }
        break;


      default:
        if ((line->count + 1) < line->buf_len)
        {
          if (line->cursor == line->count)
          {
            cliWrite(&rx_data, 1);

            line->buf[line->cursor] = rx_data;
            line->count++;
            line->cursor++;
            line->buf[line->count] = 0;
          }
          if (line->cursor < line->count)
          {
            uint8_t mov_len;

            mov_len = line->count - line->cursor;
            for (int i=0; i<mov_len; i++)
            {
              line->buf[line->count - i] = line->buf[line->count - i - 1];
            }
            line->buf[line->cursor] = rx_data;
            line->count++;
            line->cursor++;
            line->buf[line->count] = 0;

            cliOut("\x1B[4h%c\x1B[4l", rx_data);
          }
        }
        break;
    }
  }

  switch(p_cli->state)
  {
    case CLI_RX_SP1:
      p_cli->state = CLI_RX_SP2;
      break;

    case CLI_RX_SP2:
      p_cli->state = CLI_RX_SP3;
      break;

    case CLI_RX_SP3:
      p_cli->state = CLI_RX_IDLE;

      if (rx_data == CLI_KEY_LEFT)
      {
        if (line->cursor > 0)
        {
          line->cursor--;
          tx_buf[0] = 0x1B;
          tx_buf[1] = 0x5B;
          tx_buf[2] = rx_data;
          cliWrite(tx_buf, 3);
        }
      }

      if (rx_data == CLI_KEY_RIGHT)
      {
        if (line->cursor < line->count)
        {
          line->cursor++;

          tx_buf[0] = 0x1B;
          tx_buf[1] = 0x5B;
          tx_buf[2] = rx_data;
          cliWrite(tx_buf, 3);
        }
      }

      if (rx_data == CLI_KEY_UP)
      {
        cliLineChange(p_cli, true);
        cliWrite(p_cli->line.buf, p_cli->line.count);
      }

      if (rx_data == CLI_KEY_DOWN)
      {
        cliLineChange(p_cli, false);
        cliWrite(p_cli->line.buf, p_cli->line.count);
      }

      if (rx_data == CLI_KEY_HOME)
      {
        if (line->cursor > 0)
        {
          cliOut("\x1B[%dD", line->cursor);
        }
        line->cursor = 0;

        p_cli->state = CLI_RX_SP4;
      }

      if (rx_data == CLI_KEY_END)
      {
        uint16_t mov_len;

        if (line->cursor < line->count)
        {
          mov_len = line->count - line->cursor;
          cliOut("\x1B[%dC", mov_len);
        }
        if (line->cursor > line->count)
        {
          mov_len = line->cursor - line->count;
          cliOut("\x1B[%dD", mov_len);
        }
        line->cursor = line->count;
        p_cli->state = CLI_RX_SP4;
      }
      break;

    case CLI_RX_SP4:
      p_cli->state = CLI_RX_IDLE;
      break;
  }

  return ret;
}

void cliLineClean(cli_t *p_cli)
{
  p_cli->line.count   = 0;
  p_cli->line.cursor  = 0;
  p_cli->line.buf_len = CLI_LINE_BUF_MAX - 1;
  p_cli->line.buf[0]  = 0;
}

void cliLineAdd(cli_t *p_cli)
{

  p_cli->line_buf[p_cli->hist_line_last] = p_cli->line;

  if (p_cli->hist_line_count < CLI_LINE_HIS_MAX)
  {
    p_cli->hist_line_count++;
  }

  p_cli->hist_line_i    = p_cli->hist_line_last;
  p_cli->hist_line_last = (p_cli->hist_line_last + 1) % CLI_LINE_HIS_MAX;
  p_cli->hist_line_new  = true;
}

void cliLineChange(cli_t *p_cli, int8_t key_up)
{
  uint8_t change_i;


  if (p_cli->hist_line_count == 0)
  {
    return;
  }


  if (p_cli->line.cursor > 0)
  {
    cliOut("\x1B[%dD", p_cli->line.cursor);
  }
  if (p_cli->line.count > 0)
  {
    cliOut("\x1B[%dP", p_cli->line.count);
  }


  if (key_up == true)
  {
    if (p_cli->hist_line_new == true)
    {
      p_cli->hist_line_i = p_cli->hist_line_last;
    }
    p_cli->hist_line_i = (p_cli->hist_line_i + p_cli->hist_line_count - 1) % p_cli->hist_line_count;
    change_i = p_cli->hist_line_i;
  }
  else
  {
    p_cli->hist_line_i = (p_cli->hist_line_i + 1) % p_cli->hist_line_count;
    change_i = p_cli->hist_line_i;
  }

  p_cli->line = p_cli->line_buf[change_i];
  p_cli->line.cursor = p_cli->line.count;

  p_cli->hist_line_new = false;
}

bool cliRunCmd(cli_t *p_cli)
{
  bool ret = false;


  if (cliParseArgs(p_cli) == true)
  {
    cliPrintf("\r\n");

    p_cli->is_busy = true;
    for (int i=0; i<p_cli->cmd_count; i++)
    {
      if (strcasecmp(p_cli->argv[0], p_cli->cmd_list[i].cmd_str) == 0)
      {
        p_cli->cmd_args.argc =  p_cli->argc - 1;
        p_cli->cmd_args.argv = &p_cli->argv[1];
        p_cli->cmd_list[i].cmd_func(&p_cli->cmd_args);
        ret = true;
        break;
      }
    }
    p_cli->is_busy = false;
  }

  return ret;
}

bool cliParseArgs(cli_t *p_cli)
{
  bool ret = false;
  char *tok;
  char *next_ptr;
  uint16_t argc = 0;
  static const char *delim = " \f\n\r\t\v";
  char *cmdline;
  char **argv;

  p_cli->argc = 0;

  cmdline = (char *)p_cli->line.buf;
  argv    = p_cli->argv;

  argv[argc] = NULL;

  for (tok = strtok_r(cmdline, delim, &next_ptr); tok && argc < CLI_ARGS_MAX; tok = strtok_r(NULL, delim, &next_ptr))
  {
    argv[argc++] = tok;
  }

  p_cli->argc = argc;

  if (argc > 0)
  {
    ret = true;
  }

  return ret;
}

bool cliRunStr(const char *fmt, ...)
{
  bool ret;
  va_list arg;
  va_start (arg, fmt);
  cli_t *p_cli = &cli_node;

  vsnprintf((char *)p_cli->line.buf, CLI_LINE_BUF_MAX, fmt, arg);
  va_end (arg);

  ret = cliRunCmd(p_cli);

  return ret;
}

void cliPrintf(const char *fmt, ...)
{
  va_list arg;
  int32_t len;
  cli_t *p_cli = &cli_node;


  if (p_cli->is_open != true)
  {
    return;
  }

  xSemaphoreTakeRecursive(p_cli->lock, portMAX_DELAY);
  va_start (arg, fmt);
  len = vsnprintf(p_cli->print_buffer, CLI_PRINT_BUF_MAX, fmt, arg);
  va_end (arg);

  if (len > CLI_PRINT_BUF_MAX - 1)
  {
    len = CLI_PRINT_BUF_MAX - 1;
  }
  if (len > 0)
  {
    p_cli->port->write((const uint8_t *)p_cli->print_buffer, len);
  }
  xSemaphoreGiveRecursive(p_cli->lock);
}

// 내부 에코/제어 문자 출력용
void cliOut(const char *fmt, ...)
{
  va_list arg;
  char buf[32];
  int32_t len;


  va_start (arg, fmt);
  len = vsnprintf(buf, sizeof(buf), fmt, arg);
  va_end (arg);

  if (len > (int32_t)sizeof(buf) - 1)
  {
    len = sizeof(buf) - 1;
  }
  if (len > 0)
  {
    cliWrite((const uint8_t *)buf, len);
  }
}

void cliPutch(uint8_t data)
{
  cliWrite(&data, 1);
}

int32_t cliArgsGetData(uint8_t index)
{
  int32_t ret = 0;
  cli_t *p_cli = &cli_node;


  if (index >= p_cli->cmd_args.argc)
  {
    return 0;
  }

  ret = (int32_t)strtoul((const char * ) p_cli->cmd_args.argv[index], (char **)NULL, (int) 0);

  return ret;
}

float cliArgsGetFloat(uint8_t index)
{
  float ret = 0.0;
  cli_t *p_cli = &cli_node;


  if (index >= p_cli->cmd_args.argc)
  {
    return 0;
  }

  ret = (float)strtof((const char * ) p_cli->cmd_args.argv[index], (char **)NULL);

  return ret;
}

char *cliArgsGetStr(uint8_t index)
{
  char *ret = NULL;
  cli_t *p_cli = &cli_node;


  if (index >= p_cli->cmd_args.argc)
  {
    return 0;
  }

  ret = p_cli->cmd_args.argv[index];

  return ret;
}

bool cliArgsIsStr(uint8_t index, const char *p_str)
{
  bool ret = false;
  cli_t *p_cli = &cli_node;


  if (index >= p_cli->cmd_args.argc)
  {
    return 0;
  }

  if(strcasecmp(p_str, p_cli->cmd_args.argv[index]) == 0)
  {
    ret = true;
  }

  return ret;
}

bool cliKeepLoop(void)
{
  if (cliAvailable() == 0)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool cliAdd(const char *cmd_str, void (*p_func)(cli_args_t *), const char *usage)
{
  bool ret = true;
  cli_t *p_cli = &cli_node;
  uint16_t index;

  if (p_cli->cmd_count >= CLI_CMD_LIST_MAX)
  {
    return false;
  }

  index = p_cli->cmd_count;

  strncpy(p_cli->cmd_list[index].cmd_str, cmd_str, CLI_CMD_NAME_MAX - 1);
  p_cli->cmd_list[index].cmd_str[CLI_CMD_NAME_MAX - 1] = 0;
  p_cli->cmd_list[index].cmd_func = p_func;
  p_cli->cmd_list[index].usage    = usage;

  p_cli->cmd_count++;

  return ret;
}

void cliShowList(cli_args_t *args)
{
  cli_t *p_cli = &cli_node;


  cliPrintf("\r\n");
  cliPrintf("---------- cmd list ---------\r\n");

  for (int i=0; i<p_cli->cmd_count; i++)
  {
    if (p_cli->cmd_list[i].usage != NULL)
      cliPrintf("%s\r\n", p_cli->cmd_list[i].usage);
    else
      cliPrintf("%s\r\n", p_cli->cmd_list[i].cmd_str);
  }

  cliPrintf("-----------------------------\r\n");
}
