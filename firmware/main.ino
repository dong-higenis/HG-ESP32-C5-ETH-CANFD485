
/**
* @brief HG-ESP32-C5-ETH-CANFD485 Test 코드
*
* Board Manager ---------------------------
*  esp32 by Espressif Systems - ver 3.3.11
* -----------------------------------------
*
* Library ---------------------------------
*
* -----------------------------------------
*
*
* 시리얼 모니터----------------------------
*
* BaudRate : 115200 (수신 auto / 송신 CR)
* help : 도움말
* btn test : 버튼 디바운스 출력
* rs485 test : 485 상호 송수신 테스트
* can test : can 상호 송수신 테스트
* can diag : can 트랜시버 루프(TXD->버스->RXD) GPIO 진단
* eth test : w5500 초기화 -> 링크 여부/ DHCP 검사
*
* -----------------------------------------
*
* 테스트 결선 -----------------------------
*
* rs485 : J4 <-> J7 (A-A, B-B, GND), S2 1,2 ON
* can   : J6 <-> J1 (H-H, L-L),      S1 1,2 ON
* eth   : J2 -> DHCP 되는 공유기/스위치
*
* -----------------------------------------
*/

#include <Arduino.h>
#include <ETH.h>
#include <SPI.h>
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "cli.h"

/**
* @brief Button
*/
#define PIN_BTN   26  // S3 택트 스위치 / J5 외부 입력
#define PIN_BOOT  28  // DevKitC-1 BOOT 버튼

/**
* @brief W5500 (SPI)
*/
#define PIN_ETH_SCLK  6
#define PIN_ETH_MOSI  7
#define PIN_ETH_MISO  8
#define PIN_ETH_CS    10
#define PIN_ETH_INT   9
#define PIN_ETH_RST   25

/**
* @brief RS485
*/
#define PIN_RS485_1_TX  2   // U2 D  (U1_TXD)
#define PIN_RS485_1_RX  3   // U2 R  (U1_RXD)
#define PIN_RS485_2_TX  5   // U3 D  (LP_TXD)
#define PIN_RS485_2_RX  4   // U3 R  (LP_RXD)

/**
* @brief CAN
*/
#define PIN_CAN1_TX  23
#define PIN_CAN1_RX  24
#define PIN_CAN2_TX  0
#define PIN_CAN2_RX  1

/**
* @brief 테스트 파라미터
*/
#define ETH_LINK_TIMEOUT_MS  5000
#define ETH_DHCP_TIMEOUT_MS  15000

#define CAN_NOMINAL_BPS      500000UL        // CAN Classic bauds
#define CAN_DATA_BPS         2000000UL       // canfd bauds
#define CAN_FRAMES           10              // can frame 수
#define CAN_RX_TIMEOUT_MS    100

static const uint32_t RS485_BAUDS[] = {9600, 115200};
#define RS485_ROUNDS         5
#define RS485_FRAME_LEN      64

#define BTN_DEBOUNCE_MS      30


static const char *passStr(bool pass)
{
  return pass ? "PASS" : "FAIL";
}

// =====================================================================
// Button
//   btnTask 가 디바운스 후 이벤트를 큐로 전달, "btn test" 에서 출력
// =====================================================================
typedef struct
{
  uint8_t     pin;
  const char *name;
  bool        pressed;
  bool        raw;
  uint32_t    raw_ms;
  uint32_t    press_ms;
} btn_t;

typedef struct
{
  uint8_t  idx;
  bool     pressed;
  uint32_t hold_ms;
} btn_evt_t;

static btn_t btn_tbl[] =
{
  {PIN_BTN,  "btn ", false, false, 0, 0},
  {PIN_BOOT, "boot", false, false, 0, 0},
};

static QueueHandle_t btn_q;

static void btnTask(void *arg)
{
  for (auto &b : btn_tbl)
  {
    pinMode(b.pin, INPUT_PULLUP);
  }
  vTaskDelay(pdMS_TO_TICKS(10));
  for (auto &b : btn_tbl)
  {
    b.raw = b.pressed = (digitalRead(b.pin) == LOW);
  }

  while (1)
  {
    uint32_t now = millis();

    for (uint8_t i = 0; i < sizeof(btn_tbl) / sizeof(btn_tbl[0]); i++)
    {
      btn_t *b = &btn_tbl[i];
      bool raw = (digitalRead(b->pin) == LOW);

      if (raw != b->raw)
      {
        b->raw    = raw;
        b->raw_ms = now;
      }
      if (raw != b->pressed && now - b->raw_ms >= BTN_DEBOUNCE_MS)
      {
        btn_evt_t evt;

        b->pressed = raw;
        if (raw)
        {
          b->press_ms = now;
        }
        evt.idx     = i;
        evt.pressed = raw;
        evt.hold_ms = raw ? 0 : now - b->press_ms;
        xQueueSend(btn_q, &evt, 0);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

static void cliBtn(cli_args_t *args)
{
  if (args->argc == 1 && args->isStr(0, "test"))
  {
    btn_evt_t evt;

    for (auto &b : btn_tbl)
    {
      cliPrintf("%s IO%d : %s\r\n", b.name, b.pin, b.pressed ? "pressed" : "released");
    }
    cliPrintf("press button (any key to exit)\r\n");

    xQueueReset(btn_q);
    while (cliKeepLoop())
    {
      if (xQueueReceive(btn_q, &evt, pdMS_TO_TICKS(10)) == pdTRUE)
      {
        if (evt.pressed)
          cliPrintf("%s : pressed\r\n", btn_tbl[evt.idx].name);
        else
          cliPrintf("%s : released %lu ms\r\n", btn_tbl[evt.idx].name, (unsigned long)evt.hold_ms);
      }
    }
    cliRead();
    return;
  }

  cliPrintf("btn test\r\n");
}

// =====================================================================
// RS485
//   UART1 하나의 핀을 바꿔가며 상호 검사
//   1->2 : TX=IO2(U2 D), RX=IO4(U3 R)
//   2->1 : TX=IO5(U3 D), RX=IO3(U2 R)
//   쉬는 쪽 D 입력은 HIGH 로 고정 (THVD1406 자동 방향제어)
// =====================================================================
static void rs485Idle(void)
{
  pinMode(PIN_RS485_1_TX, OUTPUT);
  digitalWrite(PIN_RS485_1_TX, HIGH);
  pinMode(PIN_RS485_2_TX, OUTPUT);
  digitalWrite(PIN_RS485_2_TX, HIGH);
}

static int rs485OneWay(int tx_pin, int rx_pin, uint32_t baud, size_t *p_last_rx)
{
  static const uint8_t fixed[] = {0x00, 0xFF, 0x55, 0xAA};
  uint8_t tx[RS485_FRAME_LEN];
  uint8_t rx[RS485_FRAME_LEN];
  int ok = 0;

  rs485Idle();
  Serial1.begin(baud, SERIAL_8N1, rx_pin, tx_pin);
  Serial1.setTimeout(20 + (RS485_FRAME_LEN * 10 * 1000UL) / baud * 2);
  vTaskDelay(pdMS_TO_TICKS(20));
  while (Serial1.available()) Serial1.read();

  *p_last_rx = RS485_FRAME_LEN;
  for (int r = 0; r < RS485_ROUNDS; r++)
  {
    for (int i = 0; i < RS485_FRAME_LEN; i++)
    {
      tx[i] = (i < 4) ? fixed[i] : (uint8_t)(i * 37 + r * 11);
    }
    Serial1.write(tx, sizeof(tx));
    Serial1.flush();

    size_t n = Serial1.readBytes(rx, sizeof(rx));
    if (n == sizeof(rx) && memcmp(tx, rx, sizeof(tx)) == 0)
      ok++;
    else
      *p_last_rx = n;

    while (Serial1.available()) Serial1.read();
  }
  Serial1.end();
  rs485Idle();

  return ok;
}

static bool rs485Direction(const char *name, int tx_pin, int rx_pin)
{
  bool pass = true;

  cliPrintf("%s :", name);
  for (uint32_t baud : RS485_BAUDS)
  {
    size_t last_rx;
    int ok = rs485OneWay(tx_pin, rx_pin, baud, &last_rx);

    cliPrintf(" %lu %d/%d", (unsigned long)baud, ok, RS485_ROUNDS);
    if (ok != RS485_ROUNDS)
    {
      cliPrintf("(rx %u/%u)", (unsigned)last_rx, RS485_FRAME_LEN);
      pass = false;
    }
  }
  cliPrintf("  %s\r\n", passStr(pass));

  return pass;
}

static void cliRs485(cli_args_t *args)
{
  if (args->argc == 1 && args->isStr(0, "test"))
  {
    bool pass = true;

    pass &= rs485Direction("rs485 1->2", PIN_RS485_1_TX, PIN_RS485_2_RX);
    pass &= rs485Direction("rs485 2->1", PIN_RS485_2_TX, PIN_RS485_1_RX);
    cliPrintf("rs485 test : %s\r\n", passStr(pass));
    return;
  }

  cliPrintf("rs485 test\r\n");
}

// =====================================================================
// CAN / CAN FD (ESP32-C5 TWAI-FD x2, esp_twai_onchip 드라이버)
//   Classic : 500 kbps, 8 byte
//   FD      : 500 kbps arbitration / 2 Mbps data (BRS), 64 byte
// =====================================================================
typedef struct
{
  twai_frame_header_t hdr;
  uint8_t len;
  uint8_t data[TWAIFD_FRAME_MAX_LEN];
} can_rx_t;

typedef struct
{
  twai_node_handle_t h;
  QueueHandle_t      rx_q;
  volatile uint32_t  err_flags;  // 발생한 에러 종류 누적 (twai_error_flags_t.val)
  bool               ready;
} can_node_t;

static can_node_t can_node[2];
static bool       can_is_init = false;

static bool IRAM_ATTR canOnRx(twai_node_handle_t h, const twai_rx_done_event_data_t *e, void *ctx)
{
  can_node_t *n = (can_node_t *)ctx;
  can_rx_t item;
  twai_frame_t f = {};
  BaseType_t woken = pdFALSE;

  f.buffer     = item.data;
  f.buffer_len = sizeof(item.data);
  if (twai_node_receive_from_isr(h, &f) != ESP_OK)
    return false;

  uint16_t len = twaifd_dlc2len(f.header.dlc);
  item.hdr = f.header;
  item.len = len > sizeof(item.data) ? sizeof(item.data) : len;
  xQueueSendFromISR(n->rx_q, &item, &woken);

  return woken == pdTRUE;
}

static bool IRAM_ATTR canOnError(twai_node_handle_t h, const twai_error_event_data_t *e, void *ctx)
{
  can_node_t *n = (can_node_t *)ctx;

  n->err_flags = n->err_flags | e->err_flags.val;
  return false;
}

static bool canOpen(int idx, int tx, int rx)
{
  can_node_t *n = &can_node[idx];
  twai_onchip_node_config_t cfg = {};
  twai_event_callbacks_t cbs = {};

  n->rx_q = xQueueCreate(16, sizeof(can_rx_t));

  cfg.io_cfg.tx                = (gpio_num_t)tx;
  cfg.io_cfg.rx                = (gpio_num_t)rx;
  cfg.io_cfg.quanta_clk_out    = GPIO_NUM_NC;
  cfg.io_cfg.bus_off_indicator = GPIO_NUM_NC;
  cfg.bit_timing.bitrate       = CAN_NOMINAL_BPS;
  cfg.bit_timing.sp_permill    = 800;
  cfg.data_timing.bitrate      = CAN_DATA_BPS;
  cfg.data_timing.sp_permill   = 750;
  cfg.data_timing.ssp_permill  = 750;  // 트랜시버 지연 보상(TDC)
  cfg.fail_retry_cnt           = 3;    // ACK 없을 때 무한 재전송 방지
  cfg.tx_queue_depth           = 4;

  if (twai_new_node_onchip(&cfg, &n->h) != ESP_OK)
    return false;

  cbs.on_rx_done = canOnRx;
  cbs.on_error   = canOnError;
  twai_node_register_event_callbacks(n->h, &cbs, n);

  if (twai_node_enable(n->h) != ESP_OK)
  {
    twai_node_delete(n->h);
    return false;
  }
  return true;
}

static const char *canStateStr(twai_error_state_t s)
{
  switch (s)
  {
    case TWAI_ERROR_ACTIVE:  return "ACTIVE";
    case TWAI_ERROR_WARNING: return "WARNING";
    case TWAI_ERROR_PASSIVE: return "PASSIVE";
    case TWAI_ERROR_BUS_OFF: return "BUS_OFF";
    default:                 return "?";
  }
}

// BUS_OFF 이면 복구 시도
static void canRecover(int idx)
{
  twai_node_status_t st;

  if (twai_node_get_info(can_node[idx].h, &st, NULL) != ESP_OK || st.state != TWAI_ERROR_BUS_OFF)
    return;

  twai_node_recover(can_node[idx].h);
  for (int i = 0; i < 40; i++)
  {
    vTaskDelay(pdMS_TO_TICKS(25));
    if (twai_node_get_info(can_node[idx].h, &st, NULL) == ESP_OK && st.state != TWAI_ERROR_BUS_OFF)
      return;
  }
}

static int canOneWay(int from, int to, bool fd)
{
  static uint8_t buf[TWAIFD_FRAME_MAX_LEN];  // 송신 완료 전까지 유지되어야 함
  can_node_t *tx = &can_node[from];
  can_node_t *rx = &can_node[to];
  const int len = fd ? TWAIFD_FRAME_MAX_LEN : TWAI_FRAME_MAX_LEN;
  const uint32_t id = (fd ? 0x300 : 0x100) + from;
  int ok = 0;

  xQueueReset(tx->rx_q);
  xQueueReset(rx->rx_q);

  for (int i = 0; i < CAN_FRAMES; i++)
  {
    twai_frame_t f = {};
    can_rx_t r;

    for (int k = 0; k < len; k++)
    {
      buf[k] = (uint8_t)(i * 31 + k * 7 + from * 0x55);
    }
    f.header.id  = id;
    f.header.dlc = twaifd_len2dlc(len);
    f.header.fdf = fd;
    f.header.brs = fd;
    f.buffer     = buf;
    f.buffer_len = len;

    if (twai_node_transmit(tx->h, &f, 50) != ESP_OK)
      break;
    twai_node_transmit_wait_all_done(tx->h, 100);

    if (xQueueReceive(rx->rx_q, &r, pdMS_TO_TICKS(CAN_RX_TIMEOUT_MS)) != pdTRUE)
      continue;
    if (r.hdr.id == id && r.len == len && r.hdr.fdf == fd && memcmp(r.data, buf, len) == 0)
      ok++;
  }
  return ok;
}

static bool canDirection(int from, int to)
{
  bool pass;
  int ok_cc;
  int ok_fd;

  canRecover(from);
  canRecover(to);
  can_node[from].err_flags = 0;

  ok_cc = canOneWay(from, to, false);
  ok_fd = canOneWay(from, to, true);
  pass  = (ok_cc == CAN_FRAMES && ok_fd == CAN_FRAMES);

  cliPrintf("can %d->%d : cc %d/%d, fd %d/%d  %s\r\n", from + 1, to + 1, ok_cc, CAN_FRAMES, ok_fd, CAN_FRAMES, passStr(pass));

  if (!pass)
  {
    twai_node_status_t st;
    twai_error_flags_t ef;

    ef.val = can_node[from].err_flags;
    if (twai_node_get_info(can_node[from].h, &st, NULL) == ESP_OK)
    {
      cliPrintf("  can%d %s tec %u rec %u err%s%s%s%s%s\r\n", from + 1, canStateStr(st.state), st.tx_error_count, st.rx_error_count,
                ef.ack_err ? " ack" : "", ef.bit_err ? " bit" : "", ef.form_err ? " form" : "", ef.stuff_err ? " stuff" : "",
                ef.arb_lost ? " arb" : "");
    }
  }
  return pass;
}

// TWAI 노드 해제 (diag 에서 핀을 GPIO 로 쓰기 위해)
static void canClose(void)
{
  for (int i = 0; i < 2; i++)
  {
    if (can_node[i].ready)
    {
      twai_node_disable(can_node[i].h);
      twai_node_delete(can_node[i].h);
      can_node[i].ready = false;
    }
    if (can_node[i].rx_q != NULL)
    {
      vQueueDelete(can_node[i].rx_q);
      can_node[i].rx_q = NULL;
    }
  }
  can_is_init = false;
}

// GPIO 로 TXD 를 직접 구동해 트랜시버 루프(TXD -> 버스 -> RXD) 확인
//   idle : TXD H -> RXD H (recessive)
//   dom  : TXD L -> RXD L (dominant, 트랜시버가 버스 구동)
//   rel  : TXD H -> RXD H
//   bus  : 같은 순간 상대 채널 RXD (J6-J1 연결 시 L 이어야 함)
static bool canDiagCh(int ch, int tx_pin, int rx_pin, int other_rx_pin)
{
  int idle, dom, rel, other;
  bool pass;

  digitalWrite(tx_pin, HIGH);
  delayMicroseconds(200);
  idle = digitalRead(rx_pin);

  noInterrupts();
  digitalWrite(tx_pin, LOW);
  delayMicroseconds(20);
  dom   = digitalRead(rx_pin);
  other = digitalRead(other_rx_pin);
  digitalWrite(tx_pin, HIGH);
  interrupts();

  delayMicroseconds(200);
  rel = digitalRead(rx_pin);

  pass = (idle == HIGH && dom == LOW && rel == HIGH);

  cliPrintf("can%d IO%d->IO%d : idle %c, dom %c, rel %c, bus(can%d rx) %c  %s\r\n", ch, tx_pin, rx_pin,
            idle ? 'H' : 'L', dom ? 'H' : 'L', rel ? 'H' : 'L', ch == 1 ? 2 : 1, other ? 'H' : 'L', passStr(pass));

  if (idle == LOW)
    cliPrintf("  rxd stuck L : bus held dominant or U%d RXD fault\r\n", ch == 1 ? 6 : 7);
  else if (dom == HIGH)
    cliPrintf("  no dominant : check U%d VCC(pin3)=3.3V, STB(pin8)=0V, %s, solder\r\n", ch == 1 ? 6 : 7, ch == 1 ? "R21" : "R24");

  return pass;
}

static void canDiag(void)
{
  bool pass = true;

  canClose();

  pinMode(PIN_CAN1_TX, OUTPUT);
  digitalWrite(PIN_CAN1_TX, HIGH);
  pinMode(PIN_CAN2_TX, OUTPUT);
  digitalWrite(PIN_CAN2_TX, HIGH);
  pinMode(PIN_CAN1_RX, INPUT);
  pinMode(PIN_CAN2_RX, INPUT);
  delay(1);

  pass &= canDiagCh(1, PIN_CAN1_TX, PIN_CAN1_RX, PIN_CAN2_RX);
  pass &= canDiagCh(2, PIN_CAN2_TX, PIN_CAN2_RX, PIN_CAN1_RX);
  cliPrintf("can diag : %s\r\n", passStr(pass));
}

static void cliCan(cli_args_t *args)
{
  if (args->argc == 1 && args->isStr(0, "diag"))
  {
    canDiag();
    return;
  }

  if (args->argc == 1 && args->isStr(0, "test"))
  {
    bool pass = true;

    if (can_is_init == false)
    {
      can_node[0].ready = canOpen(0, PIN_CAN1_TX, PIN_CAN1_RX);
      can_node[1].ready = canOpen(1, PIN_CAN2_TX, PIN_CAN2_RX);
      can_is_init = true;
    }
    if (!can_node[0].ready || !can_node[1].ready)
    {
      cliPrintf("can init : can1 %s, can2 %s\r\n", can_node[0].ready ? "OK" : "FAIL", can_node[1].ready ? "OK" : "FAIL");
      cliPrintf("can test : FAIL\r\n");
      return;
    }

    pass &= canDirection(0, 1);
    pass &= canDirection(1, 0);
    cliPrintf("can test : %s\r\n", passStr(pass));
    return;
  }

  cliPrintf("can test\r\n");
  cliPrintf("can diag\r\n");
}

// =====================================================================
// ETH (W5500)
// =====================================================================
static volatile bool eth_link = false;
static volatile bool eth_got_ip = false;
static bool          eth_is_init = false;
static bool          eth_is_ok = false;

static void ethOnEvent(arduino_event_id_t event)
{
  switch (event)
  {
    case ARDUINO_EVENT_ETH_START:
      ETH.setHostname("hg-c5-hwtest");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      eth_link = true;
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      eth_got_ip = true;
      break;
    case ARDUINO_EVENT_ETH_LOST_IP:
      eth_got_ip = false;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
    case ARDUINO_EVENT_ETH_STOP:
      eth_link = false;
      eth_got_ip = false;
      break;
    default:
      break;
  }
}

static bool ethWait(volatile bool *p_flag, uint32_t timeout_ms)
{
  uint32_t pre_time = millis();

  while (!*p_flag)
  {
    if (millis() - pre_time > timeout_ms)
      return false;
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  return true;
}

static void cliEth(cli_args_t *args)
{
  if (args->argc == 1 && args->isStr(0, "test"))
  {
    if (eth_is_init == false)
    {
      Network.onEvent(ethOnEvent);
      eth_is_ok = ETH.begin(ETH_PHY_W5500, 1, PIN_ETH_CS, PIN_ETH_INT, PIN_ETH_RST, SPI2_HOST, PIN_ETH_SCLK, PIN_ETH_MISO, PIN_ETH_MOSI);
      eth_is_init = true;
    }
    cliPrintf("w5500 init : %s\r\n", eth_is_ok ? "OK" : "FAIL");
    if (!eth_is_ok)
    {
      cliPrintf("eth test : FAIL\r\n");
      return;
    }

    if (!ethWait(&eth_link, ETH_LINK_TIMEOUT_MS))
    {
      cliPrintf("link : DOWN\r\n");
      cliPrintf("eth test : FAIL\r\n");
      return;
    }
    cliPrintf("link : UP %uM %s\r\n", ETH.linkSpeed(), ETH.fullDuplex() ? "FULL" : "HALF");

    if (!ethWait(&eth_got_ip, ETH_DHCP_TIMEOUT_MS))
    {
      cliPrintf("dhcp : TIMEOUT\r\n");
      cliPrintf("eth test : FAIL\r\n");
      return;
    }
    cliPrintf("dhcp : %s\r\n", ETH.localIP().toString().c_str());
    cliPrintf("eth test : PASS\r\n");
    return;
  }

  cliPrintf("eth test\r\n");
}

// =====================================================================
// CLI Task
// =====================================================================
static void cliTask(void *arg)
{
  cliPrintf("\r\nHG-ESP32-C5-ETH-CANFD485 HW TEST\r\n");
  cliRunStr("help");
  cliPrintf("\r\ncli# ");

  while (1)
  {
    cliMain();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void setup()
{
  Serial.begin(115200);
  delay(500);

  rs485Idle();
  pinMode(PIN_RS485_1_RX, INPUT);
  pinMode(PIN_RS485_2_RX, INPUT);

  cliInit();
  cliOpen(Serial);
  cliAdd("btn",   cliBtn,   "btn test");
  cliAdd("rs485", cliRs485, "rs485 test");
  cliAdd("can",   cliCan,   "can test\r\ncan diag");
  cliAdd("eth",   cliEth,   "eth test");

  btn_q = xQueueCreate(8, sizeof(btn_evt_t));

  xTaskCreate(btnTask, "btn", 2048, NULL, 2, NULL);
  xTaskCreate(cliTask, "cli", 8192, NULL, 1, NULL);
}

void loop()
{
  vTaskDelete(NULL);
}
