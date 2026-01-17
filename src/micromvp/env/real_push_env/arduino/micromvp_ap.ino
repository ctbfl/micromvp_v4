#include <WiFi.h>
#include <WiFiUdp.h>

// ============================================================
// 配置区
// ============================================================
static const char* AP_SSID = "micromvp-ap";
static const char* AP_PASS = "12345678";   // >= 8
static const int   AP_CHANNEL = 6;

static const uint16_t CAR_CMD_PORT   = 9001;  // 车端原 UDP 端口：不改
static const uint16_t HELLO_PORT     = 9010;  // 车端向 AP 报到的端口（AP 监听这个）
static const uint32_t HELLO_TIMEOUT_MS = 5000; // 超过这个时间没 hello 就视为掉线
static const uint32_t STATUS_PRINT_MS  = 1000;

// PC 串口聚合协议：
// [AA 55][seq u16][count u8][count*(id u8, L i16, R i16)][checksum u8]
static const uint8_t  MAGIC0 = 0xAA;
static const uint8_t  MAGIC1 = 0x55;

static const size_t   AGG_FIXED_LEN = 2 + 2 + 1; // magic(2) + seq(2) + count(1)
static const size_t   ENTRY_LEN = 1 + 2 + 2;     // id + L + R = 5
static const size_t   MAX_COUNT = 32;            // 你可以按需要调大，但串口缓存也要考虑

// 若你希望 hello 的 payload 规范一点：
// 推荐车端发： "HELLO <id>\n" 或 "ID:<id>\n"
// 这里实现：从 payload 中提取第一个 0-255 的整数作为 id
// ============================================================


// ============================================================
// 数据结构：ID -> IP + alive
// ============================================================
struct RobotRecord {
  bool     used = false;
  uint8_t  id = 0;
  IPAddress ip = IPAddress(0,0,0,0);
  uint32_t lastSeenMs = 0;
  uint32_t rxHello = 0;
};

static const int MAX_ROBOTS = 64; // 允许最多注册多少个 ID（0-255 中的部分）
RobotRecord robots[MAX_ROBOTS];

// 统计
uint32_t st_rxHelloPkts = 0;
uint32_t st_rxSerialFrames = 0;
uint32_t st_serialCksumBad = 0;
uint32_t st_serialDrop = 0;
uint32_t st_udpTx = 0;
uint32_t st_idNotRegistered = 0;

uint32_t lastStatusMs = 0;

WiFiUDP udp;        // 用于收 hello + 发控制
WiFiUDP udpHello;   // （可选）同一个 udp 也行，这里就复用一个也可以

// 串口帧解析缓冲
static const size_t SERIAL_BUF_MAX = 2 + 2 + 1 + MAX_COUNT * ENTRY_LEN + 1;
uint8_t frameBuf[SERIAL_BUF_MAX];


// ============================================================
// 工具函数
// ============================================================

static void printBoot() {
  Serial.println();
  Serial.println("=== XIAO ESP32-C6 AP Bridge (Serial Aggregated -> UDP <Hhh>) ===");
  Serial.print("SSID: "); Serial.println(AP_SSID);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
  Serial.print("HELLO_PORT: "); Serial.println(HELLO_PORT);
  Serial.print("CAR_CMD_PORT: "); Serial.println(CAR_CMD_PORT);
  Serial.println("Serial Frame: AA 55 | seq(u16) | count(u8) | N*(id,u16,u16) | checksum(u8)");
  Serial.println("Hello: payload contains robot id (0..255), AP records ID->IP (alive)");
  Serial.println("=================================================================");
  Serial.println();
}

static int findSlotById(uint8_t id) {
  for (int i = 0; i < MAX_ROBOTS; i++) {
    if (robots[i].used && robots[i].id == id) return i;
  }
  return -1;
}

static int allocateSlot(uint8_t id) {
  int idx = findSlotById(id);
  if (idx >= 0) return idx;
  for (int i = 0; i < MAX_ROBOTS; i++) {
    if (!robots[i].used) {
      robots[i].used = true;
      robots[i].id = id;
      robots[i].ip = IPAddress(0,0,0,0);
      robots[i].lastSeenMs = 0;
      robots[i].rxHello = 0;
      return i;
    }
  }
  return -1;
}

static bool isAlive(const RobotRecord& r, uint32_t now) {
  if (!r.used) return false;
  if (r.ip == IPAddress(0,0,0,0)) return false;
  return (now - r.lastSeenMs) <= HELLO_TIMEOUT_MS;
}

static int parseFirstInt0to255(const char* s) {
  // 从字符串中扫第一个整数（可带空格/冒号），返回 0..255，否则 -1
  int val = -1;
  bool inNum = false;
  int cur = 0;

  for (const char* p = s; *p; p++) {
    char c = *p;
    if (c >= '0' && c <= '9') {
      if (!inNum) {
        inNum = true;
        cur = 0;
      }
      cur = cur * 10 + (c - '0');
      if (cur > 255) return -1;
    } else {
      if (inNum) { val = cur; break; }
    }
  }
  if (inNum && val < 0) val = cur;
  return val;
}

static inline uint16_t readU16LE(const uint8_t* p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline int16_t readI16LE(const uint8_t* p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static bool sendToCar(uint8_t id, IPAddress ip, uint16_t seq, int16_t l, int16_t r) {
  // 原 UDP 格式 "<Hhh"：6 bytes
  uint8_t pkt[6];
  pkt[0] = (uint8_t)(seq & 0xFF);
  pkt[1] = (uint8_t)((seq >> 8) & 0xFF);

  pkt[2] = (uint8_t)(l & 0xFF);
  pkt[3] = (uint8_t)((l >> 8) & 0xFF);

  pkt[4] = (uint8_t)(r & 0xFF);
  pkt[5] = (uint8_t)((r >> 8) & 0xFF);

  // UDP send
  if (!udp.beginPacket(ip, CAR_CMD_PORT)) {
    Serial.printf("[ERR] UDP beginPacket failed id=%u ip=%s\n", id, ip.toString().c_str());
    return false;
  }
  udp.write(pkt, 6);
  if (!udp.endPacket()) {
    Serial.printf("[ERR] UDP endPacket failed id=%u ip=%s\n", id, ip.toString().c_str());
    return false;
  }
  return true;
}


// ============================================================
// 串口帧接收（带帧头同步 + checksum）
// ============================================================
//
// 返回：true 表示成功读到一帧（已填入 frameBuf，长度写入 outLen）
//
static bool readOneSerialFrame(size_t* outLen) {
  static uint8_t state = 0;   // 0:找AA  1:找55  2:读seq/count  3:读body+checksum
  static size_t want = 0;
  static size_t got  = 0;

  while (Serial.available() > 0) {
    uint8_t b = (uint8_t)Serial.read();

    if (state == 0) {
      if (b == MAGIC0) {
        frameBuf[0] = b;
        state = 1;
      }
      continue;
    }

    if (state == 1) {
      if (b == MAGIC1) {
        frameBuf[1] = b;
        got = 2;
        // 接下来先读 seq(2) + count(1)
        want = 2 + 2 + 1; // magic2 + seq2 + count1
        state = 2;
      } else {
        // 重新找AA
        state = 0;
      }
      continue;
    }

    if (state == 2) {
      frameBuf[got++] = b;
      if (got == want) {
        // 已读完固定头，计算总长度
        uint8_t count = frameBuf[4];
        if (count == 0 || count > MAX_COUNT) {
          st_serialDrop++;
          state = 0;
          continue;
        }
        size_t total = 2 + 2 + 1 + ((size_t)count) * ENTRY_LEN + 1; // + checksum
        if (total > SERIAL_BUF_MAX) {
          st_serialDrop++;
          state = 0;
          continue;
        }
        want = total;
        state = 3;
      }
      continue;
    }

    // state == 3
    frameBuf[got++] = b;
    if (got == want) {
      *outLen = want;
      state = 0;
      return true;
    }
  }

  return false;
}

static bool verifyChecksum(const uint8_t* buf, size_t len) {
  // checksum = Sum(Seq..Body) & 0xFF
  // buf: [AA 55][Seq2][Count1][Body...][Cksum1]
  if (len < (2 + 2 + 1 + 1)) return false;
  uint8_t cksum = buf[len - 1];

  uint32_t sum = 0;
  for (size_t i = 2; i < len - 1; i++) sum += buf[i];
  return ((uint8_t)(sum & 0xFF)) == cksum;
}


// ============================================================
// Hello 处理：接车端 UDP hello，提取 ID 并登记 ID->IP
// ============================================================
static void handleHello(uint32_t now) {
  int ps = udp.parsePacket();
  if (ps <= 0) return;

  st_rxHelloPkts++;

  IPAddress rip = udp.remoteIP();
  uint16_t rport = udp.remotePort();  // ✅ 统一用 rport

  // 读 payload（限制长度避免占内存）
  char msg[64];
  int n = udp.read((uint8_t*)msg, sizeof(msg) - 1);
  if (n < 0) n = 0;
  msg[n] = '\0';

  int idVal = parseFirstInt0to255(msg);
  if (idVal < 0) {
    Serial.printf("[HELLO] from %s:%u but no valid id in payload: '%s'\n",
                  rip.toString().c_str(), (unsigned)rport, msg);
    return;
  }
  uint8_t id = (uint8_t)idVal;

  int idx = allocateSlot(id);
  if (idx < 0) {
    Serial.printf("[HELLO] table full, cannot register id=%u from %s\n",
                  id, rip.toString().c_str());
    return;
  }

  robots[idx].ip = rip;
  robots[idx].lastSeenMs = now;
  robots[idx].rxHello++;

  // 可选 ACK
  udp.beginPacket(rip, rport);
  udp.print("ACK");
  udp.endPacket();
}



// ============================================================
// setup / loop
// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL);
  if (!ok) {
    Serial.println("[ERR] softAP start failed");
    while (true) delay(1000);
  }

  // 用同一个 udp 同时：监听 hello 端口 + 发控制包
  if (!udp.begin(HELLO_PORT)) {
    Serial.println("[ERR] udp.begin(HELLO_PORT) failed");
    while (true) delay(1000);
  }

  printBoot();
}

void loop() {
  uint32_t now = millis();

  // 1) 处理车端 hello
  handleHello(now);

  // 2) 处理来自 PC 的串口聚合帧
  size_t flen = 0;
  while (readOneSerialFrame(&flen)) {
    st_rxSerialFrames++;

    if (!verifyChecksum(frameBuf, flen)) {
      st_serialCksumBad++;
      // 不要疯狂打印，偶发时打印一下
      if ((st_serialCksumBad % 20) == 1) {
        Serial.printf("[SER] checksum bad (total=%lu)\n", (unsigned long)st_serialCksumBad);
      }
      continue;
    }

    // 解包
    uint16_t seq = readU16LE(&frameBuf[2]);
    uint8_t count = frameBuf[4];

    size_t bodyOff = 5;
    for (uint8_t i = 0; i < count; i++) {
      uint8_t rid = frameBuf[bodyOff + 0];
      int16_t l = readI16LE(&frameBuf[bodyOff + 1]);
      int16_t r = readI16LE(&frameBuf[bodyOff + 3]);
      bodyOff += ENTRY_LEN;

      // 查表
      int idx = findSlotById(rid);
      if (idx < 0 || !isAlive(robots[idx], now)) {
        st_idNotRegistered++;
        // 控制打印频率，避免刷屏
        if ((st_idNotRegistered % 30) == 1) {
          if (idx < 0) {
            Serial.printf("[WARN] Robot id=%u not registered (skip)\n", rid);
          } else {
            Serial.printf("[WARN] Robot id=%u registered but timeout (skip). ip=%s\n",
                          rid, robots[idx].ip.toString().c_str());
          }
        }
        continue;
      }

      // 转发：保持车端协议不变（seq + l + r）
      bool sent = sendToCar(rid, robots[idx].ip, seq, l, r);
      if (sent) st_udpTx++;
    }
  }

  // 3) 状态输出 + 清理掉线
  if (now - lastStatusMs >= STATUS_PRINT_MS) {
    lastStatusMs = now;

    int stations = WiFi.softAPgetStationNum();
    Serial.printf("[STATUS] stations=%d hello=%lu serFrames=%lu udpTx=%lu cksumBad=%lu drop=%lu notReg=%lu\n",
                  stations,
                  (unsigned long)st_rxHelloPkts,
                  (unsigned long)st_rxSerialFrames,
                  (unsigned long)st_udpTx,
                  (unsigned long)st_serialCksumBad,
                  (unsigned long)st_serialDrop,
                  (unsigned long)st_idNotRegistered);

    // 打印当前 alive 表（最多打印前 16 个，避免太长）
    int printed = 0;
    for (int i = 0; i < MAX_ROBOTS; i++) {
      if (!robots[i].used) continue;
      bool alive = isAlive(robots[i], now);
      if (alive) {
        Serial.printf("  [ALIVE] id=%u ip=%s last=%lums rxHello=%lu\n",
                      robots[i].id,
                      robots[i].ip.toString().c_str(),
                      (unsigned long)(now - robots[i].lastSeenMs),
                      (unsigned long)robots[i].rxHello);
        printed++;
        if (printed >= 16) break;
      }
    }
    if (printed == 0) {
      Serial.println("  (no alive robots)");
    }
  }

  delay(1);
}
