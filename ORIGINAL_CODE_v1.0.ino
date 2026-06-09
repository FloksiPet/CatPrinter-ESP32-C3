#include <Arduino.h>

#include <BLEDevice.h>

#include <BLEUtils.h>

#include <BLEClient.h>



// ── Налаштування ────────────────────────────────────────────────

static std::string PRINTER_MAC = "3e:01:22:4a:a2:05";

static BLEUUID serviceUUID ("0000ae30-0000-1000-8000-00805f9b34fb");

static BLEUUID writeUUID   ("0000ae01-0000-1000-8000-00805f9b34fb");

static BLEUUID notifyUUID  ("0000ae02-0000-1000-8000-00805f9b34fb");

// ────────────────────────────────────────────────────────────────



// ── Команди (з реверс-інжинірингу APK) ─────────────────────────

#define CMD_RETRACT_PAPER  0xA0  // Data: 2 байти, кількість кроків назад

#define CMD_FEED_PAPER     0xA1  // Data: 2 байти, кількість кроків вперед

#define CMD_DRAW_BITMAP    0xA2  // Data: 50 байт = 1 рядок (400 пікселів, 1bpp)

#define CMD_GET_STATUS     0xA3  // Data: немає, відповідь: статус принтера

#define CMD_SET_QUALITY    0xA4  // Data: 1 байт, 1-5

#define CMD_SET_ENERGY     0xAF  // Data: 2 байти, рівень енергії

#define CMD_DRAWING_MODE   0xBE  // Data: 0=зображення, 1=текст

// ────────────────────────────────────────────────────────────────



// ── CRC8 таблиця з APK (верифікована по відповіді принтера) ────

static const uint8_t CRC8_TABLE[256] = {

    0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31,

    0x24, 0x23, 0x2a, 0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,

    0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9,

    0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,

    0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1,

    0xb4, 0xb3, 0xba, 0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,

    0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea, 0xb7, 0xb0, 0xb9, 0xbe,

    0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,

    0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16,

    0x03, 0x04, 0x0d, 0x0a, 0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,

    0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80,

    0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,

    0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8,

    0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,

    0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10,

    0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,

    0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f,

    0x6a, 0x6d, 0x64, 0x63, 0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,

    0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13, 0xae, 0xa9, 0xa0, 0xa7,

    0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,

    0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef,

    0xfa, 0xfd, 0xf4, 0xf3

};



uint8_t crc8(uint8_t* data, uint8_t len) {

    uint8_t crc = 0;

    for (uint8_t i = 0; i < len; i++)

        crc = CRC8_TABLE[(crc ^ data[i]) & 0xFF];

    return crc;

}



// ── Збірка пакету: 51 78 CMD 00 LEN 00 [DATA] CRC(DATA) FF ─────

uint8_t buildPacket(uint8_t cmd, uint8_t* data, uint8_t dataLen, uint8_t* out) {

    out[0] = 0x51; out[1] = 0x78;

    out[2] = cmd;  out[3] = 0x00;

    out[4] = dataLen; out[5] = 0x00;

    for (int i = 0; i < dataLen; i++) out[6+i] = data[i];

    out[6+dataLen] = (dataLen > 0) ? crc8(data, dataLen) : 0x00;

    out[7+dataLen] = 0xFF;

    return 8 + dataLen;

}



// ── Стан ────────────────────────────────────────────────────────

static BLERemoteCharacteristic* pWriteChar  = nullptr;

static BLERemoteCharacteristic* pNotifyChar = nullptr;

static BLEClient*                pClient    = nullptr;

bool connected = false;



// ── Notify callback ─────────────────────────────────────────────

void notifyCallback(BLERemoteCharacteristic* c, uint8_t* d, size_t l, bool n) {

    Serial.printf("🔔 [%d байт]: ", l);

    for (size_t i = 0; i < l; i++) Serial.printf("%02X ", d[i]);

    Serial.println();

    if (l >= 8 && d[0]==0x51 && d[1]==0x78) {

        Serial.printf("  CMD=0x%02X  LEN=%d  DATA:", d[2], d[4]);

        for (size_t i = 6; i < 6+(size_t)d[4] && i < l-2; i++)

            Serial.printf(" %02X", d[i]);

        Serial.println();

    }

}



// ── Відправка (WRITE NO RESPONSE) ──────────────────────────────

void sendCmd(const char* label, uint8_t cmd, uint8_t* data=nullptr, uint8_t len=0) {

    if (!connected || !pWriteChar) return;

    uint8_t pkt[80];

    uint8_t pktLen = buildPacket(cmd, data, len, pkt);

    Serial.printf("📤 %-18s: ", label);

    // Виводимо тільки перші 12 байт щоб не захаращувати лог

    uint8_t show = pktLen < 12 ? pktLen : 12;

    for (int i = 0; i < show; i++) Serial.printf("%02X ", pkt[i]);

    if (pktLen > 12) Serial.printf("... (%d байт)", pktLen);

    Serial.println();

    pWriteChar->writeValue(pkt, pktLen, false);

}



// ── BLE callbacks ───────────────────────────────────────────────

class MyCallbacks : public BLEClientCallbacks {

    void onConnect(BLEClient* c) override    { Serial.println("🔗 З'єднано"); }

    void onDisconnect(BLEClient* c) override { connected = false; Serial.println("❌ Відключення!"); }

};



bool connectToPrinter() {

    pClient = BLEDevice::createClient();

    pClient->setClientCallbacks(new MyCallbacks());

    if (!pClient->connect(BLEAddress(PRINTER_MAC))) return false;



    BLERemoteService* svc = pClient->getService(serviceUUID);

    if (!svc) { Serial.println("❌ Сервіс не знайдено"); return false; }



    pWriteChar  = svc->getCharacteristic(writeUUID);

    pNotifyChar = svc->getCharacteristic(notifyUUID);

    if (!pWriteChar) { Serial.println("❌ AE01 не знайдено"); return false; }



    if (pNotifyChar && pNotifyChar->canNotify()) {

        pNotifyChar->registerForNotify(notifyCallback);

        BLERemoteDescriptor* d = pNotifyChar->getDescriptor(BLEUUID((uint16_t)0x2902));

        if (d) { uint8_t v[]={0x01,0x00}; d->writeValue(v, 2, true); }

        Serial.println("⚙️  Notify: AE02");

    }

    return true;

}



// ── Setup ────────────────────────────────────────────────────────

void setup() {

    Serial.begin(115200);

    delay(2000);

    Serial.println("\n=== MX10 Gateway v6 (APK protocol) ===");

    Serial.println("⚠️  v1.0: PAPER FEED ONLY - NO PRINTING YET");

    Serial.println("📍 https://github.com/FloksiPet/CatPrinter-ESP32-C3\n");



    BLEDevice::init("ESP32_MX10_GW");

    if (!connectToPrinter()) { Serial.println("❌ Підключення не вдалось"); return; }

    connected = true;

    delay(1500);



    // ── Init sequence (точно як у Python APK коді) ──────────────

    Serial.println("\n─── Init ───");



    uint8_t energy[]  = {0x10, 0x00}; // помірна енергія

    uint8_t quality[] = {0x05};       // якість 5 (максимум)

    uint8_t mode_img[]= {0x00};       // режим: зображення

    

    sendCmd("SetEnergy (AF)",   CMD_SET_ENERGY,   energy,  2); delay(300);

    if (!connected) { Serial.println("❌ Дроп на SetEnergy!"); return; }

    

    sendCmd("SetQuality (A4)",  CMD_SET_QUALITY,  quality, 1); delay(300);

    if (!connected) { Serial.println("❌ Дроп на SetQuality!"); return; }

    

    sendCmd("DrawingMode (BE)", CMD_DRAWING_MODE, mode_img,1); delay(300);

    if (!connected) { Serial.println("❌ Дроп на DrawingMode!"); return; }



    Serial.println("✅ Init завершено!");



    // ── Тест 1: запит статусу ───────────────────────────────────

    Serial.println("\n─── Тест 1: GetStatus (A3) ───");

    sendCmd("GetStatus (A3)", CMD_GET_STATUS);

    delay(1000);

    if (!connected) { Serial.println("❌ Дроп на GetStatus!"); return; }



    // ── Тест 2: просування паперу ──────────────────────────────

    Serial.println("\n─── Тест 2: FeedPaper 30 кроків ───");

    uint8_t feed[] = {0x00, 0x1E}; // 30 кроків

    sendCmd("FeedPaper (A1)", CMD_FEED_PAPER, feed, 2);

    delay(2000);

    if (!connected) { Serial.println("❌ Дроп на FeedPaper!"); return; }



    // ── Тест 3: тестовий рядок друку ───────────────────────────

    Serial.println("\n─── Тест 3: PrintLine — суцільна чорна лінія ───");

    Serial.println("⚠️  ПРИМІТКА: DrawBitmap відправляється, але принтер НЕ ДРУКУЄ\n");

    // 50 байт = 400 пікселів, всі 0xFF = суцільно чорний рядок

    uint8_t bmp[50];

    memset(bmp, 0xFF, 50);

    sendCmd("DrawBitmap (A2)", CMD_DRAW_BITMAP, bmp, 50);

    delay(100);



    // Після кожного рядка — FeedPaper на 1 крок (як у Python)

    uint8_t feed1[] = {0x00, 0x01};

    sendCmd("FeedPaper x1 (A1)", CMD_FEED_PAPER, feed1, 2);

    delay(100);



    // Ще пара рядків для видимості

    for (int i = 0; i < 5; i++) {

        sendCmd("DrawBitmap (A2)", CMD_DRAW_BITMAP, bmp, 50);

        delay(40);

        sendCmd("FeedPaper x1", CMD_FEED_PAPER, feed1, 2);

        delay(40);

    }



    // Великий відступ в кінці щоб папір виїхав (як у Python: 0x70 0x00)

    uint8_t feed_end[] = {0x70, 0x00};

    sendCmd("FeedPaper end (A1)", CMD_FEED_PAPER, feed_end, 2);

    delay(3000);



    if (connected)

        Serial.println("\n✅ Готово! Папір має виїхати.\n❌ ДРУКУ НЕМАЄ - це v1.0 для тестування BLE");

    else

        Serial.println("\n❌ Відключення під час друку.");

}



void loop() { delay(1000); }
