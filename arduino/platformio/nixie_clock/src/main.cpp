
#include <WiFi.h>
#include <soc/rtc.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFiManager.h>

// Replace with your network credentials
const char* ssid     = "ASUS";
const char* password = "spektr6791gradient52";

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 10800;
const int   daylightOffset_sec = 0;

void hv_pwm_init(void) {
  ledcSetup(0, 50000, 8);
  ledcAttachPin(5, 0);
  ledcAttachPin(25, 0);
  ledcAttachPin(26, 0);
  ledcAttachPin(27, 0);
  ledcWrite(0, 127);
}

uint8_t addr_pins[] = {10, 4, 2, 9};
uint8_t lamp_pins[] = {18, 23, 19, 22};
uint8_t num_map[10] = {8, 9, 5, 6, 4, 1, 0, 3, 7, 2};

void init_hw_staff(void) {
  for (uint8_t i = 0; i < sizeof(addr_pins); i++) {
    pinMode(addr_pins[i], OUTPUT);
  }
  for (uint8_t i = 0; i < sizeof(lamp_pins); i++) {
    pinMode(lamp_pins[i], OUTPUT);
  }
  digitalWrite(lamp_pins[0], HIGH); // hv should be loaded
  hv_pwm_init();
}

void connect_wifi(void) {
  // Initialize Serial Monitor
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

#define CALIBRATE_ONE(cali_clk) calibrate_one(cali_clk, #cali_clk)

static uint32_t calibrate_one(rtc_cal_sel_t cal_clk, const char *name)
{

  const uint32_t cal_count = 1000;
  const float factor = (1 << 19) * 1000.0f;
  uint32_t cali_val;
  printf("%s:\n", name);
  for (int i = 0; i < 5; ++i)
  {
    printf("calibrate (%d): ", i);
    cali_val = rtc_clk_cal(cal_clk, cal_count);
    printf("%.3f kHz\n", factor / (float)cali_val);
  }
  return cali_val;
}

void init_time(void) {
  rtc_clk_32k_bootstrap(512);
  rtc_clk_32k_bootstrap(512);
  rtc_clk_32k_enable(true);

  uint32_t cal_32k = CALIBRATE_ONE(RTC_CAL_32K_XTAL);
  rtc_clk_slow_freq_set(RTC_SLOW_FREQ_32K_XTAL);

  if (cal_32k == 0)
  {
    printf("32K XTAL OSC has not started up");
  }
  else
  {
    printf("done\n");
  }

  if (rtc_clk_32k_enabled())
  {
    Serial.println("OSC Enabled");
  }
}

uint8_t addr_cnt, lamp_cnt;
void test_digits(void) {
  if (addr_cnt == 10) {
    addr_cnt = 0;
    digitalWrite(lamp_pins[lamp_cnt], LOW);
    lamp_cnt++;
    if (lamp_cnt == sizeof(lamp_pins)) lamp_cnt = 0;
    digitalWrite(lamp_pins[lamp_cnt], HIGH);
  }
  for (uint8_t i = 0; i < sizeof(addr_pins); i++) {
    digitalWrite(addr_pins[i], num_map[addr_cnt] & (1 << i));
  }
  addr_cnt++;
}

void printLocalTime()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

struct tm timeinfo;

//  int tm_sec;
//  int tm_min;
//  int tm_hour;

void IRAM_ATTR onTimer() {
  static uint8_t cur_lamp;
  static struct tm _timeinfo;
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  _timeinfo = timeinfo;
  portEXIT_CRITICAL_ISR(&timerMux);

  digitalWrite(lamp_pins[cur_lamp], LOW);
  if(++cur_lamp == 4) cur_lamp = 0;
  int temp;
  if(cur_lamp < 2) temp = _timeinfo.tm_hour;
  else temp = _timeinfo.tm_min;
  uint8_t num;
  if(cur_lamp % 2) num = temp % 10;
  else num = temp / 10; 
  for (uint8_t i = 0; i < sizeof(addr_pins); i++) {
    digitalWrite(addr_pins[i], num_map[num] & (1 << i));
  }
  digitalWrite(lamp_pins[cur_lamp], HIGH);
  
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output
}

void init_timer(void) {
  timerSemaphore = xSemaphoreCreateBinary();
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 2000, true);
  timerAlarmEnable(timer);
}

void setup() {
  Serial.begin(115200);
  init_hw_staff();

  WiFiManager wifiManager;
  wifiManager.autoConnect("AutoConnectAP");
  //connect_wifi();

  init_time();
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  init_timer();
}

void update_time(void) {
  static struct tm _timeinfo;
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE) {
    getLocalTime(&_timeinfo);
    printf("%d:%d:%d\n", _timeinfo.tm_hour, _timeinfo.tm_min, _timeinfo.tm_sec);
    portENTER_CRITICAL(&timerMux);
    timeinfo = _timeinfo;
    portEXIT_CRITICAL(&timerMux);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
//    test_digits();
  //  printLocalTime();
  update_time();
  delay(1000);
  printf("working\n");
}