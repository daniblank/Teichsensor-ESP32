#include <version.h>
#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <dht.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "esp_camera.h"
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "nvs_flash.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <esp_task_wdt.h>

//200 seconds WDT
#define WDT_TIMEOUT 200

#include "time.h"
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 3600;

#include <Syslog.h>
#define SYSLOG_SERVER "192.168.1.100"
#define SYSLOG_PORT 514
#define DEVICE_HOSTNAME "pondsensor"
#define APP_NAME "sensors"
WiFiUDP udpClient;
Syslog syslog(udpClient, SYSLOG_SERVER, SYSLOG_PORT, DEVICE_HOSTNAME, APP_NAME, LOG_KERN);  

#include "wlan_params.h"
IPAddress ip;
WiFiMulti wifiMulti;

String serverName = "http://192.168.1.100";   
String imageService = "/upload";     
String dataService = "/save";     
const int serverPort = 3456;
WiFiClient client;


#define ONE_WIRE_BUS 13
#define DHTPIN 2
#define DHTTYPE DHT22
#define HCSR04TRIGPIN 15
#define HCSR04ECHOPIN 14

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress waterThermometer;

dht DHT;
float airhumidity;
float airtemperature;
int waterlevel;
float watertemperature;
// int retry_counter = 0;
#define MAXRETRIES 10

/*
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "Arduino.h"

#include "fb_gfx.h"
#include "fd_forward.h"
#include "fr_forward.h"

typedef struct
{
  size_t size;  //number of values used for filtering
  size_t index; //current value index
  size_t count; //value count
  int sum;
  int *values; //array to be filled with values
} ra_filter_t;

typedef struct
{
  httpd_req_t *req;
  size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static ra_filter_t ra_filter;
static ra_filter_t *ra_filter_init(ra_filter_t *filter, size_t sample_size)
{
  memset(filter, 0, sizeof(ra_filter_t));

  filter->values = (int *)malloc(sample_size * sizeof(int));
  if (!filter->values)
  {
    return NULL;
  }
  memset(filter->values, 0, sample_size * sizeof(int));

  filter->size = sample_size;
  return filter;
}

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;
static mtmn_config_t mtmn_config = {0};

static int ra_filter_run(ra_filter_t *filter, int value)
{
  if (!filter->values)
  {
    return value;
  }
  filter->sum -= filter->values[filter->index];
  filter->values[filter->index] = value;
  filter->sum += filter->values[filter->index];
  filter->index++;
  filter->index = filter->index % filter->size;
  if (filter->count < filter->size)
  {
    filter->count++;
  }
  return filter->sum / filter->count;
}

static size_t jpg_encode_stream(void *arg, size_t index, const void *data, size_t len)
{
  jpg_chunking_t *j = (jpg_chunking_t *)arg;
  if (!index)
  {
    j->len = 0;
  }
  if (httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK)
  {
    return 0;
  }
  j->len += len;
  return len;
}

static esp_err_t index_handler(httpd_req_t *req)
{
  httpd_resp_set_type(req, "text/x-json");
  DynamicJsonDocument doc(1024);
  doc["airtemperature"] = airtemperature;
  doc["airhumidity"] = airhumidity;
  doc["waterlevel"] = waterlevel;
  doc["watertemperature"] = watertemperature;
  char buf[128];
  int num = serializeJson(doc, buf, sizeof(buf));
  return httpd_resp_send(req, buf, num);
}

static esp_err_t capture_handler(httpd_req_t *req)
{
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  int64_t fr_start = esp_timer_get_time();

  fb = esp_camera_fb_get();
  if (!fb)
  {
    Serial.println("Camera capture failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  size_t out_len, out_width, out_height;
  uint8_t *out_buf;
  bool s;
  bool detected = false;
  int face_id = 0;

  size_t fb_len = 0;
  if (fb->format == PIXFORMAT_JPEG)
  {
    fb_len = fb->len;
    res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  }
  else
  {
    jpg_chunking_t jchunk = {req, 0};
    res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
    httpd_resp_send_chunk(req, NULL, 0);
    fb_len = jchunk.len;
  }
  esp_camera_fb_return(fb);
  int64_t fr_end = esp_timer_get_time();
  Serial.printf("JPG: %uB %ums\n", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start) / 1000));
  return res;

  dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
  if (!image_matrix)
  {
    esp_camera_fb_return(fb);
    Serial.println("dl_matrix3du_alloc failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  out_buf = image_matrix->item;
  out_len = fb->width * fb->height * 3;
  out_width = fb->width;
  out_height = fb->height;

  s = fmt2rgb888(fb->buf, fb->len, fb->format, out_buf);
  esp_camera_fb_return(fb);
  if (!s)
  {
    dl_matrix3du_free(image_matrix);
    Serial.println("to rgb888 failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  jpg_chunking_t jchunk = {req, 0};
  s = fmt2jpg_cb(out_buf, out_len, out_width, out_height, PIXFORMAT_RGB888, 90, jpg_encode_stream, &jchunk);
  dl_matrix3du_free(image_matrix);
  if (!s)
  {
    Serial.println("JPEG compression failed");
    return ESP_FAIL;
  }

  fr_end = esp_timer_get_time();
  Serial.printf("FACE: %uB %ums %s%d\n", (uint32_t)(jchunk.len), (uint32_t)((fr_end - fr_start) / 1000), detected ? "DETECTED " : "", face_id);
  return res;
}

static esp_err_t stream_handler(httpd_req_t *req)
{
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[64];
  bool detected = false;
  int face_id = 0;
  int64_t fr_start = 0;
  int64_t fr_ready = 0;
  int64_t fr_face = 0;
  int64_t fr_recognize = 0;
  int64_t fr_encode = 0;

  static int64_t last_frame = 0;
  if (!last_frame)
  {
    last_frame = esp_timer_get_time();
  }

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK)
  {
    return res;
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true)
  {
    detected = false;
    face_id = 0;
    fb = esp_camera_fb_get();
    if (!fb)
    {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    }
    else
    {
      fr_start = esp_timer_get_time();
      fr_ready = fr_start;
      fr_face = fr_start;
      fr_encode = fr_start;
      fr_recognize = fr_start;

      if (fb->format != PIXFORMAT_JPEG)
      {
        bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
        esp_camera_fb_return(fb);
        fb = NULL;
        if (!jpeg_converted)
        {
          Serial.println("JPEG compression failed");
          res = ESP_FAIL;
        }
      }
      else
      {
        _jpg_buf_len = fb->len;
        _jpg_buf = fb->buf;
      }
    }
    if (res == ESP_OK)
    {
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK)
    {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if (res == ESP_OK)
    {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (fb)
    {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    }
    else if (_jpg_buf)
    {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK)
    {
      break;
    }
    int64_t fr_end = esp_timer_get_time();

    int64_t ready_time = (fr_ready - fr_start) / 1000;
    int64_t face_time = (fr_face - fr_ready) / 1000;
    int64_t recognize_time = (fr_recognize - fr_face) / 1000;
    int64_t encode_time = (fr_encode - fr_recognize) / 1000;
    int64_t process_time = (fr_encode - fr_start) / 1000;

    int64_t frame_time = fr_end - last_frame;
    last_frame = fr_end;
    frame_time /= 1000;
    uint32_t avg_frame_time = ra_filter_run(&ra_filter, frame_time);
    Serial.printf("MJPG: %uB %ums (%.1ffps), AVG: %ums (%.1ffps), %u+%u+%u+%u=%u %s%d\n",
                  (uint32_t)(_jpg_buf_len),
                  (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time,
                  avg_frame_time, 1000.0 / avg_frame_time,
                  (uint32_t)ready_time, (uint32_t)face_time, (uint32_t)recognize_time, (uint32_t)encode_time, (uint32_t)process_time,
                  (detected) ? "DETECTED " : "", face_id);
  }

  last_frame = 0;
  return res;
}
void startCameraServer()
{
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  httpd_uri_t index_uri = {
      .uri = "/",
      .method = HTTP_GET,
      .handler = index_handler,
      .user_ctx = NULL};

  httpd_uri_t capture_uri = {
      .uri = "/capture",
      .method = HTTP_GET,
      .handler = capture_handler,
      .user_ctx = NULL};

  httpd_uri_t stream_uri = {
      .uri = "/stream",
      .method = HTTP_GET,
      .handler = stream_handler,
      .user_ctx = NULL};

  ra_filter_init(&ra_filter, 20);

  mtmn_config.type = FAST;
  mtmn_config.min_face = 80;
  mtmn_config.pyramid = 0.707;
  mtmn_config.pyramid_times = 4;
  mtmn_config.p_threshold.score = 0.6;
  mtmn_config.p_threshold.nms = 0.7;
  mtmn_config.p_threshold.candidate_number = 20;
  mtmn_config.r_threshold.score = 0.7;
  mtmn_config.r_threshold.nms = 0.7;
  mtmn_config.r_threshold.candidate_number = 10;
  mtmn_config.o_threshold.score = 0.7;
  mtmn_config.o_threshold.nms = 0.7;
  mtmn_config.o_threshold.candidate_number = 1;

  Serial.printf("Starting web server on port: '%d'\n", config.server_port);
  if (httpd_start(&camera_httpd, &config) == ESP_OK)
  {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &capture_uri);
  }

  config.server_port += 1;
  config.ctrl_port += 1;
  Serial.printf("Starting stream server on port: '%d'\n", config.server_port);
  if (httpd_start(&stream_httpd, &config) == ESP_OK)
  {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}
*/
#define CAMERA_MODEL_AI_THINKER

#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

String sendPhoto() {
  String getAll;
  String getBody;

  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if(!fb) {
    Serial.println("Camera capture failed");
    syslog.logf(LOG_CRIT,"Camera capture failed. Restarting.");
    delay(1000);
    ESP.restart();
  }
  String serverPath = serverName ;
  //String serverPath = serverName + ":" + String(serverPort) + dataService;
  const char* post_host = "192.168.1.100";
  
  Serial.println("sendPhoto: Connecting to server: " + serverName);

  if (client.connect(post_host, serverPort)) {
    Serial.println("sendPhoto: Connection successful!");    
    String head = "--ImageBoundary\r\nContent-Disposition: form-data; name=\"imageFile\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--ImageBoundary--\r\n";

    uint32_t imageLen = fb->len;
    uint32_t extraLen = head.length() + tail.length();
    uint32_t totalLen = imageLen + extraLen;
  
    client.println("POST " + imageService + " HTTP/1.1");
    client.println("Host: " + serverName);
    client.println("Content-Length: " + String(totalLen));
    client.println("Content-Type: multipart/form-data; boundary=ImageBoundary");
    client.println();
    client.print(head);
  
    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n=0; n<fbLen; n=n+1024) {
      if (n+1024 < fbLen) {
        client.write(fbBuf, 1024);
        fbBuf += 1024;
      }
      else if (fbLen%1024>0) {
        size_t remainder = fbLen%1024;
        client.write(fbBuf, remainder);
      }
    }   
    client.print(tail);
    
    esp_camera_fb_return(fb);
    
    int timoutTimer = 10000;
    long startTimer = millis();
    boolean state = false;
    
    while ((startTimer + timoutTimer) > millis()) {
      Serial.print(".");
      delay(100);      
      while (client.available()) {
        char c = client.read();
        if (c == '\n') {
          if (getAll.length()==0) { state=true; }
          getAll = "";
        }
        else if (c != '\r') { getAll += String(c); }
        if (state==true) { getBody += String(c); }
        startTimer = millis();
      }
      if (getBody.length()>0) { break; }
    }
    Serial.println();
    client.stop();
    syslog.logf(LOG_INFO,"Image Saved. Server Message: %s", getBody.c_str());
    Serial.println(getBody);
    esp_task_wdt_reset();
  }
  else {
    getBody = "sendPhoto: Connection to " + serverName +  " failed.";
    syslog.logf(LOG_ERR,"Saving image failed. Server Message: %s", getBody.c_str());
    Serial.println(getBody);
  }
  return getBody;
}


void setup_camera()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if (psramFound())
  {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  }
  else
  {
    config.frame_size = FRAMESIZE_CIF;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    syslog.logf(LOG_CRIT,"Camera init failed. Error 0x%x", err);
    delay(1000);
    ESP.restart();
    return;
  }

  //sensor_t *s = esp_camera_sensor_get();
  //drop down frame size for higher initial frame rate
  //s->set_framesize(s, FRAMESIZE_QVGA);
}


// Set your Static IP address
IPAddress local_IP(192, 168, 1, 70);
// Set your Gateway IP address
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(8, 8, 8, 8);   //optional
IPAddress secondaryDNS(8, 8, 4, 4); //optional

void setup_wifi()
{
    // prevent Wifi connection issues
  // https://github.com/espressif/arduino-esp32/issues/2144
  ESP_ERROR_CHECK(nvs_flash_erase());
  nvs_flash_init();

  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }
  
  delay(5);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(wifi_ssid, wifi_password);
  Serial.print("Waiting for WiFi to connect...");
  while ((wifiMulti.run() != WL_CONNECTED)) {
    delay(5000);
    Serial.print(".");
    // retry_counter++;
    // if (retry_counter > MAXRETRIES){
    //   ESP.restart();
    // }  
  }
  Serial.println(" connected");
  // retry_counter = 0;
  
  ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";
        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
      })
      .onEnd([]() {
        Serial.println("\nEnd");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
          Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
          Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
          Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
          Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)
          Serial.println("End Failed");
      });

  ArduinoOTA.begin();

  //startCameraServer();
}

void printLocalTime(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}


void setup()
{
  pinMode(DHTPIN, INPUT);
  Serial.begin(115200);

  Serial.println("Configuring WDT...");
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch

  sensors.begin();

  pinMode(HCSR04TRIGPIN, OUTPUT);
  pinMode(HCSR04ECHOPIN, INPUT);
  Serial.println("Started.");
  Serial.print("Version:");
  Serial.println(VERSION);

  vTaskDelay(5000 / portTICK_RATE_MS);

  setup_wifi();
  setup_camera();

  // Init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();

  syslog.logf(LOG_INFO, "Booted Version %s",VERSION);
  syslog.log(LOG_INFO, "Setup done.");

}

static const unsigned long DHTREFRESH_INTERVAL = 10000; // ms
static unsigned long lastDHTRefreshTime = 0;

void measureDHT()
{
  if (millis() - lastDHTRefreshTime >= DHTREFRESH_INTERVAL)
  {
    lastDHTRefreshTime += DHTREFRESH_INTERVAL;
    int ret = DHT.read22(DHTPIN);
    Serial.print(F("DHT Return code:"));
    Serial.println(ret);
    if (ret == 0)
    {
      airtemperature = DHT.temperature;
      airhumidity = DHT.humidity;
    }

    if (isnan(airhumidity) || isnan(airtemperature))
    {
      Serial.println(F("Error: Could not read from DHT sensor"));
      return;
    }
    Serial.print("Air Temperature: ");
    Serial.println(airtemperature);
    Serial.print("Air Humidity: ");
    Serial.println(airhumidity);
  }
}

// Improved ultrasound measurement
// based on code by Dragos Calin
// https://www.intorobotics.com/object-detection-hc-sr04-arduino-millis/
// Improved sensor fault tolerance by averaging and eliminating zeros

// static const unsigned long ECHOREFRESH_INTERVAL = 10000; // ms, 10 sec
static const unsigned long ECHOREFRESH_INTERVAL = 10000000; // us, 10 sec
static unsigned long lastEchoRefreshTime = 0;

unsigned long startTime = 0;
int TIMER_TRIGGER_HIGH = 10;
int TIMER_LOW_HIGH = 2;
int TIMER_WAIT = 1000;

float timeDuration, distance;

/*The states of an ultrasonic sensor*/
enum SensorStates
{
  TRIG_LOW,
  TRIG_HIGH,
  ECHO_HIGH,
  WAIT
};

SensorStates _sensorState = TRIG_LOW;

void startTimer()
{
  //startTime = millis();
  startTime = micros();
}

bool isTimerReady(int uSec)
{
  // return (millis() - startTime) < mSec;
  return (micros() - startTime) < uSec;
}

static const int NUMMEASUREMENTS = 5;
int cur_mes = 0;
float ultrasound_measurements[NUMMEASUREMENTS];

float processMeasurement(float newMeasurement){
  if (newMeasurement > 50) // 50 uSec ~ 17mm. HC-SR04 detection range:20mm-4500mm
  {
    ultrasound_measurements[cur_mes] = newMeasurement * 0.34 / 2;
    cur_mes = (cur_mes+1) % NUMMEASUREMENTS;
  }
  // return average of last measurements
  int sum = 0;
  for (int i = 0; i< NUMMEASUREMENTS; i++){
    sum = sum + ultrasound_measurements[i];
  }
  return sum / NUMMEASUREMENTS;
}

void measureEcho()
{
  //long duration, distance;
  // if (millis() - lastEchoRefreshTime >= ECHOREFRESH_INTERVAL)
  if (micros() - lastEchoRefreshTime >= ECHOREFRESH_INTERVAL)
  {
    // lastEchoRefreshTime = millis();
    lastEchoRefreshTime = micros();
    Serial.print("Water Level: ");
    Serial.println(waterlevel);
  }
  /*Switch between the ultrasonic sensor states*/
  switch (_sensorState)
  {
  /* Start with LOW pulse to ensure a clean HIGH pulse*/
  case TRIG_LOW:
  {
    // Serial.println("TRIG_LOW");
    digitalWrite(HCSR04TRIGPIN, LOW);
    startTimer();
    if (isTimerReady(TIMER_LOW_HIGH))
    {
      _sensorState = TRIG_HIGH;
    }
  }
  break;

  /*Triggered a HIGH pulse of 10 microseconds*/
  case TRIG_HIGH:
  {
    // Serial.println("TRIG_HIGH");
    digitalWrite(HCSR04TRIGPIN, HIGH);
    startTimer();
    if (isTimerReady(TIMER_TRIGGER_HIGH))
    {
      _sensorState = ECHO_HIGH;
    }
  }
  break;

  /*Measures the time that ping took to return to the receiver.*/
  case ECHO_HIGH:
  {
    // Serial.println("ECHO_HIGH");
    digitalWrite(HCSR04TRIGPIN, LOW);
    timeDuration = pulseIn(HCSR04ECHOPIN, HIGH);
    /*
           distance = time * speed of sound
           speed of sound is 340 m/s => 0.034 cm/us
        */
    // waterlevel = timeDuration * 0.34 / 2;

    waterlevel = processMeasurement(timeDuration);

    // Serial.print("Distance measured is: ");
    // Serial.print(waterlevel);
    // Serial.println(" mm");
    startTimer();
    _sensorState = WAIT;
    // _sensorState = TRIG_LOW;
  }
  break;

  case WAIT:
  {
    // Serial.println("WAIT");
    if (isTimerReady(TIMER_WAIT))
    {
      _sensorState = TRIG_LOW;
    }
  }
  break;
  }
}

static const unsigned long WATERREFRESH_INTERVAL = 10000; // ms
static unsigned long lastWaterRefreshTime = 0;

void measureWater()
{
  if (millis() - lastWaterRefreshTime >= WATERREFRESH_INTERVAL)
  {
    lastWaterRefreshTime += WATERREFRESH_INTERVAL;
    sensors.requestTemperatures();
    float tempC = sensors.getTempCByIndex(0);

    // Check if reading was successful
    if (tempC != DEVICE_DISCONNECTED_C)
    {
      Serial.print("Water Temperature: ");
      Serial.println(tempC);
      watertemperature = tempC;
    }
    else
    {
      Serial.println("Error: Could not read water temperature");
    }
  }
}


void sendData(){
  DynamicJsonDocument doc(512);
  doc["airhumidity"] = airhumidity;
  doc["airtemperature"] = airtemperature;
  doc["waterlevel"] = waterlevel;
  doc["watertemperature"] = watertemperature;

  String httpRequestData;
  serializeJson(doc, httpRequestData);           
  Serial.println("sendData: "+httpRequestData);

  String serverPath = serverName + ":" + String(serverPort) + dataService;

  HTTPClient http;
  http.begin(serverPath);
  http.addHeader("Content-Type", "application/json");
  int httpCode = http.POST(httpRequestData);

  if(httpCode > 0) {
      Serial.printf("[HTTP] POST OK. Return Code %d\n", httpCode);
      syslog.logf(LOG_INFO,"POST OK. Return Code %d", httpCode);
      esp_task_wdt_reset();

      // retry_counter = 0;
      String payload = http.getString();
      Serial.println(payload);
  } else {
      Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
      syslog.logf(LOG_ERR,"POST failed, error: %s", http.errorToString(httpCode).c_str());
      // retry_counter++;
  }
  http.end();

}

unsigned long previousWLANMillis = 0;
unsigned long interval = 60000;

void loop()
{
  ArduinoOTA.handle();
  measureDHT();
  measureEcho();
  measureWater();

  if ((millis() - previousWLANMillis) > interval) {

    if((wifiMulti.run() == WL_CONNECTED)) {
      sendData();
      sendPhoto();
    }
    else {
      Serial.println("WiFi Disconnected");
      WiFi.disconnect();
      vTaskDelay(5000 / portTICK_RATE_MS);
      WiFi.reconnect();
      syslog.log(LOG_ERR,"WiFi restarted.");
      // syslog.logf(LOG_ERR,"WiFi restarted. Retry counter= %d", retry_counter);
      // retry_counter++;
    }
    // if (retry_counter > MAXRETRIES){
    //   syslog.logf(LOG_CRIT,"MAXRETRIES exceeded. Restarting.");
    //   ESP.restart();
    // }     
    previousWLANMillis = millis();
  }
}