#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <NeoPixelBus.h>
#include "settings.h"

#define UART_RATE   115200
const char wifi_dhcp_name[] = WIFI_DHCP_NAME;
const char wifi_essid[] = WIFI_ESSID;
const char wifi_passphrase[] = WIFI_PASSWORD;

#define LED_COUNT  (COLS*ROWS)
#define LED_BYTES (BYPP*LED_COUNT)

#define BYPP 3
NeoPixelBus<NeoGrbFeature, NeoEsp8266Dma800KbpsMethod> strip(LED_COUNT, LED_PIN);

void LEDPreprocess(void)
{
  /* put any data preprocessing needed here */
  // this reverses the row order of even columns (starting from 0)
  if (!strip.IsDirty()) return;
  uint8_t *buf = strip.Pixels();
  uint8_t tmp[BYPP];
  for (int i = 0; i < COLS; i += 2) {
    for (int j = 0; j < ROWS/2; ++j) {
      int idx0 = BYPP*(COLS*j + i);
      int idx1 = BYPP*(COLS*(ROWS - j - 1) + i);
      memcpy(tmp, &buf[idx0], BYPP);
      memcpy(&buf[idx0], &buf[idx1], BYPP);
      memcpy(&buf[idx1], tmp, BYPP);
    }
  }
}

WiFiUDP udp;

static bool wifi_started = false;

void setup() {
  //Serial.begin(UART_RATE);
  //Serial.println("Starting up");

  wifi_station_set_hostname((char*)wifi_dhcp_name);

  strip.Begin();

  WiFi.disconnect(true);
  //Serial.println("WiFi");
  WiFi.begin(wifi_essid, wifi_passphrase);
}

void loop() {
  int status = WiFi.status();
  if (status != WL_CONNECTED) {
    if (wifi_started) {
      //Serial.println("Disconnected");
      wifi_started = false;
      udp.stop();

      uint8_t *buf = strip.Pixels();
      memset(buf, 0, LED_BYTES);
      LEDPreprocess();
      strip.Show();
    }
    delay(500);
    return;
  } else if (!wifi_started) {
    IPAddress ip = WiFi.localIP();
    //Serial.print("Local IP: ");
    //Serial.println(ip);
    wifi_started = true;
    udp.begin(UDP_PORT);
  }

  int count;
  while ((count = udp.parsePacket()) > 0) {
    //Serial.printf("%d byte packet\r\n", count);
    if (count != LED_BYTES) {
      udp.flush();
      continue;
    }

    uint8_t *buf = strip.Pixels();
    count = udp.read(buf, LED_BYTES);
    if (count != LED_BYTES) {
      /* error */
      //Serial.printf("Only read %d bytes!\r\n", count);
    } else {
      strip.Dirty();
      LEDPreprocess();
      strip.Show();
    }

    udp.flush();
  }
}

