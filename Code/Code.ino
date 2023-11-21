/*
 * Project Name: Wireless Battery Monitor
 * Project Brief: Firmware for Wireless Battery Monitor built around ESP32 and AD7280
 * Author: Jobit Joseph
 * Copyright © Jobit Joseph
 * Copyright © Semicon Media Pvt Ltd
 * Copyright © Circuitdigest.com
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

/* AD7280 CS pin Definition */
#ifndef _SS
#define _SS 5
#endif
/* AD7280 CS pin Definition */

/* AD7280 Register mapping */
#define _PDEBUG
#define AD7280A_ACQ_TIME_400ns 0
#define AD7280A_ACQ_TIME_800ns 1
#define AD7280A_ACQ_TIME_1200ns 2
#define AD7280A_ACQ_TIME_1600ns 3
#define AD7280A_CONV_AVG_DIS 0
#define AD7280A_CONV_AVG_2 1
#define AD7280A_CONV_AVG_4 2
#define AD7280A_CONV_AVG_8 3
#define AD7280A_ALERT_REMOVE_VIN5 (1 << 2)
#define AD7280A_ALERT_REMOVE_VIN4_VIN5 (2 << 2)
#define AD7280A_ALERT_REMOVE_AUX5 (1 << 0)
#define AD7280A_ALERT_REMOVE_AUX4_AUX5 (2 << 0)
#define AD7280A_CELL_VOLTAGE_1 0x0        /* D11 to D0, Read only */
#define AD7280A_CELL_VOLTAGE_2 0x1        /* D11 to D0, Read only */
#define AD7280A_CELL_VOLTAGE_3 0x2        /* D11 to D0, Read only */
#define AD7280A_CELL_VOLTAGE_4 0x3        /* D11 to D0, Read only */
#define AD7280A_CELL_VOLTAGE_5 0x4        /* D11 to D0, Read only */
#define AD7280A_CELL_VOLTAGE_6 0x5        /* D11 to D0, Read only */
#define AD7280A_AUX_ADC_1 0x6             /* D11 to D0, Read only */
#define AD7280A_AUX_ADC_2 0x7             /* D11 to D0, Read only */
#define AD7280A_AUX_ADC_3 0x8             /* D11 to D0, Read only */
#define AD7280A_AUX_ADC_4 0x9             /* D11 to D0, Read only */
#define AD7280A_AUX_ADC_5 0xA             /* D11 to D0, Read only */
#define AD7280A_AUX_ADC_6 0xB             /* D11 to D0, Read only */
#define AD7280A_SELF_TEST 0xC             /* D11 to D0, Read only */
#define AD7280A_CONTROL_HB 0xD            /* D15 to D8, Read/write */
#define AD7280A_CONTROL_LB 0xE            /* D7 to D0, Read/write */
#define AD7280A_CELL_OVERVOLTAGE 0xF      /* D7 to D0, Read/write */
#define AD7280A_CELL_UNDERVOLTAGE 0x10    /* D7 to D0, Read/write */
#define AD7280A_AUX_ADC_OVERVOLTAGE 0x11  /* D7 to D0, Read/write */
#define AD7280A_AUX_ADC_UNDERVOLTAGE 0x12 /* D7 to D0, Read/write */
#define AD7280A_ALERT 0x13                /* D7 to D0, Read/write */
#define AD7280A_CELL_BALANCE 0x14         /* D7 to D0, Read/write */
#define AD7280A_CB1_TIMER 0x15            /* D7 to D0, Read/write */
#define AD7280A_CB2_TIMER 0x16            /* D7 to D0, Read/write */
#define AD7280A_CB3_TIMER 0x17            /* D7 to D0, Read/write */
#define AD7280A_CB4_TIMER 0x18            /* D7 to D0, Read/write */
#define AD7280A_CB5_TIMER 0x19            /* D7 to D0, Read/write */
#define AD7280A_CB6_TIMER 0x1A            /* D7 to D0, Read/write */
#define AD7280A_PD_TIMER 0x1B             /* D7 to D0, Read/write */
#define AD7280A_READ 0x1C                 /* D7 to D0, Read/write */
#define AD7280A_CNVST_CONTROL 0x1D        /* D7 to D0, Read/write */
/* Bits and Masks */
#define AD7280A_CTRL_HB_CONV_INPUT_ALL (0 << 6)
#define AD7280A_CTRL_HB_CONV_INPUT_6CELL_AUX1_3_4 (1 << 6)
#define AD7280A_CTRL_HB_CONV_INPUT_6CELL (2 << 6)
#define AD7280A_CTRL_HB_CONV_INPUT_SELF_TEST (3 << 6)
#define AD7280A_CTRL_HB_CONV_RES_READ_ALL (0 << 4)
#define AD7280A_CTRL_HB_CONV_RES_READ_6CELL_AUX1_3_4 (1 << 4)
#define AD7280A_CTRL_HB_CONV_RES_READ_6CELL (2 << 4)
#define AD7280A_CTRL_HB_CONV_RES_READ_NO (3 << 4)
#define AD7280A_CTRL_HB_CONV_START_CNVST (0 << 3)
#define AD7280A_CTRL_HB_CONV_START_CS (1 << 3)
#define AD7280A_CTRL_HB_CONV_AVG_DIS (0 << 1)
#define AD7280A_CTRL_HB_CONV_AVG_2 (1 << 1)
#define AD7280A_CTRL_HB_CONV_AVG_4 (2 << 1)
#define AD7280A_CTRL_HB_CONV_AVG_8 (3 << 1)
#define AD7280A_CTRL_HB_CONV_AVG(x) ((x) << 1)
#define AD7280A_CTRL_HB_PWRDN_SW (1 << 0)
#define AD7280A_CTRL_LB_SWRST (1 << 7)
#define AD7280A_CTRL_LB_ACQ_TIME_400ns (0 << 5)
#define AD7280A_CTRL_LB_ACQ_TIME_800ns (1 << 5)
#define AD7280A_CTRL_LB_ACQ_TIME_1200ns (2 << 5)
#define AD7280A_CTRL_LB_ACQ_TIME_1600ns (3 << 5)
#define AD7280A_CTRL_LB_ACQ_TIME(x) ((x) << 5)
#define AD7280A_CTRL_LB_MUST_SET (1 << 4)
#define AD7280A_CTRL_LB_THERMISTOR_EN (1 << 3)
#define AD7280A_CTRL_LB_LOCK_DEV_ADDR (1 << 2)
#define AD7280A_CTRL_LB_INC_DEV_ADDR (1 << 1)
#define AD7280A_CTRL_LB_DAISY_CHAIN_RB_EN (1 << 0)
#define AD7280A_ALERT_GEN_STATIC_HIGH (1 << 6)
#define AD7280A_ALERT_RELAY_SIG_CHAIN_DOWN (3 << 6)
#define AD7280A_ALL_CELLS (0x00AD << 16)
#define AD7280A_MAX_SPI_CLK_Hz 700000 /* < 1MHz */
#define AD7280A_MAX_CHAIN 8
#define AD7280A_CELLS_PER_DEV 6
#define AD7280A_BITS 12
#define AD7280A_NUM_CH (AD7280A_AUX_ADC_6 - AD7280A_CELL_VOLTAGE_1 + 1)
#define AD7280A_DEVADDR_MASTER 0
#define AD7280A_DEVADDR_ALL 0x1F
/* 5-bit device address is sent LSB first */
#define AD7280A_DEVADDR(addr) (((addr & 0x1) << 4) | ((addr & 0x2) << 3) | (addr & 0x4) | ((addr & 0x8) >> 3) | ((addr & 0x10) >> 4))
#define AD7280A_READ_TXVAL 0xF800030A
#define POLYNOM 0x2F
#define POLYNOM_ORDER 8
#define HIGHBIT 1 << (POLYNOM_ORDER - 1);
/* AD7280 Register mapping */

/* AD7280 data structure */
struct ad7280_state {
  int16_t slave_num;
  int16_t scan_cnt;
  uint8_t readback_delay_ms;
  uint8_t crc_tab[256];
  uint8_t ctrl_hb;
  uint8_t ctrl_lb;
  uint8_t cell_threshhigh;
  uint8_t cell_threshlow;
  uint8_t aux_threshhigh;
  uint8_t aux_threshlow;
  uint8_t cb_mask[AD7280A_MAX_CHAIN];
  struct spi_device *spi;
};
/* AD7280 data structure */

struct spi_device {
  int8_t spi;
};

//Global Variables
int LEDPins[] = {15, 13, 12, 14};
unsigned long previousMillis = 0;
unsigned long lastMillis = 0;
unsigned long interval = 30000;
struct ad7280_state ADinst;
uint16_t valCh[12];  

//WiFi Credentials
const char *ssid = "CircuitDigest";
const char *password = "circuitdigest123";

//Async Webserver instance
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

/*Wireless battery monitor Web page*/
const char *htmlPage = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta http-equiv="X-UA-Compatible" content="IE=edge">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Wireless Battery Monito</title>
  <script src="https://cdn.jsdelivr.net/npm/reconnecting-websocket@4.4.0/dist/reconnecting-websocket.min.js"></script>
  <style>
    body {
      display: flex;
      justify-content: center;
      align-items: center;
      height: 50vh;
      margin: 0;
      font-family: Arial, sans-serif;
      flex-wrap: wrap;
    }
    h1 {
      text-align: center;
      width: 100%;
      font-size: 24px;
      margin-top: 20px;
    }
    .total-voltage-container {
      width: 100%;
      text-align: center;
      font-size: 20px;
      font-weight: bold;
      margin: 10px 0;
    }
    .battery-container {
      display: flex;
      flex-direction: column;
      align-items: center;
      margin: 20px;
    }
    .battery {
      width: 100px;
      height: 200px;
      position: relative;
      border: 3px solid black;
    }
    .battery:after {
      content: "";
      position: absolute;
      top: -15px;
      left: 35px;
      width: 30px;
      height: 10px;
      background: black;
    }
    .battery-level {
      width: 100%;
      position: absolute;
      bottom: 0;
      background: green;
      text-align: center;
      color: white;
      font-weight: bold;
    }
    .voltage {
      font-size: 18px;
      margin-top: 10px;
    }
  </style>
</head>
<body>
  <h1>Wireless Battery Monitor</h1>
  <div class="total-voltage-container">
    <div id="totalVoltage" class="voltage">Total Voltage: 0mV</div>
    <div id="totalPercentage" class="voltage">Total Percentage: 0%</div>
  </div>
  <div class="battery-container">
    <div id="battery1" class="battery"><div class="battery-level" style="height: 0%;">0%</div></div>
    <div id="voltage1" class="voltage">0mV</div>
  </div>
  <div class="battery-container">
    <div id="battery2" class="battery"><div class="battery-level" style="height: 0%;">0%</div></div>
    <div id="voltage2" class="voltage">0mV</div>
  </div>
  <div class="battery-container">
    <div id="battery3" class="battery"><div class="battery-level" style="height: 0%;">0%</div></div>
    <div id="voltage3" class="voltage">0mV</div>
  </div>
  <div class="battery-container">
    <div id="battery4" class="battery"><div class="battery-level" style="height: 0%;">0%</div></div>
    <div id="voltage4" class="voltage">0mV</div>
  </div>
  
<script>
  var ws = new WebSocket('ws://' + location.hostname + '/ws');
  ws.onopen = function() {
    console.log("WebSocket connection opened");
  };
  ws.onclose = function() {
    console.log("WebSocket connection closed");
    setTimeout(function() {
      ws = new WebSocket('ws://' + location.hostname + '/ws');
    }, 5000);
  };
  ws.onmessage = function(event) {
    console.log("Data received:", event.data);
    var data = JSON.parse(event.data);
    for (var i = 1; i <= 4; i++) {
      var battery = document.getElementById('battery' + i);
      var level = battery.querySelector('.battery-level');
      var voltage = document.getElementById('voltage' + i);
      level.style.height = data.percentages[i - 1] + '%';
      level.textContent = data.percentages[i - 1] + '%';
      voltage.textContent = data.voltages[i - 1] + 'mV';
    }
    document.getElementById('totalVoltage').textContent = "Total Voltage: " + data.totalVoltage + "mV";
    document.getElementById('totalPercentage').textContent = "Total Percentage: " + data.totalPercentage + "%";
  };
</script>
</body>
</html>
)rawliteral";
/*Wireless battery monitor Web page*/

/*Websocket event handler*/
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.println("Websocket client connection received");
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.println("Client disconnected");
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    String msg = "";
    if (info->final && info->index == 0 && info->len == len) {
      for (size_t i = 0; i < info->len; i++) {
        msg += (char)data[i];
      }
      Serial.println(msg);
    }
  }
}

/* SPI Setup function */
void setup_spi32() {
  SPI.begin();
  /*
  SPI.setDataMode(SPI_MODE1);
  SPI.setBitOrder(MSBFIRST);             // MSB first.
  SPI.setClockDivider(SPI_CLOCK_DIV16);  // 1 Mhz Clk fo SPI. 16Mhz Div 16 = 1 Mhz.s
  */
  SPISettings mySPISettings(500000, MSBFIRST, SPI_MODE1); // 500 kHz speed, MSB first, Mode 1
  // Begin SPI transaction with the defined settings
  SPI.beginTransaction(mySPISettings);
  pinMode(_SS, OUTPUT);
}

/* 32bit to 4x 8bit conversion*/
void transferspi32(uint32_t *val) {
  byte h_h;  // D31-D24 received
  byte h_l;  // D23-D16 received
  byte l_h;  // D15 -D8 received
  byte l_l;  // D07 -D0 received

#ifndef _PDEBUG
  h_h = ((*val) >> 24) & 0xFF;
  h_l = ((*val) >> 16) & 0xFF;
  l_h = ((*val) >> 8) & 0xFF;
  l_l = (*val) & 0xFF;
  Serial.println("=================");
  Serial.println("Sending SPI32bit");
  Serial.print("HEX: ");
  Serial.print(h_h, HEX);
  Serial.print(",");
  Serial.print(h_l, HEX);
  Serial.print(",");
  Serial.print(l_h, HEX);
  Serial.print(",");
  Serial.println(l_l, HEX);
  Serial.print("BIN: ");
  Serial.print(h_h, BIN);
  Serial.print(",");
  Serial.print(h_l, BIN);
  Serial.print(",");
  Serial.print(l_h, BIN);
  Serial.print(",");
  Serial.println(l_l, BIN);
#endif

  digitalWrite(_SS, LOW);
  h_h = SPI.transfer(((*val) >> 24) & 0xFF);  // D31-D24
  h_l = SPI.transfer(((*val) >> 16) & 0xFF);  // D23-D16
  l_h = SPI.transfer(((*val) >> 8) & 0xFF);   // D15-D08
  l_l = SPI.transfer((*val) & 0xFF);          // D07-D00
  digitalWrite(_SS, HIGH);
  *val = (uint32_t)(((uint32_t)h_h << 24) | ((uint32_t)h_l << 16) | ((uint32_t)l_h << 8) | l_l);

#ifndef _PDEBUG

  Serial.println("Receiving SPI32bit");
  Serial.print("HEX: ");
  Serial.print(h_h, HEX);
  Serial.print(",");
  Serial.print(h_l, HEX);
  Serial.print(",");
  Serial.print(l_h, HEX);
  Serial.print(",");
  Serial.println(l_l, HEX);
  Serial.print("BIN: ");
  Serial.print(h_h, BIN);
  Serial.print(",");
  Serial.print(h_l, BIN);
  Serial.print(",");
  Serial.print(l_h, BIN);
  Serial.print(",");
  Serial.println(l_l, BIN);
#endif

  return;  // Always return zero. Even if the things are not successful
}

// Builds the CRC table. Returns not relevant. But fills the crc_tab
static void ad7280_crc8_build_table(uint8_t *crc_tab) {
  uint8_t bit, crc;
  int16_t cnt, i;

  for (cnt = 0; cnt < 256; cnt++) {
    crc = cnt;
    for (i = 0; i < 8; i++) {
      bit = crc & HIGHBIT;
      crc <<= 1;
      if (bit)
        crc ^= POLYNOM;
    }
    crc_tab[cnt] = crc;
  }
}

// Calculates the CRC for the transmission packets: Returns CRC(8bit)
static uint8_t ad7280_calc_crc8(uint8_t *crc_tab, uint32_t val) {
  uint8_t crc;

  crc = crc_tab[(uint16_t)(val >> 16 & 0xFF)];
  crc = crc_tab[crc ^ (val >> 8 & 0xFF)];

  return crc ^ (val & 0xFF);
}

// Check crc routine for the received packets. Returns 0 if correct else -1.
static int8_t ad7280_check_crc(struct ad7280_state *st, uint32_t val) {
  uint8_t crc = ad7280_calc_crc8(st->crc_tab, val >> 10);

  if (crc != ((val >> 2) & 0xFF))
    return -1;

  return 0;
}

// Delay
static void ad7280_delay(struct ad7280_state *st) {
  delay(st->readback_delay_ms);
}

// Send valid frame on SPI. Returns not relevant. but val will contain the received value.
static int8_t __ad7280_read32(struct spi_device *spi, uint32_t *val) {
  *val = AD7280A_READ_TXVAL;  //0xF800030A
  transferspi32(val);         // recieved value will be in <val>
  return 0;
}

// Send Data (val) to devices. Returns not relevant.
static int8_t ad7280_write(struct ad7280_state *st, uint32_t devaddr,
                           uint32_t addr, uint8_t all, uint32_t val) {
  uint32_t reg = (devaddr << 27 | addr << 21 | (val & 0xFF) << 13 | all << 12);

  reg |= ad7280_calc_crc8(st->crc_tab, reg >> 11) << 3 | 0x2;
  reg = (uint32_t)(reg);

  //return spi_write(st->spi, &reg, 4);  Wrtite to SPI.
  transferspi32(&reg);
  return 0;
}

// Read from a specific device register : Returns register value
static int8_t ad7280_read(struct ad7280_state *st, uint32_t devaddr,
                          uint32_t addr) {
  uint32_t tmp;

  /* turns off the read operation on all parts */
  ad7280_write(st, AD7280A_DEVADDR_MASTER, AD7280A_CONTROL_HB, 1,
               AD7280A_CTRL_HB_CONV_INPUT_6CELL | AD7280A_CTRL_HB_CONV_RES_READ_NO | st->ctrl_hb);


  /* turns on the read operation on the addressed part */
  ad7280_write(st, devaddr, AD7280A_CONTROL_HB, 0,
               AD7280A_CTRL_HB_CONV_INPUT_6CELL | AD7280A_CTRL_HB_CONV_RES_READ_6CELL | st->ctrl_hb);


  /* Set the address of the register on the device to be read from */
  ad7280_write(st, devaddr, AD7280A_READ, 0, addr << 2);

  /* Valid read so that the value can be received*/
  __ad7280_read32(st->spi, &tmp);

  if (ad7280_check_crc(st, tmp))  // Check the received CRC
    return -1;

  if (((tmp >> 27) != devaddr) || (((tmp >> 21) & 0x3F) != addr))
    return -1;  // Check if the received device and register address is correct.

  return (tmp >> 13) & 0xFF;
}

// Read from an Analog channel with start of Conversion as CS start. Returns the value of the ADC count received.
static uint16_t ad7280_read_channel(struct ad7280_state *st, uint32_t devaddr,
                                    uint32_t addr) {
  int8_t ret;
  uint32_t tmp, temp2;

  /* Write on Read Register of "the device" the Register address of channel*/
  ret = ad7280_write(st, devaddr, AD7280A_READ, 0, addr << 2);

  /* Write on Ctrl_HB of "all device" to convert all but not to read all.*/
  ret = ad7280_write(st, AD7280A_DEVADDR_MASTER, AD7280A_CONTROL_HB, 1,
                     AD7280A_CTRL_HB_CONV_INPUT_6CELL | AD7280A_CTRL_HB_CONV_RES_READ_NO | st->ctrl_hb);

  /* Write on Ctrl_HB of "the device" to convert all and to read all 
	and to start the conversion as CS start*/
  ret = ad7280_write(st, devaddr, AD7280A_CONTROL_HB, 0,
                     AD7280A_CTRL_HB_CONV_INPUT_6CELL | AD7280A_CTRL_HB_CONV_RES_READ_6CELL | AD7280A_CTRL_HB_CONV_START_CS | st->ctrl_hb);

  /*Delay*/
  ad7280_delay(st);

  /*Valid write to get the channel value*/
  __ad7280_read32(st->spi, &tmp);

  if (ad7280_check_crc(st, tmp))  // Check the received CRC
    return 0;

  if (((tmp >> 27) != devaddr) || (((tmp >> 23) & 0xF) != addr))
    return 1;  // Check if the received device and register address is correct.
  temp2 = (tmp >> 11) & 0xFFF;
  temp2 = temp2 + 935;
  if (temp2 < 936) {
    return 0;
  }
  return temp2;
}

// Read from all devices the channels of VIN and AUX at CS frame start. Returns the sum of all the voltages.
static uint32_t ad7280_read_all_channels(struct ad7280_state *st, uint32_t cnt,
                                         uint16_t *array) {
  uint8_t ret;
  uint32_t i;
  uint32_t tmp, sum = 0;
  /* Write to all the read registers of all the devices the address
	of first channel register i.e AD7280A_CELL_VOLTAGE_1*/
  ret = ad7280_write(st, AD7280A_DEVADDR_MASTER, AD7280A_READ, 1,
                     AD7280A_CELL_VOLTAGE_1 << 2);

  /*Write to all the Control HB of all the devices, 
	select all channels, read all, Start conversion at CS Start*/
  ret = ad7280_write(st, AD7280A_DEVADDR_MASTER, AD7280A_CONTROL_HB, 1,
                     AD7280A_CTRL_HB_CONV_INPUT_6CELL | AD7280A_CTRL_HB_CONV_RES_READ_6CELL | AD7280A_CTRL_HB_CONV_START_CS | st->ctrl_hb);

  ad7280_delay(st);

  for (i = 0; i < cnt; i++) {
    __ad7280_read32(st->spi, &tmp);

    if (ad7280_check_crc(st, tmp))
      return 0xFFF;

    array[i] = ((tmp >> 11) & 0xFFF);
    /* only sum cell voltages */
    if (((tmp >> 23) & 0xF) <= AD7280A_CELL_VOLTAGE_6) {
      sum += ((tmp >> 11) & 0xFFF);
      //Serial.println(sum,HEX); //
    }
  }

  return sum;
}

/* 4x 8bit to 32bit conversion */
void show32bit(uint32_t *val) {
  byte h_h;  // D31-D24 received
  byte h_l;  // D23-D16 received
  byte l_h;  // D15 -D8 received
  byte l_l;  // D07 -D0 received

  h_h = ((*val) >> 24) & 0xFF;
  h_l = ((*val) >> 16) & 0xFF;
  l_h = ((*val) >> 8) & 0xFF;
  l_l = (*val) & 0xFF;
  Serial.print("HEX: ");
  Serial.print(h_h, HEX);
  Serial.print(",");
  Serial.print(h_l, HEX);
  Serial.print(",");
  Serial.print(l_h, HEX);
  Serial.print(",");
  Serial.println(l_l, HEX);
  Serial.print("BIN: ");
  Serial.print(h_h, BIN);
  Serial.print(",");
  Serial.print(h_l, BIN);
  Serial.print(",");
  Serial.print(l_h, BIN);
  Serial.print(",");
  Serial.println(l_l, BIN);
  return;
}

/* AD7280 initialization function */
void init_ad7280() {
  uint32_t val;
  byte a;
  val = 0x01C2B6E2;
  transferspi32(&val);
  Serial.print("Check routine returns ");
  a = ad7280_check_crc(&ADinst, val);
  Serial.println(a, BIN);

  val = 0x038716CA;
  transferspi32(&val);
  Serial.print("Check routine returns ");
  a = ad7280_check_crc(&ADinst, val);
  Serial.println(a, BIN);

  val = 0xF800030A;
  transferspi32(&val);
  Serial.print("Check routine returns ");
  a = ad7280_check_crc(&ADinst, val);
  Serial.println(a, BIN);

  val = 0xF800030A;
  transferspi32(&val);
  Serial.print("Check routine returns ");
  a = ad7280_check_crc(&ADinst, val);
  Serial.println(a, BIN);
  delay(100);
  ad7280_crc8_build_table(&ADinst.crc_tab[0]);
  ADinst.scan_cnt = AD7280A_NUM_CH;
  delay(10);
  ad7280_write(&ADinst, AD7280A_DEVADDR(0),
               AD7280A_CELL_BALANCE, 0, 0);
  Serial.println("=================");
}

/* Setup function */
void setup() {
  Serial.begin(115200); //init serial com
  pinMode(12, OUTPUT); //set pinmode
  pinMode(13, OUTPUT); //set pinmode
  pinMode(14, OUTPUT); //set pinmode
  pinMode(15, OUTPUT); //set pinmode
  setup_spi32(); //Call SPI setup function

  /* AD7280 init register values */
  ADinst.readback_delay_ms = 10;  // 10 ms wait time.
  ADinst.cell_threshhigh = 0xFF;
  ADinst.aux_threshhigh = 0xFF;
  ADinst.ctrl_hb = 0x00;
  ADinst.ctrl_lb = 0x10;  // D4 of Ctrl_lb must be 1 ( reserved Bit )
  ADinst.cell_threshlow = 0x00;
  ADinst.aux_threshlow = 0x00;
  /* AD7280 init register values */

  /* WiFi Setup */
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print('.');
  }
  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  /* WiFi Setup */

  /* Webserver Setup */
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", htmlPage);
  });
  server.begin();
  /* Webserver Setup */

  init_ad7280(); //Call AD7280 initialization function
}

/* Loop function */
void loop() {
    unsigned long currentMillis = millis();
  // try to reconnect if WiFi is disconnected
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >=interval)) {
    Serial.print(millis());
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    previousMillis = currentMillis;
  }
  init_ad7280()
  /* Read cell voltages from AD7280, map it to corresponding variable and calculate charge status */
  int batteryVoltages[4];
  batteryVoltages[0] = ad7280_read_channel(&ADinst, AD7280A_DEVADDR_MASTER, AD7280A_CELL_VOLTAGE_1);
  batteryVoltages[1] = ad7280_read_channel(&ADinst, AD7280A_DEVADDR_MASTER, AD7280A_CELL_VOLTAGE_2);
  batteryVoltages[1] = ad7280_read_channel(&ADinst, AD7280A_DEVADDR_MASTER, AD7280A_CELL_VOLTAGE_2);
  batteryVoltages[2] = ad7280_read_channel(&ADinst, AD7280A_DEVADDR_MASTER, AD7280A_CELL_VOLTAGE_3);
  batteryVoltages[3] = ad7280_read_channel(&ADinst, AD7280A_DEVADDR_MASTER, AD7280A_CELL_VOLTAGE_6);
  int batteryPercentages[] = { map(batteryVoltages[0], 3200, 4200, 0, 100), map(batteryVoltages[1], 3200, 4200, 0, 100), map(batteryVoltages[2], 3200, 4200, 0, 100), map(batteryVoltages[3], 3200, 4200, 0, 100) };
/*ignore any values with CRC error */
  if(batteryPercentages[0] < 0 || batteryPercentages[1] < 0 || batteryPercentages[2] < 0 || batteryPercentages[3] < 0)
  {
	  return;
  }
  if(millis() - lastMillis > 500)
  {
    for(int i = 0; i < 4; i++)
    {
      if(batteryPercentages[i] < 100)
      {
        digitalWrite(LEDPins[i], !digitalRead(LEDPins[i]));
      }
      else if(batteryPercentages[i] = 100)
      {
        digitalWrite(LEDPins[i], HIGH);
      }
      else if(batteryPercentages[i] <= 0)
      {
        digitalWrite(LEDPins[i], LOW);
      }
    }
  }
  int totalVoltage = batteryVoltages[0] + batteryVoltages[1] + batteryVoltages[2] + batteryVoltages[3];
  int totalPercentage = (batteryPercentages[0] + batteryPercentages[1] + batteryPercentages[2] + batteryPercentages[3]) / 4;
  if (totalPercentage < 0) {
    totalPercentage = 0;
  }
  Serial.print("Voltages: ");
  Serial.print(batteryVoltages[0]);
  Serial.print(", ");
  Serial.print(batteryVoltages[1]);
  Serial.print(", ");
  Serial.print(batteryVoltages[2]);
  Serial.print(", ");
  Serial.println(batteryVoltages[3]);

  Serial.print("Percentages: ");
  Serial.print(batteryPercentages[0]);
  Serial.print(", ");
  Serial.print(batteryPercentages[1]);
  Serial.print(", ");
  Serial.print(batteryPercentages[2]);
  Serial.print(", ");
  Serial.println(batteryPercentages[3]);
  
  /* send voltage levels and charge level to webpage */
  String jsonData = "{\"voltages\": [" + String(batteryVoltages[0]) + "," + String(batteryVoltages[1]) + "," + String(batteryVoltages[2]) + "," + String(batteryVoltages[3]) + "], \"percentages\": [" + String(batteryPercentages[0]) + "," + String(batteryPercentages[1]) + "," + String(batteryPercentages[2]) + "," + String(batteryPercentages[3]) + "], \"totalVoltage\": " + String(totalVoltage) + ", \"totalPercentage\": " + String(totalPercentage) + "}";
  ws.textAll(jsonData);

  delay(1000);
}
