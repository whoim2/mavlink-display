#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#include <mavlink.h>
#include <EEPROM.h>

#define I2C_ADDRESS 0x3C
#define RST_PIN -1
SSD1306AsciiWire oled;

#define MAV_TIMEOUT 5000 //mavlink timeout
#define SERIAL_SPEED 57600 //mavlink input baud, 4800 for qczek 2.10
//#define DEBUG
mavlink_message_t msg;
mavlink_status_t status;
//mavlink_global_position
int32_t alt, relative_alt;
int16_t vx, vy, vz;
uint16_t hdg;
//__mavlink_sys_status_t
int8_t battery_remaining;
uint16_t current_battery, voltage_battery, cpu_load, drop_rate_comm;
//mavlink_gps_raw_int_t
int32_t lat, lon, gps_alt;
uint8_t satellites_visible, fix_type;
uint16_t cog, vel;
//mavlink_rc_channels_raw_t
uint8_t rssi;
//oth
uint8_t flag, eeprom_flag = 0;
uint32_t time_flag;
//------------------------------------------------------------------------------
void setup() {
  Wire.begin();
  Wire.setClock(400000L);
  oled.begin(&Adafruit128x32, I2C_ADDRESS);
  display_wait();
  //
  Serial.begin(SERIAL_SPEED);
  //
  time_flag = millis();
}
//------------------------------------------------------------------------------
void loop() {
  while(Serial.available()) {
    uint8_t c= Serial.read();
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
    flag = 0;
    switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
          break;
        }
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
          mavlink_global_position_int_t packet;
          mavlink_msg_global_position_int_decode(&msg, &packet);
          if(packet.hdg == 65535) packet.hdg = 0;
          //if(lat != packet.lat) { lat = packet.lat; set_flag(); }
          //if(lon != packet.lon) { lon = packet.lon; set_flag(); }
          //if(alt != packet.alt) { alt = packet.alt; set_flag(); }
          if(relative_alt != packet.relative_alt) { relative_alt = packet.relative_alt; set_flag(); }
          //if(vx != packet.vx) { vx = packet.vx; set_flag(); }
          //if(vy != packet.vy) { vy = packet.vy; set_flag(); } 
          //if(vz != packet.vz) { vz = packet.vz; set_flag(); }
          if(hdg != packet.hdg) { hdg = packet.hdg; set_flag(); }
          break;
        }
        case MAVLINK_MSG_ID_SYS_STATUS: {
          __mavlink_sys_status_t packet;
          mavlink_msg_sys_status_decode(&msg, &packet);
          if(battery_remaining != packet.battery_remaining && packet.battery_remaining >= 0) { battery_remaining = packet.battery_remaining; set_flag(); }
          if(voltage_battery != packet.voltage_battery && packet.voltage_battery != 65535) { voltage_battery = packet.voltage_battery; set_flag(); }
          if(current_battery != packet.current_battery) { current_battery = packet.current_battery; set_flag(); }
          if(cpu_load != packet.load) { cpu_load = packet.load; set_flag(); }
          if(drop_rate_comm != packet.drop_rate_comm) { drop_rate_comm = packet.drop_rate_comm; set_flag(); }
          break;
        }
        case MAVLINK_MSG_ID_ATTITUDE: {
          break;
        }
        case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN: {
          break;
        }
        case MAVLINK_MSG_ID_RC_CHANNELS_RAW: {
          __mavlink_rc_channels_raw_t packet;
          mavlink_msg_rc_channels_raw_decode(&msg, &packet);
          if(rssi != packet.rssi) { rssi = packet.rssi; set_flag(); }
          break;
        }
        case MAVLINK_MSG_ID_VFR_HUD: {
          break;
        }
        case MAVLINK_MSG_ID_GPS_RAW_INT: {
          __mavlink_gps_raw_int_t packet;
          mavlink_msg_gps_raw_int_decode(&msg, &packet);
          if(packet.cog == 65535) packet.cog = 0;
          if(packet.vel == 65535) packet.vel = 0;
          if(packet.alt == 65535) packet.alt = 0;
          if(lat != packet.lat) { lat = packet.lat; set_flag(); }
          if(lon != packet.lon) { lon = packet.lon; set_flag(); }
          //if(gps_alt != packet.alt) { gps_alt = packet.alt; set_flag(); }
          if(vel != packet.vel) { vel = packet.vel; set_flag(); }
          if(cog != packet.cog) { cog = packet.cog; set_flag(); }
          if(fix_type != packet.fix_type) { fix_type = packet.fix_type; set_flag(); }
          if(satellites_visible != packet.satellites_visible) { satellites_visible = packet.satellites_visible; set_flag(); }
          break;
        }
        default: {
          #ifdef DEBUG
          Serial.println(msg.msgid); //see unused packet types
          #endif
        break;
        }
    }//switch
    if(flag == 1) {
      display_data();
    }//print flag
    else {
      no_data();
    }
   }//if mavlink_parse_char
  }//while serial available
  no_data(); //check no serial input data fuction
}

void set_flag() {
    flag = 1;
    eeprom_flag = 0;
    time_flag = millis();
}
void display_wait() {
  oled.setFont(font8x8);
  oled.set2X();
  oled.clear();
  oled.println("WAIT FOR");
  oled.println("MAVLINK");
  oled.set1X();
  oled.setFont(font5x7);
}

void display_data() {
  oled.clear();
  printL(lat); //gps
  oled.print(" ");
  printL(lon);
  oled.println();
  oled.println((String)"S"+satellites_visible+(String)" F"+fix_type+(String)" R"+map(rssi,0,255,0,100)+(String)"% L"+cpu_load/10+(String)"% E"+drop_rate_comm/100+(String)"%");
  oled.println((String)"H"+hdg+(String)" S"+(uint8_t)(vel/100*3.6)+(String)"k A"+relative_alt/1000+(String)"m");
  oled.println(current_battery/100.0+(String)"A "+voltage_battery/1000.0+(String)"v R"+battery_remaining+(String)"%");
}

void printL(int32_t degE7) {
  // Extract and print negative sign
  if (degE7 < 0) {
    degE7 = -degE7;
    oled.print( '-' );
  }
  // Whole degrees
  int32_t deg = degE7 / 10000000L;
  oled.print( deg );
  oled.print( '.' );
  // Get fractional degrees
  degE7 -= deg*10000000L;
  // Print leading zeroes, if needed
  int32_t factor = 1000000L;
  while ((degE7 < factor) && (factor > 1L)){
    oled.print( '0' );
    factor /= 10L;
  }
  // Print fractional degrees
  oled.print( degE7 );
}

void no_data() {
  if((millis() - time_flag) > MAV_TIMEOUT ) { //no mavlink data at 2sec
     #ifdef DEBUG
     Serial.println((String)"LOST MAVLINK DATA");
     #endif
     display_wait();
     delay(300);
     if(eeprom_flag == 0 && lat != 0 && lon != 0 && fix_type > 1) { //if gps coordinates present, save it
        #ifdef DEBUG
        Serial.println("save..");
        #endif
        EEPROM_int32_write(5, lat);
        EEPROM_int32_write(16, lon);
        EEPROM.write(30, satellites_visible);
        EEPROM.write(32, fix_type);
        EEPROM.write(34, cpu_load);
        EEPROM.write(40, drop_rate_comm);
        EEPROM.write(46, hdg);
        EEPROM.write(52, vel);
        EEPROM_int32_write(56, relative_alt);
        EEPROM.write(66, current_battery);
        EEPROM.write(72, voltage_battery);
        EEPROM.write(74, battery_remaining);
        EEPROM.write(76, rssi);
        eeprom_flag = 1;
     } else { //no fresh data on mavlink, read from memory
        #ifdef DEBUG
        Serial.println("read..");
        #endif
        lat = EEPROM_int32_read(5);
        lon = EEPROM_int32_read(16);
        satellites_visible = EEPROM.read(30);
        fix_type = EEPROM.read(32);
        cpu_load = EEPROM.read(34);
        drop_rate_comm = EEPROM.read(40);
        hdg = EEPROM.read(46);
        vel = EEPROM.read(52);
        relative_alt = EEPROM_int32_read(56);
        current_battery = EEPROM.read(66);
        voltage_battery = EEPROM.read(72);
        battery_remaining = EEPROM.read(74);
        rssi = EEPROM.read(76); 
        eeprom_flag = 1;     
     }
     display_data();
     time_flag = millis();
  }
}

int32_t EEPROM_int32_read(int addr) // чтение из EEPROM 4 байта unsigned long
{   
  byte raw[4];
  for(byte i = 0; i < 4; i++) raw[i] = EEPROM.read(addr+i);
  int32_t &data = (int32_t&)raw;
  return data;
}
//*****************************************************************
void EEPROM_int32_write(int addr, int32_t data) // запись в EEPROM 4 байта unsigned long
{
  byte raw[4];
  (int32_t&)raw = data;
  for(byte i = 0; i < 4; i++) EEPROM.write(addr+i, raw[i]);
}
