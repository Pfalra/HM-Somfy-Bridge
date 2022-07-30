#define NORMAL_OPERATION // Comment if frequency test is necessary

#define SERIAL_NO "HM_SOMFY01"


#ifdef NORMAL_OPERATION
//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2017-12-14 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
// ci-test=yes board=328p aes=no
//- -----------------------------------------------------------------------------------------------------------------------

// define this to read the device id, serial and device type from bootloader section
// #define USE_OTA_BOOTLOADER

#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#include <AskSinPP.h>
#include <LowPower.h>

#include <Blind.h>


// we use a Pro Mini
// Arduino pin for the LED
// D4 == PIN 4 on Pro Mini
#define LED_PIN LED_BUILTIN
// Arduino pin for the config button
// B0 == PIN 8 on Pro Mini
#define CONFIG_BUTTON_PIN 8

// define L298N when using a H-Bridge motor driver
// #define L298N
#ifdef L298N
  // define up and down pins for L298N driver
  #define UP_PIN 16
  #define DOWN_PIN 17
#else
  // define relay pins
  #define ON_RELAY_PIN 15
  #define DIR_RELAY_PIN 14
#endif

#define UP_BUTTON_PIN 6
#define DOWN_BUTTON_PIN 3

// number of available peers per channel
#define PEERS_PER_CHANNEL 12

// all library classes are placed in the namespace 'as'
using namespace as;

// define all device properties
const struct DeviceInfo PROGMEM devinfo = {
    {0x59,0x32,0xaf},              // Device ID
    SERIAL_NO,                  // Device Serial
    {0x00,0x05},                   // Device Model
    0x24,                          // Firmware Version
    as::DeviceType::BlindActuator, // Device Type
    {0x01,0x00}                    // Info Bytes
};

/**
 * Configure the used hardware
 */
typedef AvrSPI<10,11,12,13> RadioSPI;
typedef AskSin<StatusLed<LED_PIN>,NoBattery,Radio<RadioSPI,2> > Hal;
//typedef AskSin<StatusLed<4>,NoBattery,NoRadio> Hal;

DEFREGISTER(BlindReg0,MASTERID_REGS,DREG_INTKEY,DREG_CONFBUTTONTIME,DREG_LOCALRESETDISABLE)

class BlindList0 : public RegList0<BlindReg0> {
public:
  BlindList0 (uint16_t addr) : RegList0<BlindReg0>(addr) {}
  void defaults () {
    clear();
    // intKeyVisible(false);
    confButtonTime(0xff);
    // localResetDisable(false);
  }
};

class BlChannel : public ActorChannel<Hal,BlindList1,BlindList3,PEERS_PER_CHANNEL,BlindList0,BlindStateMachine> {
public:
  typedef ActorChannel<Hal,BlindList1,BlindList3,PEERS_PER_CHANNEL,BlindList0,BlindStateMachine> BaseChannel;

  BlChannel () {}
  virtual ~BlChannel () {}

  virtual void switchState(uint8_t oldstate,uint8_t newstate, uint32_t stateDelay) {
    BaseChannel::switchState(oldstate, newstate, stateDelay);
    if( newstate == AS_CM_JT_RAMPON && stateDelay > 0 ) {
      motorUp();
    }
    else if( newstate == AS_CM_JT_RAMPOFF && stateDelay > 0 ) {
      motorDown();
    }
    else {
      motorStop();
    }
  }

  void motorUp () {
#ifdef L298N
    digitalWrite(DOWN_PIN,LOW);
    digitalWrite(UP_PIN,HIGH);
#else
    digitalWrite(DIR_RELAY_PIN,HIGH);
    digitalWrite(ON_RELAY_PIN,HIGH);
#endif
  }

  void motorDown () {
#ifdef L298N
    digitalWrite(UP_PIN,LOW);
    digitalWrite(DOWN_PIN,HIGH);
#else
    digitalWrite(DIR_RELAY_PIN,LOW);
    digitalWrite(ON_RELAY_PIN,HIGH);
#endif
  }

  void motorStop () {
#ifdef L298N
    digitalWrite(UP_PIN,LOW);
    digitalWrite(DOWN_PIN,LOW);
#else
    digitalWrite(DIR_RELAY_PIN,LOW);
    digitalWrite(ON_RELAY_PIN,LOW);
#endif
  }

  void init () {
#ifdef L298N
    pinMode(UP_PIN,OUTPUT);
    pinMode(DOWN_PIN,OUTPUT);
#else
    pinMode(ON_RELAY_PIN,OUTPUT);
    pinMode(DIR_RELAY_PIN,OUTPUT);
#endif
    motorStop();
    BaseChannel::init();
  }
};

// setup the device with channel type and number of channels
typedef MultiChannelDevice<Hal,BlChannel,1,BlindList0> BlindType;

Hal hal;
BlindType sdev(devinfo,0x20);
ConfigButton<BlindType> cfgBtn(sdev);
InternalButton<BlindType> btnup(sdev,1);
InternalButton<BlindType> btndown(sdev,2);

void initPeerings (bool first) {
  // create internal peerings - CCU2 needs this
  if( first == true ) {
    sdev.channel(1).peer(btnup.peer(),btndown.peer());
  }
}

void setup () {
  DINIT(57600,ASKSIN_PLUS_PLUS_IDENTIFIER);
  //storage().setByte(0,0);
  bool first = sdev.init(hal);
  sdev.channel(1).init();

//  sdev.channel(1).getList1().refRunningTimeBottomTop(270);
//  sdev.channel(1).getList1().refRunningTimeTopBottom(270);

  hal.radio.initReg(CC1101_FREQ2, 0x21);
  hal.radio.initReg(CC1101_FREQ1, 0x65);
  hal.radio.initReg(CC1101_FREQ0, 0xDA);
  

  buttonISR(cfgBtn,CONFIG_BUTTON_PIN);
  buttonISR(btnup,UP_BUTTON_PIN);
  buttonISR(btndown,DOWN_BUTTON_PIN);

  initPeerings(first);
  sdev.initDone();
}

void loop() {
  bool worked = hal.runready();
  bool poll = sdev.pollRadio();
  if( worked == false && poll == false ) {
    hal.activity.savePower<Idle<> >(hal);
  }
}

#else

//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2016-10-31 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
// 2019-11-16 stan23 Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
// ci-test=yes board=328p aes=no
//- -----------------------------------------------------------------------------------------------------------------------

// define this to read the device id, serial and device type from bootloader section
// #define USE_OTA_BOOTLOADER

#if defined ARDUINO_ARCH_STM32F1
  #define STORAGEDRIVER at24cX<0x50,128,32>
  #define TICKS_PER_SECOND 500UL
  #define USE_HW_SERIAL
 
  #include <SPI.h>    // when we include SPI.h - we can use LibSPI class
  #include <Wire.h>   // for I2C access to EEPROM
  #include <EEPROM.h> // the EEPROM library contains Flash Access Methods
#endif

#include <AskSinPP.h>
#include <Device.h>
#include <Register.h>


#if defined ARDUINO_ARCH_STM32F1
  // we use a BluePill
  #define CC1101_GDO0_PIN     PB0
  #define CC1101_CS_PIN       PA4
  // CC1101 communication uses HW-SPI
  #define LED_PIN             LED_BUILTIN
#else
  // we use a Pro Mini
  #define CC1101_GDO0_PIN     2     // PD2
  #define CC1101_CS_PIN       10    // PB2
  #define CC1101_MOSI_PIN     11    // PB3
  #define CC1101_MISO_PIN     12    // PB4
  #define CC1101_SCK_PIN      13    // PB5
  // Pro Mini LED
  #define LED_PIN             4     // PD4
#endif

// all library classes are placed in the namespace 'as'
using namespace as;

// define all device properties
const struct DeviceInfo PROGMEM devinfo = {
    {0x01,0x01,0x01},       // Device ID
    "FreqTest00",           // Device Serial
    {0x00,0x00},            // Device Model
    0x10,                   // Firmware Version
    as::DeviceType::Sensor, // Device Type
    {0x00,0x00}             // Info Bytes
};

/**
 * Configure the used hardware
 */
#if defined ARDUINO_ARCH_STM32F1
typedef LibSPI<CC1101_CS_PIN> RadioSPI;
#else
typedef AvrSPI<CC1101_CS_PIN, CC1101_MOSI_PIN, CC1101_MISO_PIN, CC1101_SCK_PIN> RadioSPI;
#endif
typedef Radio<RadioSPI, CC1101_GDO0_PIN> RadioType;
typedef StatusLed<LED_PIN> LedType;
typedef AskSin<LedType,NoBattery,RadioType> HalType;



#define STARTFREQ 0x656A             // frequency we start scanning
#define MINFREQ (STARTFREQ - 0x300)  // frequency we abort scanning
#define SEARCHSTEP 0x50              // step with during search
#define BOUNDSTEP 0x10               // step width during upper/lower bound analysis


// see https://github.com/AskSinPP/asksinpp.de/blob/master/Grundlagen/FAQ/Fehlerhafte_CC1101.md
// #define ACTIVE_PING
HMID PING_FROM(0x12,0x34,0x56);      // from address for status message e.g. switch
HMID PING_TO(0x99,0x66,0x99);        // to address for status message / central / CCU
#ifdef ACTIVE_PING
  #define SCANTIME seconds2ticks(5)  // maximal time to wait for a valid message
#else
  #define SCANTIME seconds2ticks(60) // maximal time to wait for a valid message
#endif



class TestDevice : public Device<HalType,DefList0>, Alarm {
  DefList0 l0;
  uint16_t freq, start, end;
  uint8_t received, rssi;
public:
  enum SearchMode { Search, Up, Down, Done };
  SearchMode mode;
  HMID id;

  typedef Device<HalType,DefList0> BaseDevice;
  TestDevice (const DeviceInfo& i,uint16_t addr) : BaseDevice(i,addr,l0,0), Alarm(0), l0(addr),
      freq(STARTFREQ), start(STARTFREQ), end(STARTFREQ),
      received(0), rssi(0), mode(Search)  {}
  virtual ~TestDevice () {}

  virtual void trigger (__attribute__ ((unused)) AlarmClock& clock) {
    DPRINT("  ");DDEC(received);
    if (!received) {
      DPRINTLN("");
    } else {
      DPRINT(" / -");DDEC(rssi);DPRINTLN("dBm");
    }

    if( mode == Search ) {
      if( received > 0 ) {
        start = end = freq;
        mode = Up; // start find upper bound
        DPRINTLN("Search for upper bound");
        setFreq(freq + BOUNDSTEP);
      }
      else {
        if( freq < MINFREQ ) mode = Done;
        if( freq <= STARTFREQ ) setFreq(STARTFREQ + (STARTFREQ-freq) + SEARCHSTEP);
        else setFreq(STARTFREQ - (freq-STARTFREQ));
      }
    }
    else if(mode == Up) {
      if( received > 0 ) {
        end = freq;
        setFreq(end + BOUNDSTEP);
      }
      else {
        mode = Down; // start find lower bound
        DPRINTLN("Search for lower bound");
        setFreq(start - BOUNDSTEP);
      }
    }
    else if(mode == Down) {
      if( received > 0 ) {
        start = freq;
        setFreq(start - BOUNDSTEP);
      }
      else {
        mode = Done;
      }
    }
    if( mode == Done ) {
      DPRINT("\nDone: 0x21");DHEX(start);DPRINT(" - 0x21");DHEXLN(end);
      if( start == end && start == STARTFREQ ) {
        DPRINT("Could not receive any message");
      }
      else {
        freq = start+((end - start)/2);
        DPRINT("Calculated Freq: 0x21");DHEX((uint8_t)(freq>>8));DHEX((uint8_t)(freq&0xff));
        printFreq(0x210000 + freq);DPRINTLN("");

        // store frequency
        DPRINT("Store into config area: ");DHEX((uint8_t)(freq>>8));DHEX((uint8_t)(freq&0xff));
        StorageConfig sc = getConfigArea();

#if defined ARDUINO_ARCH_STM32F1
        Wire.begin();
        DPRINT(".");
#endif
        // read old frequency
        uint8_t oldF1 = sc.getByte(CONFIG_FREQ1);
        uint8_t oldF2 = sc.getByte(CONFIG_FREQ2);

        sc.clear();
        DPRINT(".");
        sc.setByte(CONFIG_FREQ1, freq>>8);
        DPRINT(".");
        sc.setByte(CONFIG_FREQ2, freq&0xff);
        DPRINT(".");
        sc.validate();

        DPRINTLN("stored!");

        if ((oldF1 >= 0x60) && (oldF1 <= 0x6A)) {
          DPRINT("\nOld Config Freq was: 0x21");DHEX(oldF1);DHEX(oldF2);
          printFreq(0x210000 + oldF1*256 + oldF2);
        }
        
#if defined ARDUINO_ARCH_STM32F1
        // measurement is done, loop here forever
        while(1);
#else
        DPRINTLN("Going to sleep...");
        Serial.flush();
        activity().savePower<Sleep<> >(this->getHal());
#endif
      }
    }
  }

  virtual bool process(Message& msg) {
    msg.from().dump(); DPRINT(".");
    rssi = max(rssi,radio().rssi());
#ifdef ACTIVE_PING
    if (msg.from() == PING_TO)
#endif
    {
      received++;
      if( received > 0 ) {
        trigger(sysclock);
      }
    }
    return true;
  }

  bool init (HalType& hal) {
    this->setHal(hal);
    this->getDeviceID(id);
    hal.init(id);

#ifdef ACTIVE_PING
    DPRINT("Active ping is enabled, looking for telegrams only from ");
    PING_TO.dump(); DPRINTLN("!");
#else
    DPRINTLN("Active ping is NOT enabled, looking for any telegram");
#endif
    DPRINTLN("Start searching ...");
    setFreq(STARTFREQ);
    return false;
  }

  void setFreq (uint16_t current) {
    sysclock.cancel(*this);
    freq = current;
    rssi=0;
    received=0;
    DPRINT("Freq 0x21");DHEX(freq);
    printFreq(0x210000 + freq);
    DPRINT(": ");
    this->radio().initReg(CC1101_FREQ2, 0x21);
    this->radio().initReg(CC1101_FREQ1, freq >> 8);
    this->radio().initReg(CC1101_FREQ0, freq & 0xff);
    set(SCANTIME);
    sysclock.add(*this);
  }

  void printFreq(uint32_t freq) {
    char buffer[16];
    float val = (float)freq * 26.0 / 65536.0;
    dtostrf(val, 8, 3, buffer);
    DPRINT(buffer);DPRINT(" MHz");
  }
};

HalType hal;
TestDevice sdev(devinfo,0x20);

class InfoSender : public Alarm {
  class channel {
  public:
    uint8_t number() const {return 1; }
    uint8_t status() const {return 0; }
    uint8_t flags()  const {return 0; }
    void patchStatus(__attribute__ ((unused)) Message& msg) {}
    void changed(__attribute__ ((unused)) bool b) {}
  };
  uint8_t cnt;
  channel ch;
public:
  InfoSender () : Alarm(0), cnt(0) {}
  virtual ~InfoSender () {}

  virtual void trigger (AlarmClock& clock) {
#ifdef ACTIVE_PING
    InfoActuatorStatusMsg msg;
    msg.init(cnt++, ch, hal.radio.rssi());
    msg.to(PING_TO);
    msg.from(PING_FROM);
    msg.ackRequired();
    msg.setRpten();
    sdev.radio().write(msg,msg.burstRequired());
#endif
    sdev.led().ledOn(millis2ticks(100), 0);
    if( sdev.mode != TestDevice::SearchMode::Done ) {
      set(seconds2ticks(1));
      clock.add(*this);
    }
  }
} info;

void setup () {
#if defined ARDUINO_ARCH_STM32F1
  delay(5000);
#endif
  DINIT(57600,ASKSIN_PLUS_PLUS_IDENTIFIER);
  sdev.init(hal);
  // start sender
  info.trigger(sysclock);
}

void loop() {
  sdev.pollRadio();
  hal.runready();
}

#endif