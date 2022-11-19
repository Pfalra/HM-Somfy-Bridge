#define SERIAL_NO "HM_SOMFY01"

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

/*********************************/
/* BUTTONS */
// we use a Pro Mini
// Arduino pin for the LED
// D4 == PIN 4 on Pro Mini
#define LED_PIN LED_BUILTIN

// Arduino pin for the config button
// B0 == PIN 8 on Pro Mini
#define CONFIG_BUTTON_PIN 8


#define UP_BUTTON_PIN 7
#define DOWN_BUTTON_PIN 6
#define TOGGLE_BLIND_BUTTON_PIN 5

#define BUTTON_ACTIVE_STATE LOW
#define BUTTON_INACTIVE_STATE HIGH

/*********************************/

/*********************************/
/* DEVICE CONFIGURATION */
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
/*********************************/



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

  virtual void switchState(uint8_t oldstate, uint8_t newstate, uint32_t stateDelay) {
    BaseChannel::switchState(oldstate, newstate, stateDelay);
    if( newstate == AS_CM_JT_RAMPON && stateDelay > 0 ) {
      trigger_blindUp();
    }
    else if( newstate == AS_CM_JT_RAMPOFF && stateDelay > 0 ) {
      trigger_blindDown();
    }
    else {
      // trigger_blindToggle();
      // Do nothing. Requirement says that this should be deactivated.
    }
  }

  void trigger_blindUp () {
    digitalWrite(UP_BUTTON_PIN, BUTTON_ACTIVE_STATE);
    Serial.println("Driving blind up");
    delay(300);
    digitalWrite(UP_BUTTON_PIN, BUTTON_INACTIVE_STATE);
  }

  void trigger_blindDown () {
    digitalWrite(DOWN_BUTTON_PIN, BUTTON_ACTIVE_STATE);
    Serial.println("Driving blind down");
    delay(300);
    digitalWrite(DOWN_BUTTON_PIN, BUTTON_INACTIVE_STATE);
  }

  // Requirement explicitly says to deactivate this.
  // void trigger_blindToggle () {
  //   digitalWrite(TOGGLE_BLIND_BUTTON_PIN, BUTTON_ACTIVE_STATE);
  //   Serial.println("Toggled to next blind");
  //   digitalWrite(TOGGLE_BLIND_BUTTON_PIN, BUTTON_INACTIVE_STATE);
  // }

  void init () {
    // init pins for the blind
    pinMode(UP_BUTTON_PIN, OUTPUT);
    pinMode(DOWN_BUTTON_PIN, OUTPUT);
    pinMode(TOGGLE_BLIND_BUTTON_PIN, OUTPUT);

    digitalWrite(UP_BUTTON_PIN, BUTTON_INACTIVE_STATE);
    digitalWrite(DOWN_BUTTON_PIN, BUTTON_INACTIVE_STATE);
    digitalWrite(TOGGLE_BLIND_BUTTON_PIN, BUTTON_INACTIVE_STATE);

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
  // Setup Serial
  DINIT(57600,ASKSIN_PLUS_PLUS_IDENTIFIER);

  // Init the device (blind)
  //storage().setByte(0,0);
  bool first = sdev.init(hal);
  sdev.channel(1).init();

//  sdev.channel(1).getList1().refRunningTimeBottomTop(270);
//  sdev.channel(1).getList1().refRunningTimeTopBottom(270);

  // Setup radio frequency
  hal.radio.initReg(CC1101_FREQ2, 0x21);
  hal.radio.initReg(CC1101_FREQ1, 0x65);
  hal.radio.initReg(CC1101_FREQ0, 0xDA);
  
  // Configure interrupt for config button
  buttonISR(cfgBtn, CONFIG_BUTTON_PIN);

  // Pairing with HomeMatic CCU
  initPeerings(first);
  sdev.initDone();
}

void loop() {
  // Check if radio is still working
  bool worked = hal.runready();
  // Poll for reception 
  bool poll = sdev.pollRadio();
  // Save power
  if( worked == false && poll == false ) {
    hal.activity.savePower<Idle<> >(hal);
  }
}
