#pragma once

#include <Arduino.h>
#include <RadioLib.h>

#include "NodeConfig.h"
#include "PacketProtocol.h"

/*
  Nonblocking SX1262 radio control

  RadioLib supplies the low-level SX1262 commands. RadioManager adds a small
  state machine so the main loop can continue checking timers and UART data while
  LoRa transmission or reception is in progress. "Nonblocking" means a function
  starts an operation and returns instead of waiting in a long loop for it to
  finish.
*/
namespace AndyMesh {

// The largest valid protocol packet has a fixed local buffer. RSSI is received
// signal strength in dBm (closer to zero is stronger), and SNR is signal-to-noise
// ratio in dB (higher is generally cleaner).
struct ReceivedRadioPacket {
  uint8_t data[kMaxPacketSize];
  size_t length;
  float rssiDbm;
  float snrDb;
};

class RadioManager {
 public:
  RadioManager();

  // begin configures the SX1262 and enters continuous receive mode. poll checks
  // whether an interrupt reported completed RX/TX work. send starts a transmit
  // only when the radio is ready.
  bool begin(const RadioConfig& config, Print& log);
  bool poll(ReceivedRadioPacket& packet);
  bool send(const uint8_t* data, size_t length);
  bool readyToSend() const;
  int16_t lastError() const;

 private:
  // Stopped means setup failed or the radio is unavailable. The other states
  // tell poll whether a DIO1 interrupt means "packet received" or "send done."
  enum class State : uint8_t {
    Stopped,
    Receiving,
    Transmitting,
  };

  // RadioLib interrupt callbacks cannot be ordinary object member functions.
  // activeInstance_ lets the static callback set the flag on this one manager.
  static RadioManager* activeInstance_;
  static void onDio1();

  bool startReceiving();
  void reportError(const char* operation, int16_t error);

  SX1262 radio_;                 // RadioLib object that talks to the SX1262.
  // DIO1 changes this flag inside an interrupt. volatile tells the compiler the
  // value may change outside normal main-loop execution.
  volatile bool operationDone_;
  State state_;                  // Current asynchronous radio operation.
  Print* log_;                   // Usually Serial; used for readable errors.
  int16_t lastError_;            // Most recent RadioLib status code.
};

}  // namespace AndyMesh
