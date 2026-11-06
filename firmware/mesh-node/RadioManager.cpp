#include "RadioManager.h"

namespace AndyMesh {

// There is one physical radio in a node, so one active callback target is enough.
RadioManager* RadioManager::activeInstance_ = nullptr;

RadioManager::RadioManager()
    // Module pin order is NSS, DIO1, RESET, BUSY. These are the exact pins from
    // the working XIAO ESP32S3 + Wio-SX1262 PING/PONG sketches.
    : radio_(new Module(41, 39, 42, 40)),
      operationDone_(false),
      state_(State::Stopped),
      log_(nullptr),
      lastError_(RADIOLIB_ERR_NONE) {}

// Keep interrupt work extremely short: record that DIO1 fired and let poll do
// SPI communication later in the normal main-loop context.
void RadioManager::onDio1() {
  if (activeInstance_ != nullptr) {
    activeInstance_->operationDone_ = true;
  }
}

bool RadioManager::begin(const RadioConfig& config, Print& log) {
  log_ = &log;
  activeInstance_ = this;

  // The constructor pins mirror the tested hardware. Reject a mismatched
  // runtime configuration rather than silently talking to different pins.
  if (config.nssPin != 41 || config.dio1Pin != 39 || config.resetPin != 42 ||
      config.busyPin != 40) {
    lastError_ = RADIOLIB_ERR_UNKNOWN;
    reportError("pin configuration", lastError_);
    return false;
  }

  // begin programs the LoRa modem with the same frequency, bandwidth, coding
  // rate, private sync word, power, preamble, and TCXO voltage as the
  // bench-tested reference; the spreading factor comes from NodeConfig (SF7).
  // The private sync word helps ignore public LoRa traffic that is not part of
  // this project.
  lastError_ = radio_.begin(
      config.frequencyMhz, config.bandwidthKhz, config.spreadingFactor,
      config.codingRateDenominator, RADIOLIB_SX126X_SYNC_WORD_PRIVATE,
      config.transmitPowerDbm, config.preambleLength, config.tcxoVoltage);
  if (lastError_ != RADIOLIB_ERR_NONE) {
    reportError("radio initialization", lastError_);
    return false;
  }

  // Preserve the bench-tested XIAO/Wio-SX1262 RF-switch setup. The RF switch
  // connects one antenna to either transmit or receive circuitry. DIO2 selects
  // transmit automatically, while pin 38 enables the receive path.
  lastError_ = radio_.setDio2AsRfSwitch(true);
  if (lastError_ != RADIOLIB_ERR_NONE) {
    reportError("DIO2 RF switch", lastError_);
    return false;
  }
  radio_.setRfSwitchPins(config.receiveEnablePin, RADIOLIB_NC);
  // DIO1 is the SX1262 interrupt line. It signals both a completed receive and a
  // completed transmit; state_ tells poll which one just finished.
  radio_.setDio1Action(onDio1);

  return startReceiving();
}

bool RadioManager::startReceiving() {
  // Reset any old event before arming continuous reception. startReceive returns
  // immediately, so other mesh tasks keep running while the radio listens.
  operationDone_ = false;
  lastError_ = radio_.startReceive();
  if (lastError_ != RADIOLIB_ERR_NONE) {
    state_ = State::Stopped;
    reportError("start receive", lastError_);
    return false;
  }
  state_ = State::Receiving;
  return true;
}

bool RadioManager::poll(TransportPacket& packet) {
  // Most loop iterations have no radio event. Returning immediately keeps this
  // check inexpensive and leaves time for HELLO, timeout, and MAVLink work.
  if (!operationDone_) {
    return false;
  }
  operationDone_ = false;

  if (state_ == State::Transmitting) {
    // finishTransmit clears the TX interrupt and places the SX1262 in standby.
    // Listening is restarted right away so the node misses as little traffic as
    // possible after sending its own packet.
    lastError_ = radio_.finishTransmit();
    if (lastError_ != RADIOLIB_ERR_NONE) {
      reportError("finish transmit", lastError_);
    }
    startReceiving();
    return false;
  }

  if (state_ != State::Receiving) {
    return false;
  }

  const size_t packetLength = radio_.getPacketLength();
  // Never read beyond TransportPacket::data. A zero or oversized length is
  // not a valid Phase 2 frame, so discard it and resume listening.
  if (packetLength == 0 || packetLength > kMaxPacketSize) {
    reportError("invalid received length", RADIOLIB_ERR_PACKET_TOO_LONG);
    startReceiving();
    return false;
  }

  // RadioLib copies bytes out of the SX1262 FIFO (first-in, first-out hardware
  // buffer). RSSI and SNR describe this directly received packet and are later
  // stored in the direct-neighbor table.
  lastError_ = radio_.readData(packet.data, packetLength);
  packet.rssiDbm = radio_.getRSSI();
  packet.snrDb = radio_.getSNR();
  packet.length = packetLength;
  const bool received = lastError_ == RADIOLIB_ERR_NONE;
  if (!received) {
    reportError("read packet", lastError_);
  }
  startReceiving();
  return received;
}

bool RadioManager::send(const uint8_t* data, size_t length) {
  // A node must not start another send while TX is active or while an RX event is
  // waiting to be processed. The fixed protocol maximum also guards the buffer.
  if (!readyToSend() || data == nullptr || length == 0 ||
      length > kMaxPacketSize) {
    return false;
  }

  operationDone_ = false;
  // startTransmit loads the packet and returns while LoRa modulation continues
  // in hardware. DIO1 will call onDio1 when the final symbol is sent.
  lastError_ = radio_.startTransmit(data, length);
  if (lastError_ != RADIOLIB_ERR_NONE) {
    reportError("start transmit", lastError_);
    startReceiving();
    return false;
  }
  state_ = State::Transmitting;
  return true;
}

bool RadioManager::readyToSend() const {
  // A pending receive interrupt gets priority so an incoming packet is not
  // overwritten by switching the radio into transmit mode.
  return state_ == State::Receiving && !operationDone_;
}

int16_t RadioManager::lastError() const { return lastError_; }

void RadioManager::reportError(const char* operation, int16_t error) {
  // RadioLib uses negative numeric error codes. Printing both the failed step
  // and code makes hardware/wiring problems easier to look up during testing.
  if (log_ == nullptr) {
    return;
  }
  log_->print("[RADIO] ");
  log_->print(operation);
  log_->print(" failed: ");
  log_->println(error);
}

}  // namespace AndyMesh
