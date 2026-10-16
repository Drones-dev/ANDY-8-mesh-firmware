/*
  ANDY 8: RED - Drone 1 (LoRa ping test)
  Board:   Seeed XIAO ESP32S3 + Wio-SX1262 
  Library: RadioLib

  What it does:
    1. Starts the LoRa radio and listens.
    2. When it hears "PING" from the Ground Station, it answers "PONG".
    3. Prints what it heard and sent to the Serial Monitor (115200 baud).

  The groundStation sketch must be running on the other board with the same radio settings.
*/

#include <RadioLib.h>   // LoRa radio library for the SX1262 chip
                        // (low-level radio commands)

//The four numbers are the XIAO ESP32S3 pins wired to the SX1262:
//   NSS (chip select), DIO1 (interrupt), RESET, BUSY
// Why: the ESP32 talks to the radio chip over SPI, and the library needs to know which pins are the chip-select, 
// "event" signal, reset line and "busy" signal. 
//If wrong, the radio won't start or TX/RX failures
SX1262 radio = new Module(41, 39, 42, 40);

void setup() {
  Serial.begin(115200);   // start USB serial for the Serial Monitor
  delay(3000);   // wait 3s so the Serial Monitor has time to open before we print.
                 // This board uses native USB, which disconnects and reconnects on every reset, so without this pause the first messages are lost.

  // Start the LoRa radio with these settings
  //   915MHz

  //   125      standard LoRa default: narrower is longer range but slower, wider is faster but shorter range.

  //   9        spreading factor (SF9). Higher SF = longer range and better noise tolerance, but each message takes longer on the air. 
  //            SF9 is a middle range for test.

  //   7        coding rate, written as 4/7. For every 4 bits of real data, the radio sends 7 bits in total (3 extra bits). 
  //            If noise flips a bit, the receiver can use the backup bits to fix it.
  //            (4/5 to 4/8): a bigger number means more backup bits and better repair,but a slower message.

  //   SYNC_WORD_PRIVATE  a network ID byte. It makes our radios ignore other LoRa traffic nearby.

  //   14       transmit power in dBm. We can raise it later for range tests.

  //   8        preamble length. "wake up message" 8 is the standard default.

  //   1.8      TCXO voltage. SX1262 uses a temperature-compensated crystal oscillator powered at 1.8 V through the radio chip.
  //            If we don't tell the library to power it, the radio's clock won't be accurate and it may not start/frequency drift.
  int state = radio.begin(915.0, 125.0, 9, 7,
                          RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 14, 8, 1.8);

 // Error -2 usually means the radio module isn't attached
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("Radio failed to start. Error code: ");
    Serial.println(state);
    while (true) { delay(1000); }   // stop here
  }

  // Antenna switch setup for SX1262:
  // The module has one antenna, and an RF switch has to point it at either the transmitter or the receiver. 
  // The chip can drive that switch itself, so we tell the library how:
  // DIO2 flips the switch to transmit, pin 38 (RXEN) enables receive.
  radio.setDio2AsRfSwitch(true);
  radio.setRfSwitchPins(38, RADIOLIB_NC);

  Serial.println("Drone 1 ready. Listening...");
}

void loop() {
  String message;
  int state = radio.receive(message);   // wait for a message.

  if (state == RADIOLIB_ERR_NONE) {
    Serial.print("Drone 1 heard: ");
    Serial.println(message);

    // If ping from Ground Station, respond.
    if (message == "PING") {
      delay(50);                         // short pause before replying.
      radio.transmit("PONG");            // send the reply
      Serial.println("Drone 1 sent: PONG");
    }
  }
}
