#pragma once

#include <Arduino.h>

#include "MavlinkTelemetryProvider.h"

/*
  Optional software-only MAVLink parser check

  The test creates valid MAVLink frames in memory, converts them to raw UART-like
  bytes, and feeds those bytes through the same ingestByte path used in flight.
  It checks parsing and mapping but cannot test physical wires or PX4 settings.
*/
namespace AndyMesh {

bool runMavlinkSyntheticTest(MavlinkTelemetryProvider& provider,
                             uint32_t nowMs,
                             Print& log);

}  // namespace AndyMesh
