#!/usr/bin/env sh
set -eu
if [ "$#" -ne 1 ]; then echo "Usage: $0 /path/to/PX4-Autopilot"; exit 2; fi
cd "$1"
# PX4's maintained multi-instance launcher assigns system IDs and UDP ports for
# instances 0, 1, and 2. The matching mesh ports are 14540, 14541, and 14542.
./Tools/simulation/sitl_multiple_run.sh 3 sihsim_quadx px4_sitl_sih
