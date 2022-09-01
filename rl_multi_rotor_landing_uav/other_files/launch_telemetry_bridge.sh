#!/bin/bash

while true; do

	/home/ubuntu/LibrePilot/ground/TelemetryBridge/bridge >/home/ubuntu/Desktop/telemetrylog-$( date +%s ).opl
done
