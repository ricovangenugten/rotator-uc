#!/bin/bash

# enter ota mode
mqtt_pub rotator_01/set OTA

#wait a bit
sleep 1

# upload
pio run -t upload -e d1_mini --upload-port 192.168.1.187

# enter normal mode, triggering reboot
mqtt_pub rotator_01/set NORMAL
