# Sony ground-truth capture and fit

Record Sony's driver (under Ignition), then measure our optical pipeline
against it.

1. `use-sony.sh`, start SteamVR, then `capture.sh 60 ~/sony-capture-N`
   (moves: both controllers in view, slow turns on every axis).
2. Raw gyro vs Sony's angular velocity (device <- raw IMU):
   `python3 gyrofit.py ~/sony-capture-N`
3. LD blobs into an optical trace with Sony's poses as P lines:
   ```
   gcc -O2 -I../../userspace/lib/sie/irtracker -o pcap2f pcap2f.c ../../userspace/lib/sie/irtracker/psvr2_ld.c -lm
   off=$(python3 -c 'import time;print(time.time_ns()-time.monotonic_ns())')
   ./pcap2f ~/sony-capture-N/usb.pcap $off > f.txt
   python3 mkp.py f.txt ~/sony-capture-N 0 > trace.txt
   ```
   `off` converts usbmon's wall-clock time to CLOCK_MONOTONIC; measure it
   soon after the capture (an NTP step in between breaks it).
4. Replay and fit head <- rig and the IMU mount against Sony:
   ```
   ../../steamvr/driver_psvr2/tools/optical_replay/build.sh /tmp/rp
   /tmp/rp/optical_replay trace.txt > rep.txt
   python3 fitall.py rep.txt trace.txt ~/sony-capture-N 50.3
   ```
   A correct setup prints a head-frame correction G near 0 deg and delta
   near 0 m.
