# Hardware tests for the dev branch

These tests give the data needed to finish controller optical tracking and to
check the head-pose change. Each test says what to do and what to send back.
Run them in order: later tests need the setup from earlier ones.

Everything stays on your machine. Send back only the log lines asked for.

## 0. Setup (once)

1. Build and load the dev kernel module, and install its udev rules:

   ```bash
   cd ~/Claude/psvr2/src-dev && make -C kernel && sudo rmmod psvr2; sudo insmod kernel/psvr2.ko
   ```

   ```bash
   sudo cp ~/Claude/psvr2/src-dev/kernel/99-psvr2.rules /etc/udev/rules.d/ && sudo udevadm control --reload && sudo udevadm trigger
   ```

2. Point SteamVR at the dev driver build:

   ```bash
   ln -sfn ~/Claude/psvr2/src-dev/steamvr/driver_psvr2/build/psvr2 ~/.steam/debian-installation/steamapps/common/SteamVR/drivers/psvr2
   ```

3. With SteamVR closed, add these two lines inside the existing
   `"driver_psvr2" : { ... }` section of
   `~/.steam/debian-installation/config/steamvr.vrsettings`:

   ```
   "optical_tracking" : true,
   "optical_log_blobs" : true,
   ```

   To undo, set both to `false`.

4. Start SteamVR once, wear the headset until the view is up, then close
   SteamVR. Check that the module saw the calibration:

   ```bash
   sudo dmesg | grep -i psvr2 | tail -40
   ```

   **Send back:** the dmesg lines that mention calibration or the clock.

## 1. Camera calibration

Start SteamVR, wear the headset for about 10 s, then close SteamVR.

```bash
grep "psvr2: optical:" ~/.steam/debian-installation/logs/vrserver.txt | head -60
```

**Send back:** all the lines from the command, especially the
`cam N ... K ...`, `cam N raw ...`, `cam N model ...` and `extrinsic ...`
lines.

**Answers:** the real calibration image size (is 508/W exact?), whether the
unused distortion terms are about 0, and the extrinsic layout.

## 2. LED sync

1. Turn on both Sense controllers and start SteamVR.
2. Wear the headset. Hold both controllers in front of you, about 40 cm
   away, for 30 s. Watch the controller lights: they should flicker, then
   settle.
3. Close SteamVR.

```bash
grep -E "psvr2: (ledsync|optical):" ~/.steam/debian-installation/logs/vrserver.txt | grep -v "  cam\|extrinsic" | head -80
```

**Send back:** the output. Also say what the lights did: flickered and
settled, stayed off, stayed on, or kept flickering.

**Answers:** whether each controller reaches `STABLE`, and where it stops if
it doesn't.

## 3. Which camera is which (cover test)

The headset has four cameras on the front. Name them from **your** point of
view while wearing it: top-left, top-right, bottom-left, bottom-right.

1. Start SteamVR and wear the headset. Hold one controller still in front of
   you, about 40 cm away, at chest height. The other controller stays off.
2. Wait 10 s. Then cover **one** camera fully with a finger or a piece of
   tape for 10 s and uncover it. Wait 5 s.
3. Repeat step 2 for each camera, in this order: top-left, top-right,
   bottom-left, bottom-right.
4. Close SteamVR.

```bash
grep "psvr2: blobs:" ~/.steam/debian-installation/logs/vrserver.txt | tail -400 > ~/psvr2-test3.txt
```

**Send back:** `~/psvr2-test3.txt` and the order in which you covered the
cameras, if it differed.

**Answers:** which camera index (0–3) belongs to which physical camera. The
`seen` count of the covered camera drops to 0.

## 4. Pixel range of each camera

1. Start SteamVR and wear the headset. Turn on one controller.
2. Slowly sweep the controller across your whole field of view for 30 s: far
   left to far right, top to bottom, and near (20 cm) to far (arm's length).
   Make sure it reaches the edges where you can barely see it.
3. Close SteamVR.

```bash
grep "psvr2: blobs:" ~/.steam/debian-installation/logs/vrserver.txt | tail -400 > ~/psvr2-test4.txt
```

**Send back:** `~/psvr2-test4.txt`.

**Answers:** the pixel range of the blob centres (`x ..`, `y ..`). This shows
whether blob coordinates are in the 508×508 model image or in the sensor's
own pixels, and the largest blob size (`box`).

## 5. Camera directions (fixed positions)

This gives each camera's viewing direction relative to the headset.

1. Sit still and keep your head still, looking straight ahead. Start SteamVR
   with one controller on.
2. Hold the controller at each position below for 10 s, with the light ring
   facing you. Say "now" to yourself and count to 10; the exact time doesn't
   matter, but keep the order:
   1. straight ahead, 50 cm, at eye level;
   2. 50 cm ahead, 30 cm to your left;
   3. 50 cm ahead, 30 cm to your right;
   4. 50 cm ahead, 30 cm above eye level;
   5. 50 cm ahead, 30 cm below eye level (about chest height).
3. Close SteamVR.

```bash
grep "psvr2: blobs:" ~/.steam/debian-installation/logs/vrserver.txt | tail -400 > ~/psvr2-test5.txt
```

**Send back:** `~/psvr2-test5.txt`.

**Answers:** where each position lands in each camera image, and so the
cameras' rotations (camera 0's built-in rotation is not documented).

## 6. Head pose (lever-arm change)

The dev driver now places the head origin between your eyes, not at the
headset's motion sensor. This needs no controllers. Set
`"optical_tracking" : false` first if test 2 showed problems.

1. Start SteamVR and stand in a room with a clear object about 1 m away,
   such as a table edge or a SteamVR menu.
2. Keep your eyes on the object. Turn your head left and right (about 45°
   each way), then nod up and down, several times.
3. Watch whether the object stays still, or slides and swims as you turn.
4. Repeat with the original driver for comparison:

   ```bash
   ln -sfn ~/Claude/psvr2/src/steamvr/driver_psvr2/build/psvr2 ~/.steam/debian-installation/steamapps/common/SteamVR/drivers/psvr2
   ```

   Switch back to dev afterwards with the command from step 0.2.

**Send back:** for each driver, whether the object stays still, slides a
little, or slides a lot, and in which motion (turning or nodding).

**Answers:** whether the 2.08 cm / 9.255 cm head offset is right in our
frame.

## After testing

To return to the original driver:

```bash
ln -sfn ~/Claude/psvr2/src/steamvr/driver_psvr2/build/psvr2 ~/.steam/debian-installation/steamapps/common/SteamVR/drivers/psvr2
```
