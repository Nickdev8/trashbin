# Disco Trashbin - Planning and Electronics Design
**Author:** Nick  
**Time spent:** 5.5 hours

## Intention
Plan the Arduino system, sensors, motor control, and audio.

## What I built
- Sketched layout for two ultrasonic sensors, relay motor control, and the Adafruit Audio Shield.
- Breadboarded the audio shield and tested SD playback.
- Planned serial messages for syncing with Nasi’s ESP32.

## Decisions
- Use two sensors for wider detection.
- Shared ground across all boards.

## Issues and fixes
- SD card not detected. Reformatted and set correct CS pin.

## Picture
![](https://hc-cdn.hel1.your-objectstorage.com/s/v3/ebb72fcc63f11f2fd794074e41d1da6c29974ced_tmp_3Achat_3A2025-11-02_3Acmh69pxsq00cop401dxr6jvx8_3Ab91478f2a7887025)
![](https://hc-cdn.hel1.your-objectstorage.com/s/v3/0f264e4f6b700de5d3028cf9c54e851addd2ef26_tmp_3Achat_3A2025-11-02_3Acmh69pxsq00cop401dxr6jvx8_3Ac7cbe5561932f6a4)




# Disco Trashbin - Main Wooden Frame Build
**Author:** Nick  
**Time spent:** 6.25 hours

## Intention
Build a sturdy frame for the trashbin.

## What I built
- Cut beams and base, predrilled and assembled the frame.
- Reinforced corners and checked squareness.
- Measured internal spacing for shelves.

## Decisions
- Dimensions set to 150 x 50 x 50 cm.
- Added a brace near the top plate.

## Issues and fixes
- Slight racking fixed with a diagonal brace.

## Picture
![](https://hc-cdn.hel1.your-objectstorage.com/s/v3/989bdccbb063d76f13ebe8bf9132b20f00e648e4_tmp_3Achat_3A2025-11-02_3Acmh69pxsq00cop401dxr6jvx8_3Ae2413984584cd47c)
![](https://hc-cdn.hel1.your-objectstorage.com/s/v3/165c696238023d49441acffe68c9e15f3129da64_tmp_3Achat_3A2025-11-02_3Acmh69pxsq00cop401dxr6jvx8_3Aad568ad0ceeac898)




# Disco Trashbin - Shelves, Panels, Internal Mounts
**Author:** Nick  
**Time spent:** 5.75 hours

## Intention
Create internal structure for electronics and wiring.

## What I built
- Installed shelves and chute behind the slot.
- Marked mounting points for boards and speakers.
- Drilled pass-through holes for LED and sensor cables.

## Decisions
- Separate electronics shelf and motor zone.
- Cable tie points along the wall.

## Issues and fixes
- Shelf interfering with the crate lip. Trimmed by 5 mm.

## Picture
![](https://hc-cdn.hel1.your-objectstorage.com/s/v3/159a80ae517080b0abd7c3f49f723a3ef269ff31_tmp_3Achat_3A2025-11-02_3Acmh69pxsq00cop401dxr6jvx8_3A27308a175cab3c18)






# Disco Trashbin - Arduino Sensors, Motor, Audio
**Author:** Nick  
**Time spent:** 8.0 hours

## Intention
Program the Arduino for detection, motor control, and sound.

## What I built
- Wrote sensor code with filtering and entry detection.
- Added relay motor control with PWM speed ramp.
- Integrated the audio shield to play sounds.
- Added START_SHOW and END_SHOW serial messages.

## Decisions
- Entry threshold tuned to avoid false triggers.
- Added cooldown timer.

## Issues and fixes
- Sensor spikes at startup fixed with boot delay.

## Source Code
Here is the [Source Code](https://github.com/Nickdev8/trashbin/blob/main/trashbin.ino) running on the ardiuno

## Picture
![](https://hc-cdn.hel1.your-objectstorage.com/s/v3/f85d969ce94a9c3882896d554be96e0c96b3acfa_tmp_3Achat_3A2025-11-02_3Acmh69pxsq00cop401dxr6jvx8_3Ab82ec6639c4d5a46)





# Disco Trashbin - Soldering and Power Distribution
**Author:** Nick  
**Time spent:** 7.5 hours

## Intention
Permanently wire the electronics.

## What I built
- Soldered sensor leads and connectors.
- Made fused 5 V and 12 V power lines.
- Crimped motor and relay connections.
- Labeled and heat-shrank everything.

## Decisions
- Inline fuse holders for quick replacement.
- Zip-tie anchors for cable routing.

## Issues and fixes
- Short on audio ground fixed by redoing a wire.

## Picture
![](https://hc-cdn.hel1.your-objectstorage.com/s/v3/76eeb58849a9863c13ebae54611a44874a18a1de_tmp_3Achat_3A2025-11-02_3Acmh69pxsq00cop401dxr6jvx8_3A59b199bb94c9c47e)




# Disco Trashbin - Painting and Finish
**Author:** Nick  
**Time spent:** 5.0 hours

## Intention
Paint and protect the wooden body.

## What I built
- Sanded surfaces and applied two blue coats.
- Painted the top plate black.
- Touched up edges and screw areas.

## Decisions
- Matte blue body and black top.

## Issues and fixes
- Paint drips near slot smoothed and repainted.

## Picture
![](https://hc-cdn.hel1.your-objectstorage.com/s/v3/5a137b9317d621e580915ca840fde54bdaa9fcc8_tmp_3Achat_3A2025-11-02_3Acmh69pxsq00cop401dxr6jvx8_3Acc882ab8a2f03d66)



# Disco Trashbin - Final Assembly and Mounting
**Author:** Nick  
**Time spent:** 5.75 hours

## Intention
Install electronics and hardware into the bin.

## What I built
- Mounted Arduino, relay, speakers, and power blocks.
- Installed disco ball motor.
- Placed acrylic window and slot plate.

## Decisions
- Rubber washers on motor mount.
- Velcro under audio shield.

## Issues and fixes
- Motor wobble fixed by shimming the mount.

## Picture
![](https://hc-cdn.hel1.your-objectstorage.com/s/v3/149afaedf39db36b87f60bb306c910ab8609ffc7_tmp_3Achat_3A2025-11-02_3Acmh69pxsq00cop401dxr6jvx8_3A834f83a2f8bd675b)
![](https://hc-cdn.hel1.your-objectstorage.com/s/v3/dd60e2131c776eab9303bd1f67220cb7cd7ee349_tmp_3Achat_3A2025-11-02_3Acmh69pxsq00cop401dxr6jvx8_3A3349b8bd9e253b4b)


# Disco Trashbin - Full System Test
**Author:** Nick  
**Time spent:** 5.0 hours

## Intention
Finalize timing and test all effects together.

## What I built
- Tuned sensor thresholds and cooldown.
- Synced audio and LED start.
- Fixed loose LED connector and rerouted one cable.
- Ran full test cycles.

## Decisions
- Show length capped at 6 seconds.
- Safe RPM limit for motor.

## Issues and fixess
- Double triggers fixed by adding ignore window.

## Picture
<video class="journal-video" controls playsinline style="aspect-ratio: 390/848;" src="https://hc-cdn.hel1.your-objectstorage.com/s/v3/9b8a4719610281b391557538d4913e0102d56557_tmp_3Achat_3A2025-11-02_3Acmh69pxsq00cop401dxr6jvx8_3Ac2061d1ff6119ea6"></video>
<video class="journal-video" controls playsinline style="aspect-ratio: 848/390;" src="https://hc-cdn.hel1.your-objectstorage.com/s/v3/8584a01e76710bc87a8fb53d56194626dee00ec8_tmp_3Achat_3A2025-11-02_3Acmh69pxsq00cop401dxr6jvx8_3Aa3edd433144af89e"></video>

<video class="journal-video" controls playsinline style="aspect-ratio: 2160/3840;" src="https://hc-cdn.hel1.your-objectstorage.com/s/v3/cc011139dd6296c27912b7d7d9b3e75693dc8d07_tmp_3Achat_3A2025-11-02_3Acmh69pxsq00cop401dxr6jvx8_3Adb80fe575ddba916"></video>
<video class="journal-video" controls playsinline style="aspect-ratio: 2160/3840;" src="https://hc-cdn.hel1.your-objectstorage.com/s/v3/378a4e8e3ee834eae5dc67edbf6bcdab10e5cb80_tmp_3Achat_3A2025-11-02_3Acmh69pxsq00cop401dxr6jvx8_3A7c3b90f674708059"></video>