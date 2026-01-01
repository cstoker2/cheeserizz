/*
12-11-25 changing logger outputs
12-8-25 logging hotloop is working, aux6 is regulating pulse strength edit G1 offset in mix , aux7 trigger and throttle down for data dump w/ 17.17 as magic number value
want to try changing library to have a packet flag for labels vs data to eliminate duplicate transmission of labels.
12-1-25 attempt at recording hotloop for later logging telem output.
11-19-25 added aux6 & aux7 inputs for parameter tweaking from radio values.
11-19-25 NMRC Botsgiving notes: may try TRANSL_STRENGTH of 0.75 to see. Felt like trans was poor, also may try having leds always show fixed heading vs following stick.
9-28-25 Botbash notes: want to improve translation by adding parameters to knobs to fiddle while driving. Also want to record telemetry data to card.
8-4-25 driving pretty well, changed radius range 25-35 and make rudder more authoritative +/- 3mm so trim can be used well
7-30-25 finally seems to work on the bench, needs testing
7-30-25 hardware kinda working, changed serial2 pin to correct val.  escs resetting frequently, trying always send values.
7-19-25 integrated with basic dshot, crsf, dotstar example
7-15-25 removing excess from the files, going to integrate CRSFAccelLEDforRP2350
7-7-25 plan for new protocols:
dchot, crsf, adafuitneopixel, h3lis331 libraries
5-18-2025 mapping pins for new hardwre config
5-17-25 split into several files, going to integrate dshot, crsf, and fast led based on teensy_dshot_crsf_fastled.ino
2-28-25 finally correct translation directions once sign reversed on stichAngle
2-12-25 redoing teensy based control for 1.5lbs bot
need to update:  heading control in mbd(), remove log?, led patterns, pause interrupts for mbd(), rid blink mode, test sensor fusion?

1/28/25 compiles ok for teensy. Preparing to move to esp32 and try new library, again
1/13/25  testing shows more stability with klman filter and only using z accel data. kalman Q of about 1 seems most stable.

rotation is CW, when translating forward, the heading seems to bounce about 90 deg CCW. 
crappy loose battery managment circuit made for dropouts on the teensy power bus.
1/14 tweaked stickangle to start at 12oclock instead of 3oclock...

*/
//Pin Summary:
//Serial2 8,7 leds
//Serial3 14,15 ibus
//Wire 18,19 accel, mag
//Wire1 16,17 
//esc's 6,5