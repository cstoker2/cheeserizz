/*
3-3-25 continuous phase working and keeping it much less jumpy. Magnetometer is still not being used, too low signal. 

2-28-25 finally correct translation directions once sign reversed on stickAngle
2-12-25 redoing teensy based control for 1.5lbs bot
need to update:  heading control in mbd(), remove log?, led patterns, pause interrupts for mbd(), rid blink mode, test sensor fusion?

1/28/25 compiles ok for teensy. Preparing to move to esp32 and try new library, again
1/13/25  testing shows more stability with klman filter and only using z accel data. kalman Q of about 1 seems most stable.

rotation is CW, when translating forward, the heading seems to bounce about 90 deg CCW. 
crappy loose battery managment circuit made for dropouts on the teensy power bus.
1/14 tweaked stickangle to start at 12oclock instead of 3oclock...

*/
// teensy 4.0, accelerometer mounted vertically w/ z+ pointed in towards center
//Pin Summary:
//Serial2 8,7 leds
//Serial3 14,15 ibus
//Wire 18,19 accel, mag interrupt 16
//esc's 6,5