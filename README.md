# BASSTURD
ESP32 bluetooth soundcard with rotary encoder volume control, oled display and needle vu-meters

Warning. This is by no means quality code. Use at your own risk.
The oled code original source can be found in the respective headers.

This project is based on the ESP-IDF example for a2dp-sink which was changed and adapted to suit my needs.

For the DAC part, I used an IQaudIO DAC Pro  ( https://cdn.sparkfun.com/assets/e/6/5/a/9/20201209_IQaudIO_v32.pdf page 8 ) which uses PCM5242 and which had to have a small HW change to make it work automatically with the ESP. Basically one resistor had to be moved from one place to another to make the DAC setup itself in a default generic state that just takes and converts data without any software config as it is done when used on the raspberryPI. I used this DAC board because it is very high quality regarding signal, has optional differential outputs that I can connect to my M-AUDIO BX8's and has a nice integrated headphone amp that is the best one I got to test so far.
The rotary encoder is any generic rotary encoder. Nothig special about it.

The OLED display (I2C 0.91 inch white generic oled display) and the needle vu-meters should be optional. There is absolutely no problem if you do not connect the needle vu-meters. The ones I used are from an old Tesla B115 reel-to-reel player. Connect them  through trimpots (preferably multi-turn) because the pwm signal comming from the ESP32 is DEFINITELY too strong for them. Just adjust the pots to hit max scale at you max signal level.

The display currently shows signal level for each channel, with a peak indicator that slowly falls off.
Also when changing volume through rotary encoder or through the source (PC or some phones), the volume updated value is overlayed in negative color, realtime, over the OLED level meters.
IF you don't wanna use the Oled display, it's better to comment out all the code (init and write) that uses the display. It's I2C so it might not like it if the device is missing and not ACK-ing back the messages. I don't know, I did not run it without it.

