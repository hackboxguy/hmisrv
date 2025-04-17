# hmisrv
An hmi server for headless Linux boards to allow humans to interact with machine through hid(/dev/input/eventx) and display(/dev/ttyACMx)

# Hardware
As shown in [this git repo](https://github.com/hackboxguy/usb-hid-display), prepare the rpi-pico(rp2040) based USB-HMI-input-output hardware along with ssd1306 and rotary-encoder.

## How to build and run
- ```make```
- Ensure your usb-hid-display hw is plugged in
- Using ```dmesg``` check for the /dev/input/eventX and /dev/ttyACMX device nodes
- e.g ```sudo ./hmisrv -i /dev/input/event11 -s /dev/ttyACM0```
