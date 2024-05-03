
## MACOS Specific:
https://learnembeddedsystems.co.uk/pico-usb-serial-code#:~:text=Connect%20your%20Pico%20over%20usb,your%20Pico%20is%20connected%20to.
find the serial port:
```bash
ls /dev/cu.*
```
connect to the serial port:
```bash
screen /dev/cu.usbserial... 115200
```