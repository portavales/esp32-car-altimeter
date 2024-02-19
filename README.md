

# Pre-requisites and installation

## Install the Arduino IDE
```
$ apt-get install arduino
```
https://learn.adafruit.com/adafruit-trinkey-qt2040/arduino-ide-setup
https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json


## Install the Arduino command line interface (arduino-cli)
I prefer using a command line interfance instead of the IDE, so in this tutorial I will show the commands using the arduino-cli.
```
$ brew update
$ brew install arduino-cli
```
https://arduino.github.io/arduino-cli/0.21/installation/

### How to use arduino-cli
https://www.pcbway.com/blog/Activities/Arduino_cli__compile__upload_and_manage_libraries__cores__and_boards.html



## Initialize the arduino env
```
$ arduino-cli config init
```

## Install the board config
Listing all boards:
```
$ arduino-cli core update-index
$ arduino-cli board listall
```

Add the additional repository from Espressif to the arduino libraries manager.
```
$ arduino-cli config add board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_dev_index.json
$ arduino-cli core update-index
$ arduino-cli core install esp32:esp32@2.0.14
```

The board we are using is the **lilygo t-display-s3**
```
$ arduino-cli board listall esp32 | grep lilygo
$ arduino-cli core install esp32:esp32:lilygo_t_display_s3
```

If you have troubles, follow the guide from this YT video:
https://www.youtube.com/watch?v=gpyeMjM9cOU&t=352s


## Install the necessary libraries
Install the TFT_eSPI library from Bodmer (https://github.com/Bodmer/TFT_eSPI)
```
$ arduino-cli lib install "TFT_eSPI"
```
Other libraries needed:
```
$ arduino-cli lib install "BSEC Software Library"
$ arduino-cli lib install "TinyGPSPlus-ESP32"
$ arduino-cli lib install "BH1750"
$ arduino-cli lib install "EspSoftwareSerial"
$ arduino-cli lib install "Preferences"
```

# Compile & flash it
Plug the Lilygo into the USB port and run
```
$ ./build.sh
```
