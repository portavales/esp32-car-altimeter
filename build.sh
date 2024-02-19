set -e
DIR=./
arduino-cli compile --fqbn esp32:esp32:lilygo_t_display_s3 $DIR
arduino-cli upload --port /dev/ttyACM0 --fqbn esp32:esp32:lilygo_t_display_s3 $DIR -v

python3 read_serial.py
