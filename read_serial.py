import time
import serial

# https://pyserial.readthedocs.io/en/latest/shortintro.html#readline
ss = serial.Serial("/dev/ttyACM0")

# read string
_ = ss.readline() # first read may be incomplete, just toss it
raw_string = ss.readline().strip().decode()
print (raw_string)

while True:
    #print("------------")
    raw_string = ss.readline().strip().decode()
    print (raw_string)
    continue
