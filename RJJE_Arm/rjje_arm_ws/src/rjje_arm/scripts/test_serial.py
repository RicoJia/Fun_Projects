import serial 

ser = serial.Serial("/dev/ttyUSB0", 9600, timeout=2)
byte = (0xFF).to_bytes(1, "little")
print(byte)
ser.write(byte)
while True: 
    a = ser.read(1)
    if a: 
        print(ord(a))
