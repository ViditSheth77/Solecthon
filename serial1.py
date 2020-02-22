import serial

s=serial.Serial('/dev/ttyACM1',115200)
while True:
	ip = input('Enter char 0 to 9: ')
	s.write(str.encode(ip))

