import smbus, time
bus = smbus.SMBus(1)
addr = 0x40

bus.write_byte_data(addr, 0, 0x20) # enables word writes
time.sleep(.25)
bus.write_byte_data(addr, 0, 0x10) # enable Prescale change as noted in the datasheet
time.sleep(.25) # delay for reset
bus.write_byte_data(addr, 0xfe, 0x79) #changes the Prescale register value for 50 Hz, using the equation in the datasheet.
bus.write_byte_data(addr, 0, 0x20) # enables word writes

ports = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]

time.sleep(.25)

for p in ports:
	port = p*4+0x06
	bus.write_word_data(addr, port, 0) # chl 0 start time = 0us

time.sleep(.25)
for p in ports:
	port = p*4+0x06
	bus.write_word_data(addr, port+0x02, 312) # chl 0 end time = 2.0ms (90 degrees)
