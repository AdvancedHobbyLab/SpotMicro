import smbus, time
import argparse
bus = smbus.SMBus(1)
addr = 0x40

## Running this program will move the servo to 0, 45, and 90 degrees with 5 second pauses in between with a 50 Hz PWM signal.
def initialize():
  bus.write_byte_data(addr, 0, 0x20) # enables word writes
  time.sleep(.25)
  bus.write_byte_data(addr, 0, 0x10) # enable Prescale change as noted in the datasheet
  time.sleep(.25) # delay for reset
  bus.write_byte_data(addr, 0xfe, 0x79) #changes the Prescale register value for 50 Hz, using the equation in the datasheet.
  bus.write_byte_data(addr, 0, 0x20) # enables word writes
  time.sleep(.25)

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Tool to measure servo position offsets")
  parser.add_argument('port', type=int, default=0, help="The servo index")
  
  args = parser.parse_args()
  
  # Translate port to internal adress
  port = args.port*4+0x06
  
  initialize()
  
  # Set start pulse
  bus.write_word_data(addr, port, 0)
  
  time.sleep(2)
  
  # Set end pulse (90deg)
  value = 312
  bus.write_word_data(addr, port+0x02, value)
  
  print("Adjust center position")
  print("Enter amount to adjust or leave blank to exit")
  
  done = False
  while not done:
    text = input("adjustment: ")
    if text == "":
      done = True
    else:
      value += int(text)
      bus.write_word_data(addr, port+0x02, value)
  
  print("Finished: ", (value-312))
  
