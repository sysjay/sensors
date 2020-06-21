import uos, machine

from machine import RTC
#import ntptime
#ntptime.settime()
#rtc = RTC()
#rtc.datetime()

#uos.dupterm(None, 1) # disable REPL on UART(0)
import gc
import webrepl

def do_connect():
    import network
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('connecting to network...')
        wlan.connect('hogwarts', 'cousteau')
        while not wlan.isconnected():
            pass
        webrepl.start()
    print('network config:', wlan.ifconfig())
do_connect()


gc.collect()


import machine
i2c = machine.I2C(scl=machine.Pin(16), sda=machine.Pin(17))

print('Scan i2c bus...')
devices = i2c.scan()

if len(devices) == 0:
  print("No i2c device !")
else:
  print('i2c devices found:',len(devices))

  for device in devices:  
    print("Decimal address: ",device," | Hexa address: ",hex(device))
    
