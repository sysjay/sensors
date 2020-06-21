#!/bin/bash
# transfer files to ESP microcontroller
echo Send data to the ESP devide via webrpl_cli.pi 
echo parm1 = $1
webrepl_cli.py -p $1 main.py 192.168.1.163:main.py
webrepl_cli.py -p $1 BME280.py 192.168.1.163:BME280.py
