################################################################################
# Modbus communications via tcp.
#
# Created: 2020-10-15
# Author: S. Torneo
################################################################################

import streams

# Needed for internet connectivity.
# This is dependent on your device.
from wireless import wifi 
from espressif.esp32net import esp32wifi as wifi_driver
from modbus import modbus

wifi_driver.auto_init()
streams.serial()

try:
    try:
        wifi.link(SSID, 3, PASSWORD)
    except Exception as e:
        print("Exception while trying to connect to the network ", e)
            
    master = modbus.ModbusTCP(1)

    try:
        master.connect(IP_ADDRESS)
        print("Connected.")
    except Exception as e:
        print("Couldn't connect to the slave device ", e)

    result = master.write_coil(2, 0xff00)
    print("Written coil 2: ", result)

    result = master.write_register(55, 745)
    print("Written discrete register: ", result)

    result = master.write_multiple_coils(10, 10, [1, 0, 1, 1, 0, 0, 1, 1, 1, 0])
    print("Written ", result, " coils")

    result = master.write_multiple_registers(20, 10, [55, 555, 123, 42, 352, 546, 754, 34, 643, 23])
    print("Written ", result, " registers")

    coils = master.read_coils(10, 10)
    print("Coils 10-19 status: ", coils)

    discrete = master.read_discrete(10, 5)
    print("Discrete inputs 10-14 values: ", discrete)

    holding = master.read_holding(20, 10)
    print("Holding registers 20-29 values: ", holding)

    inputs = master.read_input(20, 10)
    print("Input registers 20-29 values: ", inputs)

    master.close()
    print("Closed.")
	
except Exception as e:
	print("Error ", e)

