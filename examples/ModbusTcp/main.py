################################################################################
# Modbus communications via tcp.
#
# Created: 2019-03-21
# Author: L. Marsicano
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
        wifi.link("SSID", 3, "password")
    except Exception as e:
        print("Exception while trying to connect to the network ", e)
            
    main = modbus.ModbusTCP(1)

    try:
        main.connect("ip-address")
        print("Connected.")
    except Exception as e:
        print("Couldn't connect to the subordinate device ", e)

    result = main.write_coil(2, 0xff00)
    print("Written coil 2: ", result)

    result = main.write_register(55, 745)
    print("Written discrete register: ", result)

    result = main.write_multiple_coils(10, 10, [1, 0, 1, 1, 0, 0, 1, 1, 1, 0])
    print("Written ", result, " coils")

    result = main.write_multiple_registers(20, 10, [55, 555, 123, 42, 352, 546, 754, 34, 643, 23])
    print("Written ", result, " registers")

    coils = main.read_coils(20, 19)
    print("Coils 20-38 status: ", coils)

    discrete = main.read_discrete(20, 15)
    print("Discrete inputs 20-34 values: ", discrete)

    holding = main.read_holding(20, 10)
    print("Holding registers 20-29 values: ", holding)

    inputs = main.read_input(20, 10)
    print("Input registers 20-29 values: ", inputs)

    main.close()
    print("Closed.")
	
except Exception as e:
	print("Error ", e)
