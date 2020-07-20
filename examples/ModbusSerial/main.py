################################################################################
# Modbus communications via serial line.
#
# Created: 2019-03-21
# Author: L. Marsicano
################################################################################

import rs485
from modbus import modbus
import streams

try:
    device = rs485.RS485(100000)
    # Wait for the serial port to open. This may not be necessary
    sleep(10000)
    main = modbus.ModbusSerial(1, serial_device = device)

    result = main.write_register(5, 745)
    print("Written discrete register: ", result)

    result = main.write_multiple_registers(2, 4, [55, 555, 123, 42])
    print("Written ", result, " registers")

    discrete = main.read_discrete(2, 4)
    print("Discrete inputs values: ", discrete)

    holding = main.read_holding(3, 2)
    print("Holding registers values: ", holding)

    main.close()
    print("Closed.")
	
except Exception as e:
	print("Exception ", e)
