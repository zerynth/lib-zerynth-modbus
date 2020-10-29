################################################################################
# Modbus communications via serial line.
#
# Created: 2020-10-12
# Author: S. Torneo
################################################################################

from semtech.sx1503 import sx1503
import gpio
from modbus import modbus
import streams

RS485EN  = D40
LED_R    = D47
LED_G    = D54
LED_B    = D55

# add pins definition for the expander
pinmap = {
    LED_R   : 7, # LED R mapped to pin 7 on sx1503
    LED_G   : 14, # LED G mapped to pin 14 on sx1503
    LED_B   : 15, # LED B mapped to pin 15 on sx1503
    RS485EN : 0, # RS485 pin trasmit/recieve
}

streams.serial()

try:
    port_expander = sx1503.SX1503(I2C0, 400000)
    gpio.add_expander(1, port_expander, pinmap)
    
    config_serial = modbus.ConfigSerial(SERIAL1, 9600, rs485en=RS485EN)
    master = modbus.ModbusSerial(1, cfg = config_serial)
    
    register = master.write_register(2, 10)
    print("Value written in the register: ", register)
    
    holding = master.read_holding(2, 1)
    print("Value of holding register read: ", holding)

    result = master.write_multiple_registers(3, 4, [55, 555, 123, 42])
    print(result, "registers written")
    
    holding = master.read_holding(2, 5)
    print("Values of holding register read: ", holding)
    
    coil = master.read_coils(1, 1)
    print("Status of coil read: ", coil)
    
    discrete = master.read_discrete(1, 1)
    print("Status of discrete register read: ", discrete)

    master.close()
    print("Closed.")
	
except Exception as e:
	print("Exception ", e)
