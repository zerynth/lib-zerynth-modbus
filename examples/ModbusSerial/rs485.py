import gpio
import streams
import mcu
import sfw

from semtech.sx1503 import sx1503


RS485EN  = D40
LED_R    = D47
LED_G    = D54
LED_B    = D55

pinmap = {                      # now add pins definition for the expander
    LED_R   : 7,                       # LED R mapped to pin 7 on sx1503
    LED_G   : 14,                      # LED G mapped to pin 14 on sx1503
    LED_B   : 15,                      # LED B mapped to pin 15 on sx1503
    RS485EN : 0,                       # RS485 pin trasmit/recieve
}

class RS485():
    def __init__(self, watchdog_time):
       sfw.watchdog(0, watchdog_time)
       port_expander = sx1503.SX1503(I2C0, 400000)
       gpio.add_expander(1, port_expander, pinmap)
       gpio.mode(LED_R, OUTPUT)
       gpio.mode(LED_G, OUTPUT)
       gpio.mode(LED_B, OUTPUT)
       gpio.mode(RS485EN, OUTPUT)
       gpio.low(LED_G)
       gpio.low(LED_R)
       gpio.high(LED_B)
       gpio.low(RS485EN)
       self.port = streams.serial(drvname=SERIAL1, baud=9600, set_default=False)
    
    def read(self):
        gpio.low(RS485EN)
        bc = self.port.available()
        sfw.kick()
        return self.port.read(bc)

    def write(self, packet):
        gpio.high(RS485EN)
        self.port.write(packet)
        gpio.low(RS485EN)
        sfw.kick()

    def close(self):
        self.port.close()
