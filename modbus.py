# Zerynth - libs - zerynth-modbus/modbus.py
#
# Zerynth library for Modbus
#
# @Author: Stefano Torneo
#
# @Date: 2020-10-12
# @Last Modified by:   m.cipriani
# @Last Modified time: 2020-10-21 16:38:42

"""
.. module:: modbus

***************
MODBUS Library
***************
    This module contains all functionalities to communicate with a Modbus device.
    The communication can be of the serial or TCP type.
    When a connection with a slave device has been established, the registers can be read and write.

    """
import socket
import gpio
import streams
import math
import threading
import timers

# Functions
FN_READ_COILS = 0x01
FN_READ_DISCRETE = 0x02
FN_READ_HOLDING = 0x03
FN_READ_INPUT = 0x04

FN_WRITE_COIL = 0x05
FN_WRITE_REGISTER = 0x06
FN_WRITE_MULTIPLE_COILS = 0x0F
FN_WRITE_MULTIPLE_REGISTERS = 0x10

new_exception(ModbusBadAddress,Exception)
new_exception(ModbusBadLength,Exception)
new_exception(ModbusBadData, Exception)
new_exception(ModbusException, Exception)

class Modbus():
    
    def exchange_data(self, pdu, sz):
        pass

    def _req_pdu(self,fncode,*args):
        # calc size: each even *args is the length of the following item
        sz = 0
        for i in range(0, len(args),2):
            sz+=int(args[i])
        pdu = bytearray(sz + 1)
        c=0
        pdu[c]=fncode
        c+=1
        for i in range(0, len(args),2):
            l = int(args[i])
            a = args[i+1]
            if l == 2:
                # big endian encode
                bigE = bytes_to_bytearray(a)
                hb = bigE[0]
                lb = bigE[1]
                pdu[c]=hb
                pdu[c+1]=lb
                c+=2
            elif l==1:
                pdu[c]=a
                c+=1
            else:
                raise ModbusBadLength
        return pdu
    
    def _check_pdu(self, pdu, sz, fncode):
        #-if MODBUS_DEBUG
        print("[INFO] PDU:", [x for x in pdu])
        #-endif
        if (sz == len(pdu) and pdu[0] == fncode):
            return True
        return False
    
    def _parse_pdu_coils_discrete(self, pdu, output_count):
        byte_count = pdu[1]
        status = []
        for i in range(0, byte_count):
            byte = pdu[i+2]
            bits = to_bit_list(byte)
            for b in bits:
                status.append(b)
        return status[0 : output_count]

    def _parse_pdu_read(self, pdu, output_count=None):
        if (output_count != None):
            return self._parse_pdu_coils_discrete(pdu, output_count)
        byte_count = pdu[1]
        ret = []
        for i in range(0, byte_count, 2):
            hb = pdu[i + 2]
            lb = pdu[i + 3]
            ret.append((hb << 8) | lb)
        return ret

    def _parse_pdu_write(self, pdu):
        return (pdu[3] << 8) | pdu[4]

    def read_coils(self, address, n):
        """
    .. method:: read_coils(address, n)
            
            :param address: The starting address
            :param n: the number of coils to read, starting from **address**.

            Reads the status of **n** coils, starting from **address**.

            Returns a list containing the status of coils read if the read has been successfull,
            otherwhise an exception will be raised.
        """
        if address<0 or address>0xffff:
            raise ModbusBadAddress
        if n<0x0001 or n>0x07D0:
            raise ModbusBadLength
        pdu = self._req_pdu(FN_READ_COILS,2,address,2,n)
        sz = 1 + 1 + int(math.ceil(n/8)) # cmd + len + data
        rsp = self.exchange_data(pdu, sz) 
        if self._check_pdu(rsp, sz, FN_READ_COILS):
            return self._parse_pdu_read(rsp, n)
        raise ValueError

    def read_discrete(self, address, n):
        """
    .. method:: read_discrete(address, n)
            
            :param address: The starting address
            :param n: the number of discrete register to read, starting from **address**.

            Reads the status of **n** discrete registers, starting from **address**.

            Returns a list containing the status of discrete registers read if the read has been successfull,
            otherwhise an exception will be raised.
        """
        if address<0 or address>0xffff:
            raise ModbusBadAddress
        if n < 0x0001 or n > 0x07D0:
            raise ModbusBadLength
        pdu = self._req_pdu(FN_READ_DISCRETE, 2, address, 2, n)
        sz = 1 + 1 + int(math.ceil(n/8)) # cmd + len + data
        rsp = self.exchange_data(pdu, sz) 
        if self._check_pdu(rsp, sz, FN_READ_DISCRETE):
            return self._parse_pdu_read(rsp, n)
        raise ValueError

    def read_holding(self, address, n):
        """
    .. method:: read_holding(address, n)
            
            :param address: The starting address
            :param n: the number of holding register to read, starting from **address**.

            Reads the values of **n** holding registers, starting from **address**.

            Returns a list containing the values of holding registers read if the read has been successfull,
            otherwhise an exception will be raised.
        """
        if address<0 or address>0xffff:
            raise ModbusBadAddress
        if n < 0 or n > 0x007D:
            raise ModbusBadLength
        pdu = self._req_pdu(FN_READ_HOLDING, 2, address, 2, n)
        sz = 1 + 1 + (n*2) # cmd + len + data
        rsp = self.exchange_data(pdu, sz)
        if self._check_pdu(rsp, sz, FN_READ_HOLDING):
            return self._parse_pdu_read(rsp)
        raise ValueError

    def read_input(self, address, n):
        """
    .. method:: read_input(address, n)
            
            :param address: The starting address
            :param n: the number of input register to read, starting from **address**.

            Reads the values of **n** input registers, starting from **address**.

            Returns a list containing the values of input registers read if the read has been successfull,
            otherwhise an exception will be raised.
        """
        if address < 0 or address > 0xffff:
            raise ModbusBadAddress
        if n < 0 or n > 0x007D:
            raise ModbusBadLength
        pdu = self._req_pdu(FN_READ_INPUT, 2, address, 2, n)
        sz = 1 + 1 + (n*2) # cmd + len + data
        rsp = self.exchange_data(pdu, sz)
        if self._check_pdu(rsp, sz, FN_READ_INPUT):
            return self._parse_pdu_read(rsp)
        raise ValueError

    def write_coil(self, address, value):
        """
    .. method:: write_coil(address, value)
            
            :param address: The address of the coil
            :param value: is the value to write.

            Returns the value written in the coil if the write has been successfull, 
            otherwhise an exception will be raised.
        """
        if address < 0 or address > 0xffff:
            raise ModbusBadAddress
        if value != 0x0000 and value != 0xff00:
            raise ModbusBadData
        pdu = self._req_pdu(FN_WRITE_COIL, 2, address, 2, value)
        sz = 5
        rsp = self.exchange_data(pdu, sz)
        if self._check_pdu(rsp, sz, FN_WRITE_COIL):
            return self._parse_pdu_write(rsp)
        raise ValueError

    def write_register(self, address, value):
        """
    .. method:: write_register(address, value)
            
            :param address: The address of the register
            :param value: is the value to write.

            Returns the value written in the holding register if the write has been successfull, 
            otherwhise an exception will be raised.
        """
        if address < 0 or address > 0xffff:
            raise ModbusBadAddress
        if value < 0 or value > 0xffff:
            raise ModbusBadData
        pdu = self._req_pdu(FN_WRITE_REGISTER, 2, address, 2, value)
        sz = 5
        rsp = self.exchange_data(pdu, sz)
        if self._check_pdu(rsp, sz, FN_WRITE_REGISTER):
           return self._parse_pdu_write(rsp)
        raise ValueError
    
    def _write_multiple_coils(self, address, n, count, values):
        if address < 0 or address > 0xffff:
            raise ModbusBadAddress
        if n < 0x0001 or n > 0x07b0:
            raise ModbusBadLength
        pdu = self._req_pdu(FN_WRITE_MULTIPLE_COILS, 2, address, 2, n, 1, count, *values)
        sz = 5
        rsp = self.exchange_data(pdu, sz)
        if self._check_pdu(rsp, sz, FN_WRITE_MULTIPLE_COILS):
            return self._parse_pdu_write(rsp)
        raise ValueError
    
    def write_multiple_coils(self, address, n, values):
        """
        .. method:: write_multiple_coils(address, n, values)
            
            :param address: The address of the first coil
            :param n: the number of coils 
            :param values: a list containing the new values to write.

            Returns the number of coils written if the write has been successfull, 
            otherwhise an exception will be raised.
        """
        # Insert padding
        while len(values) % 8 != 0:
            values.append(0)
        count = int(len(values) / 8)
        # Invert every byte
        for i in range(0, count):
            values[i * 8 : i * 8 + 8] = values[i * 8 : i * 8 + 8][::-1]
        # Obtain the value
        out = 0
        for b in values:
            out = (out << 1) | b
        
        # Split each byte
        lst = []
        for i in range(0, count):
            # Append value and length of the value
            lst.append((out >> i * 8) & 0xff)
            lst.append(1)
        return self._write_multiple_coils(address, n, count, lst[::-1])

    def _write_multiple_address(self, address, n, count, values):
        if address < 0 or address > 0xffff:
            raise ModbusBadAddress
        if n < 0x0001 or n > 0x07b0 or count != 2 * n:
            raise ModbusBadLength
        pdu = self._req_pdu(FN_WRITE_MULTIPLE_REGISTERS, 2, address, 2, n, 1, count, *values)
        sz = 5
        rsp = self.exchange_data(pdu, sz)
        if self._check_pdu(rsp, sz, FN_WRITE_MULTIPLE_REGISTERS):
            return self._parse_pdu_write(rsp)
        raise ValueError
    
    def write_multiple_registers(self, address, n, values):
        """
    .. method:: write_multiple_registers(address, n, values)
            
            :param address: The address of the first holding register
            :param n: the number of registers
            :param values: a list containing the new values to write.

            Returns the number of holding registers written if the write has been successfull, 
            otherwhise an exception will be raised.
        """
        count = len(values)

        vals = []
        for i in range(0, len(values)):
            r = bytes_to_bytearray(values[i])
            # Add value and append its length
            vals.append(1)
            vals.append(r[0])
            vals.append(1)
            vals.append(r[1])

        return self._write_multiple_address(address, n, 2 * count, vals)

#-if MODBUS_TCP
class ModbusTCP(Modbus):
    """

===============
ModbusTCP class
===============

.. class:: ModbusTCP(identifier)

    Creates an instance of the ModbusTCP class which allows Modbus communication with slave device using TCP.

    It inherits all of its methods from :class:`MODBUS Library` described above.
    
    :param identifier: The slave device identifier, used in the header of every packet.

    """
    def __init__(self, identifier):
        self.endpoint = identifier
        self._socket = socket.socket(socket.AF_INET, type=socket.SOCK_STREAM)
        self.transmissionID = 1
        self.lock_tcp = threading.Lock()

    def exchange_data(self, pdu, sz):
        self.lock_tcp.acquire()
        self._send_tcp(self.endpoint, pdu)
        pdu = self._recv_tcp()
        self.lock_tcp.release()
        return pdu

    def _recv_tcp(self):
        header = self._socket.recv(7)
        pdu_len = header[4] << 8 | header[5]
        pdu = self._socket.recv(pdu_len - 1)
        return pdu

    def _send_tcp(self, endpoint, pdu):
        # 2 bytes (TransmissionID) + 2 bytes (Protocol ID) + 2 bytes (Length) + 1 byte (Slave Device ID) + N bytes (pdu) + 2 bytes (CRC)
        packet = bytearray(7 + len(pdu))
        bs = bytes_to_bytearray(self.transmissionID)
        # Transmission ID
        packet[0] = bs[0]
        packet[1] = bs[1]
        # Protocol ID
        packet[2] = 0x00
        packet[3] = 0x00
        # Length
        length = bytes_to_bytearray(len(pdu) + 1)
        packet[4] = length[0]
        packet[5] = length[1]
        # Slave Device ID
        packet[6] = endpoint
        # PDU
        packet[7:] = pdu
        self._socket.send(packet)

    def connect(self, address, port = 502):
        """
        .. method:: connect(address, port = 502)
            
            :param address: The ip address of the slave device
            :param port: port on which the slave device is listening to (default value is 502).

            Create the connection with the slave device.
        """
        self._socket.connect((address, port))
        
    def close(self):
        """
        .. method:: close()

            Close the connection with the slave device.

        """
        self._socket.close()
#-endif

#-if MODBUS_SERIAL
PARITY_NONE = 0
PARITY_EVEN = 1

STOPBIT_1 = 0
STOPBIT_1_HALF = 1
STOPBIT_2 = 2

# a dictionary contaning the serial ports used
# the keys are the drvname of serial ports and 
# the value of each key is formed by a Serial() object with adding of field 'port' getting from streams.serial(...)
used_serial = {}

class ConfigSerial():
    """
================
ConfigSerial class
================

.. class:: ConfigSerial(drvname, baud, rs485en = None, stopbits = STOPBIT_1, parity = PARITY_NONE, recv_timeout = 250)

        Creates an object to set some parameters to communicate with serial port.
    
        :param drvname: is the serial name
        
        :param baud: is the baudrate

        :param rs485en: the pin used for RS485 communication (default value is None)

        :param stopbits: :samp:`STOPBIT_1` , :samp:`STOPBIT_1_HALF`, :samp:`STOPBIT_2` (default value is STOPBIT_1)

        :param parity: :samp:`PARITY_NONE` or :samp:`PARITY_EVEN` (default value is PARITY_NONE)

        :param recv_timeout: timeout on the receiving function (default value is 250).

    """
    def __init__(self,
            drvname,
            baud,
            rs485en=None,
            stopbits=STOPBIT_1, 
            parity=PARITY_NONE, 
            recv_timeout = 250):
        self.drvname = drvname
        self.baud = baud
        self.rs485en = rs485en
        self.stopbits = stopbits
        self.parity = parity
        self.recv_timeout = recv_timeout

class ModbusSerial(Modbus):
    """
==================
ModbusSerial class
==================

.. class:: ModbusSerial(identifier, drvname = None, baud = None, rs485en = None, cfg = None)

    Creates an instance of the ModbusSerial class which allows Modbus communication with slave device using RTU.

    It inherits all of its methods from :class:`MODBUS Library` described above.
    
    :param identifier: The slave device identifier

    :param drvname: is the serial name (default value is None)
        
    :param baud: is the baudrate (default value is None)

    :param rs485en: is the pin used for RS485 communication (default value is None)

    :param cfg: is an object of the ConfigSerial class described above (default value is None). 

    If you want communicate with a device through serial communication, you can create a ConfigSerial 
    class object, in order to set more parameters, like showing in this example: ::


        from modbus import modbus
        
        ...
        
        config_serial = modbus.ConfigSerial(drvname=SERIAL1, baud=9600)
        master = modbus.ModbusSerial(1, cfg = config_serial)
        result = master.write_register(2, 10)
        print("Value written in the register: ", result)
    
    or you can pass the drvname, baudrate and rs485en directly to ModbusSerial like showing in this other example: ::

        from modbus import modbus
        
        ...
        
        master = modbus.ModbusSerial(1, drvname=SERIAL1, baud=9600, rs485en=D40)
        result = master.write_register(2, 10)
        print("Value written in the register: ", result)

    """
    def __init__(self, identifier, drvname=None, baud=None, rs485en=None, cfg = None):
        self.endpoint = identifier
        if cfg == None:
            if (drvname == None and baud == None):
                raise UnsupportedError
            self._cfg = ConfigSerial(drvname, baud, rs485en)
        else:
            try:
                self._cfg = ConfigSerial(cfg.drvname, cfg.baud, cfg.rs485en, cfg.stopbits, cfg.parity, cfg.recv_timeout)
            except Exception:
                raise ValueError
        if (self._cfg.rs485en != None):
           gpio.mode(self._cfg.rs485en, OUTPUT)
           gpio.low(self._cfg.rs485en)
        self._check_serial_port(self._cfg)
        self.recv_timeout = self._cfg.recv_timeout
        self.lock_serial = threading.Lock()
        self.tim = None

    def _check_serial_port(self, cfg):
        res = None
        try: 
            res = used_serial[cfg.drvname]
        except Exception:
            pass
        # if serial port never used before
        if res == None:
            self.port = streams.serial(
                drvname=cfg.drvname, 
                baud=cfg.baud,
                stopbits=cfg.stopbits,
                parity=cfg.parity,
                set_default=False)
            used_serial[cfg.drvname] = cfg
            used_serial[cfg.drvname].port = self.port
        else:
            # check the old values of serial port with the new config
            if (cfg.baud != res.baud or 
                cfg.parity != res.parity or 
                cfg.stopbits != res.stopbits):
                raise ValueError # if something is different, raise an exception
            else:
                self.port = res.port # otherwise set self.port to the old value saved

    def _crc_calc(self, packet):
        crc = 0xffff
        for b in packet:
            crc ^= b
            for i in range(0, 8):
                if ((crc & 0x0001) != 0):
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return bytes_to_bytearray(crc)

    def exchange_data(self, pdu, sz):
        self.lock_serial.acquire()
        self._send_serial(self.endpoint, pdu)
        pdu = self._recv_serial(sz)
        self.lock_serial.release()
        return pdu

    def _send_serial(self, endpoint, pdu):
        # 1 Byte for the device address + the length of the pdu + 2 bytes for the crc
        packet = bytearray(1 + len(pdu) + 2)
        packet[0] = endpoint
        packet[1:-2] = pdu
        crc = self._crc_calc(packet[:-2])
        packet[-2] = crc[1]
        packet[-1] = crc[0]
        if (self._cfg.rs485en != None):
            gpio.high(self._cfg.rs485en)
        self.port.write(packet)
        if (self._cfg.rs485en != None):
            gpio.low(self._cfg.rs485en)
        
    def _recv_serial(self, sz):
        if self.tim == None:
            self.tim = timers.timer()
            self.tim.start()
        else:
            self.tim.reset()	
        sz += 3
        pdu = []
        while self.port.available() < sz:
            if self.tim.get() > self.recv_timeout:
                break
            sleep(1)
        n = self.port.available()
        if n > 0:
            packet = self.port.read(n)
            pdu = packet[1:-2]
        return pdu

    def close(self):
        """ 
        .. method:: close()

            Close the serial port.
        """ 
        self.port.close()
#-endif

def bytes_to_bytearray(num):
    ret = bytearray(2)
    ret[0] = (num & 0xff00) >> 8
    ret[1] = num & 0x00ff
    return ret

def to_bit_list(num):
    if num == 0:
        return [0, 0, 0, 0, 0, 0, 0, 0]
    s = ''
    while num:
        if num & 1 == 1:
            s = "1" + s
        else:
            s = "0" + s
        num //= 2
    ret = []
    for b in s:
        ret.append(b)
    ret = ret[::-1]
    while len(ret) < 8:
        ret.append(0)
    return ret