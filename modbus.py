"""
.. module:: modbus

***************
MODBUS Library
***************
    To talk with a slave device an object of type ModbusTCP or ModbusSerial must be\
        initialized, depending on what kind of communication is needed.

    If serial communication is used, a device object that implements write, read and close methods is needed:

    Example::

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
        
        
    When a connection with a slave device has been established, coils and registers can be accessed with the methods in each class.

    """

import socket
import streams

# Functions
FN_READ_COILS = 0x01
FN_READ_DISCRETE = 0x02
FN_READ_HOLDING = 0x03
FN_READ_INPUT = 0x04

FN_WRITE_COIL = 0x05
FN_WRITE_REGISTER = 0x06
FN_WRITE_MULTIPLE_COILS = 0x0F
FN_WRITE_MULTIPLE_REGISTERS = 0x10

# Serial Line only
FN_READ_EXCEPTION_STATUS = 0x07
FN_DIAGNOSTICS = 0x08
FN_GET_COMM_EVENT_COUNTER = 0x0B
FN_GET_COMM_EVENT_LOG = 0x0C

# Exceptions
EX_ILLEGAL_FUNCTION = 0x01 + 0x80
EX_ILLEGAL_DATA_ADDRESS = 0X02 + 0x80
EX_ILLEGAL_DATA_VALUE = 0x03 + 0x80
EX_SERVER_DEVICE_FAILURE = 0x04 + 0x80
EX_ACK = 0x05+ 0x80
EX_SERVER_DEVICE_BUSY = 0x06+ 0x80
EX_MEMORY_PARITY_ERROR = 0x00+ 0x80
EX_GATEWAY_PATH_UNAVAIABLE = 0x0A+ 0x80
EX_GATEWAY_TARGET_DEVICE_FAILED = 0x0B+ 0x80


new_exception(ModbusBadAddress,Exception)
new_exception(ModbusBadLength,Exception)
new_exception(ModbusBadData, Exception)

new_exception(ModbusException, Exception)
new_exception(ModbusIllegalFunction, ModbusException, "The specified function code is not implemented.")
new_exception(ModbusIllegalDataAddress, ModbusException, "The address received by the server is not allowed.")
new_exception(ModbusIllegalDataValue, ModbusException, "The value received by the server is not allowed.")
new_exception(ModbusServerDeviceFailure, ModbusException,"Unexpected error while performing the requested action.")
new_exception(ModbusAck, ModbusException, "The server received the request and will be unavaiable for the time needed for the processing.")
new_exception(ModbusServerDeviceBusy, ModbusException, "The server is engaged in processing a long-duration program command.")
new_exception(ModbusMemoryParityError, ModbusException, "The server attempted to read record file, but detected a parity error in the memory.")
new_exception(ModbusGatewayPathUnavaiable, ModbusException, "The gateway was unable to allocate an internal communication path from the input port to the output port for processing the request.")
new_exception(ModbusGatewayTargetDeviceFailed, ModbusException, "No response was obtained from the target device.")

class Modbus():
    def __init__(self, endpoint, send_fn, recv_fn):
        self.callbacks = {}
        self.endpoint = endpoint
        self.send_fn = send_fn
        self.recv_fn = recv_fn
    
    def _register_callback(self, code, fun):
        self.callbacks[code] = fun

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
    
    def _read_coils(self,address,n):
        if address<0 or address>0xffff:
            raise ModbusBadAddress
        if n<0x0001 or n>0x07D0:
            raise ModbusBadLength
        pdu = self._req_pdu(FN_READ_COILS,2,address,2,n)
        self.send_fn(self.endpoint, pdu)
        rsp = self.recv_fn()
        return self.callbacks[rsp[0]](rsp, n)

    def _read_discrete(self, address, n):
        if address<0 or address>0xffff:
            raise ModbusBadAddress
        if n < 0x0001 or n > 0x07D0:
            raise ModbusBadLength
        pdu = self._req_pdu(FN_READ_DISCRETE, 2, address, 2, n)
        self.send_fn(self.endpoint, pdu)
        rsp = self.recv_fn()
        return self.callbacks[rsp[0]](rsp, n)


    def _read_holding(self, address, n):
        if address<0 or address>0xffff:
            raise ModbusBadAddress
        if n < 0 or n > 0x007D:
            raise ModbusBadLength
        pdu = self._req_pdu(FN_READ_HOLDING, 2, address, 2, n)
        self.send_fn(self.endpoint, pdu)
        rsp = self.recv_fn()
        return self.callbacks[rsp[0]](rsp)

    def _read_input(self, address, n):
        if address < 0 or address > 0xffff:
            raise ModbusBadAddress
        if n < 0 or n > 0x007D:
            raise ModbusBadLength
        pdu = self._req_pdu(FN_READ_INPUT, 2, address, 2, n)
        self.send_fn(self.endpoint, pdu)
        rsp = self.recv_fn()
        return self.callbacks[rsp[0]](rsp)


    def _write_coil(self, address, value):
        if address < 0 or address > 0xffff:
            raise ModbusBadAddress
        if value != 0x0000 and value != 0xff00:
            raise ModbusBadData
        pdu = self._req_pdu(FN_WRITE_COIL, 2, address, 2, value)
        self.send_fn(self.endpoint, pdu)
        rsp = self.recv_fn()
        return self.callbacks[rsp[0]](rsp)


    def _write_register(self, address, value):
        if address < 0 or address > 0xffff:
            raise ModbusBadAddress
        if value < 0 or value > 0xffff:
            raise ModbusBadData
        pdu = self._req_pdu(FN_WRITE_REGISTER, 2, address, 2, value)
        self.send_fn(self.endpoint, pdu)
        rsp = self.recv_fn()
        return self.callbacks[rsp[0]](rsp)


    def _write_multiple_coils(self, address, n, count, values):
        if address < 0 or address > 0xffff:
            raise ModbusBadAddress
        if n < 0x0001 or n > 0x07b0:
            raise ModbusBadLength
        pdu = self._req_pdu(FN_WRITE_MULTIPLE_COILS, 2, address, 2, n, 1, count, *values)
        self.send_fn(self.endpoint, pdu)
        rsp = self.recv_fn()
        return self.callbacks[rsp[0]](rsp)


    def _write_multiple_address(self, address, n, count, values):
        if address < 0 or address > 0xffff:
            raise ModbusBadAddress
        if n < 0x0001 or n > 0x07b0 or count != 2 * n:
            raise ModbusBadLength
        pdu = self._req_pdu(FN_WRITE_MULTIPLE_REGISTERS, 2, address, 2, n, 1, count, *values)
        self.send_fn(self.endpoint, pdu)
        rsp = self.recv_fn()
        return self.callbacks[rsp[0]](rsp)


class ModbusTCP():
    """

===============
ModbusTCP class
===============

.. class:: ModbusTCP(identifier)

    Create an instance of the ModbusTCP class which allow modbus communication with slave device using TCP.
    
    :param identifier: The slave device identifier, used in the header of every packet.

    """
    def __init__(self, identifier):
        self.modbus = Modbus(identifier, self._send_tcp, self._recv_tcp)
        self._socket = socket.socket(socket.AF_INET, type=socket.SOCK_STREAM)
        self.transmissionID = 1
        self.identifier = identifier

        self.modbus._register_callback(FN_READ_COILS, self._rsp_read_coils)
        self.modbus._register_callback(FN_READ_DISCRETE, self._rsp_read_discrete)
        self.modbus._register_callback(FN_READ_HOLDING, self._rsp_read_holding)
        self.modbus._register_callback(FN_READ_INPUT, self._rsp_read_input)
        self.modbus._register_callback(FN_WRITE_COIL, self._rsp_write_coil)
        self.modbus._register_callback(FN_WRITE_REGISTER, self._rsp_write_register)
        self.modbus._register_callback(FN_WRITE_MULTIPLE_COILS, self._rsp_write_multiple_coils)
        self.modbus._register_callback(FN_WRITE_MULTIPLE_REGISTERS, self._rsp_write_multiple_registers)

        self.modbus._register_callback(EX_ILLEGAL_FUNCTION, self._ex_illegal_function)
        self.modbus._register_callback(EX_ILLEGAL_DATA_ADDRESS, self._ex_illegal_data_address)
        self.modbus._register_callback(EX_ILLEGAL_DATA_VALUE, self._ex_illegal_data_value)
        self.modbus._register_callback(EX_SERVER_DEVICE_FAILURE, self._ex_server_device_failure)
        self.modbus._register_callback(EX_SERVER_DEVICE_BUSY, self._ex_server_device_busy)
        self.modbus._register_callback(EX_ACK, self._ex_ack)
        self.modbus._register_callback(EX_MEMORY_PARITY_ERROR, self._ex_parity_error)
        self.modbus._register_callback(EX_GATEWAY_PATH_UNAVAIABLE, self._ex_gateway_path_unavaiable)
        self.modbus._register_callback(EX_GATEWAY_TARGET_DEVICE_FAILED, self._ex_gateway_target_device_failed)
       
    def read_coils(self, address, n):
        """
        .. method:: read_coils(address, n)
            
            :param address: The starting address
            :param n: the number of coils to read from address


            Read the status of **n** coils, starting from **address**.

            Returns:
                a python list containing the values of the coils.
        """
        return self.modbus._read_coils(address, n)
    
    def _rsp_read_coils(self, pdu, output_count):
        byte_count = pdu[1]
        status = []
        for i in range(0, byte_count):
            byte = pdu[i + 2]
            bits = to_bit_list(byte)
            for b in bits:
                status.append(b)
        return status[0 : output_count]

    def read_input(self, address, n):
        """
        .. method:: read_input(address, n)
            
            :param address: The starting address
            :param n: the number of input register to read, starting from **address**

            Returns:
                a python list containing the values of the input registers
        """
        return self.modbus._read_input(address, n)

    def _rsp_read_input(self, pdu, input_count):
        byte_count = pdu[1]
        ret = []
        for i in range(0, byte_count, 2):
            hb = pdu[i + 2]
            lb = pdu[i + 3]
            ret.append((hb << 8) | lb)
        return ret

    def read_holding(self, address, n):
        """
        .. method:: read_holding(address, n)
            
            :param address: The starting address
            :param n: the number of holding register to read, starting from **address**

            Returns:
                a python list containing the values of the holding registers
        """
        return self.modbus._read_holding(address, n)

    def _rsp_read_holding(self, pdu):
        byte_count = pdu[1]
        ret = []
        for i in range(0, byte_count, 2):
            hb = pdu[i + 2]
            lb = pdu[i + 3]
            ret.append((hb << 8) | lb)
        return ret

    def read_discrete(self, address, n):
        """
        .. method:: read_discrete(address, n)
            
            :param address: The starting address
            :param n: the number of discrete register to read, starting from **address**

            Returns:
                a python list containing the values of the discrete registers
        """
        return self.modbus._read_discrete(address, n)

    def _rsp_read_discrete(self, pdu, input_count):
        byte_count = pdu[1]
        ret = []
        for i in range(0, byte_count):
            byte = pdu[i + 2]
            bits = to_bit_list(byte)
            for b in bits:
                ret.append(b)
        return ret[0 : input_count]

    def write_coil(self, address, value):
        """
        .. method:: write_coil(address, n)
            
            :param address: the address of the coil
            :param value: the new value

            Returns:
                1 if the write has been successfull. Otherwhise an exception will be thrown
        """
        return self.modbus._write_coil(address, value)

    def _rsp_write_coil(self, pdu):
        return 1

    def write_register(self, address, value):
        """
        .. method:: write_register(address, n)
            
            :param address: the address of the register
            :param value: the new value

            Returns:
                1 if the write has been successfull. Otherwhise an exception will be thrown
        """
        return self.modbus._write_register(address, value)

    def _rsp_write_register(self, pdu):
        return 1

    def write_multiple_coils(self, address, n, values):
        """
        .. method:: write_multiple_coils(address, n, values)
            
            :param address: the address of the first coil
            :param n: the number of coils 
            :param value: a python list containing the new values

            Returns:
                the number of coils written
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
            # APpend value and length of the value
            lst.append((out >> i * 8) & 0xff)
            lst.append(1)
        return self.modbus._write_multiple_coils(address, n, count, lst[::-1])

    def _rsp_write_multiple_coils(self, pdu):
        return (pdu[3] << 8) | pdu[4]

    def write_multiple_registers(self, address, n, values):
        """
        .. method:: write_multiple_registers(address, n, values)
            
            :param address: the address of the first holding register
            :param n: the number of registers
            :param value: a python list containing the new values

            Returns:
                the number of holding registers written
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

        return self.modbus._write_multiple_address(address, n, 2 * count, vals)
        
    def _rsp_write_multiple_registers(self, pdu):
        return (pdu[3] << 8) | pdu[4]

    def _ex_illegal_function(self):
        raise ModbusIllegalFunction

    def _ex_illegal_data_address(self):
        raise ModbusIllegalDataAddress

    def _ex_illegal_data_value(self):
        raise ModbusIllegalDataValue

    def _ex_server_device_failure(self):
        raise ModbusServerDeviceFailure 

    def _ex_ack(self):
        raise ModbusAck

    def _ex_server_device_busy(self):
        raise ModbusServerDeviceBusy

    def _ex_parity_error(self):
        raise ModbusMemoryParityError

    def _ex_gateway_path_unavaiable(self):
        raise ModbusGatewayPathUnavaiable 

    def _ex_gateway_target_device_failed(self):
        raise ModbusGatewayTargetDeviceFailed

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
        .. method:: connect(address, [port = 502])
            
            :param address: the ip address of the slave device
            :param port: port on which the slave device is listening to
        """
        self._socket.connect((address, port))
        
    def close(self):
        """
        .. method:: close()

            close the connection with the slave device 

        """
        self._socket.close()


class ModbusSerial():
    """

==================
ModbusSerial class
==================

.. class:: ModbusSerial(identifier, serial_device)

    Create an instance of the ModbusSerial class which allow modbus communication with slave device using RTU.
    
    :param identifier: The slave device identifier

    :param serial_device: an object representing the device. It must implement read, write and close methods to \
        communicate with the serial port. See **rs485** in the example folder.

    :param receive_sleep: timeout on the receiving function; 
    """
    def __init__(self, identifier, serial_device, receive_sleep = 250):
        self.modbus = Modbus(identifier, self._send_serial, self._recv_serial)
        self.identifier = identifier
        self.serial_device = serial_device
        self.receive_sleep = receive_sleep

        self.modbus._register_callback(FN_READ_COILS, self._rsp_read_coils)
        self.modbus._register_callback(FN_READ_DISCRETE, self._rsp_read_discrete)
        self.modbus._register_callback(FN_READ_HOLDING, self._rsp_read_holding)
        self.modbus._register_callback(FN_READ_INPUT, self._rsp_read_input)
        self.modbus._register_callback(FN_WRITE_COIL, self._rsp_write_coil)
        self.modbus._register_callback(FN_WRITE_REGISTER, self._rsp_write_register)
        self.modbus._register_callback(FN_WRITE_MULTIPLE_COILS, self._rsp_write_multiple_coils)
        self.modbus._register_callback(FN_WRITE_MULTIPLE_REGISTERS, self._rsp_write_multiple_registers)

        self.modbus._register_callback(EX_ILLEGAL_FUNCTION, self._ex_illegal_function)
        self.modbus._register_callback(EX_ILLEGAL_DATA_ADDRESS, self._ex_illegal_data_address)
        self.modbus._register_callback(EX_ILLEGAL_DATA_VALUE, self._ex_illegal_data_value)
        self.modbus._register_callback(EX_SERVER_DEVICE_FAILURE, self._ex_server_device_failure)
        self.modbus._register_callback(EX_SERVER_DEVICE_BUSY, self._ex_server_device_busy)
        self.modbus._register_callback(EX_ACK, self._ex_ack)
        self.modbus._register_callback(EX_MEMORY_PARITY_ERROR, self._ex_parity_error)
        self.modbus._register_callback(EX_GATEWAY_PATH_UNAVAIABLE, self._ex_gateway_path_unavaiable)
        self.modbus._register_callback(EX_GATEWAY_TARGET_DEVICE_FAILED, self._ex_gateway_target_device_failed)
       

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

    def _send_serial(self, endpoint, pdu):
        # 1 Byte for the device address + the length of the pdu + 2 bytes for the crc
        packet = bytearray(1 + len(pdu) + 2)
        packet[0] = endpoint
        packet[1:-2] = pdu
        crc = self._crc_calc(packet[:-2])
        packet[-2] = crc[1]
        packet[-1] = crc[0]
        
        self.serial_device.write(packet)
        
    def _recv_serial(self):
        sleep(self.receive_sleep)
        packet = self.serial_device.read()
        return packet[1:]

    def read_coils(self, address, n):
        """
        .. method:: read_coils(address, n)
            
            :param address: The starting address
            :param n: the number of coils to read from address


            Read the status of **n** coils, starting from **address**.

            Returns:
                a python list containing the values of the coils.
        """
        return self.modbus._read_coils(address, n)

    def _rsp_read_coils(self, pdu, output_count):
        byte_count = pdu[1]
        status = []
        for i in range(0, byte_count):
            byte = pdu[i+2]
            bits = to_bit_list(byte)
            for b in bits:
                status.append(b)
        return status[0 : output_count]
    
    def read_input(self, address, n):
        """
        .. method:: read_input(address, n)
            
            :param address: The starting address
            :param n: the number of input register to read, starting from **address**

            Returns:
                a python list containing the values of the input registers
        """
        return self.modbus._read_input(address, n)

    def _rsp_read_input(self, pdu, input_count):
        byte_count = pdu[1]
        ret = []
        for i in range(0, byte_count, 2):
            hb = pdu[i + 2]
            lb = pdu[i + 3]
            ret.append((hb << 8) | lb)
        return ret
    
    def read_holding(self, address, n):
        """
        .. method:: read_holding(address, n)
            
            :param address: The starting address
            :param n: the number of holding register to read, starting from **address**

            Returns:
                a python list containing the values of the holding registers
        """
        return self.modbus._read_holding(address, n)

    def _rsp_read_holding(self, pdu):
        byte_count = pdu[1]
        ret = []
        for i in range(0, byte_count, 2):
            hb = pdu[i + 2]
            lb = pdu[i + 3]
            ret.append((hb << 8) | lb)
        return ret

    def read_discrete(self, address, n):
        """
        .. method:: read_discrete(address, n)
            
            :param address: The starting address
            :param n: the number of discrete register to read, starting from **address**

            Returns:
                a python list containing the values of the discrete registers
        """
        return self.modbus._read_discrete(address, n)

    def _rsp_read_discrete(self, pdu, input_count):
        byte_count = pdu[1]
        ret = []
        for i in range(0, byte_count):
            byte = pdu[i + 2]
            bits = to_bit_list(byte)
            for b in bits:
                ret.append(b)
        return ret[0 : input_count]
    
    def write_coil(self, address, value):
        """
        .. method:: write_coil(address, n)
            
            :param address: the address of the coil
            :param value: the new value

            Returns:
                1 if the write has been successfull. Otherwhise an exception will be thrown
        """
        return self.modbus._write_coil(address, value)

    def _rsp_write_coil(self, pdu):
        return 1

    def write_register(self, address, value):
        """
        .. method:: write_register(address, n)
            
            :param address: the address of the register
            :param value: the new value

            Returns:
                1 if the write has been successfull. Otherwhise an exception will be thrown
        """
        return self.modbus._write_register(address, value)

    def _rsp_write_register(self, pdu):
        return 1

    def write_multiple_coils(self, address, n, values):
        """
        .. method:: write_multiple_coils(address, n, values)
            
            :param address: the address of the first coil
            :param n: the number of coils 
            :param value: a python list containing the new values

            Returns:
                the number of coils written
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
        return self.modbus._write_multiple_coils(address, n, count, lst[::-1])

    def _rsp_write_multiple_coils(self, pdu):
        return (pdu[3] << 8) | pdu[4]

    def write_multiple_registers(self, address, n, values):
        """
        .. method:: write_multiple_registers(address, n, values)
            
            :param address: the address of the first holding register
            :param n: the number of registers
            :param value: a python list containing the new values

            Returns:
                the number of holding registers written
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

        return self.modbus._write_multiple_address(address, n, 2 * count, vals)
        
    def _rsp_write_multiple_registers(self, pdu):
        return (pdu[3] << 8) | pdu[4]

    def _ex_illegal_function(self):
        raise ModbusIllegalFunction

    def _ex_illegal_data_address(self):
        raise ModbusIllegalDataAddress

    def _ex_illegal_data_value(self):
        raise ModbusIllegalDataValue

    def _ex_server_device_failure(self):
        raise ModbusServerDeviceFailure 

    def _ex_ack(self):
        raise ModbusAck

    def _ex_server_device_busy(self):
        raise ModbusServerDeviceBusy

    def _ex_parity_error(self):
        raise ModbusMemoryParityError

    def _ex_gateway_path_unavaiable(self):
        raise ModbusGatewayPathUnavaiable 

    def _ex_gateway_target_device_failed(self):
        raise ModbusGatewayTargetDeviceFailed

    def close(self):
        """ 
        .. method:: close()

            Close the serial port by calling the close function implemented by the device class.
        """ 
        self.serial_device.close()

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
