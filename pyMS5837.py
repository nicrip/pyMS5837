#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import smbus
import time
import math
import signal
import sys
import curses

# Sensor-specific definitions
# MS5837
MS5837_ADDR             =0x76
MS5837_RESET            =0x1E
MS5837_ADC_READ         =0x00
MS5837_PROM_READ        =0xA0
MS5837_CONVERT_D1_256   =0x40
MS5837_CONVERT_D2_256   =0x50

# BlueRobotics depth sensor models
MODEL_02BA              =0
MODEL_30BA              =1

# Oversampling options
OSR_256                 =0
OSR_512                 =1
OSR_1024                =2
OSR_2048                =3
OSR_4096                =4
OSR_8192                =5

# Water density constants kg/m^3
DENSITY_FRESHWATER      =997
DENSITY_SALTWATER       =1029

# Pressure conversion factors from mbar
UNITS_Pa                =100.0
UNITS_hPa               =1.0
UNITS_kPa               =0.1
UNITS_mbar              =1.0
UNITS_bar               =0.001
UNITS_atm               =0.000986923
UNITS_Torr              =0.750062
UNITS_psi               =0.014503773773022

# Temperature valid units
UNITS_Celsius           =1
UNITS_Farenheit         =2
UNITS_Kelvin            =3

# initialize i2c bus
def initBus(bus_num):
    return smbus.SMBus(bus_num)

# write a byte to an open i2c bus
def writeByteToBus(bus, address, offset, data_byte):
    bus.write_byte_data(address, offset, data_byte)

# read 'count' bytes from an open i2c bus
def readBytesFromBus(bus, address, offset, count):
    bus.write_byte(address, offset)
    return [bus.read_byte(address) for k in range(count)]

def signal_handler(sig, frame):
        curses.nocbreak()
        curses.echo()
        curses.endwin()
        print('Ctrl+C force quit.')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

class MS5837(object):
    def __init__(self, bus_num, model=MODEL_30BA, fluid_density=DENSITY_FRESHWATER, oversampling=OSR_8192):
        self.bus_num = bus_num
        self.model = model
        self.fluid_density = fluid_density
        self.oversampling = oversampling
        self.pressure = 0
        self.temperature = 0
        self.D1 = 0
        self.D2 = 0

        if self.oversampling < OSR_256 or self.oversampling > OSR_8192:
            print('Invalid oversampling (averaging) option requested!')
            exit(1)

        # initialize i2c bus
        try:
            self.MS5837 = initBus(bus_num)
        except:
            print('No i2c device detected on bus{:2d}!'.format(bus_num))
            exit(1)
        print('i2c bus initialized on bus{:2d}.'.format(bus_num))

        # reset the device
        self.MS5837.write_byte(MS5837_ADDR, MS5837_RESET)
        time.sleep(0.1)

        # read calibration values and cyclic redundancy check (CRC)
        self.C = []
        for i in range(7):
            c = self.MS5837.read_word_data(MS5837_ADDR, MS5837_PROM_READ + 2*i)
            c =  ((c & 0xFF) << 8) | (c >> 8) # SMBus is little-endian for word transfers, we need to swap MSB and LSB
            self.C.append(c)
        crc = (self.C[0] & 0xF000) >> 12
        if crc != self.crc4(self.C):
            print('EEPROM read error, CRC failed!')
            exit(1)

    def run(self):
        prev_loop_start = time.time()
        console = curses.initscr()
        curses.noecho()
        curses.cbreak()
        console.keypad(True)
        console.nodelay(True)
        input = None
        while True:
            console.clear()
            loop_start = time.time()
            loop_time = loop_start - prev_loop_start
            prev_loop_start = loop_start

            self.read()

            console.addstr(0,0,'Reading MS5837 Pressure Sensor')
            console.addstr(1,0,'')
            console.addstr(2,0,'Temperature:')
            console.addstr(3,0,'    Celsius: {:6.2f} C'.format(self.get_temperature(UNITS_Celsius)))
            console.addstr(4,0,'    Farenheit: {:6.2f} F'.format(self.get_temperature(UNITS_Farenheit)))
            console.addstr(5,0,'    Kelvin: {:6.2f} K'.format(self.get_temperature(UNITS_Kelvin)))
            console.addstr(6,0,'')
            console.addstr(7,0,'Temperature:')
            console.addstr(8,0,'    Millibar: {:6.2f} mbar'.format(self.get_pressure(UNITS_mbar)))
            console.addstr(9,0,'    Atmospheres: {:6.2f} atm'.format(self.get_pressure(UNITS_atm)))
            console.addstr(10,0,'    Pounds-per-square-inch: {:6.2f} psi'.format(self.get_pressure(UNITS_psi)))
            console.addstr(11,0,'')

            self.fluid_density = DENSITY_FRESHWATER
            console.addstr(12,0,'Depth:')
            console.addstr(13,0,'    Freshwater: {:6.2f} m'.format(self.get_depth()))
            self.fluid_density = DENSITY_SALTWATER
            console.addstr(14,0,'    Saltwater: {:6.2f} m'.format(self.get_depth()))
            console.addstr(15,0,'')
            console.addstr(16,0,'Altitude:')
            console.addstr(17,0,'    Air: {:6.2f} m'.format(self.get_altitude()))
            console.addstr(18,0,'')
            console.addstr(19,0,'Loop Cycle Time: {:4.2f} ms'.format(loop_time*1e3))
            console.addstr(20,0,'')
            console.addstr(21,0,'Press `q` to quit.')
            console.refresh()

            try:
                input = console.getkey()
            except:
                pass

            if input == 'q':
                curses.nocbreak()
                console.nodelay(False)
                console.keypad(False)
                curses.echo()
                curses.endwin()
                print('Quitting MS5837 program.')
                sys.exit(0)

    def read(self):
        # Request D1 conversion (temperature)
        self.MS5837.write_byte(MS5837_ADDR, MS5837_CONVERT_D1_256 + 2*self.oversampling)
        # Maximum conversion time increases linearly with oversampling
        # max time (seconds) ~= 2.2e-6(x) where x = OSR = (2^8, 2^9, ..., 2^13)
        # We use 2.5e-6 for some overhead
        time.sleep(2.5e-6 * 2**(8+self.oversampling))

        d = self.MS5837.read_i2c_block_data(MS5837_ADDR, MS5837_ADC_READ, 3)
        self.D1 = d[0] << 16 | d[1] << 8 | d[2]

        # Request D2 conversion (pressure)
        self.MS5837.write_byte(MS5837_ADDR, MS5837_CONVERT_D2_256 + 2*self.oversampling)
        time.sleep(2.5e-6 * 2**(8+self.oversampling))

        d = self.MS5837.read_i2c_block_data(MS5837_ADDR, MS5837_ADC_READ, 3)
        self.D2 = d[0] << 16 | d[1] << 8 | d[2]

        # Calculate compensated pressure and temperature using raw ADC values and internal calibration
        self.calculate()

    # Pressure in requested units
    def get_pressure(self, conversion=UNITS_mbar):
        return self.pressure * conversion

    # Temperature in requested units
    def get_temperature(self, conversion=UNITS_Celsius):
        degC = self.temperature / 100.0
        if conversion == UNITS_Farenheit:
            return (9.0/5.0)*degC + 32
        elif conversion == UNITS_Kelvin:
            return degC + 273
        return degC

    # Depth relative to mean sea level (MSL) pressure in given fluid density
    def get_depth(self):
        return (self.get_pressure(UNITS_Pa)-101300)/(self.fluid_density*9.80665)

    # Altitude relative to mean sea level (MSL) pressure
    def get_altitude(self):
        return (1-pow((self.get_pressure()/1013.25),.190284))*145366.45*.3048

    # Calculation according to MS5837 datasheet
    def calculate(self):
        OFFi = 0
        SENSi = 0
        Ti = 0

        dT = self.D2-self.C[5]*256
        if self.model == MODEL_02BA:
            SENS = self.C[1]*65536+(self.C[3]*dT)/128
            OFF = self.C[2]*131072+(self.C[4]*dT)/64
            self.pressure = (self.D1*SENS/(2097152)-OFF)/(32768)
        else:
            SENS = self.C[1]*32768+(self.C[3]*dT)/256
            OFF = self.C[2]*65536+(self.C[4]*dT)/128
            self.pressure = (self.D1*SENS/(2097152)-OFF)/(8192)

        self.temperature = 2000+dT*self.C[6]/8388608

        # Second order compensation
        if self.model == MODEL_02BA:
            if (self.temperature/100) < 20: # Low temp
                Ti = (11*dT*dT)/(34359738368)
                OFFi = (31*(self.temperature-2000)*(self.temperature-2000))/8
                SENSi = (63*(self.temperature-2000)*(self.temperature-2000))/32

        else:
            if (self.temperature/100) < 20: # Low temp
                Ti = (3*dT*dT)/(8589934592)
                OFFi = (3*(self.temperature-2000)*(self.temperature-2000))/2
                SENSi = (5*(self.temperature-2000)*(self.temperature-2000))/8
                if (self.temperature/100) < -15: # Very low temp
                    OFFi = OFFi+7*(self.temperature+1500)*(self.temperature+1500)
                    SENSi = SENSi+4*(self.temperature+1500)*(self.temperature+1500)
            elif (self.temperature/100) >= 20: # High temp
                Ti = 2*(dT*dT)/(137438953472)
                OFFi = (1*(self.temperature-2000)*(self.temperature-2000))/16
                SENSi = 0

        OFF2 = OFF-OFFi
        SENS2 = SENS-SENSi

        if self.model == MODEL_02BA:
            self.temperature = (self.temperature-Ti)
            self.pressure = (((self.D1*SENS2)/2097152-OFF2)/32768)/100.0
        else:
            self.temperature = (self.temperature-Ti)
            self.pressure = (((self.D1*SENS2)/2097152-OFF2)/8192)/10.0

    def writeRegister(self, offset, data):
        writeByteToBus(self.MS5837, MS5837_ADDR, offset, data)

    # Cyclic redundancy check (CRC) calculation according to MS5837 datasheet
    def crc4(self, n_prom):
        n_rem = 0
        n_prom[0] = ((n_prom[0]) & 0x0FFF)
        n_prom.append(0)
        for i in range(16):
            if i%2 == 1:
                n_rem ^= ((n_prom[i>>1]) & 0x00FF)
            else:
                n_rem ^= (n_prom[i>>1] >> 8)
            for n_bit in range(8,0,-1):
                if n_rem & 0x8000:
                    n_rem = (n_rem << 1) ^ 0x3000
                else:
                    n_rem = (n_rem << 1)
        n_rem = ((n_rem >> 12) & 0x000F)
        self.n_prom = n_prom
        self.n_rem = n_rem
        return n_rem ^ 0x00

ms5837 = MS5837(2, model=MODEL_30BA, fluid_density=DENSITY_FRESHWATER, oversampling=OSR_8192)
ms5837.run()
