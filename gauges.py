
import time, math

# Import the ADS1x15 module.
import Adafruit_ADS1x15
import RPi.GPIO as GPIO

# Create an ADS1115 ADC (16-bit) instance.
adc = Adafruit_ADS1x15.ADS1115()

# Or create an ADS1015 ADC (12-bit) instance.
#adc = Adafruit_ADS1x15.ADS1015()
class max31856(object):
	"""Read Temperature on the Raspberry PI from the MAX31856 chip using GPIO
	   Any pins can be used for CS (chip select), MISO, MOSI and CLK
	"""

	def __init__(self, csPin = 8, misoPin = 9, mosiPin = 10, clkPin = 11):
		self.csPin = csPin
		self.misoPin = misoPin
		self.mosiPin = mosiPin
		self.clkPin = clkPin
		self.setupGPIO()
		#
		# Config Register 2
		# ------------------
		# bit 7: Reserved                                -> 0 
		# bit 6: Averaging Mode 1 Sample                 -> 0 (default)
		# bit 5: Averaging Mode 1 Sample                 -> 0 (default)
		# bit 4: Averaging Mode 1 Sample                 -> 0 (default)
		# bit 3: Thermocouple Type -> K Type (default)   -> 0 (default)
		# bit 2: Thermocouple Type -> K Type (default)   -> 0 (default)
		# bit 1: Thermocouple Type -> K Type (default)   -> 1 (default)
		# bit 0: Thermocouple Type -> K Type (default)   -> 1 (default)
		#
		#Uncomment one of the following to select thermocouple type
		#self.writeRegister(1, 0x00) #for B Type
		#self.writeRegister(1, 0x01) #for E Type
		#self.writeRegister(1, 0x02) #for J Type
		self.writeRegister(1, 0x03) #for K Type
		#self.writeRegister(1, 0x04) #for N Type
		#self.writeRegister(1, 0x05) #for R Type
		#self.writeRegister(1, 0x06) #for S Type
		#self.writeRegister(1, 0x07) #for T Type
		
	def setupGPIO(self):
		GPIO.setwarnings(False)
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.csPin, GPIO.OUT)
		GPIO.setup(self.misoPin, GPIO.IN)
		GPIO.setup(self.mosiPin, GPIO.OUT)
		GPIO.setup(self.clkPin, GPIO.OUT)

		GPIO.output(self.csPin, GPIO.HIGH)
		GPIO.output(self.clkPin, GPIO.LOW)
		GPIO.output(self.mosiPin, GPIO.LOW)	
	
	def readThermocoupleTemp(self):
		self.requestTempConv()

		# read 4 registers starting with register 12
		out = self.readRegisters(0x0c, 4) 
		
		[tc_highByte, tc_middleByte, tc_lowByte] = [out[0], out[1], out[2]]	
		temp = ((tc_highByte << 16) | (tc_middleByte << 8) | tc_lowByte) >> 5
		
		if (tc_highByte & 0x80):
			temp -= 0x80000
		
		temp_C = temp * 0.0078125
		
		fault = out[3]
		
		if ((fault & 0x80) == 1):
			raise FaultError("Cold Junction Out-of-Range")
		if ((fault & 0x40) == 1):
			raise FaultError("Thermocouple Out-of-Range")
		if ((fault & 0x20) == 1):
			raise FaultError("Cold-Junction High Fault")
		if ((fault & 0x10) == 1):
			raise FaultError("Cold-Junction Low Fault")
		if ((fault & 0x08) == 1):
			raise FaultError("Thermocouple Temperature High Fault")
		if ((fault & 0x04) == 1):
			raise FaultError("Thermocouple Temperature Low Fault")
		if ((fault & 0x02) == 1):
			raise FaultError("Overvoltage or Undervoltage Input Fault")
		if ((fault & 0x01) == 1):
			raise FaultError("Thermocouple Open-Circuit Fault")
				
		return temp_C
				
	def readJunctionTemp(self):
		self.requestTempConv()
		
		# read 3 registers starting with register 9
		out = self.readRegisters(0x09, 3)
		
		offset = out[0]
		
		[junc_msb, junc_lsb] = [out[1], out[2]]
		
		temp = ((junc_msb << 8) | junc_lsb) >> 2
		temp = offset + temp
		
		if (junc_msb & 0x80):
			temp -= 0x4000
		
		temp_C = temp * 0.015625
		
		return temp_C
	
	def requestTempConv(self):
		#
		# Config Register 1
		# ------------------
		# bit 7: Conversion Mode                         -> 0 (Normally Off Mode)
		# bit 6: 1-shot                                  -> 1 (ON)
		# bit 5: open-circuit fault detection            -> 0 (off)
		# bit 4: open-circuit fault detection            -> 0 (off)
		# bit 3: Cold-junction temerature sensor enabled -> 0 (default)
		# bit 2: Fault Mode                              -> 0 (default)
		# bit 1: fault status clear                      -> 1 (clear any fault)
		# bit 0: 50/60 Hz filter select                  -> 0 (60Hz)
		#
		# write config register 0
		self.writeRegister(0, 0x42)
		# conversion time is less than 150ms
		time.sleep(.2) #give it 200ms for conversion

	def writeRegister(self, regNum, dataByte):
		GPIO.output(self.csPin, GPIO.LOW)
		
		# 0x8x to specify 'write register value'
		addressByte = 0x80 | regNum;
		
		# first byte is address byte
		self.sendByte(addressByte)
		# the rest are data bytes
		self.sendByte(dataByte)

		GPIO.output(self.csPin, GPIO.HIGH)

		
	def readRegisters(self, regNumStart, numRegisters):
		out = []
		GPIO.output(self.csPin, GPIO.LOW)
		
		# 0x to specify 'read register value'
		self.sendByte(regNumStart)
		
		for byte in range(numRegisters):	
			data = self.recvByte()
			out.append(data)

		GPIO.output(self.csPin, GPIO.HIGH)
		return out

	def sendByte(self,byte):
		for bit in range(8):
			GPIO.output(self.clkPin, GPIO.HIGH)
			if (byte & 0x80):
				GPIO.output(self.mosiPin, GPIO.HIGH)
			else:
				GPIO.output(self.mosiPin, GPIO.LOW)
			byte <<= 1
			GPIO.output(self.clkPin, GPIO.LOW)

	def recvByte(self):
		byte = 0x00
		for bit in range(8):
			GPIO.output(self.clkPin, GPIO.HIGH)
			byte <<= 1
			if GPIO.input(self.misoPin):
				byte |= 0x1
			GPIO.output(self.clkPin, GPIO.LOW)
		return byte	

class FaultError(Exception):
	pass

	
# Note you can change the I2C address from its default (0x48), and/or the I2C
# bus by passing in these optional parameters:
#adc = Adafruit_ADS1x15.ADS1015(address=0x49, busnum=1)

# Choose a gain of 1 for reading voltages from 0 to 4.09V.
# Or pick a different gain to change the range of voltages that are read:
#  - 2/3 = +/-6.144V
#  -   1 = +/-4.096V
#  -   2 = +/-2.048V
#  -   4 = +/-1.024V
#  -   8 = +/-0.512V
#  -  16 = +/-0.256V
# See table 3 in the ADS1015/ADS1115 datasheet for more info on gain.
GAIN = 1

print('Reading ADS1x15 values, press Ctrl-C to quit...')
# Print nice channel column headers.
print('| {0:>6} | {1:>6} | {2:>6} | {3:>6} |'.format(*range(4)))
print('-' * 37)
# Main loop.
while True:
    # Read all the ADC channel values in a list.
    values = [0]*4
    for i in range(4):
        # Read the specified ADC channel using the previously set gain value.
        values[i] = adc.read_adc(i, gain=GAIN)
        # Note you can also pass in an optional data_rate parameter that controls
        # the ADC conversion time (in samples/second). Each chip has a different
        # set of allowed data rate values, see datasheet Table 9 config register
        # DR bit values.
        #values[i] = adc.read_adc(i, gain=GAIN, data_rate=128)
        # Each value will be a 12 or 16 bit signed integer value depending on the
        # ADC (ADS1015 = 12-bit, ADS1115 = 16-bit).
    # Print the ADC values.
    print('| {0:>6} | {1:>6} | {2:>6} | {3:>6} |'.format(*values))
    # Pause for half a second.
    import max31856
	csPin = 8
	misoPin = 9
	mosiPin = 10
	clkPin = 11
	max = max31856.max31856(csPin,misoPin,mosiPin,clkPin)
	thermoTempC = max.readThermocoupleTemp()
	thermoTempF = (thermoTempC * 9.0/5.0) + 32
	print "Thermocouple Temp: %f degF" % thermoTempF
	juncTempC = max.readJunctionTemp()
	juncTempF = (juncTempC * 9.0/5.0) + 32
	print "Cold Junction Temp: %f degF" % juncTempF
	GPIO.cleanup()
	time.sleep(0.5)