from time import *
from machine import *
from mcp3004 import MCP3004


""" SPI Parameters """
SPI_CLK = Pin(2)
SPI_MOSI = Pin(3)
SPI_MISO = Pin(4)
SPI_CS = Pin(5, Pin.OUT)

spi = SPI(0, sck=SPI_CLK, mosi=SPI_MOSI, miso=SPI_MISO, baudrate=100000)
cs = SPI_CS

mcp = MCP3004(spi, cs)


""" ADC Parameters """
SPANEL_PIN = Pin(26, Pin.IN) # Sense PANEL PIN
SPOUT_PIN = Pin(27, Pin.IN)  # Not Used! Sense Power OUT PIN

sPanel = ADC(SPANEL_PIN)
sPout = ADC(SPOUT_PIN)


""" Control Parameters """
SERVO_LR_PIN = Pin(15, Pin.OUT)
SERVO_UD_PIN = Pin(14, Pin.OUT)

servoLR = PWM(SERVO_LR_PIN)
servoUD = PWM(SERVO_UD_PIN)
servoLR.freq(50) # 50Hz
servoUD.freq(50) # 50Hz


""" System Parameters """
onboardLED = Pin(25, Pin.OUT)

SYS_ADC_ACCURACY = 0xFFFF  # system ADC is 16 bit
MCP_ADC_ACCURACY = 0xFFF   # mcp ADC is 12 bit

NUM_LDR = 4 # number of Light Dependent Resistors in our system

SYS_mV_REF = 3300 # system votlage reference in mV
SENSE_RES_VALUE = 1  # value of sensing resistors

SLEEP_TIME = 2000 # changes if isDemo is not 0

SENSITIVITY = 3
SERVO_STEP = 1


# sensor-mcp channel mapping
LDR_UR = 1 # up right ldr when looking from the front
LDR_UL = 2 # up left ldr when looking from the front
LDR_DR = 0 # down right ldr when looking from the front
LDR_DL = 3 # down left ldr when looking from the front

LR_MIN = 30
LR_MAX = 150
UD_MIN = 90
UD_MAX = 150


""" =========================
=== System init functions ===
============================= """
def CalcSensorNormalization():
    acc_ldr = [0, 0, 0, 0]
    
    for i in range(10):
        sensorValues = ReadLightSensors_raw()
        for j in range(NUM_LDR):
            acc_ldr[j] += sensorValues[j]
        sleep_ms(100)
    
    for i in range(NUM_LDR):
        print(acc_ldr[i], end=' ')
    print()
    
    normValues = [1., 1., 1., 1.]
    for i in range(NUM_LDR):
        normValues[i] = round(sensorValues[0] / sensorValues[i], 2)
        print(normValues[i], end=" ")
    print()


def ReadCurrent(sourceADC):
    rawValue = sourceADC.read_u16()
    voltage = rawValue * SYS_mV_REF / SYS_ADC_ACCURACY
    current = voltage / SENSE_RES_VALUE # I = V/R (R=1 in our case)
    
    return round(current) # return current in mA


""" ================================
=== Energy production monitoring ===
==================================== """
def UpdateEnergyValues():
    global panelCurrent, panelWatt
    
    panelCurrent = (panelCurrent + ReadCurrent(sPanel)) >> 1 # division by shifting
    panelWatt = pow(panelCurrent, 2) * SENSE_RES_VALUE / 1000 # divide by 1000 because of mA ^ 2
    panelWatt = round(panelWatt)


def CalcPowerProduction():
    global productionWatt, productionTime, productionWSeconds
    
    productionWatt += panelWatt # in mW
    productionTime += SLEEP_TIME # in ms
    productionWSeconds = round(productionWatt * (productionTime/1000) / 3600, 2) # power production in mWHours


""" ================
=== Read sensors ===
==================== """
def ReadLightSensors_raw():
    data = []
    
    for i in range(NUM_LDR):
        data.append(mcp.read(i))
    
    return data


def ReadLightSensors():
    global normValues
    
    data = []
    
    for i in range(NUM_LDR):
        data.append(round(mcp.read(i) * normValues[i] * 100 / MCP_ADC_ACCURACY, 1))
    
    return data

""" ===========================
=== Correct system position ===
=============================== """
def ConstraintValues(value, low, high):
    if value > high:
        return high
    elif value < low:
        return low
    else:
        return value
    

def ServoDegrees(myServo, angle):
    if angle > 180:
        angle = 180
    elif angle < 0:
        angle = 0
    
    duty = (angle / 180) * 6881.175 + 1310.7
    myServo.duty_u16(int(duty))


def SetServos(angleUD, angleLR):
    ServoDegrees(servoUD, angleUD)
    ServoDegrees(servoLR, angleLR)


def CalcServoUD(ldrUR, ldrUL, ldrDR, ldrDL):
    if ldrUR > ldrDR + SENSITIVITY or ldrUL > ldrDL + SENSITIVITY:
        return -SERVO_STEP
    elif ldrUR < ldrDR - SENSITIVITY or ldrUL < ldrDL - SENSITIVITY:
        return SERVO_STEP
    else:
        return 0
    
    
def CalcServoLR(ldrUR, ldrUL, ldrDR, ldrDL):
    if ldrUR > ldrUL + SENSITIVITY or ldrDR > ldrDL + SENSITIVITY:
        return SERVO_STEP
    elif ldrUR < ldrUL - SENSITIVITY or ldrDR < ldrDL - SENSITIVITY:
        return -SERVO_STEP
    else:
        return 0


""" ===========
=== Display ===
=============== """
def PrintValues():
    print("\nCurrent //", end='\t')
    print("Panel: " + str(panelCurrent) + " mA", end='\n')
    
    print("Power //", end='\t')
    print("Panel: " + str(panelWatt) + " mW", end='\n')
    
    for i in range(len(sensorValues)):
        print("LDR" + str(i) + ": " + str(sensorValues[i]), end='\t')
    print()
    
    print("UD: " + str(angleUD) + "\tLR: " + str(angleLR))


""" ============================
=== Delay and low power mode ===
================================ """
def BlinkAndWait():
    onboardLED.on()
    sleep_ms(50) # wait for pulses to be sent to the servos
    
    onboardLED.off()
    lightsleep(SLEEP_TIME - 50) # enter low power state and keeping RAM


""" ==========
    Main
    ========== """
ldrUR = 0
ldrUL = 0
ldrDR = 0
ldrDL = 0

angleLR = 90 # set in the middle
angleUD = 135 # set at 45 degrees from being parallel from the ground

panelCurrent = 0
productionWatt = 0
productionTime = 0

# === Select "Is Demo" mode and values ===
isDemo = 1 # set to 0 for normal mode, set to 1 for demo mode
if not isDemo:
    normValues = [1.0, 1.05, 1.01, 1.01] # measured on direct sunlight
    sensorThreshold = 17
else:
    normValues = [1.0, 1.54, 1.1, 1.1] # measured indoors
    sensorThreshold = 7
    SLEEP_TIME = 500


# === System init ===
SetServos(angleUD, angleLR)

# while True:  # Loop normalization calculation for debugging
#     CalcSensorNormalization()  # Loop normalization calculation for debugging

CalcSensorNormalization()

while True:
# === Energy production monitoring === 
    UpdateEnergyValues()
    CalcPowerProduction()

# === Read sensors === 
    sensorValues = ReadLightSensors()

    ldrUR = (ldrUR + sensorValues[LDR_UR]) / 2
    ldrUL = (ldrUL + sensorValues[LDR_UL]) / 2
    ldrDR = (ldrDR + sensorValues[LDR_DR]) / 2
    ldrDL = (ldrDL + sensorValues[LDR_DL]) / 2

# === Correct system position === 
    if max(sensorValues) > sensorThreshold:
        angleUD += CalcServoUD(ldrUR, ldrUL, ldrDR, ldrDL)
        angleUD = ConstraintValues(angleUD, UD_MIN, UD_MAX)
        
        angleLR += CalcServoLR(ldrUR, ldrUL, ldrDR, ldrDL)
        angleLR = ConstraintValues(angleLR, LR_MIN, LR_MAX)
    
    SetServos(angleUD, angleLR)

# === Display and save === 
    PrintValues()

# === Delay and low power mode === 
    BlinkAndWait()