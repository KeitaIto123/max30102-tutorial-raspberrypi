# -*-coding:utf-8-*-
import RPi.GPIO as GPIO
import smbus

REG_INTR_STATUS_1 = 0x00
REG_INTR_STATUS_2 = 0x01

REG_INTR_ENABLE_1 = 0x02
REG_INTR_ENABLE_2 = 0x03

REG_FIFO_WR_PTR = 0x04
REG_OVF_COUNTER = 0x05
REG_FIFO_RD_PTR = 0x06
REG_FIFO_DATA = 0x07
REG_FIFO_CONFIG = 0x08

REG_MODE_CONFIG = 0x09
REG_SPO2_CONFIG = 0x0A
REG_LED1_PA = 0x0C

REG_LED2_PA = 0x0D
REG_MULTI_LED_CTRL1 = 0x11
REG_MULTI_LED_CTRL2 = 0x12

REG_TEMP_INTR = 0x1F
REG_TEMP_FRAC = 0x20
REG_TEMP_CONFIG = 0x21

class MAX30102():
    # by default, this assumes that physical pin 7 (GPIO 4) is used as interrupt
    # by default, this assumes that the device is at 0x57 on channel 1
    def __init__(self, channel=1, address=0x57, gpio_pin=7):
        self.address = address
        self.channel = channel
        self.bus = smbus.SMBus(self.channel)
        self.interrupt = gpio_pin

        # set gpio mode
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.interrupt, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.reset()

        # PWR_RDY
        # On power-up or after a brownout condition,
        # when the supply voltage VDD transitions from below the undervoltage lockout(UVLO) voltage to above the UVLO voltage,
        # a power-ready interrupt is triggered to signal that the module is powered-up and ready to collect data.
        # After the power is established, an interrupt occurs to alert the system that the MAX30102 is ready for operation.
        # Reading the I2C interrupt register clears the interrupt.
        self.bus.read_i2c_block_data(self.address, REG_INTR_STATUS_1, 1)
        self.setup()

    def shutdown(self):
        """
        Shutdown the device.
        """
        # SHDN
        # The part can be put into a power-save mode by setting this bit to one.
        # While in power-save mode, all registers retain their values, and write/read operations function as normal.
        # All interrupts are cleared to zero in this mode.
        self.bus.write_i2c_block_data(self.address, REG_MODE_CONFIG, [0x80])

    def reset(self):
        """
        Reset the device, this will clear all settings,
        so after running this, run setup() again.

        """
        # RESET
        # When the RESET bit is set to one,
        # all configuration, threshold, and data registers are reset to their power-on-state through a power-on reset.
        # The RESET bit is cleared automatically back to zero after the reset sequence is completed.
        # Note: Setting the RESET bit does not trigger a PWR_RDY interrupt event.
        self.bus.write_i2c_block_data(self.address, REG_MODE_CONFIG, [0x40])
        # Waiting for RESET register turning to 0
        while(True):
            res = self.bus.read_i2c_block_data(self.address, REG_MODE_CONFIG, 1)
            if (res[0] >> 6) & 1 == 0: break

    def setup(self):
        """
        This will setup the device with the values written in sample Arduino code.
        """
        # INTR setting
        # 0x40 : PPG_RDY_EN = Interrupt will be triggered when new fifo data ready
        # In SpO2 and HR modes, this interrupt triggers when there is a new sample in the data FIFO.
        # The interrupt is cleared by reading the Interrupt Status 1 register (0x00), or by reading the FIFO_DATA register.
        self.bus.write_i2c_block_data(self.address, REG_INTR_ENABLE_1, [0x40])
        self.bus.write_i2c_block_data(self.address, REG_INTR_ENABLE_2, [0x00])

        # When starting a new SpO2 or heart rate conversion,
        # it is recommended to first clear the FIFO_WR_PTR, OVF_COUNTER, and FIFO_RD_PTR registers
        # to all zeroes (0x00) to ensure the FIFO is empty and in a known state.
        self.bus.write_i2c_block_data(self.address, REG_FIFO_WR_PTR, [0x00])
        self.bus.write_i2c_block_data(self.address, REG_OVF_COUNTER, [0x00])
        self.bus.write_i2c_block_data(self.address, REG_FIFO_RD_PTR, [0x00])

        SMP_AVE = 0b001 # 2
        FIFO_ROLLOVER_EN = 0b00 # disable
        FIFO_A_FULL = 0b00 # 0
        REG_FIFO_CONFIG_DATA = (SMP_AVE << 5 | FIFO_ROLLOVER_EN << 4 | FIFO_A_FULL)
        self.bus.write_i2c_block_data(self.address, REG_FIFO_CONFIG, [REG_FIFO_CONFIG_DATA])

        # 0x02 for HR mode, 0x03 for SpO2 mode, 0x07 multimode LED
        self.bus.write_i2c_block_data(self.address, REG_MODE_CONFIG, [0x03])

        SPO2_ADC_RANGE = 0b01 # 4096nA
        SPO2_SAMPLE_RATE = 0b011 # 400Hz
        LED_PULSE_WIDTH = 0b11 # 411us
        REG_SPO2_CONFIG_DATA = (SPO2_ADC_RANGE << 5 | SPO2_SAMPLE_RATE << 2 | LED_PULSE_WIDTH)
        self.bus.write_i2c_block_data(self.address, REG_SPO2_CONFIG, [REG_SPO2_CONFIG_DATA])

        # choose value for ~7mA for LED1(Red for SpO2)
        self.bus.write_i2c_block_data(self.address, REG_LED1_PA, [0x24])
        # choose value for ~7mA for LED2(IR for HR)
        self.bus.write_i2c_block_data(self.address, REG_LED2_PA, [0x24])

    def read_fifo(self):
        red_led = None
        ir_led = None

        # The interrupt is cleared by reading the Interrupt Status 1 register
        self.bus.read_i2c_block_data(self.address, REG_INTR_STATUS_1, 1)

        # read 6-byte data from the device
        d = self.bus.read_i2c_block_data(self.address, REG_FIFO_DATA, 6)

        # mask MSB [23:18]
        red_led = (d[0] << 16 | d[1] << 8 | d[2]) & 0x03FFFF
        ir_led = (d[3] << 16 | d[4] << 8 | d[5]) & 0x03FFFF

        return red_led, ir_led

    def read_sequential(self, amount=100):
        """
        This function will read the red-led and ir-led `amount` times.
        This works as blocking function.
        """
        red_buf = []
        ir_buf = []
        for _ in range(amount):
            while(GPIO.input(self.interrupt) == 1):
                # wait for interrupt signal, which means the data is available
                pass

            red, ir = self.read_fifo()

            red_buf.append(red)
            ir_buf.append(ir)

        return red_buf, ir_buf
