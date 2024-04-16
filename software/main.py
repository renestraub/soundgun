from machine import ADC, PWM, Pin
from machine import mem32
from time import ticks_us, sleep
import math
import gc


CORE_FREQ = const(125_000_000)
ADC_FREQ = const(48_000_000)

# Accroding to datasheet of US transmitter the working frequency is 40 kHz
# Empirical tests have shown best volume and lowest power consumption at 42.0 kHz
PWM_FREQ = const(42000)
SAMPLING_FREQ = const(PWM_FREQ // 2)


# ADC Hardware Register
ADC_BASE = const(0x4004c000)
ADC_CS_REG = const(ADC_BASE + 0x0)
ADC_FCS_REG = const(ADC_BASE + 0x8)
ADC_FIFO_REG = const(ADC_BASE + 0xC)
ADC_DIV_REG = const(ADC_BASE + 0x10)
ADC_INTR_REG = const(ADC_BASE + 0x14)

# PWM HW Registers
PWM_BASE = const(0x40050000)
PWM_CH7_CSR_REG = const(PWM_BASE + 0x08c)   # Control and status
# PWM_CH7_DIV_REG = const(PWM_BASE + 0x90)
PWM_CH7_CC_REG = const(PWM_BASE + 0x98)     # Compare register, defines pulse width


PWM_RANGE = const(CORE_FREQ // PWM_FREQ) # Max. value for PWM output (pulse = 100% on)
PWM_CENTER_VALUE = const(PWM_RANGE // 2) # Value for 50% duty cycle
print(f"PWM_RANGE {PWM_RANGE}, PWM_CENTER_VALUE {PWM_CENTER_VALUE}")

#
# Pre-Compute multiplication factor from analog input value to PWM output setting
# 
# ADC values are from 98 to 161 at 880 mVpp audio signal, center is at 129.
# PWM value shall use 25% (1/4th) of possible range = 3125 / 4.
# Multiplier is then (3125 / 4) / (161 - 98)
# 

# Values obtained by running the function sample_audio_input() while playing a
# 440 hz tone at maximum volume.
# AIN_LOW = const(98)
# AIN_HIGH = const(161)
AIN_LOW = const(355)
AIN_HIGH = const(3861)

AIN_RANGE = const(AIN_HIGH - AIN_LOW)
AIN_CENTER = const((AIN_HIGH + AIN_LOW) // 2)
print(f"AIN_RANGE {AIN_RANGE}, AIN_CENTER {AIN_CENTER}")
PWM_VOLUME_IN_PERCENT = const(25)

# PWM_GAIN = const(PWM_RANGE * PWM_VOLUME_IN_PERCENT // 100 // AIN_RANGE)
PWM_GAIN = const(1)
print(f"PWM_GAIN {PWM_GAIN}")
PWM_ATTENUATION = const(2)
print(f"PWM_ATTENUATION {PWM_ATTENUATION}")

pwm_min = PWM_CENTER_VALUE + (AIN_LOW - AIN_CENTER) * PWM_GAIN / PWM_ATTENUATION
pwm_0 = PWM_CENTER_VALUE + (AIN_CENTER - AIN_CENTER) * PWM_GAIN / PWM_ATTENUATION
pwm_max = PWM_CENTER_VALUE + (AIN_HIGH - AIN_CENTER) * PWM_GAIN / PWM_ATTENUATION
print(pwm_min, pwm_0, pwm_max)


# Global variables, could do nicer by using a class with members
led = None
led_module = None
pwm14 = None
pwm15 = None
ain = None


def setup(pwm_freq, sampling_freq):
    """
    Initialize hardware
    """
    global led, led_module
    global ain
    global pwm14, pwm15

    #
    # Make some light
    #

    led_module = Pin(25, Pin.OUT)
    led_module.on()

    led = Pin(16, Pin.OUT)
    led.on()

    #
    # Setup ADC Input 
    # - 8'000 samples per second
    # - 8 bit samples
    #

    # Get ADC controller with default settings (500 ksps, 12 bit resolution)
    ain = ADC(0)        

    # Change settings as needed

    # Enable FIFO, 8 bit right aligned, IRQ after one byte
    FIFO_EN = 1 << 0
    FIFO_SHIFT_RIGHT = 0    # 1 << 1
    FIFO_THRESHOLD_ONE_BYTE = 1 << 24
    adc_cs = mem32[ADC_CS_REG]
    # print(f'ADC_CS = {adc_cs:08x}')
    mem32[ADC_FCS_REG] = FIFO_EN | FIFO_SHIFT_RIGHT | FIFO_THRESHOLD_ONE_BYTE
    # adc_fcs = mem32[ADC_FCS_REG]
    # print(f'ADC_FCS = {adc_fcs:08x}')

    adc_div = int(ADC_FREQ / sampling_freq)
    print(f'ADC clock divider = {adc_div}, 0x{adc_div:x}')
    mem32[ADC_DIV_REG] = (adc_div) << 8
    # adc_div = mem32[ADC_DIV_REG]
    # print(f'ADC_DIV = 0x{adc_div:x}')

    #
    # Setup two PWM Outputs
    # - 40'000 Hz, for ultrasonic speakers
    # - A and B with opposite polarity
    # 

    # print(f'max duty cycle = {PWM_RANGE}')

    pwm14 = PWM(Pin(14), freq = pwm_freq)
    pwm15 = PWM(Pin(15), freq = pwm_freq)
    pwm14.duty_u16(int(PWM_CENTER_VALUE))
    pwm15.duty_u16(int(PWM_CENTER_VALUE))

    # ch7_div = mem32[PWM_DIV_REG]
    # print(f'ch7_div = 0x{ch7_div:04x}')

    # Invert the B output
    mem32[PWM_CH7_CSR_REG] |= (1 << 3)
    # ch7_csr = mem32[PWM_CH7_CSR_REG] 
    # print(f'ch7_csr = 0x{ch7_csr:04x}')


def adc_start():
    """
    Starts ADC

    Set START_MANY bit, ADC will provide sample values at 8 kHz rate
    """
    mem32[ADC_CS_REG] |= (1 << 3)
    # adc_cs = mem32[ADC_CS_REG]
    # print(f'ADC_CS = {adc_cs:08x}')


def adc_stop():
    """
    Stops ADC
    """
    mem32[ADC_CS_REG] &= ~(1 << 3)
    # adc_cs = mem32[ADC_CS_REG]
    # print(f'ADC_CS = {adc_cs:08x}')


def pwm_stop():
    """
    Stops PWM outputs

    This is necessary as the PWM driver otherwise overheats
    """
    global pwm14, pwm15

    pwm14.deinit()
    pwm15.deinit()


def get_sinewave(freq: int):
    """
    Computes a sinewave for the frequency <freq>.

    Amplitude is similar to a 880 mVpp audio input signal.

    Returns list with samples
    """
    phase_shift = (freq * 360) / SAMPLING_FREQ
    print(f"phase shift = {phase_shift}°")

    count = int(SAMPLING_FREQ / freq)
    print(f"have {count} samples")

    res = []

    for sample in range(count):
        val = math.sin(2*math.pi * sample / count)
        pwm = int(val * AIN_RANGE / 2)
        res.append(pwm)

    return res


def sample_audio_input():
    """
    Samples audio for one second. 
    
    Checks highest and lowest value to know if everything is working as expected.
    """
    print("checking ADC. sampling for one second. audio source must be started")

    sum = 0

    adc_start()

    low = 65535
    high = 0
    t_start = ticks_us()

    for pos in range(SAMPLING_FREQ):
        while mem32[ADC_INTR_REG] & 0x1 == 0:
            pass

        val = mem32[ADC_FIFO_REG] # & 0xFF
        sum += val
        if val < low:
            low = val
        if val > high:
            high = val

    t_end = ticks_us()

    adc_stop()

    avg = int((low + high) / 2)
    print(low, high, avg)
    print(sum/SAMPLING_FREQ)

    print(f"done {pos}: {(t_end - t_start)/1e6}")


def play_samples(samples):
    """
    Plays the sample values in <samples> in an endless loop.
    Values in <sample> are expected to be in range 98 .. 161 as from real audio signal.
    """
    step = 0
    count = len(samples) - 1

    adc_start()

    while True:
        # Wait for new ADC value to get timing right
        while mem32[ADC_INTR_REG] & 0x1 == 0:
            pass
        _ = mem32[ADC_FIFO_REG] # Read FIFO so next value can be stored, ignore value

        # Compute PWM value from given sample value
        val = samples[step]
        pwm = PWM_CENTER_VALUE + val * PWM_GAIN
        mem32[PWM_CH7_CC_REG] = (pwm << 0) | (pwm << 16)

        # Look for next sample
        if step < count:
            step += 1
        else:
            step = 0


def playback() -> None:
    """
    Plays audio as sampled from analog input on ultrasonic speakers.
    
    Uses ADC unit as loop timer. Whenever an ADC sample is available, the PWM duty
    cycle is immediately updated. This modulates the audio signal on the 40 kHz
    carrier required by the US speaker.
    """
    global led

    gc.collect()
    adc_start()

    pwm = PWM_CENTER_VALUE
    pwm_reg_value = (pwm) | (pwm << 16)
    while True:
        led.off()
        # Wait for new ADC value. This will generate our timing of the loop
        while mem32[ADC_INTR_REG] & 0x1 == 0:
            pass

        # Write next value to PWM controller as fast as possible
        mem32[PWM_CH7_CC_REG] = pwm_reg_value
        led.on()

        # Compute next value while we have a lot of time
        # - Create a zero (0) centered value from unsigned integer with average value 129
        #   98..129..161 -> -31..0..32
        # - Center in middle of PWM range, multiply to get 25% amplitude
        #   -31..0..32 -> 1190..1562..1946
        val = mem32[ADC_FIFO_REG] - AIN_CENTER
        pwm = PWM_CENTER_VALUE + val #* PWM_GAIN // PWM_ATTENUATION
        pwm_reg_value = (pwm) | (pwm << 16)


def playback2(duration: int = 1000) -> None:
    """
    Performance optimized version of playback() that used pre-compiled code.
    """
    gc.collect()
    adc_start()

    # Repeatedly call the optimized playback function. The loop
    # gives the interpreter a chance to break into our code.
    # while True:
    for _ in range(duration * 4):
        playback_fast()


@micropython.viper
def playback_fast():
    global led

    adc_intr = ptr32(ADC_INTR_REG)
    adc_fifo = ptr32(ADC_FIFO_REG)
    pwm_cc = ptr32(PWM_CH7_CC_REG)

    pwm = PWM_CENTER_VALUE
    pwm_reg_value = (pwm) | (pwm << 16)

    # Can't loop forever, as Python can't stop us otherwise
    for _ in range(SAMPLING_FREQ // 4):
        led.off()
        # Wait for new ADC value. This will generate our timing of the loop
        while adc_intr[0] & 0x1 == 0:
            pass

        # Write next value to PWM controller as fast as possible
        pwm_cc[0] = pwm_reg_value
        led.on()

        val = adc_fifo[0] - AIN_CENTER
        pwm = PWM_CENTER_VALUE + val # * 2 // 3
        pwm_reg_value = (pwm) | (pwm << 16)


def test_led():
    global led

    while True:
        led.on()
        sleep(0.500)
        led.off()
        sleep(0.500)


def test_sinewave():
    sinewave = get_sinewave(400)
    print(sinewave)
    play_samples(sinewave)


if __name__ == '__main__':
    setup(PWM_FREQ, SAMPLING_FREQ)

    try:
        # test_led()
        # test_sinewave()
        # sample_audio_input()

        # playback()
        playback2()
    except KeyboardInterrupt:
        pass
    finally:
        pwm_stop()
