from machine import ADC, PWM, Pin
from machine import mem32
from machine import Timer
from time import ticks_us, sleep_us, sleep
import math



PWM_FREQ = 40000
SAMPLING_FREQ = 8000

# ADC CS.START_MANY = Sample continuosly
ADC_BASE = 0x4004c000
ADC_CS = ADC_BASE + 0x0
ADC_FCS = ADC_BASE + 0x8
ADC_FIFO = ADC_BASE + 0xC
ADC_DIV = ADC_BASE + 0x10
ADC_INTR = ADC_BASE + 0x14

PWM_INTR = 0x400500a4

def test_led():
    while True:
        led.on()
        sleep(0.500)
        led.off()
        sleep(0.500)


def test_pwm_outputs():
    pwm15 = PWM(Pin(15), freq=PWM_FREQ)
    pwm14 = PWM(Pin(14), freq=PWM_FREQ)
    pwm14.duty_u16(32768)
    pwm15.duty_u16(32768)


def test():
    pwm14 = PWM(Pin(14), freq=PWM_FREQ)
    pwm15 = PWM(Pin(15), freq=PWM_FREQ)

    adc_rate_in_ns = int(1/8000*1e9)
    print(adc_rate_in_ns)

    # 500 ksps, 12 bit
    ain = ADC(0)

    pwm14.duty_u16(32768)
    pwm15.duty_u16(32768)

    return

    max_duty = mem32[0x4005009C]
    print(f'max duty cycle = 0x{max_duty:04x}')

    ADC_BASE = 0x4004c000
    ADC_DIV = ADC_BASE + 0x10
    # ADC CS.START_MANY = Sample continuosly
    # ADC DIV: 48000 = 1k Hz sampling rate

    # adc_div = mem32[ADC_DIV]
    # print(f'ADC_DIV = {adc_div}')

    # mem32[ADC_DIV] = int(48_000_000 / 1000)

    ts = ticks_us()
    for i in range(1000):
        x = ain.read_u16()
        # duty = i % max_duty
        # duty = duty | duty << 16
        # mem32[0x4005009C] = duty

    te = ticks_us()
    print(te-ts)
    t_per_sample = (te-ts)/1000
    print(f'time per sample {t_per_sample} us')
    print(f'frequency = {1e6/t_per_sample}')

    # tim = Timer(period=8000, mode=Timer.ONE_SHOT, callback=lambda t:print(1))
    # tim.init(period=2000, mode=Timer.PERIODIC, callback=lambda t:print(2))



def get_sinewave(freq: int):
    res = []
    max_duty = int(125_000_000 / 40_000)

    phase_shift = (freq * 360) / SAMPLING_FREQ
    print(f"phase shift = {phase_shift}°")

    count = int(SAMPLING_FREQ / freq)
    print(f"have {count} samples")

    for sample in range(count):
        val = math.sin(2*math.pi * sample / count)
        # print(val)
        # pwm = int(max_duty/2 + val * max_duty/3)
        pwm = int(val * 127)
        # print(pwm)
        res.append(pwm)

    return res


def test_adc():
    # 500 ksps, 12 bit
    ain = ADC(0)

    adc_cs = mem32[ADC_CS]
    print(f'ADC_CS = {adc_cs:08x}')

    # Enable FIFO, 8 Bit right aligned, IRQ after one byte
    mem32[ADC_FCS] = 1 << 0 | 1 << 1 | 1 << 24
    adc_fcs = mem32[ADC_FCS]
    print(f'ADC_FCS = {adc_fcs:08x}')

    adc_div = int(48_000_000 / SAMPLING_FREQ)   # 48 MHz ADC clock
    print(f'ADC clock divider = {adc_div}')
    mem32[ADC_DIV] = adc_div << 8
    adc_div = mem32[ADC_DIV]
    print(f'ADC_DIV = {adc_div}')

    mem32[ADC_CS] = adc_cs | 1 << 3     # START_MANY
    adc_cs = mem32[ADC_CS]
    print(f'ADC_CS = {adc_cs:08x}')

    print("checking ADC. sampling for one second. audio source must be started")
    t_start = ticks_us()
    i = 0
    adc = []

    for _ in range(SAMPLING_FREQ):
        while mem32[ADC_INTR] & 0x1 == 0:
            pass
        val = mem32[ADC_FIFO]
        adc.append(val)
        i += 1

    low = min(adc)
    high = max(adc)
    avg = int((low + high) / 2)
    print(low, high, avg)

    # gain = int(max_duty / 128)
    # print(f'pwm gain = {gain}')

    t_end = ticks_us()
    print(f"done {i}: {(t_end - t_start)/1e6}")


def test_pwm(samples):
    # 500 ksps, 12 bit
    ain = ADC(0)

    pwm15 = PWM(Pin(15), freq=PWM_FREQ)
    pwm14 = PWM(Pin(14), freq=PWM_FREQ)
    pwm14.duty_u16(32768)
    pwm15.duty_u16(32768)

    ch7_div = mem32[0x40050090] 
    print(f'ch7_div = 0x{ch7_div:04x}')

    # Invert the B output
    ch7_csr = mem32[0x4005008c] 
    print(f'ch7_csr = 0x{ch7_csr:04x}')
    mem32[0x4005008c] = ch7_csr | (1 << 3)
    ch7_csr = mem32[0x4005008c] 
    print(f'ch7_csr = 0x{ch7_csr:04x}')

    max_duty = int(125_000_000 / 40_000)  # 125 MHz base clock, 40 kHz PWM
    print(f'max duty cycle = {max_duty}')


    adc_cs = mem32[ADC_CS]
    print(f'ADC_CS = {adc_cs:08x}')

    # Enable FIFO, 8 Bit right aligned, IRQ after one byte
    mem32[ADC_FCS] = 1 << 0 | 1 << 1 | 1 << 24
    adc_fcs = mem32[ADC_FCS]
    print(f'ADC_FCS = {adc_fcs:08x}')

    adc_div = int(48_000_000 / SAMPLING_FREQ)   # 48 MHz ADC clock
    print(f'ADC clock divider = {adc_div}')
    mem32[ADC_DIV] = adc_div << 8
    adc_div = mem32[ADC_DIV]
    print(f'ADC_DIV = {adc_div}')

    mem32[ADC_CS] = adc_cs | 1 << 3     # START_MANY
    adc_cs = mem32[ADC_CS]
    print(f'ADC_CS = {adc_cs:08x}')

    step = 0
    count = len(samples) - 1

    gain = 2
    
    while True:
        # Wait for new ADC value
        while mem32[ADC_INTR] & 0x1 == 0:
            pass
        val = mem32[ADC_FIFO]
        
        # Override with current sample value
        val = samples[step]

        pwm = int(max_duty / 2)
        # pwm += (val - 128) * gain
        pwm += val * gain

        mem32[0x40050098] = (pwm << 0) | (pwm << 16)
        if step < count:
            step += 1
        else:
            step = 0


def play():
    # 500 ksps, 12 bit
    ain = ADC(0)

    pwm15 = PWM(Pin(15), freq=PWM_FREQ)
    pwm14 = PWM(Pin(14), freq=PWM_FREQ)
    pwm14.duty_u16(32768)
    pwm15.duty_u16(32768)

    ch7_div = mem32[0x40050090] 
    print(f'ch7_div = 0x{ch7_div:04x}')

    # Invert the B output
    ch7_csr = mem32[0x4005008c] 
    print(f'ch7_csr = 0x{ch7_csr:04x}')
    mem32[0x4005008c] = ch7_csr | (1 << 3)
    ch7_csr = mem32[0x4005008c] 
    print(f'ch7_csr = 0x{ch7_csr:04x}')

    max_duty = int(125e6/40e3)
    print(f'max duty cycle = {max_duty}')


    adc_cs = mem32[ADC_CS]
    print(f'ADC_CS = {adc_cs:08x}')

    # Enable FIFO, 8 Bit right aligned, IRQ after one byte
    mem32[ADC_FCS] = 1 << 0 | 1 << 1 | 1 << 24
    adc_fcs = mem32[ADC_FCS]
    print(f'ADC_FCS = {adc_fcs:08x}')

    adc_div = int(48e6 / SAMPLING_FREQ)
    print(f'ADC clock divider = {adc_div}')
    mem32[ADC_DIV] = adc_div << 8
    adc_div = mem32[ADC_DIV]
    print(f'ADC_DIV = {adc_div}')

    mem32[ADC_CS] = adc_cs | 1 << 3     # START_MANY
    adc_cs = mem32[ADC_CS]
    print(f'ADC_CS = {adc_cs:08x}')

    print("checking ADC. sampling for one second. audio source must be started")
    t_start = ticks_us()
    i = 0
    adc = []

    for _ in range(SAMPLING_FREQ):
        while mem32[ADC_INTR] & 0x1 == 0:
            pass
        val = mem32[ADC_FIFO]
        adc.append(val)
        i += 1

    low = min(adc)
    high = max(adc)
    avg = int((low + high) / 2)
    print(low, high, avg)

    gain = int(max_duty / 128)
    print(f'pwm gain = {gain}')
    gain = 16

    # t_end = ticks_us()
    # print(f"done {i}: {(t_end - t_start)/1e6}")

    step = 0
    while True:
        # Wait for new ADC value
        while mem32[ADC_INTR] & 0x1 == 0:
            pass
        val = mem32[ADC_FIFO]
        
        pwm = int(max_duty / 2)
        pwm += (val - 128) * gain

        mem32[0x40050098] = (pwm << 0) | (pwm << 16)



print("Hello 4")

led_module = Pin(25, Pin.OUT)
led_module.on()

led = Pin(16, Pin.OUT)
led.on()

# test_pwm_outputs()

sinewave = get_sinewave(800)
print(sinewave)
test_pwm(sinewave)

# test_adc()
