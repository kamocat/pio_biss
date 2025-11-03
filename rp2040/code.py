import time

import board
import rp2pio
import digitalio
import array

from adafruit_pioasm import Program

tx_enable = digitalio.DigitalInOut(board.GP21)
tx_enable.direction = digitalio.Direction.OUTPUT
tx_enable.value = True

period = 9 # cycles per half-clock
dly = 0 #cycles before sampling
asm = Program(f'''
.program biss_master
.side_set 1
    set pins, 1
    pull
    out x, 8
    wait 1 pin 0 [{period}] ;Set MA high and wait for SLO to go high
get_ack:  ;Wait for ACK falling edge
    set pins, 0 [{period}]
    set pins, 1 [{period-1}]
    jmp pin get_ack
get_ready: ;Wait for ACK rising edge
    set pins, 0 [{period}]
    set pins, 1 [{period-2}]
    jmp pin get_set
    jmp get_ready
get_set: ;Clear the first bit
    set pins, 0 [{period}]
    set pins, 1 [{period-1}]
    jmp x-- get_bits
get_bits:
    set pins, 0 [{dly}]
    in pins, 1 [{period-dly-1}]
    set pins, 1 [{period-1}]
    jmp x-- get_bits
cleanup:
    push ;FIXME: Only push if there is data
''')

assembled = asm.assembled

sm = rp2pio.StateMachine(
    assembled,
    frequency=20000000,
    first_in_pin=board.GP18,
    first_set_pin=board.GP20,
    in_pin_count=1,
    sideset_pin_count=1,
    jmp_pin=board.GP18,
    auto_push=True,
    pull_threshold=8,
    push_threshold=32,
)
print("real frequency", sm.frequency)

while True:
    x = array.array('l',range(4))
    sm.write_readinto(bytes((44)),x)
    print(x)
    time.sleep(0.5)