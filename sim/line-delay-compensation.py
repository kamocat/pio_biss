#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Oct 18 07:01:50 2025

@author: marshal
"""

"""
References
https://biss-interface.com/wp-content/uploads/2025/07/BiSS_BP3S_profile_A1en-2.pdf
https://biss-interface.com/wp-content/uploads/2025/07/BiSS_BP3_profile_A3en-19.pdf
https://www.renishaw.com/resourcecentre/download/data-sheet-biss-c-mode-unidirectional-for-resolute-and-fortis-encoders--131419

A typical request cycle proceeds as follows:
1. When idle, the master holds MA high. The encoder indicates it is ready by holding SLO high.
2. The master requests position acquisition by starting to transmit clock pulses on MA.
3. The encoder responds by setting SLO low on the second rising edge on MA.
4. After the Ack period is complete, the encoder transmits data to the master synchronised with the clock as shown above.
5. When all data has been transferred, the master stops the clock and sets MA high.
6. If the encoder is not yet ready for the next request cycle, it sets SLO low (the Timeout period).
7. When the encoder is ready for the next request cycle, it indicates this to the master by setting SLO high
"""
# BISS Encoding

import matplotlib.pyplot as plt
from pioemu import conditions, emulate, State
from adafruit_pioasm import Program
from collections import deque
from vcdvcd import VCDVCD

line_delay = '''
.program biss_line_delay_measurement
    wait 0 pin 2 ; Wait for idle signaled by another PIO
    wait 0 pin 1
    wait 1 pin 1
    wait 0 pin 1
    wait 1 pin 1 ;Wait for the second rising edge
ack:
    jmp x-- count_ack
count_ack:
    jmp pin ack
public send_line_delay:
    mov x, !x
    in x, 32
    push noblock
'''

#FIXME: Add the appropriate delays to keep consistent counts
detect_idle = '''
count_idle:
    jmp x-- wait_idle
wait_idle:
    jmp pin count_idle
    in x, 32
    push noblock
    set pins, 1
    mov x, !null
count_lo:
    set pins, 1
    set y, 31 ;Adjust to suit timeout
    jmp pin count_hi
    jmp x-- count_lo
count_hi:
    jmp x-- count_hi2
count2_hi:
    jmp y-- wait_hi
    set pins, 0
    jmp count_idle
wait_hi:
    jmp pin count_hi
'''

asm = Program(line_delay)

program = asm.assembled

# %% Emulation

class Vcd:
    def __init__(self, fname:str, clkrate:float = 1):
        self.d = VCDVCD(fname).data
        self.clkrate = clkrate
    def get(self, state:State) -> int:
        clk = int(state.clock * self.clkrate)
        accum = 0
        for k in list(self.d.keys()):
            accum <<= 1
            accum |= int(self.d[k][clk])
        return accum

init = State(pin_directions=0x00,
             clock=130,
             #program_counter=asm.public_labels['entry_point']
            )

data = Vcd('1MHz.vcd', 500)


trace = [
    (state.clock, state.pin_values, state.program_counter, state.receive_fifo.pop() if state.receive_fifo else None)
    for _, state in emulate(program,
                    stop_when=conditions.clock_cycles_reached(18100),
                    initial_state = init,
                    input_source=data.get,
                    auto_push=False,
                    shift_isr_right=True,
                    #wrap_target=asm.pio_kwargs['wrap_target'],
                    #wrap_top=asm.pio_kwargs['wrap'],
                   )
]

t = [x[0] for x in trace]

execution = [x[2] for x in trace]
crossref = {value:key for key,value in asm.public_labels.items()}
execution = [crossref[x] if x in crossref else x for x in execution]

def binary_plot(names):
    n = len(names)
    fig, axes = plt.subplots(n,1,sharex=True,sharey=True, figsize=(16,3))
    try:
        axes[0]
    except:
        axes = [axes]
    for i in range(n):
        axes[i].step(t,  [1 if x[1]&(1<<i) else 0 for x in trace])
        axes[i].set_ylabel(names[i])
    axes[-1].set_xlabel('PIO Clock Cycles')
    axes[0].margins(x=0, y=0.05)
    plt.yticks([0,1])
    plt.xticks([n for n,e in zip(t,execution) if type(e) is type('')]) #Only label the named transitions

binary_plot(['SLO','MA'])
print( [hex(x[3]) for x in trace if x[3] is not None])
