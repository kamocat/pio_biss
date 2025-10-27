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

period = 9 # cycles per half-clock
dly = 0 #cycles before sampling
asm = Program(f'''
.program biss_master
.side_set 1
    out x, 32 side 1
    wait 1 pin 0 side 1 [{period}] ;Set MA high and wait for SLO to go high
public get_ack:  ;Wait for ACK falling edge
    nop side 0 [{period}]
    nop side 1 [{period-1}]
    jmp pin get_ack side 1
get_ready: ;Wait for ACK rising edge
    nop side 0 [{period}]
    nop side 1 [{period-2}]
    jmp pin get_set side 1
    jmp get_ready side 1
public get_set: ;Clear the first bit
    nop side 0 [{period}]
    nop side 1 [{period-1}]
    jmp x-- get_bits side 1
get_bits:
    nop side 0 [{dly}]
    in pins, 1 side 0 [{period-dly-1}]
    nop side 1 [{period-1}]
    jmp x-- get_bits side 1
public cleanup:
    push side 1 ;FIXME: Only push if there is data
public finis:
    nop side 1
''')

program = asm.assembled

# %% Emulation

class Clocked:
    def __init__(self, data, bits):
        self.data = data
        self.i = bits
        self.prev = 1
    def next(self, state:State) -> int:
        clk = state.pin_values & 2
        if not clk and self.prev:
            self.i -= 1
        self.prev = clk
        mask = 1<<self.i if self.i>=0 else 0
        return 1 if self.data & mask else 0
    def complete(self, opcode:int, state:State) -> bool:
        return self.i <= 0

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

init = State(pin_directions=0x04,
             transmit_fifo=deque([48]),
             clock=130,
             #program_counter=asm.public_labels['entry_point']
            )

data = Vcd('1MHz.vcd', 500)



def tx_complete(opcode:int, state:State) -> bool:
    return conditions.transmit_fifo_empty(state) and state.program_counter == asm.public_labels['finis']

trace = [
    (state.clock, state.pin_values, state.program_counter, state.receive_fifo.pop() if state.receive_fifo else None)
    for _, state in emulate(program,
                    #stop_when=conditions.clock_cycles_reached(1810),
                    stop_when=tx_complete,
                    initial_state = init,
                    input_source=data.get,
                    auto_push=True,
                    push_threshold=32,
                    auto_pull=True,
                    pull_threshold=32,
                    shift_isr_right=False,
                    shift_osr_right=False,
                    side_set_base=2,
                    side_set_count=asm.pio_kwargs['sideset_pin_count'],
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

binary_plot(['SLO','ref','MA'])
captures = [f'{x[3]:08X}' for x in trace if x[3] is not None]
print(captures)
