#! /usr/bin/bash
/class/ece411/software/spike_new/bin/spike --isa=rv32imc -m0x40000000:0x80000000  --log-commits "sim/bin/$1.elf" &> "$1_spike.log"
