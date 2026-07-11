
# z386 - an 80386-class FPGA CPU built around original microcode

z386 is an 80386-compatible CPU core written in SystemVerilog and built around the original Intel 386 microcode. Instead of implementing each x86 instruction as a separate RTL behavior, z386 implements the hardware structures the microcode expects to control: instruction prefetch, decode, the microcode sequencer, segmentation, paging, protection checks, ALU, shifter, and bus access.

This `z386x` branch contains the eXtended core used by current z386_MiSTer
builds. It adds a faster frontend and bounded hardwired fast paths around the
original microcode engine. The `master` branch remains the 80386-faithful z386
implementation.

