// Simple Verilator C++ wrapper for z386 testbench
#include "Vtb_z386.h"
#include "verilated.h"
#include "verilated_vcd_c.h"
#include <iostream>

int main(int argc, char **argv) {
    auto contextp = new VerilatedContext;
    contextp->commandArgs(argc, argv);

    Vtb_z386 *tb = new Vtb_z386{contextp};
    VerilatedVcdC *tfp = nullptr;

    // Enable tracing unless +notrace is specified
    const char* notrace_arg = contextp->commandArgsPlusMatch("notrace");
    bool enable_trace = !(notrace_arg && notrace_arg[0] != '\0');

    if (enable_trace) {
        contextp->traceEverOn(true);
        tfp = new VerilatedVcdC;
        tb->trace(tfp, 99);
        tfp->open("trace.vcd");
    }

    // Run simulation
    while (!contextp->gotFinish()) {
        tb->eval();
        if (tfp) {
            tfp->dump(contextp->time());
        }
        contextp->timeInc(1);
    }

    printf("Simulation finished, time=%llu\n", contextp->time());
    // Cleanup
    if (tfp) {
        tfp->close();
        delete tfp;
        printf("Trace file closed\n");
    }
    delete tb;

    return 0;
}
