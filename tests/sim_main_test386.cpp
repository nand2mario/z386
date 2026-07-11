#include <cstdio>
#include <cstring>
#include "Vtb_test386.h"
#include "verilated.h"
#if VM_TRACE_FST
#include "verilated_fst_c.h"
#elif VM_TRACE
#include "verilated_vcd_c.h"
#endif

int main(int argc, char** argv) {
    setvbuf(stdout, NULL, _IOLBF, 0);  // Line-buffered stdout for real-time output
    Verilated::commandArgs(argc, argv);
    Vtb_test386* top = new Vtb_test386;
    bool trace_enabled = false;
    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "+trace") == 0) {
            trace_enabled = true;
            break;
        }
    }

#if VM_TRACE_FST
    VerilatedFstC* tfp = nullptr;
    if (trace_enabled) {
        Verilated::traceEverOn(true);
        tfp = new VerilatedFstC;
        top->trace(tfp, 99);
        tfp->open("test386.fst");
    }
#elif VM_TRACE
    VerilatedVcdC* tfp = nullptr;
    if (trace_enabled) {
        Verilated::traceEverOn(true);
        tfp = new VerilatedVcdC;
        top->trace(tfp, 99);
        tfp->open("test386.vcd");
    }
#endif

    while (!Verilated::gotFinish()) {
        top->eval();
#if VM_TRACE_FST || VM_TRACE
        if (tfp) tfp->dump(Verilated::time());
#endif
        Verilated::timeInc(1);
    }

#if VM_TRACE_FST || VM_TRACE
    if (tfp) {
        tfp->close();
        delete tfp;
    }
#endif
    delete top;
    return 0;
}
