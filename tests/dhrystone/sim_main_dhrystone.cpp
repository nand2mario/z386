#include "Vtb_dhrystone.h"
#include "verilated.h"
#include "verilated_fst_c.h"

#include <cstring>

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Vtb_dhrystone* top = new Vtb_dhrystone;

    VerilatedFstC* tfp = nullptr;
    bool trace = false;
    const char* tracefile = "trace.fst";

    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "+trace") == 0) {
            trace = true;
        }
        const char* prefix = "+tracefile=";
        if (std::strncmp(argv[i], prefix, std::strlen(prefix)) == 0) {
            tracefile = argv[i] + std::strlen(prefix);
        }
    }

    if (trace) {
        Verilated::traceEverOn(true);
        tfp = new VerilatedFstC;
        top->trace(tfp, 99);
        tfp->open(tracefile);
    }

    while (!Verilated::gotFinish()) {
        top->eval();
        if (tfp) tfp->dump(Verilated::time());
        Verilated::timeInc(1);
    }

    if (tfp) {
        tfp->close();
        delete tfp;
    }
    top->final();   // run SV `final` blocks (e.g. z386x dead-slot breakdown)
    delete top;
    return 0;
}
