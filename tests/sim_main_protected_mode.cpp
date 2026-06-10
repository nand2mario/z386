#include "Vtb_protected_mode.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Vtb_protected_mode* top = new Vtb_protected_mode;

    VerilatedVcdC* tfp = nullptr;
    if (Verilated::commandArgsPlusMatch("trace")) {
        Verilated::traceEverOn(true);
        tfp = new VerilatedVcdC;
        top->trace(tfp, 99);
        tfp->open("trace.vcd");
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
    delete top;
    return 0;
}
