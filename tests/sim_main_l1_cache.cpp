#include "Vtb_l1_cache.h"
#include "verilated.h"

int main(int argc, char **argv) {
    auto contextp = new VerilatedContext;
    contextp->commandArgs(argc, argv);
    Vtb_l1_cache *top = new Vtb_l1_cache{contextp};

    while (!contextp->gotFinish() && contextp->time() < 200000) {
        top->eval();
        contextp->timeInc(1);
    }

    int failed = contextp->gotFinish() ? 0 : 1;
    delete top;
    delete contextp;
    return failed;
}
