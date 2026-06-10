# Makefile for z386 Testing
#
# Builds and runs single-instruction tests for the z386 core

# Paths
Z386_DIR := ..
VERILATOR := verilator

# Source files
# Note: muldiv.sv and bus_prefetch.sv are standalone modules that can optionally
# replace the inline mul/div and bus/prefetch code in z386.sv.
# Currently z386.sv uses inline code; uncomment modules below when integrating.
RTL_SRCS := $(Z386_DIR)/z386_pkg.sv $(Z386_DIR)/z386.sv $(Z386_DIR)/decoder.sv \
			$(Z386_DIR)/alu.sv $(Z386_DIR)/biu.sv $(Z386_DIR)/prefetch.sv \
			$(Z386_DIR)/paging_unit.sv $(Z386_DIR)/paging_tlb.sv $(Z386_DIR)/paging_walker.sv \
			$(Z386_DIR)/l1_cache.sv $(Z386_DIR)/l1_icache.sv \
			$(Z386_DIR)/dsp_mul.sv $(Z386_DIR)/protection.sv $(Z386_DIR)/segmentation_unit.sv
# Optional modules (not yet integrated):
# RTL_SRCS += $(Z386_DIR)/muldiv.sv $(Z386_DIR)/bus_prefetch.sv
TB_SRCS := tb_z386.sv
CPP_SRCS := sim_main.cpp

# Verilator flags (common, no --top-module — set per target)
VFLAGS_BASE := --timing
VFLAGS_BASE += -Wall -Wno-fatal -Wno-UNOPTFLAT -Wno-SYNCASYNCNET
VFLAGS_BASE += -cc --exe
VFLAGS_BASE += -O3
VFLAGS_BASE += --build
VFLAGS_BASE += -j 0
VFLAGS_BASE += -I$(Z386_DIR)
VFLAGS_BASE += -CFLAGS "-std=c++14 -O3"

# With VCD trace support (singlestep tests — short runs, need waveforms)
VFLAGS := $(VFLAGS_BASE) --trace --trace-structs
# Without trace
VFLAGS_NOTRACE := $(VFLAGS_BASE)

# protected mode testbench sources
TB_PROT_SRCS := tb_protected_mode.sv
CPP_PROT_SRCS := sim_main_protected_mode.cpp

# Targets
.PHONY: all build test-simple test-protected clean help

all: build

# Build all testbenches
build: obj_dir/Vtb_z386 obj_dir/Vtb_protected_mode

obj_dir/Vtb_z386: $(RTL_SRCS) $(TB_SRCS) $(CPP_SRCS)
	@echo "Building z386 testbench with Verilator..."
	cp -f ../*.hex .
	$(VERILATOR) $(VFLAGS) --top-module tb_z386 $(TB_SRCS) $(RTL_SRCS) $(CPP_SRCS)
	@echo "Build complete: obj_dir/Vtb_z386"

obj_dir/Vtb_protected_mode: $(RTL_SRCS) $(TB_PROT_SRCS) $(CPP_PROT_SRCS)
	@echo "Building protected mode testbench with Verilator..."
	cp -f ../*.hex .
	$(VERILATOR) $(VFLAGS) --top-module tb_protected_mode $(TB_PROT_SRCS) $(RTL_SRCS) $(CPP_PROT_SRCS)
	@echo "Build complete: obj_dir/Vtb_protected_mode"

test: test-simple

# Run tests
test-simple: build
	@echo "Running z386 simple instruction tests..."
	./test_simple.py

test-protected: obj_dir/Vtb_protected_mode
	@echo "Running protected mode tests..."
	./test_protected_mode.py

# Clean build artifacts
clean:
	rm -rf obj_dir
	rm -f trace.fst trace.vcd
	rm -f result.txt
	rm -f *.hex

# Help
help:
	@echo "z386 Test Makefile"
	@echo "=================="
	@echo ""
	@echo "Targets:"
	@echo "  make build          - Build all testbenches with Verilator"
	@echo "  make test-simple    - Run default test suite (~13 test files)"
	@echo "  make test-protected - Run protected mode regression tests"
	@echo "  make clean          - Remove build artifacts"
	@echo "  make help           - Show this help message"
