// z386x - An x86 core with the original 386 microcode and some 486-style pipelining 
//
// nand2mario, July 2026
//
// Functional units
//   1. Prefetch and Bus Interface
//   2. Decode1 (structural decode)
//   3. Decode2 (literals capture and early-address)
//   4. Segmentation Unit
//   5. Paging Unit (including TLB)
//   6. Protection Test Unit
//   7. Execution (control unit for both microcode and fast instructions)
//   8. Write-back
//   9. Data Unit (ALU, register file, barrel shifter)

module z386
    import z386_pkg::*;
#(
    parameter PROTECT_UMA_ROM = 0,
    parameter DCACHE_SET_BITS = 8,   // dcache size: 8 = 16KB, 7 = 8KB
    parameter ICACHE_SET_BITS = 8    // icache size: 8 = 16KB, 7 = 8KB
)
(
    input              clk,
    input              reset_n,

    // 32-bit bus interface (ready/valid handshake)
    output     [31:2]  addr,        // Physical address [31:2]
    output      [3:0]  be,          // Byte enables
    output      [7:0]  burstcount,  // Burst length in DWORDs
    input      [31:0]  din,         // Data input
    output     [31:0]  dout,        // Data output
    output             valid,       // Request valid (held until ready)
    input              ready,       // Handshake: transfer on valid && ready
    output             write,       // 1=write, 0=read (stable while valid)
    output             io,          // I/O vs memory (1=I/O, 0=memory)
    input              resp_valid,  // Read data valid (1-cycle pulse)

    // Interrupts
    input              intr,        // Maskable interrupt request
    input              nmi,         // Non-maskable interrupt
    output             inta,        // Interrupt acknowledge

    // External memory writers can invalidate matching L1 lines.
    input      [31:0]  snoop_addr,
    input              snoop_valid,

    input              a20_enable,  // A20 gate input

    // Debug/test control
    input              single_step, // Halt after each instruction (for single-step tests)

    output     [15:0]  dbg_CS,
    output     [31:0]  dbg_EIP,
    output     [31:0]  dbg_CS_base,
    output             dbg_pe,
    output             dbg_vm
);

reg dbg_first_done;                 // Debug: first instruction finished execution
reg halted;                         // Tracks when the core is halted
reg [31:0] debug_ip;                // Debug: IP at instruction completion
wire [31:0] dbg_addr = {addr, 2'b0};  // Debug: full 32-bit address

reg [31:0] CR0, CR2, CR3;
reg [31:0] DR6, DR7;
reg [31:0] EAX, ECX, EDX, EBX, ESP, EBP, ESI, EDI;
reg [31:0] EIP = 32'h0000FFF0;      // Architectural IP (next instruction) - reset vector
wire [15:0] AX = EAX[15:0];
wire [7:0]  AL = EAX[7:0];
wire [7:0]  AH = EAX[15:8];
wire [15:0] CX = ECX[15:0];
wire [15:0] DX = EDX[15:0];
wire [15:0] BX = EBX[15:0];
wire [15:0] SP = ESP[15:0];
wire [15:0] BP = EBP[15:0];
wire [15:0] SI = ESI[15:0];
wire [15:0] DI = EDI[15:0];

seg_desc_t cs_seg_desc;
assign cs_seg_desc = seg_cache[SEG_CS];
wire D = cs_seg_desc.D_B;  // Default operand size (0=16-bit in real mode, 1=32-bit)

// Bits: 31..22 21 20 19 18 17 16 15..14 13..12 11 10 9  8  7  6  5  4  3  2  1  0
//       Rsvd   ID VIP VIF AC VM RF Rsvd  IOPL   OF DF IF TF SF ZF 0  AF 0  PF 1  CF
reg [31:0] EFLAGS;
reg [31:0] uc_flags;                // Internal ALU flags for microcode conditionals
wire       DF = EFLAGS[10];         // Direction Flag (used for string ops)

reg [31:0] TMPB, TMPC, TMPD, TMPE, TMPF;
reg [31:0] TMPG, TMPH, PROTUN, CSOPCD, FSVeIP, OPROFF;
reg [31:0] SIGMA;                   // ALU result
reg [31:0] FLAGSB;                  // FLAGS backup for INT

wire [31:0] OPR_R;                  // Read operand register, owned by the paging unit
reg [31:0] OPR_W;                   // Bus operation data registers
reg [31:0] IND;                     // Internal address register
reg [31:0] ind_linear;              // linear address of IND
reg        ind_linear_valid;

reg [1:0]  op_size;                 // Runtime operand size: 0=byte, 1=word, 2=dword (modifiable by BITS8/16/32)
reg [1:0]  op_size_decode;          // Decoded operand size (saved at i_pop, restored by BITSDE)
reg [1:0]  srcreg_size;             // Same as op_size most of the time, different for MOVZX/MOVSX and etc
reg [1:0]  srcreg_size_decode;      // Decoded srcreg_size (saved at i_pop, restored by BITSDE)
(* preserve *) reg [1:0] op_size_src;            // Local copy for generic source mux fanout
(* preserve *) reg [1:0] op_size_src_decode;
(* preserve *) reg [1:0] srcreg_size_src;
(* preserve *) reg [1:0] srcreg_size_src_decode;
wire       is_dword = (op_size == 2'd2); // Runtime dword flag
wire       is_dword_src = (op_size_src == 2'd2);

// ALU signals (35-bit for MUL/DIV iteration support)
reg [31:0] alu_dst, alu_src;        // ALU inputs this cycle
reg [4:0]  alu_op5;                 // ALU operation this cycle
reg [31:0] alu_src_r;               // Registered alu_src for jumps (32-bit)
wire [31:0] alu_result;
wire [2:0]  alu_zsp_ahead;

// Shifter
wire [31:0] shift_result;

// ALU source data for operand access
wire [31:0] alu_src_data = read_uc_alu_source(uc_alu_src);

reg [15:0] ES = 16'h0000;
reg [15:0] CS = 16'hF000;           // Reset value
reg [15:0] SS = 16'h0000;
reg [15:0] DS = 16'h0000;
reg [15:0] FS = 16'h0000;
reg [15:0] GS = 16'h0000;
reg [15:0] LDTR, TR;                // Task Register
reg [31:0] SLCTR;                   // Selector temp used by protected-mode descriptor microcode (32-bit: LAR/LSL store full descriptor hi DWORD)
// Forward SLCTR from dest_value when being written in the same cycle
wire        slctr_fwd_en = uc_exec && (uc_dest == DEST_SLCTR || uc_dest == DEST_TMP_TR) && !prot_is_ptovrr;
wire [31:0] slctr_fwd = slctr_fwd_en ? dest_value : SLCTR;

reg [31:0] desc_raw_hi;             // raw high DWORD saved at TSTDES time (before barrel-shift modifies PROTUN)
reg [5:0]  prot_saved_test_const;   // Protection test: PTSAV saves test constant for later PTOVRR to use

seg_desc_t seg_cache [0:10];        // Indexed by SEG_* constants
wire [31:0] CS_base = seg_cache[SEG_CS].base;

wire       pe = CR0[0];             // Protected mode enable
wire       vm = EFLAGS[17];         // Virtual 8086 mode

assign dbg_CS  = CS;
assign dbg_EIP = EIP;
assign dbg_CS_base = CS_base;
assign dbg_pe  = pe;
assign dbg_vm  = vm;
wire [1:0] cpl = vm ? 2'd3 : !pe ? 2'd0 : CS[1:0];

reg [2:0]  latched_pf_code;         // Latched page fault error code (for LPCR microcode access)
reg [31:0] latched_pf_addr;         // Latched faulting linear address (for LPCR microcode access)

wire [63:0] win_d1;                 // registered raw window at the prefetcher's D1 cursor
wire [5:0]  d1_avail;               // bytes fetched beyond the D1 cursor
wire [3:0]  d1_adv;                 // D1 cursor advance this cycle (a prefix, or
                                    //   the instruction rest at handoff, 0-11)
wire [31:0] win_lit;                // D2 literal window at pop_cursor + lit_off
wire [5:0]  lit_avail;              // bytes fetched beyond that point
wire [4:0]  dec_lit_off;            // literal offset from the pop cursor
wire        dec_pop_now;            // one instruction completed D2: pop its bytes
wire [4:0]  dec_pop_len;            //   (registered length from the skeleton)
wire       pf_full;
wire       q_flush;                 // Flush queue (branch/jump) - combinational for i.immediate gating
wire       pe_mode_toggle_now;      // CR0.PE changed this cycle: re-decode next bytes in new mode
reg        uc_ctl_pref;             // Previous-cycle predecode: current uop is BUSOP_PREF

// The live compare read dest_value[0] -- doc/z386x/core_notes_v51.md #1
wire cr0_wr_bit0 = (uc_source == SRC_MDTMP) ? RESULT[0] :
                   (uc_source == SRC_SIGMA) ? SIGMA[0]  : 1'b0;
assign pe_mode_toggle_now = uc_exec && (uc_dest == DEST_CR0) && (cr0_wr_bit0 != CR0[0]);
assign q_flush = (uc_exec && uc_ctl_pref && !uc_cond_jump_taken_prev && !early_redirected)
               || early_redirect || pe_mode_toggle_now;
// synthesis translate_off
always @(posedge clk)
    if (reset_n && uc_exec && (uc_dest == DEST_CR0) && (dest_value[0] !== cr0_wr_bit0))
        $fatal(1, "CR0-WRITE SOURCE INVARIANT BROKEN: uc_addr=%03x src=%02x dv0=%b fast0=%b",
               uc_addr, uc_source, dest_value[0], cr0_wr_bit0);
// synthesis translate_on

wire        page_fault;             // Page fault (declared fully at paging unit instantiation)
wire [1:0]  prot_cpl;               // CPL for protection unit (declared fully near protection logic)
reg  [1:0]  arpl_rpl_latch;         // ARPL RPL latch (declared fully near ARPL logic)

// Memory requests
wire        mem_servicing;          // memory request in flight
wire        mem_dly_grace;          // optimistic read: DLY may execute this (lookup) cycle
wire        mem_write_dly_grace;    // posted write in PG_MEM_TLB: next non-bus uop may execute now
wire        mem_write_wait;         // unposted demand write still fault-capable: stall ALL uops
                                    // (its instruction may be FAST-chained away already)
wire        mem_opt_wait;           // optimistic read missed: stall all uops until fill done
wire        mem_accepted;           // memory request accepted (ready pulse)
wire        mem_complete_now;       // combinational, request completing THIS cycle

// Prefetch ↔ paging unit toggle signals
wire        pf_req_toggle;
wire [31:0] pf_linear_addr;
wire        pf_redirect_queued;
wire        pf_ack_toggle;
wire [127:0] pf_rdata;
wire        pf_fault;

//
// Microcode Sequencer State
//
// After a jump, the next micro-op still executes (delay slot) before jump takes effect.
reg [11:0] uaddr_now;               // Next address, launched early to the ucode ROM
reg [11:0] uaddr;                   // Address being fetched in the current ucode pipeline
reg [11:0] uc_addr;                 // Address of current uc (for debug)
reg [11:0] uc_addr_mem_r;           // Address aligned with the ROM q_mem stage
wire [50:0] uc;                     // Current microcode word + pre-computed bits (50:37)
wire [50:0] uc_next;

// Instruction lifecycle: D2 start -> D2 fire -> first EX -> RNI -> delay slot.
// Keep i_entry/i_pop as waveform-compatible names during the v53 migration.
wire       i_entry;                 // Normal (non-chained) D2 start
wire       i_pop;                   // Legacy alias of d2_fire
reg        i_first;                 // First ucode execution cycle after i_pop
wire       i_rni;                   // RNI detected in this cycle (combinational from uc bits)
reg        i_rni_delay;             // RNI delay slot - RNI has been executed. this is last instruction cycle

wire       d2_start;                // Launch a macro entry address into the ROM
wire [11:0] d2_start_entry;         // Entry address selected on d2_start
wire       d2_valid;                // A launched macro entry is resident in D2
wire       d2_ready;                // D2 may transfer its instruction to EX
wire       d2_fire;                 // D2 -> EX transfer; instruction state latches once
reg        d2_valid_r;
reg        d2_waited_r;             // D2 held after predecessor delay slot
reg        d2_stale_slot_r;         // D2 also waited during predecessor delay slot
reg        d2_rom_mem_r;            // D2 entry tag aligned with ROM q_mem
reg        d2_rom_q_r;              // D2 entry tag aligned with execution q
reg        d2_launch_id_r;          // toggles for every macro entry launch
reg        d2_id_r;                 // generation owned by resident D2
reg        d2_rom_mem_id_r;
reg        d2_rom_q_id_r;
reg [2:0]  d2_rom_q_kind_r;
reg        d2_slot_prefetched_r;    // q_mem holds entry+1 while D2 q is held
wire       init_cycle = d2_valid_r; // Temporary waveform alias; not control logic
reg        uc_active;               // Tracks when instruction execution has begun
reg        fault_suppress_delay_slot;   // Fault handling: suppress delay slot after fault triggers

// z386x FAST path (doc/z386x/design.md): a FAST instruction completes in its
// entry microcode word; hardware commits the result in that same cycle, so the
// RNI delay slot carries no work and its (stale) ROM word must not execute.
reg        fast_i_r;                // `i` holds a FAST-class instruction (latched at i_pop)
reg        fast_multi_r;            // ... and its RNI word is not the entry word
reg        fast_jcc_r;              // ... and it is a Jcc (chain on not-taken only)
reg        fast_wf_r;               // ... and it produces arithmetic flags
reg [2:0]  fast_commit_sel_r;       // FAST sideband select (FAST_COMMIT_*) at the RNI-word cycle
reg        fast_keep_slot_r;        // memory class: the slot word is real work when unchained
reg        branch_ustep_r;          // Relative JMP/Jcc uses one bounded EX ustep
reg        branch_ustep_jcc_r;      // The bounded branch evaluates condition code
reg        fast_memc_pending;       // deferred MEM commit: write OPR_R to a GPR this cycle
reg [2:0]  fast_memc_dst;           //   destination selector (dst_reg_sel of the load)
reg [1:0]  fast_memc_size;          //   operand size of the load
reg        fast_shc_pending;        // deferred SHIFT commit: write the latched
reg [2:0]  fast_shc_dst;            //   shifter result to a GPR this cycle
reg [1:0]  fast_shc_size;           //   (see the fast_shc_* block for why)
reg [31:0] fast_shc_data;
reg        fast_dead_slot;          // delay slot of a FAST instruction: suppress uc_exec
reg        fast_off = 1'b0;         // sim chicken bit: +z386x_fast_off forces all-SEQ (21.z386 behavior)
// synthesis translate_off
initial if ($test$plusargs("z386x_fast_off")) fast_off = 1'b1;
// synthesis translate_on
wire       fast_active;             // executing a FAST instruction's entry word this cycle
wire       fast_last;               // executing a FAST instruction's RNI word this cycle

// Empty D2 may launch directly from D1. Otherwise the resident D2 skeleton
// supplies a registered entry address when the current instruction ends.
wire       d1_issue_direct;
wire [11:0] d1_issue_entry_point;
dec_entry_t d1_issue_entry;
wire       i_entry_raw = (i_rni || i_rni_delay || ~uc_active) && ~halted && !stall &&
                         (d1_issue_direct || !decq_empty) && !q_flush && !d2_valid &&
                         !fault_suppress_delay_slot && !interrupt_entry;
assign     i_entry = i_entry_raw && !any_fault_pop;

// D2 -> EX readiness. Keep this factored from the valid bit so v53 can move
// D2 ownership without changing the meaning of the transfer edge.
wire       interrupt_deliverable = nmi_request_active ||
                                   (intr_pending && EFLAGS[9] && !inhibit_interrupts);
wire       interrupt_at_boundary = i_rni_delay && interrupt_deliverable && !single_step;
assign     d2_valid = d2_valid_r;
assign     d2_ready = d2_push && !stall && !any_fault_pop &&
                      !interrupt_at_boundary && !q_flush;
assign     d2_fire  = d2_valid && d2_ready;
assign     i_pop    = d2_fire;

wire       core_live = !halted && uc_active && !fault_suppress_delay_slot && !interrupt_entry;
wire       dly_grace_now = mem_dly_grace && uc_p_pure_dly;
wire       posted_write_release = mem_write_dly_grace && !uc_busreq;    // release non-busop writes after one cycle
wire       mem_block_busy = (uc_bus_or_dly && !dly_grace_now && !posted_write_release) ||
                            mem_opt_wait || mem_write_wait; // demand op in flight
wire       mem_block_idle = (uc_busreq && !mem_accepted);  // uop wants the bus, paging not ready
wire       stall_mem = mem_servicing ? mem_block_busy : (mem_req_current && !mem_accepted);
wire       stall_wio = (uc_is_wio && !interrupt_pending && !single_step);
// An entry may reach the ROM before its D2 literals arrive. Let the ending
// instruction execute its architectural RNI delay slot, then hold the ROM word
// until D2 can transfer it to EX.
wire       stall_d2 = d2_valid && !d2_push && !i_rni_delay;
wire       stall = stall_mem || stall_wio || stall_d2;

// Repeat
wire       prot_result_now = prot_result_valid && prot_test_inflight;
wire       repeat_active = uc_is_rpt && (COUNTR[4:0] != 0 || prot_test_inflight) && !prot_result_now
                           && !(uc_is_wio && interrupt_pending);

// uc_exec: master enable for microcode execution
wire       d2_release_hold = d2_valid && (d2_waited_r || d2_stale_slot_r);
wire       uc_exec = core_live && !(mem_servicing ? mem_block_busy : mem_block_idle) &&
                     !stall_wio && !stall_d2 && !d2_release_hold && !fast_dead_slot;
wire       uc_exec_writeback = uc_exec;  // local copies for reducing fanout
wire       uc_exec_mul_start = uc_exec;
wire       uc_exec_result = uc_exec;
wire       uc_exec_shift = uc_exec;

assign     seg_cmd_valid = i_pop || uc_exec;

dec_entry_t i_bus;            // Instruction resident in unified D2
wire       decq_has_jmp_call; // D1/D2 holds a JMP/CALL rel (halt speculative prefetch)
wire       decq_empty;        // Legacy name: unified D2 has no skeleton
dec_entry_t i_bus2;           // Registered D1 skid successor
dec_entry_t i;                // Current instruction (latched at i_pop; written far below)
wire       decq_has2;         // i_bus2 is valid

// Combinational EA-operand decode shared by resident D2 and skid successor.
typedef struct packed {
    logic [7:0]  base_sel;    // onehot GPR selector (bit i = GPR i)
    logic [7:0]  index_sel;   // onehot GPR selector
    logic [1:0]  scale;
    logic [31:0] disp;
    logic        is16;
    logic        s2b;         // SIB scale-to-base (no index)
} ea_dec_t;

dec_entry_t d2_entry;         // entry completing D2 this cycle (AGU/i_entry source)
ea_dec_t    d2_agu_dec;       // EA decode for d2_entry

// z386x FAST->FAST chaining: while popping a FAST instruction
fast_class_t fast_pop_fc, fast_next_fc, fast_head_fc;
assign     fast_pop_fc    = recipe_fast_class(i_bus);
assign     fast_next_fc   = recipe_fast_class(i_bus2);
assign     fast_head_fc   = fast_pop_fc;   // resident D2 successor during i_first
wire       fast_pop_fast  = fast_pop_fc.fast && !fast_off;
// reads_flags successors may chain when the predecessor writes -- doc/z386x/core_notes_v51.md #5
wire       fast_next_ok   = decq_has2 && fast_next_fc.fast &&
                            (!fast_next_fc.reads_flags || !fast_pop_fc.writes_flags ||
                             fast_next_fc.jcc) &&
                            (!fast_next_fc.uses_ea || !fast_ea1_conflict) &&
                            !fast_memc_conf1 && !fast_shc_conf1;
// Load-use gate: a MEM-commit load's value lands one cycle after a chained
// successor reads its operands, so never chain a dst-sourcing successor out
// of a load. Conservative reg-index overlap (byte regs match on [1:0]).
function automatic logic fast_gpr_overlap(input [2:0] a, input logic a_byte,
                                          input [2:0] b, input logic b_byte);
    if (!a_byte && !b_byte)      fast_gpr_overlap = (a == b);
    else if (a_byte && b_byte)   fast_gpr_overlap = (a[1:0] == b[1:0]);
    else if (a_byte)             fast_gpr_overlap = !b[2] && (a[1:0] == b[1:0]);
    else                         fast_gpr_overlap = !a[2] && (b[1:0] == a[1:0]);
endfunction
wire       fast_load_dst_byte = (op_size == 2'd0);
// Successor operand byte-ness must be exact: byte selectors encode
// {high,reg[1:0]}, so a wrong guess can MISS a real overlap (e.g. AH vs
// EAX). Precomputed in D1 (fc.op_byte).
wire       head_byte_op = fast_pop_fc.op_byte;
wire       fast_loaduse_conflict =
               (fast_head_fc.reads_dst &&
                fast_gpr_overlap(i.dst_reg_sel, fast_load_dst_byte, i_bus.dst_reg_sel, head_byte_op)) ||
               (fast_head_fc.reads_src &&
                fast_gpr_overlap(i.dst_reg_sel, fast_load_dst_byte, i_bus.src_reg_sel, head_byte_op)) ||
               (fast_head_fc.reads_ecx &&
                fast_gpr_overlap(i.dst_reg_sel, fast_load_dst_byte, 3'd1, 1'b0));
wire       fast_head_ok   = !decq_empty && d2_push && fast_head_fc.fast &&
                            (!fast_head_fc.reads_flags || !fast_wf_r ||
                             fast_head_fc.jcc) &&
                            (!fast_head_fc.uses_ea || !fast_ea2_conflict) &&
                            !((fast_commit_sel_r == FAST_COMMIT_MEM ||
                               fast_commit_sel_r == FAST_COMMIT_SHIFT) && fast_loaduse_conflict) &&
                            !fast_memc_confN && !fast_shc_confN;
// One-word FAST: the RNI word executes one cycle after i_pop
wire       fast_uc_next_rni = (uc_next[10:8] == 3'b000);
wire jcc_pop_unsafe = uc_exec && ((uc_aluop == ALUJMP_SHIFT2) ||
                                  (uc_aluop == ALUJMP_SEZF));
wire alu_wf_now = uc_exec && alu_update_flags;
// Z/S/P from the ALU's dedicated z
wire ahead_sf = alu_zsp_ahead[2];
wire ahead_zf = alu_zsp_ahead[1];
wire ahead_pf = alu_zsp_ahead[0];
wire [31:0] eflags_ahead = !alu_wf_now ? eflags_fwd : {
    eflags_fwd[31:12],
    alu_flags[11],                                   // OF
    eflags_fwd[10:8],
    alu_zsp_update ? ahead_sf : eflags_fwd[7],       // SF
    alu_zsp_update ? ahead_zf : eflags_fwd[6],       // ZF
    eflags_fwd[5],
    alu_flags[4],                                    // AF
    eflags_fwd[3],
    alu_zsp_update ? ahead_pf : eflags_fwd[2],       // PF
    eflags_fwd[1],
    alu_flags[0]};                                   // CF (incl. preserve)
// The FOLD's decision is consumed LIVE by fast_issue1 -> uaddr
wire       jcc_fold_unsafe = alu_wf_now || jcc_pop_unsafe;
wire       jcc_fold_now = i_pop && fast_pop_fast && fast_pop_fc.jcc &&
                          !jcc_fold_unsafe &&
                          !check_condition(i_bus.opcode[3:0], eflags_fwd);
reg        jcc_fold_r;
wire       jcc_fold_active = jcc_fold_r && i_first;   // the folded 065 cycle (level through stalls)
always_ff @(posedge clk) begin
    if (!reset_n)
        jcc_fold_r <= 1'b0;
    else if (q_flush || interrupt_entry || any_fault)
        jcc_fold_r <= 1'b0;
    else if (i_pop)
        jcc_fold_r <= jcc_fold_now;
    else if (!stall)
        jcc_fold_r <= 1'b0;
end
// synthesis translate_off
// The pop-time fold decision and the 065-cycle evaluation must agree: a
// mismatch means a flag write escaped the jcc_fold_unsafe gate.
always @(posedge clk)
    if (reset_n && jcc_fold_active && uc_exec && br_jcc_taken)
        $display("%0t JCC-FOLD MISMATCH: pop said not-taken, 065 says taken. opcode=%02x EFLAGS=%08x",
                 $time, i.opcode, EFLAGS);
// synthesis translate_on

// Jcc: chain fires at i_first (065) only when the settled-EFLAGS condition resolves NOT taken
wire       fast_issue1    = i_pop && fast_pop_fast &&
                            ((!fast_pop_fc.multi_word && !fast_pop_fc.jcc) ||
                             jcc_fold_now) && fast_next_ok;
wire       fast_issueN    = fast_i_r && fast_multi_r && uc_exec && fast_uc_next_rni &&
                            !i_pop && !i_rni_delay && !q_flush && fast_head_ok;
// jcc_pop_cond_v_r && !jcc_pop_taken_r: the registered pop-time NOT-taken
// decision (see its definition near early_redirect) - keeps the live
// eflags_fwd condition cone out of the chain/uaddr paths. Pop-invalid Jcc
// simply doesn't chain (normal boundary).
wire       fast_issueJ    = fast_active && fast_jcc_r && uc_exec &&
                            jcc_pop_cond_v_r && !jcc_pop_taken_r &&
                            !q_flush && fast_head_ok && !jcc_fold_r;
wire       fast_issue     = (fast_issue1 || fast_issueN || fast_issueJ) &&
                            (!d2_valid || d2_fire) &&
                            !d2_waited_r &&
                            !interrupt_pending && !single_step && !any_fault_pop;

// A normal boundary uses i_entry. FAST chaining launches the successor on the
// predecessor's transfer/execute edge and may overlap d2_fire.
assign d2_start = i_entry || fast_issue;
assign d2_start_entry = fast_issue
                      ? (fast_issue1 ? i_bus2.entry_point : i_bus.entry_point)
                      : (decq_empty ? d1_issue_entry_point : i_bus.entry_point);

// The physical microcode ROM contains 37-bit native words plus the v52
// three-bit early kind.  The execution bus remains native ucode + 14-bit
// predecode.
// Track the D2 entry through both synchronous ROM stages. The generation bit
// distinguishes a replacement D2 from the predecessor word still in q.
wire        d2_rom_mem_resident = d2_rom_mem_r &&
                                  (d2_rom_mem_id_r == d2_id_r);
wire        d2_rom_resident = d2_rom_q_r && (d2_rom_q_id_r == d2_id_r);
// q is EX-owned. A completed ROM lookup may wait in q_mem, but it advances
// into q only on the D2->EX transfer edge.
wire        d2_rom_cancel = interrupt_at_boundary || any_fault || any_fault_pop;
wire        d2_rom_hold = d2_valid && d2_rom_mem_resident && !d2_fire &&
                          !d2_rom_cancel;
wire        microcode_rom_base_ce = !stall_mem && !stall_wio && !repeat_active;
wire        d2_delay_preload = d2_valid && d2_rom_mem_resident && d2_fire &&
                               !d2_start && !d2_slot_prefetched_r;
wire        microcode_rom_addr_ce = microcode_rom_base_ce && !d2_rom_hold;
// A redirect replaces the parked lookup but must not expose the canceled word.
wire        microcode_rom_ce = microcode_rom_base_ce && !d2_rom_hold &&
                               !d2_rom_cancel;
// A branch may redirect uaddr in the same cycle that q_mem must preread the
// architectural delay slot. Keep the physical ROM address independent; the
// normal uaddr update below still advances loads and other multi-word entries.
wire [11:0] microcode_rom_addr = d2_delay_preload
                                ? (i_bus.entry_point + 12'd1)
                               : uaddr_now;
wire [50:0] uc_rom_q;
wire [50:0] uc_rom_early;
wire [2:0]  uc_kind_early;
(* noprune *) reg [2:0] early_kind_probe_r;
wire [5:0]  uc_source_shift;
wire [1:0]  uc_shift2_source;
wire [5:0]  uc_alu_src_shift;
wire [6:0]  uc_aluop_shift;
wire [2:0]  uc_dly_source;
wire [8:0]  uc_mem_ctrl;
assign uc = uc_rom_q;
assign uc_next = uc_rom_early;

ucode_rom microcode_rom (
    .clk(clk),
    .addr_ce(microcode_rom_addr_ce),
    .q_ce(microcode_rom_ce),
    .addr(microcode_rom_addr),
    .q_early(uc_rom_early),
    .q(uc_rom_q),
    .q_kind_early(uc_kind_early),
    .q_shift_source(uc_source_shift),
    .q_shift2_source(uc_shift2_source),
    .q_shift_alu_src(uc_alu_src_shift),
    .q_shift_aluop(uc_aluop_shift),
    .q_dly_source(uc_dly_source),
    .q_mem_ctrl(uc_mem_ctrl)
);

wire [2:0] d2_kind = d2_rom_resident ? d2_rom_q_kind_r : uc_kind_early;
always_ff @(posedge clk) begin
    if (!reset_n || q_flush) begin
        d2_rom_mem_r <= 1'b0;
        d2_rom_q_r <= 1'b0;
        d2_launch_id_r <= 1'b0;
        d2_id_r <= 1'b0;
        d2_rom_mem_id_r <= 1'b0;
        d2_rom_q_id_r <= 1'b0;
        d2_rom_q_kind_r <= 3'b000;
        d2_slot_prefetched_r <= 1'b0;
    end else begin
        if (microcode_rom_addr_ce) begin
            d2_rom_mem_r <= d2_start;
            if (d2_start) begin
                d2_launch_id_r <= !d2_launch_id_r;
                d2_id_r <= !d2_launch_id_r;
                d2_rom_mem_id_r <= !d2_launch_id_r;
            end
        end
        if (microcode_rom_ce) begin
            d2_rom_q_r <= d2_rom_mem_r;
            d2_rom_q_id_r <= d2_rom_mem_id_r;
            d2_rom_q_kind_r <= uc_kind_early;
        end
        if (d2_start) begin
            d2_slot_prefetched_r <= 1'b0;
        end else if (d2_delay_preload && microcode_rom_addr_ce) begin
            d2_slot_prefetched_r <= 1'b1;
        end else if (d2_fire) begin
            d2_slot_prefetched_r <= 1'b0;
        end
    end
    if (i_pop)
        early_kind_probe_r <= d2_kind;
end

// synthesis translate_off
// At i_pop, d2_kind is the annotation captured with the entry word. Prove it
// physical ROM annotation agrees with the optimizer-generated entry lookup;
// instruction legality comes from recipe_fast_class() and registered operands.
always @(posedge clk) begin
    if (reset_n && i_pop &&
        (d2_kind !== recipe_early_kind(i_bus.entry_point)))
        $fatal(1, "ucode early-kind mismatch: entry=%03x rom=%0d expected=%0d",
               i_bus.entry_point, d2_kind,
               recipe_early_kind(i_bus.entry_point));
end
// synthesis translate_on

// ROM1 decoder for instruction layout decoding
`include "pla_control.svh"

// Decoder23 PLA: Opcode → Microcode Entry Point
`include "pla_entry.svh"


//=============================================================================
// Unit 1: Prefetch queue and Bus Interface
//=============================================================================
// Forward declarations
wire [31:0] pf_flush_addr;          // Prefetch flush address
wire [5:0]  uc_buscode;             // Bus operation code from microcode
wire [6:0]  uc_dest;                // Destination field from microcode
wire [5:0]  uc_source;              // Source field from microcode
wire [31:0] dest_value;             // Destination value for writes
wire        gp_fault_trigger;       // GP fault trigger
wire        div_overflow;           // Division overflow

wire [11:0] prot_jump_addr;         // Microcode jump address from protection unit
wire        prot_jump_valid;        // jump_addr is a redirect (non-zero)
wire        prot_validation_ok;     // M flag: Descriptor validated
wire        prot_result_valid;      // Pipelined result is valid (2 cycles after test)

wire        dcache_req_valid;
wire [31:0] dcache_req_phys_addr_raw;
wire        dcache_req_write;
wire [3:0]  dcache_req_be;
wire [31:0] dcache_req_wdata;
wire        dcache_req_is_io;
wire        dcache_req_is_inta;
wire        dcache_req_accepted;
wire        dcache_req_complete;
wire [31:0] dcache_rdata;
wire        icache_req_valid;
wire [31:0] icache_req_phys_addr_raw;
wire        icache_req_accepted;
wire        icache_req_complete;
wire [127:0] icache_rdata;

// A20 gate (bit-20 mask) applied BEFORE the L1 caches
wire [31:0] dcache_req_phys_addr = (!a20_enable && !dcache_req_is_io)
                                 ? (dcache_req_phys_addr_raw & ~32'h0010_0000)
                                 : dcache_req_phys_addr_raw;
wire [31:0] icache_req_phys_addr = !a20_enable
                                 ? (icache_req_phys_addr_raw & ~32'h0010_0000)
                                 : icache_req_phys_addr_raw;

wire [31:0] dcache_cpu_dout;
wire        dcache_cpu_ready;
wire        dcache_cpu_resp_valid;
wire [31:0] dcache_mem_addr;
wire [31:0] dcache_mem_din;
wire  [3:0] dcache_mem_be;
wire  [7:0] dcache_mem_burstcount;
wire        dcache_mem_valid;
wire        dcache_mem_write;
wire        dcache_mem_ready;
wire        dcache_mem_resp_valid;

wire [127:0] icache_cpu_line;
wire        icache_cpu_ready;
wire        icache_cpu_resp_valid;
wire [31:0] icache_mem_addr;
wire  [3:0] icache_mem_be;
wire  [7:0] icache_mem_burstcount;
wire        icache_mem_valid;
wire        icache_mem_ready;
wire        icache_mem_resp_valid;

reg   [7:0] dcache_rd_pending;
reg   [7:0] icache_rd_pending;
reg         dcache_cpu_rd_pending;
reg         icache_cpu_rd_pending;
reg         direct_rd_pending;

wire dcache_cpu_req = dcache_req_valid && !dcache_req_is_io && !dcache_req_is_inta;
wire dcache_direct_req = dcache_req_valid && (dcache_req_is_io || dcache_req_is_inta);
wire dcache_read_pending = (dcache_rd_pending != 8'd0);
wire icache_read_pending = (icache_rd_pending != 8'd0);
wire dcache_read_accept = dcache_cpu_req && !dcache_req_write && dcache_cpu_ready;
wire icache_read_accept = icache_req_valid && icache_cpu_ready;
wire dcache_read_done = dcache_cpu_resp_valid && (dcache_cpu_rd_pending || dcache_read_accept);
wire icache_read_done = icache_cpu_resp_valid && (icache_cpu_rd_pending || icache_read_accept);
wire ext_direct_req = dcache_direct_req && !direct_rd_pending &&
                      !dcache_read_pending && !icache_read_pending;
wire ext_dcache_req = dcache_mem_valid && !ext_direct_req &&
                      !direct_rd_pending && !icache_read_pending;
wire ext_icache_req = icache_mem_valid && !ext_direct_req && !ext_dcache_req &&
                      !direct_rd_pending && !dcache_read_pending;

localparam [1:0] EXT_SRC_NONE   = 2'd0;
localparam [1:0] EXT_SRC_DIRECT = 2'd1;
localparam [1:0] EXT_SRC_DCACHE = 2'd2;
localparam [1:0] EXT_SRC_ICACHE = 2'd3;

reg        ext_valid_r;
reg [1:0]  ext_src_r;
reg [31:2] ext_addr_r;
reg [3:0]  ext_be_r;
reg [7:0]  ext_burstcount_r;
reg [31:0] ext_dout_r;
reg        ext_write_r;
reg        ext_io_r;
reg        ext_inta_r;

wire ext_direct_accept = ext_valid_r && ready && (ext_src_r == EXT_SRC_DIRECT);
wire ext_dcache_accept = ext_valid_r && ready && (ext_src_r == EXT_SRC_DCACHE);
wire ext_icache_accept = ext_valid_r && ready && (ext_src_r == EXT_SRC_ICACHE);
wire direct_rd_resp_now = resp_valid && (direct_rd_pending || (ext_direct_accept && !ext_write_r));
wire icache_write_snoop = dcache_cpu_req && dcache_req_write && dcache_cpu_ready;
reg        icache_write_snoop_pending;
reg [31:0] icache_write_snoop_addr_r;
reg [31:0] icache_write_snoop_data_r;
reg  [3:0] icache_write_snoop_be_r;
wire [31:0] icache_snoop_addr = snoop_valid ? snoop_addr : icache_write_snoop_addr_r;
wire icache_snoop_valid = snoop_valid || icache_write_snoop_pending;
wire [31:0] icache_snoop_data = snoop_valid ? 32'h0 : icache_write_snoop_data_r;
wire  [3:0] icache_snoop_be = snoop_valid ? 4'h0 : icache_write_snoop_be_r;
wire        icache_snoop_patch = !snoop_valid && icache_write_snoop_pending;

assign dcache_req_accepted = dcache_cpu_req ? dcache_cpu_ready : ext_direct_accept;
assign dcache_req_complete = dcache_cpu_resp_valid ||
                             (dcache_cpu_req && dcache_req_write && dcache_cpu_ready) ||
                             direct_rd_resp_now ||
                             (ext_direct_accept && ext_write_r);
assign dcache_rdata = dcache_cpu_resp_valid ? dcache_cpu_dout : din;
assign icache_req_accepted = icache_cpu_ready;
assign icache_req_complete = icache_cpu_resp_valid;
assign icache_rdata = icache_cpu_line;

assign addr       = ext_addr_r;
assign be         = ext_be_r;
assign burstcount = ext_burstcount_r;
assign dout       = ext_dout_r;
assign valid      = ext_valid_r;
assign write      = ext_valid_r && ext_write_r;
assign io         = ext_valid_r && ext_io_r;
assign inta       = ext_valid_r && ext_inta_r;

assign dcache_mem_ready = ext_dcache_accept;
assign icache_mem_ready = ext_icache_accept;
assign dcache_mem_resp_valid = dcache_read_pending && resp_valid;
assign icache_mem_resp_valid = icache_read_pending && resp_valid;

always_ff @(posedge clk) begin
    if (!reset_n) begin
        ext_valid_r <= 1'b0;
        ext_src_r <= EXT_SRC_NONE;
        ext_addr_r <= 30'h0;
        ext_be_r <= 4'h0;
        ext_burstcount_r <= 8'h0;
        ext_dout_r <= 32'h0;
        ext_write_r <= 1'b0;
        ext_io_r <= 1'b0;
        ext_inta_r <= 1'b0;
        dcache_rd_pending <= 8'd0;
        icache_rd_pending <= 8'd0;
        dcache_cpu_rd_pending <= 1'b0;
        icache_cpu_rd_pending <= 1'b0;
        direct_rd_pending <= 1'b0;
        icache_write_snoop_pending <= 1'b0;
        icache_write_snoop_addr_r <= 32'h0;
        icache_write_snoop_data_r <= 32'h0;
        icache_write_snoop_be_r <= 4'h0;
    end else begin
        if (icache_write_snoop) begin
            // Keep dcache write finalization off the icache RAM write path.
            // The icache snoop is posted one cycle later.
            icache_write_snoop_pending <= 1'b1;
            icache_write_snoop_addr_r <= dcache_req_phys_addr;
            icache_write_snoop_data_r <= dcache_req_wdata;
            icache_write_snoop_be_r <= dcache_req_be;
        end else if (icache_write_snoop_pending && !snoop_valid) begin
            icache_write_snoop_pending <= 1'b0;
        end

        if (ext_valid_r) begin
            if (ready) begin
                ext_valid_r <= 1'b0;
                ext_src_r <= EXT_SRC_NONE;
            end
        end else if (ext_direct_req) begin
            ext_valid_r <= 1'b1;
            ext_src_r <= EXT_SRC_DIRECT;
            ext_addr_r <= dcache_req_phys_addr[31:2];
            ext_be_r <= dcache_req_be;
            ext_burstcount_r <= 8'd1;
            if (dcache_req_write)
                ext_dout_r <= dcache_req_wdata;
            ext_write_r <= dcache_req_write;
            ext_io_r <= dcache_req_is_io;
            ext_inta_r <= dcache_req_is_inta;
        end else if (ext_dcache_req) begin
            ext_valid_r <= 1'b1;
            ext_src_r <= EXT_SRC_DCACHE;
            ext_addr_r <= dcache_mem_addr[31:2];
            ext_be_r <= dcache_mem_be;
            ext_burstcount_r <= dcache_mem_burstcount;
            if (dcache_mem_write)
                ext_dout_r <= dcache_mem_din;
            ext_write_r <= dcache_mem_write;
            ext_io_r <= 1'b0;
            ext_inta_r <= 1'b0;
        end else if (ext_icache_req) begin
            ext_valid_r <= 1'b1;
            ext_src_r <= EXT_SRC_ICACHE;
            ext_addr_r <= icache_mem_addr[31:2];
            ext_be_r <= icache_mem_be;
            ext_burstcount_r <= icache_mem_burstcount;
            ext_dout_r <= 32'h0;
            ext_write_r <= 1'b0;
            ext_io_r <= 1'b0;
            ext_inta_r <= 1'b0;
        end

        if (ext_dcache_accept && !dcache_mem_write)
            dcache_rd_pending <= dcache_mem_burstcount;
        else if (dcache_read_pending && resp_valid)
            dcache_rd_pending <= dcache_rd_pending - 8'd1;

        if (ext_icache_accept)
            icache_rd_pending <= icache_mem_burstcount;
        else if (icache_read_pending && resp_valid)
            icache_rd_pending <= icache_rd_pending - 8'd1;

        if (dcache_read_accept && !dcache_read_done)
            dcache_cpu_rd_pending <= 1'b1;
        else if (dcache_read_done)
            dcache_cpu_rd_pending <= 1'b0;

        if (icache_read_accept && !icache_read_done)
            icache_cpu_rd_pending <= 1'b1;
        else if (icache_read_done)
            icache_cpu_rd_pending <= 1'b0;

        if (ext_direct_accept && !ext_write_r && !resp_valid)
            direct_rd_pending <= 1'b1;
        else if (direct_rd_pending && resp_valid)
            direct_rd_pending <= 1'b0;
    end
end

l1_cache #(
    .PROTECT_UMA_ROM(PROTECT_UMA_ROM),
    .SET_BITS(DCACHE_SET_BITS)
) dcache_inst (
    .clk(clk),
    .reset(!reset_n),

    .cpu_addr(dcache_req_phys_addr),
    .cpu_din(dcache_req_wdata),
    .cpu_dout(dcache_cpu_dout),
    .cpu_be(dcache_req_be),
    .cpu_valid(dcache_cpu_req),
    .cpu_write(dcache_req_write),
    .cpu_ready(dcache_cpu_ready),
    .cpu_resp_valid(dcache_cpu_resp_valid),

    .mem_addr(dcache_mem_addr),
    .mem_din(dcache_mem_din),
    .mem_dout(din),
    .mem_be(dcache_mem_be),
    .mem_burstcount(dcache_mem_burstcount),
    .mem_busy(ext_valid_r || ext_direct_req || direct_rd_pending || icache_read_pending),
    .mem_valid(dcache_mem_valid),
    .mem_write(dcache_mem_write),
    .mem_ready(dcache_mem_ready),
    .mem_resp_valid(dcache_mem_resp_valid),

    .snoop_addr(snoop_addr),
    .snoop_valid(snoop_valid),
    .cache_enable(1'b1)
);

l1_icache #(
    .SET_BITS(ICACHE_SET_BITS)
) icache_inst (
    .clk(clk),
    .reset(!reset_n),

    .cpu_addr(icache_req_phys_addr),
    .cpu_line(icache_cpu_line),
    .cpu_valid(icache_req_valid),
    .cpu_ready(icache_cpu_ready),
    .cpu_resp_valid(icache_cpu_resp_valid),

    .mem_addr(icache_mem_addr),
    .mem_dout(din),
    .mem_be(icache_mem_be),
    .mem_burstcount(icache_mem_burstcount),
    .mem_busy(ext_valid_r || ext_direct_req || direct_rd_pending || dcache_read_pending || dcache_mem_valid),
    .mem_valid(icache_mem_valid),
    .mem_ready(icache_mem_ready),
    .mem_resp_valid(icache_mem_resp_valid),

    .snoop_addr(icache_snoop_addr),
    .snoop_data(icache_snoop_data),
    .snoop_be(icache_snoop_be),
    .snoop_patch(icache_snoop_patch),
    .snoop_valid(icache_snoop_valid),
    .cache_enable(1'b1)
);

// Prefetch Unit: 16-byte circular buffer
prefetch prefetch_inst (
    .clk(clk),
    .reset_n(reset_n),
    // Queue output to decoder
    .win_d1(win_d1),
    .d1_avail(d1_avail),
    .d1_adv(d1_adv),
    .win_lit(win_lit),
    .lit_avail(lit_avail),
    .lit_off(dec_lit_off),
    .q_full(pf_full),
    .pop_now(dec_pop_now),
    .pop_len(dec_pop_len),
    // Flush
    .q_flush(q_flush),
    .pf_flush_addr(pf_flush_addr),
    // Toggle interface to paging unit
    .pf_req_toggle(pf_req_toggle),
    .pf_linear_addr(pf_linear_addr),
    .pf_redirect_queued(pf_redirect_queued),
    .pf_ack_toggle(pf_ack_toggle),
    .pf_rdata(pf_rdata),
    .pf_fault(pf_fault),
    // Control
    .pf_suspend(page_fault),
    .halt_speculative(decq_has_jmp_call),

    // z386x speculative branch-target line
    .spec_req(pf_spec_req),
    .spec_linear(spec_target_lin),
    .spec_owner(pf_spec_owner_r),
    .spec_store_valid(pf_spec_store),
    .spec_store_linear(pf_spec_store_linear),
    .spec_global_kill(pf_spec_global_kill)
);

// z386x speculative branch-target fetch
wire        spec_br_rel8   = !i_bus.has_0f && (i_bus.opcode[7:4] == 4'h7 || i_bus.opcode == 8'hEB);
wire [31:0] spec_disp      = spec_br_rel8 ? {{24{i_bus.displacement[7]}}, i_bus.displacement[7:0]}
                                          : i_bus.displacement;
// Stale-EIP pop guard: a pop CHAINED into a control transfer's
wire        spec_eip_stale = uc_exec && fast_last && (uc_dest == DEST_eIP);
wire [31:0] spec_target_eip = EIP + ({27'd0, i_bus.length} + spec_disp);
wire [31:0] spec_target_lin = CS_base + spec_target_eip;
wire        pf_spec_req    = i_pop && fast_pop_fc.br_rel && i_bus.data32 && !fast_off &&
                             !spec_eip_stale;
// Ownership: set when an instruction's i_pop requests a spec fetch, cleared
// by any later pop, flush, or interrupt entry - so it is only up while the
// requesting branch itself is the current instruction, which is exactly when
// its taken-flush address provably equals the spec target.
reg pf_spec_owner_r;
always_ff @(posedge clk) begin
    if (!reset_n)
        pf_spec_owner_r <= 1'b0;
    else begin
        if (i_pop)
            pf_spec_owner_r <= pf_spec_req;
        if (q_flush || interrupt_entry || any_fault)
            pf_spec_owner_r <= 1'b0;
    end
end

// Store invalidation is line-selective inside prefetch. External coherence or
// an address-space change invalidates the speculative line conservatively.
wire        pf_spec_store;
wire [31:0] pf_spec_store_linear;
wire        pf_spec_global_kill = snoop_valid || cr3_write ||
                                  (uc_exec && (uc_dest == DEST_CR0));

//=============================================================================
// Unit 2: Decode1 (structural decode)
//=============================================================================
decoder decoder_inst (
    .clk        (clk),
    .reset_n    (reset_n),

    // Prefetch queue interface (two-cursor protocol)
    .win_d1     (win_d1),
    .d1_avail   (d1_avail),
    .d1_adv     (d1_adv),
    .win_lit    (win_lit),
    .lit_avail  (lit_avail),
    .lit_off    (dec_lit_off),
    .pop_now    (dec_pop_now),
    .pop_len    (dec_pop_len),

    // Mode signals
    .D          (D),
    .pe_enable  (pe & ~vm),  // Native protected mode: PE=1 and VM=0 (V86 uses real-mode entry points)

    // Control signals
    .q_flush    (q_flush),
    .i_pop      (i_pop),

    // Decoded instruction output
    .i_bus      (i_bus),
    .decq_empty (decq_empty),
    .i_bus2     (i_bus2),
    .decq_has2  (decq_has2),
    .decq_has_jmp_call(decq_has_jmp_call),

    // Unified D2 payload and registered D1 skid
    .d2_entry   (d2_entry),
    .d2_push    (d2_push),
    .d1_issue_direct(d1_issue_direct),
    .d1_issue_entry_point(d1_issue_entry_point),
    .d1_issue_entry(d1_issue_entry)
);

wire        d2_push;

//=============================================================================
// Unit 3: Decode2 - literals capture and early-address (EA decode, relocate,
//                    and required forwarding to start memory operations at i_pop)
//=============================================================================

// Decode an entry's precomputed EA selectors for the head and chain targets.
function automatic ea_dec_t ea_decode_of(input dec_entry_t e);
    ea_dec_t r;
    // Defaults (also the "no modrm / has moffs" case)
    r = '0;
    // base/index onehot selectors are precomputed in D1 (decq-registered)
    r.base_sel  = e.ea_base_onehot;
    r.index_sel = e.ea_index_onehot;
    if (e.has_modrm && !e.has_moffs) begin
        if (e.addr32) begin
            // 32-bit addressing mode
            r.scale     = e.has_sib ? e.sib[7:6] : 2'b00;
            r.s2b       = e.has_sib && (e.sib[5:3] == 3'b100);  // No index, scale to base
        end else begin
            // 16-bit addressing mode
            r.is16      = 1'b1;
        end

        // D2 literal capture has already normalized disp8 and leaves this zero
        // for addressing forms without a displacement.
        r.disp = e.displacement;

        // POP r/m (8F) with ESP base: Intel 386 says EA uses post-increment ESP.
        if (e.opcode == 8'h8F && e.addr32 && e.has_sib && e.sib[2:0] == 3'b100)
            r.disp = e.displacement + (e.data32 ? 32'd4 : 32'd2);
    end
    ea_decode_of = r;
endfunction

ea_dec_t ea_dec_cur;    // queue head (i_entry latch source, chain2 target)
assign ea_dec_cur = ea_decode_of(i_bus);

// z386x chain-INTO memory/LEA (EA reg-match): the target's i_p
ea_dec_t fast_ea_dec2;  // chain1 target = the entry behind the head
assign fast_ea_dec2 = ea_decode_of(i_bus2);
function automatic [2:0] fast_wide_widx(input [2:0] sel, input logic is_byte);
    fast_wide_widx = is_byte ? {1'b0, sel[1:0]} : sel;   // byte regs live in GPR 0-3
endfunction

function automatic logic fast_ea_conflict(
    input logic we, input logic [2:0] widx,
    input ea_dec_t ea, input dec_entry_t entry, input logic ignore_esp
);
    fast_ea_conflict = we &&
        (ea.base_sel[widx] || ea.index_sel[widx] ||
         (entry.stack_op && (widx == 3'd4) && !ignore_esp));
endfunction

// chain1 predecessor = the popping one-word instruction
wire        fast_pred1_we   = (fast_pop_fc.commit_sel == FAST_COMMIT_ALU) ||
                              (fast_pop_fc.commit_sel == FAST_COMMIT_ESP) ||
                              fast_pop_fc.writes_srcreg;
wire [2:0]  fast_pred1_widx = (fast_pop_fc.commit_sel == FAST_COMMIT_ESP) ? 3'd4 :
                fast_wide_widx(
                fast_pop_fc.writes_srcreg ? i_bus.src_reg_sel : i_bus.dst_reg_sel,
                !fast_pop_fc.writes_srcreg && fast_pop_fc.op_byte);
wire        fast_ea1_conflict = fast_ea_conflict(
                fast_pred1_we, fast_pred1_widx, fast_ea_dec2, i_bus2,
                fast_pop_fc.commit_sel == FAST_COMMIT_ESP);
// chainN predecessor = the executing multi-word instruction (`i`); its
// commit lands at fast_last (or one cycle later for a load's deferred MEM
// commit), so any committing predecessor gates. SIGSRC (MOVZX/CBW) writes
// SRCREG, always word/dword.
wire        fast_pred2_we   = (fast_commit_sel_r != FAST_COMMIT_NONE);
wire [2:0]  fast_pred2_widx = (fast_commit_sel_r == FAST_COMMIT_SIGSRC)
                              ? i.src_reg_sel
                              : fast_wide_widx(i.dst_reg_sel, op_size == 2'd0);
wire        fast_ea2_conflict = fast_ea_conflict(
                fast_pred2_we, fast_pred2_widx, ea_dec_cur, i_bus, 1'b0);
// In-flight deferred MEM (load/POP) commit gate -- doc/z386x/core_notes_v51.md #15
wire        fast_memc_set  = fast_last && uc_exec && i_pop && !any_fault_pop &&
                             (fast_commit_sel_r == FAST_COMMIT_MEM);
wire        fast_memc_haz  = fast_memc_set || fast_memc_pending;
wire [2:0]  fast_memc_hreg = fast_memc_set ? i.dst_reg_sel : fast_memc_dst;
wire [1:0]  fast_memc_hsz  = fast_memc_set ? op_size        : fast_memc_size;
wire [2:0]  fast_memc_widx = fast_wide_widx(fast_memc_hreg, fast_memc_hsz == 2'd0);
function automatic logic fast_hazard_uses(
    input ea_dec_t ea, input dec_entry_t entry, input fast_class_t fc,
    input logic haz, input logic [2:0] widx, input logic [2:0] hreg,
    input logic [1:0] hsz
);
    fast_hazard_uses = haz &&
        (ea.base_sel[widx] || ea.index_sel[widx] ||
         (fc.reads_dst && fast_gpr_overlap(hreg, hsz == 2'd0,
                                           entry.dst_reg_sel, fc.op_byte)) ||
         (fc.reads_src && fast_gpr_overlap(hreg, hsz == 2'd0,
                                           entry.src_reg_sel, fc.op_byte)) ||
         (fc.reads_ecx && fast_gpr_overlap(hreg, hsz == 2'd0, 3'd1, 1'b0)) ||
         (entry.stack_op && (widx == 3'd4)));
endfunction
wire        fast_memc_conf1 = fast_hazard_uses(
                fast_ea_dec2, i_bus2, fast_next_fc, fast_memc_haz, fast_memc_widx,
                fast_memc_hreg, fast_memc_hsz);  // chain1 successor
wire        fast_memc_confN = fast_hazard_uses(
                ea_dec_cur, i_bus, fast_pop_fc, fast_memc_haz, fast_memc_widx,
                fast_memc_hreg, fast_memc_hsz);  // chainN successor

// z386x deferred SHIFT commit (same pattern; see fast_memc abo -- doc/z386x/core_notes_v51.md #16
wire        fast_shc_set  = fast_last && uc_exec && i_pop && !any_fault_pop &&
                            (fast_commit_sel_r == FAST_COMMIT_SHIFT);
wire        fast_shc_haz  = fast_shc_set || fast_shc_pending;
wire [2:0]  fast_shc_hreg = fast_shc_set ? i.dst_reg_sel : fast_shc_dst;
wire [1:0]  fast_shc_hsz  = fast_shc_set ? op_size        : fast_shc_size;
wire [2:0]  fast_shc_widx = fast_wide_widx(fast_shc_hreg, fast_shc_hsz == 2'd0);
wire        fast_shc_conf1 = fast_hazard_uses(
                fast_ea_dec2, i_bus2, fast_next_fc, fast_shc_haz, fast_shc_widx,
                fast_shc_hreg, fast_shc_hsz);  // chain1 successor
wire        fast_shc_confN = fast_hazard_uses(
                ea_dec_cur, i_bus, fast_pop_fc, fast_shc_haz, fast_shc_widx,
                fast_shc_hreg, fast_shc_hsz);  // chainN successor
logic [7:0]  ea_dec_base_sel, ea_dec_index_sel;
logic [1:0]  ea_dec_scale;
logic [31:0] ea_dec_disp;
logic        ea_dec_is_16bit, ea_dec_scale_to_base;
assign ea_dec_base_sel      = ea_dec_cur.base_sel;
assign ea_dec_index_sel     = ea_dec_cur.index_sel;
assign ea_dec_scale         = ea_dec_cur.scale;
assign ea_dec_disp          = ea_dec_cur.disp;
assign ea_dec_is_16bit      = ea_dec_cur.is16;
assign ea_dec_scale_to_base = ea_dec_cur.s2b;

// z386x: at a chained PUSH's RNI cycle, SIGMA holds the post-push ESP
// (precomputed at its i_pop) - the minimal ESP tracker. push;push chains
// read it here for their own i_pop precompute and stack addressing.
wire        fast_esp_fwd = fast_last && (fast_commit_sel_r == FAST_COMMIT_ESP);

// RNI-slot architectural GPR writes use only these four sources in the Intel
// ROM. Keep this mux narrow: it feeds the next instruction's D2 EA bypass.
function automatic [31:0] dly_fwd_mux(input [2:0] s);
    case (s)
        3'd1:    dly_fwd_mux = SIGMA;
        3'd2:    dly_fwd_mux = OPR_R;
        3'd3:    dly_fwd_mux = COUNTR;
        3'd4:    dly_fwd_mux = 32'hFFFF_FFFF;
        default:    dly_fwd_mux = 32'h0;
    endcase
endfunction
wire [31:0] dly_fwd_value = dly_fwd_mux(uc_dly_source);

// synthesis translate_off
// Re-prove the narrow mux against the full source read on every delay-slot
// GPR write (the ROM can change; a new source field must be added here).
always @(posedge clk)
    if (reset_n && dly_gpr_we && uc_exec &&
        (dly_fwd_value !== ((uc_source == SRC_IRF2) ? IND : dest_value)))
        $fatal(1, "DLY-FWD MUX MISMATCH: uc_addr=%03x src=%02x narrow=%08x full=%08x",
               uc_addr, uc_source, dly_fwd_value, dest_value);
// synthesis translate_on

// Delay-slot GPR write descriptor for early-EA forwarding (which GPR the
// delay-slot uop writes, and how)
localparam [1:0] FWD_BLO = 2'd0, FWD_BHI = 2'd1, FWD_W = 2'd2, FWD_D = 2'd3;

// {we, sel[2:0], mode[1:0]} for a delay-slot write to microcode dest `dest`
function automatic [5:0] decode_dly_gpr(input [6:0] dest);
    reg       we; reg [2:0] sel; reg [1:0] mode; reg [2:0] rs;
    begin
        we = 1'b0; sel = 3'd0; mode = FWD_D;
        case (dest)
            DEST_DSTREG, DEST_SRCREG: begin
                rs = (dest == DEST_DSTREG) ? i.dst_reg_sel : i.src_reg_sel;
                we = 1'b1;
                if (op_size == 2'd0) begin               // byte: rs[2]=high-byte, rs[1:0]=GPR
                    sel  = {1'b0, rs[1:0]};
                    mode = rs[2] ? FWD_BHI : FWD_BLO;
                end else begin
                    sel  = rs;
                    mode = (op_size == 2'd1) ? FWD_W : FWD_D;
                end
            end
            DEST_EAX, DEST_ECX, DEST_EDX, DEST_EBX,
            DEST_ESP, DEST_EBP, DEST_ESI, DEST_EDI:
                begin we = 1'b1; sel = dest[2:0]; mode = FWD_D; end
            DEST_eSP:
                begin we = 1'b1; sel = 3'd4;
                      mode = (pe && seg_cache[SEG_SS].D_B) ? FWD_D : FWD_W; end
            DEST_AX, DEST_CX, DEST_DX, DEST_BX, DEST_SP, DEST_BP, DEST_SI, DEST_DI:
                begin we = 1'b1; sel = dest[2:0]; mode = FWD_W; end
            DEST_AL, DEST_CL, DEST_DL, DEST_BL:
                begin we = 1'b1; sel = {1'b0, dest[1:0]}; mode = FWD_BLO; end
            DEST_AH, DEST_CH, DEST_DH, DEST_BH:
                begin we = 1'b1; sel = {1'b0, dest[1:0]}; mode = FWD_BHI; end
            DEST_eAX_AL:
                begin we = 1'b1; sel = 3'd0;
                      mode = (op_size == 2'd0) ? FWD_BLO : (op_size == 2'd1) ? FWD_W : FWD_D; end
            DEST_eDX_AH: begin
                we = 1'b1;
                if (op_size == 2'd0) begin sel = 3'd0; mode = FWD_BHI; end  // AH
                else begin sel = 3'd2; mode = (op_size == 2'd1) ? FWD_W : FWD_D; end
            end
            DEST_eCX: begin we = 1'b1; sel = 3'd1; mode = i.addr32 ? FWD_D : FWD_W; end
            DEST_eSI: begin we = 1'b1; sel = 3'd6; mode = i.addr32 ? FWD_D : FWD_W; end
            DEST_eDI: begin we = 1'b1; sel = 3'd7; mode = i.addr32 ? FWD_D : FWD_W; end
            DEST_IRF: if (COUNTR[5:3] != 3'b100)
                begin we = 1'b1; sel = COUNTR[2:0]; mode = is_dword ? FWD_D : FWD_W; end
            default: ;
        endcase
        decode_dly_gpr = {we, sel, mode};
    end
endfunction

// Predecode from uc_next (the microword that becomes uc next cycle); register on
// the same enable as uc so dly_gpr_*_pre_r tracks decode_dly_gpr(uc_dest).
wire [5:0] dly_gpr_pre   = decode_dly_gpr(uc_next[30:24]);
wire       dly_is_irf_pre = (uc_next[30:24] == DEST_IRF);
reg        dly_gpr_we_pre_r;
reg [2:0]  dly_gpr_sel_pre_r;
reg [1:0]  dly_gpr_mode_pre_r;
reg        dly_is_irf_pre_r;
always_ff @(posedge clk) begin
    if (!reset_n) begin
        dly_gpr_we_pre_r <= 1'b0; dly_gpr_sel_pre_r <= 3'd0;
        dly_gpr_mode_pre_r <= FWD_D; dly_is_irf_pre_r <= 1'b0;
    end else if (microcode_rom_ce) begin
        dly_gpr_we_pre_r   <= dly_gpr_pre[5];
        dly_gpr_sel_pre_r  <= dly_gpr_pre[4:2];
        dly_gpr_mode_pre_r <= dly_gpr_pre[1:0];
        dly_is_irf_pre_r   <= dly_is_irf_pre;
    end
end

// Functional descriptor: registered predecode, IRF index taken live from COUNTR.
// !fast_dead_slot: a FAST instruction's slot word is stale and writes nothing;
// its result committed at the entry-word cycle, so the register file is current.
wire       dly_gpr_we   = i_rni_delay && !fast_dead_slot &&
                          (dly_is_irf_pre_r ? (COUNTR[5:3] != 3'b100) : dly_gpr_we_pre_r);
wire [2:0] dly_gpr_sel  = dly_is_irf_pre_r ? COUNTR[2:0]              : dly_gpr_sel_pre_r;
wire [1:0] dly_gpr_mode = dly_is_irf_pre_r ? (is_dword ? FWD_D : FWD_W) : dly_gpr_mode_pre_r;

wire       dly_esp_fwd = dly_gpr_we && (dly_gpr_sel == 3'd4);
wire       shc_esp_fwd = fast_shc_pending && (fast_shc_dst == 3'd4) &&
                         (fast_shc_size != 2'd0);
wire [31:0] forwarded_esp = fast_esp_fwd ? SIGMA :
                            dly_esp_fwd  ? dly_fwd_value :
                            shc_esp_fwd  ? (fast_shc_size == 2'd1
                                            ? {ESP[31:16], fast_shc_data[15:0]}
                                            : fast_shc_data) : ESP;

// EA decode registered at i_entry
reg [7:0]  ea_dec_base_sel_r, ea_dec_index_sel_r;
reg [2:0]  ea_dec_base_idx_r, ea_dec_index_idx_r;  // priority-encoded at the latch
reg [1:0]  ea_dec_scale_r;
reg        ea_dec_base_valid_r, ea_dec_index_valid_r;
reg        ea_dec_is_16bit_r, ea_dec_scale_to_base_r;
function automatic [2:0] onehot_idx(input [7:0] oh);
    onehot_idx = oh[1] ? 3'd1 : oh[2] ? 3'd2 : oh[3] ? 3'd3 : oh[4] ? 3'd4 :
                 oh[5] ? 3'd5 : oh[6] ? 3'd6 : oh[7] ? 3'd7 : 3'd0;
endfunction
// D1 supplies only registered selectors to D2. Literal displacement remains a
// D2 value and is consumed from d2_entry when the instruction fires.
always_ff @(posedge clk) begin
    if (d2_start) begin
        automatic ea_dec_t sel = (fast_issue && fast_issue1) ? fast_ea_dec2 :
                                 (i_entry && decq_empty)     ? ea_decode_of(d1_issue_entry) :
                                                               ea_dec_cur;
        ea_dec_base_sel_r      <= sel.base_sel;
        ea_dec_index_sel_r     <= sel.index_sel;
        ea_dec_base_idx_r      <= onehot_idx(sel.base_sel);
        ea_dec_index_idx_r     <= onehot_idx(sel.index_sel);
        ea_dec_base_valid_r    <= |sel.base_sel;
        ea_dec_index_valid_r   <= |sel.index_sel;
        ea_dec_scale_r         <= sel.scale;
        ea_dec_is_16bit_r      <= sel.is16;
        ea_dec_scale_to_base_r <= sel.s2b;
    end
end

// Live i_pop EA path, SHARED between IND (offset) and ind_linear (offset + segment base)
wire [31:0] ea_fwd_base  = fwd_ea_gpr(ea_dec_base_valid_r,  ea_dec_base_idx_r);
wire [31:0] ea_fwd_index = fwd_ea_gpr(ea_dec_index_valid_r, ea_dec_index_idx_r);

wire [63:0] pop_ea_prep = calc_ea_prep(ea_fwd_base, ea_fwd_index,
                                       ea_dec_scale_r, ea_dec_scale_to_base_r);
wire [31:0] pop_ea_a = pop_ea_prep[63:32];  // base_val
wire [31:0] pop_ea_b = pop_ea_prep[31:0];   // scaled_val
wire [31:0] pop_ea_c = d2_agu_dec.disp;
wire [31:0] pop_ea_off = pop_ea_a + pop_ea_b + pop_ea_c;   // ternary (one ALM level)
wire [31:0] ea_early = ea_dec_is_16bit_r ? {16'h0, pop_ea_off[15:0]} : pop_ea_off;

// Fused linear -- doc/z386x/core_notes_v51.md #18
wire [31:0] pop_csa_s = pop_ea_a ^ pop_ea_b ^ pop_ea_c;
wire [31:0] pop_csa_c = ((pop_ea_a & pop_ea_b) | (pop_ea_a & pop_ea_c) |
                         (pop_ea_b & pop_ea_c)) << 1;
wire [31:0] pop_lin32 = pop_csa_s + pop_csa_c + seg_base_pending;
wire [31:0] pop_lin16 = {16'h0, pop_ea_off[15:0]} + seg_base_pending;
wire [31:0] pop_ind_linear = (ea_dec_is_16bit_r || !eff_mask_pending)
                           ? pop_lin16 : pop_lin32;
// synthesis translate_off
// The fused adder must equal the two-stage reference on every modrm capture.
always @(posedge clk)
    if (reset_n && i_pop && i_bus.has_modrm && !i_bus.stack_op && !i_bus.has_moffs &&
        (pop_ind_linear !== ((eff_mask_pending ? ea_early : {16'h0, ea_early[15:0]})
                            + seg_base_pending)))
        $fatal(1, "POP-LIN FUSE MISMATCH: fused=%08x ref=%08x ea=%08x seg=%08x m16=%b e16=%b",
               pop_ind_linear,
               (eff_mask_pending ? ea_early : {16'h0, ea_early[15:0]}) + seg_base_pending,
               ea_early, seg_base_pending, ea_dec_is_16bit_r, eff_mask_pending);
// synthesis translate_on

// D2-AGU observer
assign d2_agu_dec = ea_decode_of(d2_entry);
wire [31:0] d2_agu_base  = onehot_gpr_mux(d2_agu_dec.base_sel);
wire [31:0] d2_agu_index = onehot_gpr_mux(d2_agu_dec.index_sel);
wire [63:0] d2_agu_prep  = calc_ea_prep(d2_agu_base, d2_agu_index,
                                        d2_agu_dec.scale, d2_agu_dec.s2b);
wire [31:0] d2_agu_a = d2_agu_prep[63:32];
wire [31:0] d2_agu_b = d2_agu_prep[31:0];
wire [31:0] d2_agu_c = d2_agu_dec.disp;
wire [3:0] d2_agu_seg = apply_seg_override_type(
    calc_default_seg_type(d2_entry.modrm, d2_entry.sib, d2_entry.has_sib,
                          d2_entry.addr32), d2_entry.seg);
wire [31:0] d2_agu_segbase = seg_cache[d2_agu_seg].base;
wire [31:0] d2_agu_lin = (d2_agu_a ^ d2_agu_b ^ d2_agu_c)
                       + (((d2_agu_a & d2_agu_b) | (d2_agu_a & d2_agu_c) |
                           (d2_agu_b & d2_agu_c)) << 1)
                       + d2_agu_segbase;

// GPR-write snoop: one-hot of architectural GPRs written THIS cycle
function automatic [7:0] gpr_wr_expand(input [2:0] sel);
    gpr_wr_expand = (8'h1 << sel) | (8'h1 << {1'b0, sel[1:0]});
endfunction
// {inval_all, onehot} for the current uc word's destination
function automatic [8:0] gpr_dest_probe(input [6:0] dst);
    gpr_dest_probe = {1'b1, 8'h00};
    case (dst)
        DEST_EAX, DEST_AX, DEST_eAX_AL: gpr_dest_probe = {1'b0, 8'h01};
        DEST_ECX, DEST_CX, DEST_eCX:    gpr_dest_probe = {1'b0, 8'h02};
        DEST_EDX, DEST_DX:              gpr_dest_probe = {1'b0, 8'h04};
        DEST_EBX, DEST_BX:              gpr_dest_probe = {1'b0, 8'h08};
        DEST_ESP, DEST_SP, DEST_eSP:    gpr_dest_probe = {1'b0, 8'h10};
        DEST_EBP, DEST_BP:              gpr_dest_probe = {1'b0, 8'h20};
        DEST_ESI, DEST_SI, DEST_eSI:    gpr_dest_probe = {1'b0, 8'h40};
        DEST_EDI, DEST_DI, DEST_eDI:    gpr_dest_probe = {1'b0, 8'h80};
        DEST_DSTREG: gpr_dest_probe = {1'b0, gpr_wr_expand(i.dst_reg_sel)};
        DEST_SRCREG: gpr_dest_probe = {1'b0, gpr_wr_expand(i.src_reg_sel)};
        DEST_USTEP_ALU: gpr_dest_probe = {1'b0, gpr_wr_expand(i.dst_reg_sel)};
        DEST_IRF:    gpr_dest_probe = (COUNTR[5:3] != 3'b100)
                                    ? {1'b0, 8'h01 << COUNTR[2:0]}
                                    : {1'b1, 8'h00};   // seg selector row: clear all
        // Frequent, provably GPR/segment-free destinations: no-op.
        DEST_TMPB, DEST_TMPC, DEST_TMPD, DEST_TMPE, DEST_TMPF,
        DEST_TMPG, DEST_TMPH, DEST_TMPeIP, DEST_TMPeSP,
        DEST_OPR_W, DEST_OPROFF, DEST_CSOPCD, DEST_CR2,
        DEST_EIP, DEST_eIP, DEST_IP, DEST_EFLAGS, DEST_FLAGS,
        DEST_FLAGSB, DEST_FLAGSL, DEST_COUNTR, DEST_COUNT5,
        DEST_MDTMP, DEST_MDTMP4, DEST_PROTUN, DEST_SLCTR,
        DEST_TMP_TR, DEST_FSVeIP, DEST_DR6,
        7'h7F:                          gpr_dest_probe = 9'h0;
        default: ;
    endcase
endfunction
wire [8:0] d2_agu_ucprobe = uc_exec ? gpr_dest_probe(uc_dest) : 9'h0;
wire [7:0] ea_inval_gpr =
    d2_agu_ucprobe[7:0] |
    (fast_shc_pending ? gpr_wr_expand(fast_shc_dst) : 8'h0) |
    ((uc_exec && fast_memc_pending) ? gpr_wr_expand(fast_memc_dst) : 8'h0) |
    ((uc_exec && fast_last && !any_fault && fast_commit_sel_r == FAST_COMMIT_ALU)
        ? gpr_wr_expand(i.dst_reg_sel) : 8'h0) |
    ((uc_exec && fast_last && !any_fault && fast_commit_sel_r == FAST_COMMIT_SIGSRC)
        ? gpr_wr_expand(i.src_reg_sel) : 8'h0) |
    ((uc_exec && fast_last && !any_fault && fast_commit_sel_r == FAST_COMMIT_ESP)
        ? 8'h10 : 8'h0) |
    ((uc_exec && uc_aluop == ALUJMP_CLZF && i.has_0f && i.opcode == 8'hBD)
        ? gpr_wr_expand(i.src_reg_sel) : 8'h0);
// Clear-all events: segment state may change under any committed seg
// command or descriptor load; the effective-mask mode must be stable.
reg d2_agu_effmask_r;
always_ff @(posedge clk) d2_agu_effmask_r <= eff_mask_pending;
// Only cache-MUTATING segment commands clear the sidecars; INIT_SEG /
// UPDATE_SEG select which base to read (every memory pop issues one) and
// mutate nothing.
wire seg_cmd_mutates = (seg_cmd != SEG_CMD_NONE) &&
                       (seg_cmd != SEG_CMD_INIT_SEG) &&
                       (seg_cmd != SEG_CMD_UPDATE_SEG) &&
                       (seg_cmd != SEG_CMD_SPCR);
wire ea_inval_all = d2_agu_ucprobe[8] ||
                    (seg_cmd_valid && seg_cmd_mutates) ||
                    (d2_agu_effmask_r != eff_mask_pending);

// Eligibility (MVP): plain 32-bit MEMORY modrm EA (mod!=11), no
// moffs/stack, 32-bit mask active, and no conflicting write in the
// compute cycle itself.
wire d2_agu_valid = d2_push &&
                    d2_entry.has_modrm && (d2_entry.modrm[7:6] != 2'b11) &&
                    !d2_entry.has_moffs &&
                    !d2_entry.stack_op && d2_entry.addr32 &&
                    eff_mask_pending &&
                    (((d2_agu_dec.base_sel | d2_agu_dec.index_sel) & ea_inval_gpr) == 8'h00);
wire [31:0] head_ea_lin = d2_agu_lin;
wire        head_ea_v   = d2_agu_valid;

// Same-cycle conflict mask: a write committing at the CONSUMING edge
wire head_ea_usable = head_ea_v &&
    (((i_bus.ea_base_onehot | i_bus.ea_index_onehot) & ea_inval_gpr) == 8'h00) &&
    !ea_inval_all;

// synthesis translate_off
// +ldtrace: memory-instruction anatomy (pop/first/request/complete edges).
always @(posedge clk) if ($test$plusargs("ldtrace")) begin
    if (i_pop)   $display("%0t LD pop  op=%02x eav=%b", $time, i_bus.opcode, head_ea_usable);
    if (i_first) $display("%0t LD first op=%02x uaddr=%03x", $time, i.opcode, uc_addr);
    if (mem_req_to_paging && mem_accepted)
        $display("%0t LD req  wr=%b lin=%08x", $time, mem_write_now, ind_linear);
    if (dcache_req_complete)
        $display("%0t LD done", $time);
end
// Debug: catch GPR writes the snoop missed (+agu_dbg).
reg [31:0] dbg_esi_prev; reg [7:0] dbg_inval_prev; reg [6:0] dbg_ucdest_prev;
reg dbg_exec_prev;
always @(posedge clk) begin
    dbg_esi_prev <= ESI; dbg_inval_prev <= ea_inval_gpr;
    dbg_ucdest_prev <= uc_dest; dbg_exec_prev <= uc_exec;
    if ($test$plusargs("agu_dbg") && (ESI !== dbg_esi_prev) && !dbg_inval_prev[6])
        $display("%0t ESIWR-MISSED %08x->%08x prev_ucdest=%02x prev_exec=%b prev_inval=%02x uaddr=%03x",
                 $time, dbg_esi_prev, ESI, dbg_ucdest_prev, dbg_exec_prev,
                 dbg_inval_prev, uc_addr);
end
always @(posedge clk) if ($test$plusargs("agu_dbg")) begin
    if (d2_push)
        $display("%0t AGUDBG land op=%02x v=%b lin=%08x bs=%02x agubase=%08x ESI=%08x aguseg=%0d segbase=%08x disp=%08x",
                 $time, d2_entry.opcode, d2_agu_valid, d2_agu_lin,
                 d2_entry.ea_base_onehot, d2_agu_base, ESI, d2_agu_seg,
                 d2_agu_segbase, d2_agu_dec.disp);
    if (ea_inval_gpr != 8'h0 || ea_inval_all)
        $display("%0t AGUDBG inval gpr=%02x all=%b ucdest=%02x", $time,
                 ea_inval_gpr, ea_inval_all, uc_dest);
    if (i_pop)
        $display("%0t AGUDBG pop op=%02x headeav=%b headlin=%08x", $time,
                 i_bus.opcode, head_ea_v, head_ea_lin);
    if (ESI !== dbg_esi_prev)
        $display("%0t AGUDBG ESI %08x->%08x ucdest=%02x memc=%b dly_we=%b dly_sel=%0d",
                 $time, dbg_esi_prev, ESI, dbg_ucdest_prev, fast_memc_pending,
                 dly_gpr_we, dly_gpr_sel);
end
// THE 3a PROOF: a valid sidecar at pop must equal the pop-time compute.
always @(posedge clk)
    if (reset_n && i_pop && head_ea_usable && i_bus.has_modrm && !i_bus.stack_op &&
        !i_bus.has_moffs && !$isunknown({head_ea_lin, pop_ind_linear}) &&
        (head_ea_lin !== pop_ind_linear))
        $fatal(1, "D2-AGU MISMATCH: sidecar=%08x pop=%08x op=%02x modrm=%02x EIP=%08x a32=%b seg=%0d bs=%02x is=%02x effm=%b i_pop=%b fi=%b",
               head_ea_lin, pop_ind_linear, i_bus.opcode, i_bus.modrm, EIP,
               i_bus.addr32, i_bus.seg, i_bus.ea_base_onehot,
               i_bus.ea_index_onehot, eff_mask_pending, i_pop, fast_issue);
reg [31:0] d2agu_pops, d2agu_hits;
initial begin d2agu_pops = 0; d2agu_hits = 0; end
int eaf_base, eaf_idx, eaf_both, eaf_disp, eaf_none, eaf_16;
always @(posedge clk)
    if (reset_n && i_pop && i_bus.has_modrm && !i_bus.stack_op && !i_bus.has_moffs &&
        i_bus.modrm[7:6] != 2'b11) begin
        if (!i_bus.addr32) eaf_16 <= eaf_16 + 1;
        else case ({|ea_dec_base_sel_r, |ea_dec_index_sel_r})
            2'b10: eaf_base <= eaf_base + 1;
            2'b01: eaf_idx  <= eaf_idx + 1;
            2'b11: eaf_both <= eaf_both + 1;
            2'b00: eaf_disp <= eaf_disp + 1;
        endcase
    end
final if ($test$plusargs("ea_form_stats"))
    $display("ea forms: base=%0d idx=%0d BOTH=%0d disp=%0d a16=%0d",
             eaf_base, eaf_idx, eaf_both, eaf_disp, eaf_16);
final if ($test$plusargs("agu_stats"))
    $display("D2-AGU coverage: %0d/%0d eligible pops had a valid sidecar",
             d2agu_hits, d2agu_pops);
always @(posedge clk)
    if (reset_n && i_pop && i_bus.has_modrm && !i_bus.stack_op &&
        !i_bus.has_moffs && i_bus.addr32) begin
        d2agu_pops <= d2agu_pops + 1;
        if (head_ea_usable) d2agu_hits <= d2agu_hits + 1;
    end
// synthesis translate_on

// Relocate an about-to-be-committed IND value to its linear address
function automatic [31:0] reloc(input [31:0] off);
    reloc = (eff_mask_pending ? off : {16'h0, off[15:0]}) + seg_base_pending;
endfunction

// Dedicated reloc for the uc_exec/busop ind_linear write.
(* keep *) wire [31:0] seg_base_pending_uc = seg_base_pending;
function automatic [31:0] reloc_uc(input [31:0] off);
    reloc_uc = (eff_mask_pending ? off : {16'h0, off[15:0]}) + seg_base_pending_uc;
endfunction

// 3-term relocation using ALM's fused 3-input add
function automatic [31:0] reloc_add2(input [31:0] a, input [31:0] b, input mask16);
    reloc_add2 = mask16 ? ({16'h0, a[15:0] + b[15:0]} + seg_base_pending_uc)
                        : (a + b + seg_base_pending_uc);
endfunction

always_ff @(posedge clk) begin
    if (i_pop)
        ea_reg <= fast_pop_fc.br_rel ? spec_target_eip : ea_early;
end

//=============================================================================
// Unit 4: Segmentation Unit
//=============================================================================
wire [3:0]  mem_seg_sel;
wire        mem_seg_is_io;
wire        descsw_mode;
wire        mem_is_dtable;
wire        tss_access_flag;
wire [31:0] seg_base_pending;  // next seg_base_r from seg unit; for unified linear_address relocate
wire        eff_mask_pending;  // next (addr_size||is_dtable) from seg unit; 0 => mask offset to 16b
wire [31:0] seg_lar_result, seg_llim_result, seg_lbas_result;

// Segmentation unit command encoder
reg  [3:0]  seg_cmd;
reg  [3:0]  seg_cmd_target;
reg  [31:0] seg_cmd_data;
wire        seg_cmd_valid;          // 1 when seg_cmd should execute (i_pop or uc_exec)

// Decoded instruction register (all fields from decoder, latched at i_pop)
// dec_entry_t i; -- declaration moved up beside i_bus2 (Quartus cannot
// forward-reference struct members from the fast-chain gates)
wire [7:0]  i_modrm = i.modrm;
wire [7:0]  i_sib = i.sib;
wire        i_has_modrm = i.has_modrm;
wire        i_has_sib = i.has_sib;
wire [31:0] i_reg_immediate = i.immediate;
wire [31:0] i_reg_displacement = i.displacement;
wire        i_reg_addr32 = i.addr32;
wire [2:0]  i_seg = i.seg;
wire [2:0] i_reg_dst_reg_sel = i.dst_reg_sel;
wire [2:0] i_reg_src_reg_sel = i.src_reg_sel;

wire [3:0] modrm_resolved_seg = apply_seg_override_type(
    calc_default_seg_type(i_modrm, i_sib, i_has_sib, i_reg_addr32), i_seg);

// Pre-computed default segment for new instruction (combinational, used by INIT_SEG)
wire [3:0] init_default_seg = i_bus.stack_op ? SEG_SS :
                              i_bus.has_moffs ? SEG_DS :
                              i_bus.has_modrm ? calc_default_seg_type(i_bus.modrm, i_bus.sib, i_bus.has_sib, i_bus.addr32) :
                              SEG_DS;
wire [3:0] init_final_seg = i_bus.stack_op ? init_default_seg :
                            apply_seg_override_type(init_default_seg, i_bus.seg);

// Pre-computed access size for limit check (replaces op_size + is_dword in seg unit)
// Limit-check the actual access width: RD W/WR W = word (seg/limit reads; o32
// stride only bumps ESP); else srcreg_size (byte for MOVSX/MOVZX, not dest op_size).
wire [1:0] gp_access_adj = uc_is_word_op ? 2'd1 :
                           (srcreg_size == 2'd0) ? 2'd0 : (srcreg_size == 2'd2) ? 2'd3 : 2'd1;

wire        mem_op_eligible, gp_fault_mem_op, gp_fault_wr_op, ss_segment_fault;
reg         copy_stack_dpl_s2, conform_dpl_s2;
reg  [1:0]  copy_dpl_s2;
reg  [1:0]  conform_dpl_value_s2;

segmentation_unit seg_unit (
    .clk              (clk),
    .reset_n          (reset_n),
    // Command interface — descriptor cache manipulation
    .seg_cmd_valid    (seg_cmd_valid),
    .stssaf_pulse     (uc_exec && uc_aluop == ALUJMP_STSSAF),
    .ctssaf_pulse     (uc_exec && uc_aluop == ALUJMP_CTSSAF),
    .seg_cmd          (seg_cmd),
    .seg_target       (seg_cmd_target),
    .seg_data         (seg_cmd_data),
    .desc_lo          (TMPC),
    .desc_hi          (desc_raw_hi),
    .slctr            (SLCTR[15:0]),
    .copy_stack_dpl_s2(copy_stack_dpl_s2),
    .copy_dpl_s2      (copy_dpl_s2),
    .conform_dpl_s2   (conform_dpl_s2),
    .conform_dpl_value_s2(conform_dpl_value_s2),
    .seg_cache        (seg_cache),
    .lar_result       (seg_lar_result),
    .llim_result      (seg_llim_result),
    .lbas_result      (seg_lbas_result),
    // Segment state
    .seg_sel          (mem_seg_sel),
    .seg_is_io        (mem_seg_is_io),
    .is_dtable        (mem_is_dtable),
    .descsw_mode      (descsw_mode),
    .tss_access_flag  (tss_access_flag),
    // Address translation
    .pe               (pe),
    .vm               (vm),
    .cpl              (cpl),
    .offset           (IND),
    .access_size      (gp_access_adj),
    .check_en         (mem_op_eligible),
    .is_mem_op        (gp_fault_mem_op),
    .is_write         (gp_fault_wr_op),
    .seg_base_pending (seg_base_pending),
    .eff_mask_pending (eff_mask_pending),
    .seg_fault        (gp_fault_trigger),
    .is_stack_fault   (ss_segment_fault)
);

always_comb begin
    seg_cmd = SEG_CMD_NONE;
    seg_cmd_target = SEG_NONE;
    seg_cmd_data = dest_value;

    if (d2_valid) begin
        seg_cmd_target = init_final_seg;
        seg_cmd_data = {30'd0, i_bus.stack_op, i_bus.addr32};
    end else if ((uc_buscode == BUSOP_IND_PLUS_ALU || uc_buscode == BUSOP_IND_SRC)
                 && (uc_dest == DEST_DES_OS || uc_dest == DEST_DES_SR)) begin
        seg_cmd_target = modrm_resolved_seg;
    end else begin
        seg_cmd_target = resolve_seg_target(uc_dest, seg_reg_sel, COUNTR[5:0]);
    end

    if (d2_valid) begin
        seg_cmd = SEG_CMD_INIT_SEG;
    end else if (uc_dest == DEST_DESCSW) begin
        seg_cmd = SEG_CMD_DESCSW;
    end else begin
        case (uc_buscode)
            BUSOP_IND_PLUS_ALU,
            BUSOP_IND_SRC: begin
                seg_cmd = SEG_CMD_UPDATE_SEG;
                // DESSTK: set clear_descsw flag in seg_data[0]
                if (uc_dest == DEST_DESSTK)
                    seg_cmd_data = {31'd0, 1'b1};
                else
                    seg_cmd_data = 32'd0;
            end
            BUSOP_SBRM: begin
                if (!pe || vm)
                    seg_cmd = SEG_CMD_SBRM;
            end
            BUSOP_SAR: begin
                seg_cmd = SEG_CMD_SAR;
            end
            BUSOP_SLIM: begin
                seg_cmd = (uc_dest == DEST_DESPTR) ? SEG_CMD_SLIM_TABLE : SEG_CMD_SLIM;
            end
            BUSOP_SBAS: begin
                if (uc_dest == DEST_DESPTR)
                    seg_cmd = SEG_CMD_SBAS;
            end
            BUSOP_SDEH: begin
                if (pe && !gate_detect_cond)   // use cond, not _now (uc_exec already in valid)
                    seg_cmd = SEG_CMD_SDEH;
            end
            BUSOP_SDES: begin
                if (pe && !gate_detect_cond) begin
                    seg_cmd = SEG_CMD_SDES;
                    seg_cmd_data = alu_src_data;
                end
            end
            BUSOP_SDEL: begin
                if (pe && !gate_detect_cond) begin
                    seg_cmd = SEG_CMD_SDEL;
                    // SDEL's descriptor-low operand is encoded in the ALU source
                    // field. Most sites use TMPC, but cross-privilege CALL uses TMPD.
                    seg_cmd_data = alu_src_data;
                end
            end
            BUSOP_SPCR: begin
                seg_cmd = SEG_CMD_SPCR;
            end
            default: ;
        endcase
    end
end  // always_comb


//=============================================================================
// Unit 5: Paging Unit (including TLB)
//=============================================================================

// WR W / RD W access width = |IND_DELTA| (the stack/TSS slot stride).
// Ordinary accesses use source width: MOVZX/MOVSX read byte/word operands
// even though their architectural destination and op_size are dword.
wire ind_delta_dword = (IND_DELTA == 32'd4) || (IND_DELTA == -32'd4);
wire [1:0] mem_eff_size = uc_is_word_op ? (ind_delta_dword ? 2'd2 : 2'd1) :
                          uc_is_dword_op ? 2'd2 : srcreg_size;

wire [31:0] mem_wdata = (uc_buscode == BUSOP_WR_OPR) ? OPR_R :
    uc_is_word_op ? read_uc_source(uc_source) :
    (uc_dest == DEST_OPR_W) ? (stack_init_pending ? read_uc_source(uc_source) : dest_value) :
    OPR_W;

// div_overflow fires only at the first DIV7/PREDIV word
wire        any_fault_pop = gp_fault_trigger || page_fault;
wire        any_fault = any_fault_pop || div_overflow;
reg         any_fault_r;  // Registered any_fault: used for deferred SIGMA/TMPeSP writes
always_ff @(posedge clk) any_fault_r <= any_fault;
wire [2:0]  pg_fault_code;        // Page fault error code
wire [31:0] pg_cr2_out;           // Faulting address for CR2

// CR3 write detection for TLB flush
wire cr3_write = uc_exec && uc_buscode == BUSOP_SPCR && uc_dest == DEST_PDBR;

// IO request detection
wire mem_is_io = mem_seg_is_io;     // registered in segmentation_unit alongside seg_sel
wire io_busop_rd = uc_p_io_rd && mem_is_io;
wire io_busop_wr = uc_p_io_wr && mem_is_io;

wire iack_busop = uc_p_iack;        // IACK bus operation (interrupt acknowledge)

// Interrupt pending: NMI has priority over INTR
wire nmi_edge = nmi && !nmi_prev && !nmi_blocked;
wire nmi_request_active = nmi_pending || nmi_edge;
wire interrupt_pending = nmi_request_active || (intr_pending && EFLAGS[9]);
wire nmi_accept_boundary = i_rni_delay && !stall && !page_fault &&
                           nmi_request_active && !single_step;

reg inhibit_interrupts;     // STI shadow: real 386 suppresses interrupt recognition for one instruction after STI

// A delayed D2 entry may already occupy the shared ROM q while it is still
// waiting for literals. It is not an EX uop yet and must not issue its bus op.
assign mem_op_eligible = core_live && !mem_servicing &&
                         !stall_d2 && !d2_release_hold;
wire uc_data_busreq = (uc_is_mem_busop && !mem_is_io) ||
                      io_busop_rd || io_busop_wr;
wire uc_busreq = uc_data_busreq || iack_busop;
wire mem_req_current = mem_op_eligible && uc_busreq;    // drives paging unit

// Delay prefetch on upcoming demand memory
wire mem_req_upcoming = uc_next[39] && !halted && (uc_active || d2_valid);

// Implicit supervisor access: descriptor table and TSS reads, cross-privilege
// stack writes use CPL=0 for paging regardless of current CPL.
wire implicit_supervisor = mem_is_dtable || (mem_seg_sel == SEG_TR) ||
                           descsw_mode || (vm && CS[1:0] == 2'b00);
wire [1:0] pg_cpl = implicit_supervisor ? 2'b00 : cpl;

// Registered fault redirect state.
reg         gp_fault_r;
reg         ss_fault_r;

wire        mem_req_to_paging = mem_op_eligible && uc_data_busreq && !gp_fault_trigger;
wire        iack_req_to_paging = mem_op_eligible && iack_busop && !gp_fault_trigger;
wire        mem_write_now = uc_is_write || (io_busop_wr && mem_is_io);
wire [3:0]  mem_be_now = iack_busop ? 4'b1111 :
                          calc_be(mem_eff_size, ind_linear[1:0]);
wire [31:0] paging_linear_addr = ind_linear;
assign pf_spec_store = mem_req_to_paging && mem_write_now && mem_accepted;
assign pf_spec_store_linear = paging_linear_addr;
wire        paging_live_valid  = ind_linear_valid;
wire        paging_mem_rd_ind = (uc_buscode == BUSOP_RD_IND);
wire        paging_is_write_access = uc_is_write || uc_is_check_write;
wire        mem_ea_read = i_first && instr_ind_is_ea;

// Paging unit instantiation
paging_unit paging_inst (
    .clk                (clk),
    .reset_n            (reset_n),
    .cr0                (CR0),
    .cr3                (CR3),
    .cr3_write          (cr3_write),

    // Memory/IO request: current RD/WR/IACK uop is held by stall until accepted.
    .mem_req            (mem_req_to_paging),
    .mem_inta_req       (iack_req_to_paging),
    .mem_inta_addr      (IND),
    .mem_ea_read        (mem_ea_read),      // modrm/stack/moffs reads SET-read (linear relocated at i_pop); microcode IND reads excluded
    .mem_req_precheck   (mem_op_eligible && uc_data_busreq),
    .mem_req_upcoming   (mem_req_upcoming), // suppresses prefetch start to minimize contention
    .mem_accepted       (mem_accepted),     // ready: request accepted this cycle
    .mem_servicing      (mem_servicing),
    .mem_complete_now   (mem_complete_now), // combinational: bus op completing this cycle
    .mem_dly_grace      (mem_dly_grace),
    .mem_write_dly_grace(mem_write_dly_grace),
    .mem_opt_wait       (mem_opt_wait),
    .mem_write_wait     (mem_write_wait),
    .linear_addr        (paging_linear_addr),
    .live_valid         (paging_live_valid),
    .mem_op_size        (mem_eff_size),
    .mem_write          (mem_write_now),
    .mem_wdata          (mem_wdata),
    .mem_rd_ind         (paging_mem_rd_ind),
    .is_write_access    (paging_is_write_access),
    .mem_check_only     (uc_is_check_write),
    .cpl                (pg_cpl),
    .mem_is_io          (mem_is_io),
    .mem_be             (mem_be_now),

    // Prefetch (toggle protocol)
    .pf_req_toggle      (pf_req_toggle),
    .pf_ack_toggle      (pf_ack_toggle),
    .pf_redirect_queued (pf_redirect_queued),
    .pf_linear_addr     (pf_linear_addr),
    .pf_rdata           (pf_rdata),
    .pf_fault           (pf_fault),

    // Demand-side physical request interface
    .dcache_req_valid   (dcache_req_valid),
    .dcache_req_phys_addr(dcache_req_phys_addr_raw),
    .dcache_req_write   (dcache_req_write),
    .dcache_req_be      (dcache_req_be),
    .dcache_req_wdata   (dcache_req_wdata),
    .dcache_req_is_io   (dcache_req_is_io),
    .dcache_req_is_inta (dcache_req_is_inta),
    .dcache_req_accepted(dcache_req_accepted),
    .dcache_req_complete(dcache_req_complete),
    .dcache_rdata       (dcache_rdata),

    // Instruction-prefetch physical request interface
    .icache_req_valid   (icache_req_valid),
    .icache_req_phys_addr(icache_req_phys_addr_raw),
    .icache_req_accepted(icache_req_accepted),
    .icache_req_complete(icache_req_complete),
    .icache_rdata       (icache_rdata),

    // OPR_R
    .OPR_R              (OPR_R),

    // Status
    .page_fault         (page_fault),
    .fault_code         (pg_fault_code),
    .cr2_out            (pg_cr2_out)
);

always_ff @(posedge clk) begin
    if (!reset_n) begin
        gp_fault_r <= 1'b0;
        ss_fault_r <= 1'b0;
    end else begin
        gp_fault_r <= gp_fault_trigger;
        ss_fault_r <= ss_segment_fault;
    end
end

// synthesis translate_off
// Debug: log every protected-mode (non-V86) fault delivery
always @(posedge clk) begin
    if (reset_n && uc_addr == 12'h890 && !EFLAGS[17])
        $display("%0t: PM FAULT CS:EIP=%0x:%0x SIGMA=%08x TMPF=%08x EFL=%08x", $time, CS, EIP, SIGMA, TMPF, EFLAGS);
end
// synthesis translate_on

// CR3 register update
always_ff @(posedge clk) begin
    if (!reset_n)
        CR3 <= 32'h0;
    else if (cr3_write) begin
        CR3 <= IND;
    end
end


//=============================================================================
// Unit 6: Protection Test Unit (PLA4)
//=============================================================================
// Pipeline enable: advance PLA4 pipeline in sync with microcode.
wire prot_pipe_en = !stall;

// Protection test enable and constant routing
// aluop 0x6? range: bit3=0 is PTSAV (save only), bit3=1 fires test
wire prot_is_6x = (uc_aluop[6:4] == 3'b110);
wire prot_is_ptsav = prot_is_6x && !uc_aluop[3];  // PTSAV1(0x61), PTSAV3(0x63), PTSAV7(0x67)
wire prot_is_ptovrr = (uc_aluop == ALUJMP_PTOVRR); // 0x68: uses saved test constant
wire [5:0] prot_test_const = prot_is_ptovrr ? prot_saved_test_const : uc_alu_src[5:0];
// FPU tests (test_const 0x34, 0x38-0x3F) must fire even in real mode
wire is_fpu_prot_test = prot_test_const[5] && prot_test_const[4] && (prot_test_const[3] || prot_test_const[2]);
wire prot_test_en = uc_exec && prot_is_6x && !prot_is_ptsav && (pe || is_fpu_prot_test);
wire selector_null_wire = (slctr_fwd[15:3] == 13'b0) && !slctr_fwd[2]; // Null selector: Index=0, TI=0
wire [15:0] selector_desc_end = {slctr_fwd[15:3], 3'b111}; // Last byte offset of 8-byte descriptor
wire selector_oob_wire = slctr_fwd[2] ?
    ({12'h0, seg_cache[SEG_LDT].limit} < {4'h0, selector_desc_end}) :  // LDT: compare against LDTR limit
    (seg_cache[SEG_GDT].limit[15:0] < selector_desc_end);              // GDT: compare against GDTR limit

// PROTUN forwarding
wire        protun_writing = uc_exec && (uc_dest == DEST_PROTUN);
wire        tstdes_set_accessed = pe && uc_exec && (uc_aluop == ALUJMP_PTOVRR);
wire [31:0] protun_write_value = read_protun_source_fast(uc_source);
wire [31:0] protun_next = (tstdes_set_accessed && protun_write_value[12]) ? (protun_write_value | 32'h100) :
                           protun_write_value;
wire [31:0] protun_fwd = protun_writing ? protun_next : PROTUN;
wire [31:0] prot_desc_value = prot_is_ptovrr ? OPR_R :
                              (uc_alu_src[5:0] == TST_DES_GRANUL) ? desc_raw_hi : protun_fwd;
wire        prot_desc_g = prot_desc_value[23];
wire        prot_desc_p = prot_desc_value[15];
wire [1:0]  prot_desc_dpl = prot_desc_value[14:13];
wire        prot_desc_s = prot_desc_value[12];
wire [3:0]  prot_desc_type = prot_desc_value[11:8];
wire [1:0]  prot_desc_rpl = prot_desc_value[1:0];
wire        prot_desc_low16_nonzero = |prot_desc_value[15:0];

protection_unit protection_unit_inst (
    .clk              (clk),
    .reset_n          (reset_n),
    .pipe_en          (prot_pipe_en),

    // Descriptor state: narrowed attribute bundle with forwarding for same-cycle writes
    .descriptor_g     (prot_desc_g),
    .descriptor_p     (prot_desc_p),
    .descriptor_dpl   (prot_desc_dpl),
    .descriptor_s     (prot_desc_s),
    .descriptor_type  (prot_desc_type),
    .descriptor_rpl   (prot_desc_rpl),
    .descriptor_low16_nonzero(prot_desc_low16_nonzero),
    .selector_rpl     (slctr_fwd[1:0]),             // RPL from selector (forwarded)
    .selector_ti      (slctr_fwd[2]),               // Table indicator (forwarded)
    .selector_null    (selector_null_wire),         // Null selector (Index=0, TI=0)
    .selector_oob     (selector_oob_wire),          // Selector exceeds GDT/LDT limit

    // Processor state
    .cpl              (prot_cpl),                   // CPL (pending after WRITE_RPL, else effective CPL)
    .pe_mode          (pe),                         // Protected mode active

    // CR0 flags for FPU tests
    .cr0_et           (CR0[4]),                     // Extension type (287 vs 387)
    .cr0_ts           (CR0[3]),                     // Task switched
    .cr0_em           (CR0[2]),                     // Emulation
    .cr0_mp           (CR0[1]),                     // Monitor coprocessor

    // ARPL support
    .arpl_rpl         (arpl_rpl_latch),             // Latched source RPL from READ_RPL

    // Test control (from microcode)
    // PTSAV? (aluop 0x6?, bit3=0): saves test constant for later PTOVRR, does NOT fire test
    // PTOVRR (0x68): fires test using saved test constant from PTSAV
    // PTSELE (0x6E) and others (bit3=1): fires test with inline test constant
    .test_const       (prot_test_const),
    .aluop_type       (uc_aluop[3:0]),              // Lower 4 bits of aluop (controls Tiny PLA mux)
    .test_en          (prot_test_en),

    // Test mode (disabled in normal operation)
    .test_mode        (1'b0),
    .test_state_vector(10'h000),

    // Outputs
    .jump_addr        (prot_jump_addr),
    .jump_valid       (prot_jump_valid),
    .validation_ok    (prot_validation_ok),
    .result_valid     (prot_result_valid)
);


// PROTUN register
always_ff @(posedge clk) begin
    if (!reset_n) begin
        PROTUN <= 32'h0;
    end else if (uc_exec && (uc_dest == DEST_PROTUN)) begin
        PROTUN <= protun_next;
    end else if (uc_exec && arpl_m_flag_s2 && prot_validation_ok) begin
        PROTUN[1:0] <= arpl_rpl_latch;
    end
end

// Protection test state
always_ff @(posedge clk) begin
    if (!reset_n) begin
        prot_saved_test_const <= 6'h0;
        desc_raw_hi <= 32'h0;
        prot_test_inflight <= 1'b0;
        prot_redirect_prev <= 1'b0;
    end else if (uc_exec_writeback) begin
        casez (uc_aluop)
            7'h6?: if (pe) begin
                if (prot_is_ptsav)
                    prot_saved_test_const <= uc_alu_src[5:0];
                if (uc_aluop == ALUJMP_PTOVRR) begin
                    desc_raw_hi <= OPR_R;
                end
            end
            default: ;
        endcase
        if (prot_test_en)
            prot_test_inflight <= 1'b1;
        else if (prot_result_now)
            prot_test_inflight <= 1'b0;

        prot_redirect_prev <= 1'b0;
        if (prot_redirect_taken)
            prot_redirect_prev <= 1'b1;
    end
end

//=============================================================================
// Unit 7: Execution - control unit for microcode and fast instructions 
// "chaining" = FAST issue pipelining
//=============================================================================

wire [5:0] uc_alu_src   = uc[36:31];  // ABCDEF: ALU source / jump offset
assign uc_dest          = uc[30:24];  // GHIJKLM: destination
assign uc_source        = uc[23:18];  // NOPQRS: source
wire [6:0] uc_aluop     = uc[17:11];  // TUVWXYZ: ALU operation / jump condition
wire [2:0] uc_opcode    = uc[10:8];   // 012: opcode (RNI, RPT, etc.)
// subcode field uc[7:6] (DLY/UNL/WIO) is consumed via ROM predecode bits only
assign uc_buscode       = uc[5:0];    // 56789&: bus operation code
wire [5:0] uc_next_buscode = uc_next[5:0];

always_ff @(posedge clk) begin
    if (!reset_n) begin
        uc_ctl_pref <= 1'b0;
    end else if (microcode_rom_ce) begin
        uc_ctl_pref <= (uc_next_buscode == BUSOP_PREF);
    end
end

reg [11:0] microcode_return_stack [0:3];// 4-entry return address stack
reg [1:0]  microcode_sp;                // Stack pointer (0-3)

reg        uc_jump_taken_prev;          // Jump taken last cycle (for RNi: terminate only in delay slot)
reg        uc_cond_jump_taken_prev;     // Conditional jump taken last cycle (for PREF suppression)

//   RNI/RnI terminate unless we're in a delay slot of a taken jump (loop continues)
//   RNi only terminates when in delay slot (after a jump)
//   jcc_fold_active: pop-time-resolved not-taken Jcc terminates at 065 (M4 fold)
wire branch_ustep_exec;
wire branch_ustep_redirect;
assign i_rni = ((uc_is_rni || uc_is_rni_inhibit) && !uc_jump_taken_prev) ||
                           (uc_is_rni_lc && uc_jump_taken_prev) ||
                           jcc_fold_active || branch_ustep_exec;

reg        instr_is_shxd;           // Instruction is a SHxD operation
reg        instr_cf;                // CF bit at start of instruction
reg        instr_is_cmp;
reg        instr_ind_is_ea;
reg  [4:0] alu_grp_op;              // Pre-decoded ALU op for ALUJMP_ALU/INCDEC (from i_bus at i_pop)
reg        instr_is_loop;           // E0/E1: LOOPNE/LOOPE (eliminates 7-bit compare from jump path)
reg  [1:0] instr_bt_sel;            // BT operation selector (eliminates 8-bit compare from ALU path)
reg  [4:0] instr_szext_op;          // Pre-decoded MOVZX/MOVSX/CBW ALU op
reg        stack_init_pending;      // Cycle after i_pop for stack op - ALU computes new SP
reg        prot_test_inflight;      // Protection test is in pipeline (waiting for result)
reg        prot_redirect_prev;      // Protection redirect fired last cycle (suppresses LJUMP + relative jumps in delay slot)

reg [31:0] ea_reg;                  // Early EA registered at i_pop (ALU path / stack-dest EA, e.g. POP [mem]); read via SRC_EA, off the i_first seg/TLB cone

reg [2:0]  seg_reg_sel;             // Segment register index (0=ES,1=CS,2=SS,3=DS,4=FS,5=GS)

reg [31:0] COUNTR;                  // Counter register
wire [31:0] countr_masked = i.addr32 ? COUNTR : {16'h0, COUNTR[15:0]};
reg [31:0] TMPeIP;                  // Saved EIP for RPTI (repeat instruction)
reg [31:0] wr_restart_eip;          // TMPeIP captured at every demand-write issue: a write
                                    // fault (perm/walk/crossing) may surface after the issuing
                                    // instruction was FAST-chained away and TMPeIP moved on
reg [31:0] TMPeSP;                  // Saved ESP for fault handling
reg        flags_backup_active;     // Set at i_pop/FLGSBA, cleared on interrupt_entry - guards FLAGSB writes
reg        clear_if_pending;        // Set by {-2E-}, used by {-2F-} to clear IF during INT
reg        misc1_flag;              // Set by SMISC1 {-33-}, tested by JMISC1 {-53-}
reg        misc2_flag;              // Set by SMISC2 {-35-}, tested by JMISC2 {-55-}
reg        error_code_flag;         // Set by SERRCF {-36-}, tested by JNERRC {-56-}
reg        interrupt_hw;            // Set for hardware interrupts, tested by JINTSW {-52-}
reg        intr_pending;            // Latched INTR request (level-sampled, cleared by CINTLA)
reg        intr_latch_inhibit;      // Suppress re-latching after CINTLA until intr deasserts
reg        nmi_pending;             // Latched NMI request (edge-detected, cleared on NMI entry)
reg        nmi_blocked;             // NMI service in progress (set by SETNMI, cleared by CLRNMI)
reg        nmi_prev;                // Previous NMI value for edge detection
reg        interrupt_entry;         // Interrupt handler being entered (suppress i_entry/i_pop)
reg        jcc_active;              // Currently executing a Jcc instruction (for alu_src_r in BUSOP_IND_PLUS_ALU)
reg        instr_eip_written;       // EIP was written during instruction (RPTI restart)
reg        gate_in_progress;        // Prevent second LDTST (at 5C3) from re-triggering gate detection

// Microcode PREF restarts from IND
wire [31:0] pf_flush_ip = IND;
assign pf_flush_addr = branch_ustep_redirect ? (CS_base + ea_reg) :
                       early_redirect        ? (CS_base + br_target) :
                       pe_mode_toggle_now    ? (CS_base + EIP) :
                                               (CS_base + pf_flush_ip);

wire        br_is_jcc      = (i.opcode[7:4] == 4'b0111 && !i.has_0f) ||
                             (i.opcode[7:4] == 4'b1000 && i.has_0f);
wire        br_is_jmp_rel  = !i.has_0f && (i.opcode == 8'hEB || i.opcode == 8'hE9);
wire        br_is_call_rel = !i.has_0f && (i.opcode == 8'hE8);
wire        br_is_rel8     = !i.has_0f && (i.opcode[7:4] == 4'b0111 || i.opcode == 8'hEB);
wire [31:0] br_disp        = br_is_rel8 ? {{24{i.displacement[7]}}, i.displacement[7:0]}
                                        : i.displacement;
wire [31:0] br_target      = EIP + br_disp;
// synthesis translate_off
always @(posedge clk) begin
    // Validate the microcode-PREF flush path
    if (reset_n && q_flush && !early_redirect && is_dword && (br_is_jcc || br_is_jmp_rel || br_is_call_rel) &&
        (pf_flush_ip !== (CS_base + br_target)))   // compare LINEAR vs LINEAR (pf_flush_ip is IND = CS_base+EIP+disp)
        $display("%0t: BR TARGET MISMATCH computed=%08x actual=%08x op=%02x CS:EIP=%0x:%0x",
                 $time, CS_base + br_target, pf_flush_ip, i.opcode, CS, EIP);
end
// synthesis translate_on

// i_first PRECISE early branch redirect (NOT a prediction).
wire br_jcc_taken = br_is_jcc && check_condition(i.opcode[3:0], eflags_fwd);
// Registered pop-time condition (same validity gate as the NT 
reg jcc_pop_taken_r;    // condition value captured at the Jcc's i_pop
reg jcc_pop_cond_v_r;   // capture was valid (flags settled at the pop)
always_ff @(posedge clk) begin
    if (!reset_n) begin
        jcc_pop_taken_r  <= 1'b0;
        jcc_pop_cond_v_r <= 1'b0;
    end else if (q_flush || interrupt_entry || any_fault) begin
        jcc_pop_cond_v_r <= 1'b0;
    end else if (i_pop) begin
        jcc_pop_cond_v_r <= fast_pop_fc.jcc && !jcc_pop_unsafe;
        jcc_pop_taken_r  <= check_condition(i_bus.opcode[3:0], eflags_ahead);
    end
end
// synthesis translate_off
// The registered pop-time decision must match the live 065 evaluation.
always @(posedge clk)
    if (reset_n && fast_active && fast_jcc_r && uc_exec && jcc_pop_cond_v_r &&
        (jcc_pop_taken_r != br_jcc_taken))
        $display("%0t JCC-POPCOND MISMATCH: pop=%b live=%b op=%02x", $time,
                 jcc_pop_taken_r, br_jcc_taken, i.opcode);
// synthesis translate_on
wire branch_ustep_taken = !branch_ustep_jcc_r || jcc_pop_taken_r;
assign branch_ustep_exec = i_first && branch_ustep_r && uc_exec;
assign branch_ustep_redirect = branch_ustep_exec && branch_ustep_taken;
wire early_redirect = branch_ustep_redirect ||
                      (i_first && is_dword && br_is_call_rel);
reg  early_redirected;
// Cleared at the NEXT instruction's arrival
always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n)                            early_redirected <= 1'b0;
    else if (early_redirect)                 early_redirected <= 1'b1;
    else if (i_entry || i_pop || interrupt_entry) early_redirected <= 1'b0;
end

// RNI variants (opcode field):
//   000 = RNI  : Run Next Instruction (normal termination)
//   001 = RNi  : RNI only if in delay slot (lowercase i)
//   010 = RnI  : RNI with interrupt inhibit for next instruction
wire uc_is_rni = (uc_opcode == 3'b000);
wire uc_is_rni_lc = (uc_opcode == 3'b001);
wire uc_is_rni_inhibit = (uc_opcode == 3'b010);
wire uc_is_wio = uc_p_wio;  // WIO: wait for interrupt/IO (HLT, only with RPT)
wire uc_is_rpt = uc_p_rpt;

// LOOP/REP Condition Logic
wire loop_zf_sense = instr_is_loop ? i.opcode[0] : i.rep_lock[0];  // ZF sense for branch
wire countr_will_be_nonzero = instr_is_loop ? (countr_masked != 32'h1) : (countr_masked != 32'h0);
wire zf_check = instr_is_loop ? (loop_zf_sense == EFLAGS[6]) : (loop_zf_sense != EFLAGS[6]);
wire loopne_condition = instr_is_loop ? (countr_will_be_nonzero && zf_check)
                                      : (!countr_will_be_nonzero || zf_check);

// GP Fault Detection — handled by segmentation_unit
assign gp_fault_mem_op = uc_is_mem_busop && (uc_buscode != BUSOP_RD_D);
assign gp_fault_wr_op = uc_is_write || uc_is_check_write;

// DIV/IDIV Overflow Detection
localparam logic DIV_OP_MASK   = 1'b0;
localparam logic DIV_OP_NEGATE = 1'b1;

wire [31:0] div_upper_dividend = div_op_by_size(DIV_OP_MASK, SIGMA, op_size);
wire [31:0] div_divisor = div_op_by_size(DIV_OP_MASK, TMPB, op_size);

// IDIV2 overflow: check if quotient magnitude fits in signed range
wire [31:0] idiv_quotient = div_op_by_size(DIV_OP_MASK, RESULT, op_size);
wire [31:0] idiv_signed_max = (op_size == 2'd0) ? 32'h80 :
                              (op_size == 2'd1) ? 32'h8000 : 32'h80000000;
wire idiv_signs_differ = idiv_dividend_neg ^ idiv_divisor_neg;
wire idiv2_overflow = uc_exec && (uc_aluop == ALUJMP_IDIV2) &&
    (idiv_signs_differ ? (idiv_quotient > idiv_signed_max) : (idiv_quotient >= idiv_signed_max));

assign div_overflow = (div_first_cycle && uc_exec && (
    // DIV: unsigned overflow check at first DIV7
    (uc_aluop == ALUJMP_DIV7 && (div_divisor == 32'h0 || div_upper_dividend >= div_divisor)) ||
    // IDIV: early signed overflow check at PREDIV (uses absolute values)
    (uc_aluop == ALUJMP_PREDIV && (div_divisor_abs == 32'h0 || prediv_r_in >= div_divisor_abs))
)) || idiv2_overflow;

// Relative jump condition evaluator: returns true when relative jump should be taken
function automatic logic is_reljump_taken(input [6:0] aluop);
    case (aluop)
        ALUJMP_JNcond:  is_reljump_taken = !check_condition(i.opcode[3:0], eflags_fwd);
        ALUJMP_JCNTZ:   is_reljump_taken = (countr_masked == 32'h0);
        ALUJMP_JCNTNZ:  is_reljump_taken = (countr_masked != 32'h0);
        ALUJMP_JCT4N1:  is_reljump_taken = (countr_masked[3:0] != 4'h1);
        ALUJMP_JCNZNI:  is_reljump_taken = (countr_masked != 32'h0);
        ALUJMP_JCNTN1:  is_reljump_taken = (countr_masked != 32'h1);
        ALUJMP_JCNT1:   is_reljump_taken = (countr_masked == 32'h1);
        ALUJMP_LOOPnE:  is_reljump_taken = instr_is_loop ? !loopne_condition : loopne_condition;
        ALUJMP_JG:      is_reljump_taken = !uc_flags[6] && (uc_flags[7] == uc_flags[11]);
        ALUJMP_JNC:     is_reljump_taken = !uc_flags[0];
        ALUJMP_JNO:     is_reljump_taken = !uc_flags[11];
        ALUJMP_JPEREQ:  is_reljump_taken = uc_jpereq_fwd;       // No FPU: pre-computed in ROM bit 44
        ALUJMP_JNFLGB:  is_reljump_taken = !flags_backup_active;
        ALUJMP_JTSSAF:  is_reljump_taken = tss_access_flag;     // Jump if TSS access flag is set
        ALUJMP_JINTSW:  is_reljump_taken = !interrupt_hw;
        ALUJMP_JMISC1:  is_reljump_taken = misc1_flag;
        ALUJMP_JMISC2:  is_reljump_taken = misc2_flag;
        ALUJMP_JNERRC:  is_reljump_taken = !error_code_flag;
        ALUJMP_JNT:     is_reljump_taken = EFLAGS[14];
        ALUJMP_JIO_OK:  is_reljump_taken = !pe || cpl <= EFLAGS[13:12]; // JIO_OK: CPL <= IOPL
        ALUJMP_JMP:     is_reljump_taken = 1'b1;                // Unconditional jump
        ALUJMP_JNOINT:  is_reljump_taken = !interrupt_pending;  // Jump if NO interrupt
        ALUJMP_JNBUSY:  is_reljump_taken = 1'b1;                // FPU busy — always taken (no FPU)
        ALUJMP_J16BIT:  is_reljump_taken = !seg_cache[SEG_TR].seg_type[3];
        default:        is_reljump_taken = 1'b0;
    endcase
endfunction

// TODO: fix special casing
// Suppress JMP in LD_DESCRIPTOR at 5D3 when Accessed bit needs GDT write-back.
// When A=0 in the descriptor, fall through to 5D5-5D7 which writes A=1 back to GDT.
// When A=1, take JMP to skip write-back (A already set).
wire desc_accessed_writeback = pe && (uc_aluop == ALUJMP_JMP) &&
                               (uc_addr == 12'h5D3) && !desc_raw_hi[8];

// !jcc_fold_active: a folded not-taken Jcc terminates at 065 — its JNcond
// jump (to Jcond_DONE) must not fire, or it would steal uaddr from the
// entry-point load of the next instruction.
wire uc_reljump_taken = uc_exec && !repeat_active && is_reljump_taken(uc_aluop) &&
                        !desc_accessed_writeback && !prot_redirect_prev &&
                        !jcc_fold_active && !branch_ustep_exec;

wire uc_cond_jump_taken = (uc_reljump_taken &&
    uc_aluop != ALUJMP_JMP &&
    uc_aluop != ALUJMP_JNOINT && uc_aluop != ALUJMP_JNBUSY) ||
    (uc_exec && (
        (uc_aluop == ALUJMP_LJMPP && pe && !vm) ||
        (uc_aluop == ALUJMP_LJMPNP && (pe && (cpl != 2'b00))) ||
        (uc_aluop == ALUJMP_LJMP86 && vm)
    ));

wire [11:0] uc_ljump_target = {uc_source, uc_alu_src};
wire ljump_taken = (uc_aluop == ALUJMP_LJUMP) && !prot_redirect_prev;

reg set_rpl_redirect_s1, set_rpl_redirect_s2;
reg copy_stack_dpl_s1;
reg [1:0] copy_dpl_s1;
reg conform_dpl_s1;
reg [1:0] conform_dpl_value_s1;
reg write_rpl_s1, write_rpl_s2;
reg cpl_transition;

reg        arpl_m_flag_s1, arpl_m_flag_s2;
assign     prot_cpl = cpl_transition ? SLCTR[1:0] : cpl;
always_ff @(posedge clk) begin
    if (!reset_n) begin
        set_rpl_redirect_s1 <= 0;
        set_rpl_redirect_s2 <= 0;
        copy_stack_dpl_s1 <= 0;
        copy_stack_dpl_s2 <= 0;
        copy_dpl_s1 <= 2'b0;
        copy_dpl_s2 <= 2'b0;
        write_rpl_s1 <= 0;
        write_rpl_s2 <= 0;
        cpl_transition <= 0;
        arpl_rpl_latch <= 2'b00;
        arpl_m_flag_s1 <= 0;
        arpl_m_flag_s2 <= 0;
        conform_dpl_s1 <= 0;
        conform_dpl_s2 <= 0;
        conform_dpl_value_s1 <= 2'b00;
        conform_dpl_value_s2 <= 2'b00;
    end else if (prot_pipe_en) begin
        set_rpl_redirect_s1 <= uc_exec && pe &&
            (uc_aluop == ALUJMP_PTGEN) && (uc_alu_src == 6'h2D) &&
            (seg_cache[SEG_CS].DPL != cpl) &&
            !(seg_cache[SEG_CS].seg_type[3] && seg_cache[SEG_CS].seg_type[2]);
        set_rpl_redirect_s2 <= set_rpl_redirect_s1;
        // Conforming code: set seg_cache[SEG_CS].DPL = CPL (no privilege change)
        conform_dpl_s1 <= uc_exec && pe &&
            (uc_aluop == ALUJMP_PTGEN) && (uc_alu_src == 6'h2D) &&
            seg_cache[SEG_CS].seg_type[3] && seg_cache[SEG_CS].seg_type[2];
        conform_dpl_s2 <= conform_dpl_s1;
        conform_dpl_value_s1 <= CS[1:0];
        conform_dpl_value_s2 <= conform_dpl_value_s1;
        copy_stack_dpl_s1 <= uc_exec && pe &&
            (uc_aluop == ALUJMP_PTGEN) && (uc_alu_src == 6'h2E);
        copy_stack_dpl_s2 <= copy_stack_dpl_s1;
        copy_dpl_s1 <= prot_desc_dpl;  // Capture descriptor DPL at PTGEN time
        copy_dpl_s2 <= copy_dpl_s1;
        write_rpl_s1 <= uc_exec && pe &&
            (uc_aluop == ALUJMP_PTGEN) && (uc_alu_src[5:0] == 6'h2C);
        write_rpl_s2 <= write_rpl_s1;
        // ARPL: latch source RPL when READ_RPL fires (at uc=6B6: SRCREG → PROTUN)
        // dest_value = source selector, so dest_value[1:0] = source RPL
        if (uc_exec && pe && (uc_aluop == ALUJMP_PTSELA) && (uc_alu_src == 6'h2B))
            arpl_rpl_latch <= dest_value[1:0];
        // ARPL: track TST_SEL_ARPL success for PROTUN[1:0] writeback
        arpl_m_flag_s1 <= uc_exec && pe &&
            (uc_aluop == ALUJMP_PTSELA) && (uc_alu_src == 6'h05);
        arpl_m_flag_s2 <= arpl_m_flag_s1;

        // cpl_transition: set by RETF_OUTER redirect and WRITE_RPL, cleared by COPY_STACK_DPL
        if (prot_result_now && prot_jump_valid && prot_jump_addr == 12'h686)
            cpl_transition <= 1;  // RETF/IRETD outer-level redirect
        if (write_rpl_s2)
            cpl_transition <= 1;  // WRITE_RPL
        if (copy_stack_dpl_s2)
            cpl_transition <= 0;  // COPY_STACK_DPL
    end
end

wire prot_redirect_taken = uc_exec && prot_result_now && prot_jump_valid;
wire uc_jump_taken = uc_reljump_taken || prot_redirect_taken || (uc_exec && (
    (uc_aluop == ALUJMP_LJMPP && pe && !vm) ||
    (uc_aluop == ALUJMP_LJMPNP && (pe && (cpl != 2'b00))) ||
    (uc_aluop == ALUJMP_LJMP86 && vm) ||
    (uc_aluop == ALUJMP_LCALL) ||
    ljump_taken
));

wire gate_detect_cond = pe && (uc_buscode == BUSOP_SDEL) &&
                        !gate_in_progress && !desc_raw_hi[12] && (desc_raw_hi[11:8] == 4'hC);
wire gate_detect_now = uc_exec && gate_detect_cond;

always_comb begin
    uaddr_now = uaddr;

    if (((((i_pop && !d2_waited_r) | uc_exec) |
          (fault_suppress_delay_slot & !stall)) & !halted && !repeat_active))
        uaddr_now = uaddr + 12'd1;

    // A held D2 entry keeps q stable while q_mem primes its architectural
    // delay-slot word. Branching entries override only sequencer state; the
    // Jcc ROM-address exception above still reads the slot.
    if (d2_delay_preload)
        uaddr_now = i_bus.entry_point + 12'd1;

    if (i_entry_raw)
        // Empty pipe uses the live D1 PLA result. A resident D2 instruction
        // supplies a registered entry address.
        uaddr_now = decq_empty ? d1_issue_entry_point : i_bus.entry_point;

    if (uc_exec) begin
        if (uc_reljump_taken)
            uaddr_now = uaddr + {{6{uc_alu_src[5]}}, uc_alu_src};

        casez (uc_aluop)
            ALUJMP_LJMP86: begin
                if (vm)
                    uaddr_now = uc_ljump_target;
            end
            ALUJMP_LJMPP: begin
                if (pe && !vm)
                    uaddr_now = uc_ljump_target;
            end
            ALUJMP_LJMPNP: begin
                if (pe && (cpl != 2'b00))
                    uaddr_now = uc_ljump_target;
            end
            ALUJMP_LCALL: begin
                uaddr_now = uc_ljump_target;
            end
            ALUJMP_LJUMP: begin
                if (!prot_redirect_prev)
                    uaddr_now = uc_ljump_target;
            end
            ALUJMP_RETURN: begin
                uaddr_now = microcode_return_stack[microcode_sp - 2'd1];
            end
            default: ;
        endcase

        if (prot_redirect_taken)
            uaddr_now = prot_jump_addr;

        if (set_rpl_redirect_s2)
            uaddr_now = 12'h5FB;

        if (div_overflow)
            uaddr_now = UADDR_DIVIDE_ERROR;

        if (page_fault)
            uaddr_now = UADDR_PAGE_FAULT;

        if (gate_detect_now)
            uaddr_now = 12'h5BE;
    end

    // z386x chained entry: load the next instruction's address one
    if (fast_issue)
        uaddr_now = fast_issue1 ? i_bus2.entry_point : i_bus.entry_point;
    if (gp_fault_r)
        uaddr_now = ss_fault_r ? UADDR_STACK_FAULT : UADDR_GENERAL_FAULT1;

    if (i_rni_delay && !stall && !page_fault) begin // interrupt dispatch
        if (nmi_request_active && !single_step)
            uaddr_now = UADDR_NMI;
        else if (intr_pending && EFLAGS[9] && !single_step && !inhibit_interrupts)
            uaddr_now = UADDR_HARDWARE_IRQ;
    end

    if (!reset_n)
        uaddr_now = 12'h000;
end

// Main microcode sequencer
always_ff @(posedge clk) begin
    if (!reset_n) begin
        uaddr <= 12'h000;
        uc_active <= 1'b0;
        d2_valid_r <= 1'b0;
        d2_waited_r <= 1'b0;
        d2_stale_slot_r <= 1'b0;
        microcode_sp <= 2'h0;
        i_rni_delay <= 1'b0;
        instr_eip_written <= 1'b0;
        uc_jump_taken_prev <= 1'b0;
        uc_cond_jump_taken_prev <= 1'b0;
        stack_init_pending <= 1'b0;
        dbg_first_done <= 1'b0;
        debug_ip <= 32'h0;
        gate_in_progress <= 1'b0;
        interrupt_entry <= 1'b0;
    end else begin
        if (d2_start || d2_fire || q_flush || any_fault) begin
            d2_waited_r <= 1'b0;
            d2_stale_slot_r <= 1'b0;
        end else if (stall_d2) begin
            d2_waited_r <= 1'b1;
        end else if (d2_valid && !d2_push && i_rni_delay &&
                     (uc_exec || fast_dead_slot)) begin
            // q remains on the predecessor's slot while D2 waits. Suppress
            // that stale word when D2 becomes ready on the following cycle.
            d2_stale_slot_r <= 1'b1;
        end

        // Commit the same next address that is launched to the ROM.
        uaddr <= uaddr_now;

        // Clear interrupt_entry pulse each cycle (set by NMI/INTR handlers below)
        if (!stall)
            interrupt_entry <= 1'b0;

        // Delay slot completion handling:
        if (i_rni_delay && !stall && !page_fault) begin
            dbg_first_done <= 1'b1;
            i_rni_delay <= 1'b0;
            if (single_step)
                halted <= 1'b1;
            if (!i_pop && !d2_valid)
                uc_active <= 1'b0;
        end

        // Microcode execution: flow-control side effects
        if (uc_exec) begin
            casez (uc_aluop)
                ALUJMP_PTSELE: begin
                    if (gate_in_progress)
                        gate_in_progress <= 1'b0;
                end
                ALUJMP_LCALL: begin // LCALL: Indirect Call (with delay slot)
                    microcode_return_stack[microcode_sp] <= uaddr + 12'd1;
                    microcode_sp <= microcode_sp + 2'd1;
                end
                ALUJMP_RETURN: begin // RETURN: Return from subroutine (with delay slot)
                    microcode_sp <= microcode_sp - 2'd1;
                end
                default: ;
            endcase

            // Latch jump_taken for next cycle (to suppress RNI in delay slot)
            uc_jump_taken_prev <= uc_jump_taken;
            uc_cond_jump_taken_prev <= uc_cond_jump_taken;

            // Page fault handling: latch fault info for microcode access via LPCR bus op.
            if (page_fault) begin
                i_rni_delay <= 1'b0;
                d2_valid_r <= 1'b0;
                latched_pf_code <= pg_fault_code;
                latched_pf_addr <= pg_cr2_out;
            end

            // Track RNI/RnI for delay slot handling and capture EIP at termination
            if (i_rni && uc_active && !instr_eip_written && !any_fault) begin
                // z386x: i_pop coinciding with RNI means the next instruction
                // was chain-entered and executes in the slot — no delay slot.
                if (!i_pop)
                    i_rni_delay <= 1'b1;
                if (branch_ustep_redirect)
                    debug_ip <= ea_reg;
                else if (uc_dest == DEST_EIP || uc_dest == DEST_eIP)
                    debug_ip <= is_dword ? alu_result : {EIP[31:16], alu_result[15:0]};
                else
                    debug_ip <= EIP;  // Capture EIP at termination, before next instruction increments it
            end

            if ((uc_dest == DEST_EIP || uc_dest == DEST_eIP) && in_rpti_routine) begin
                instr_eip_written <= 1'b1;
            end

            // RPTI restart
            if (i_rni && uc_active && instr_eip_written && !stall) begin
                uc_active <= 1'b0;  // Stop microcode execution until restart
                // Don't set halted or dbg_first_done - instruction is restarting, not completing
            end

            // Call gate detection
            if (gate_detect_now) begin
                // Push return addr so second LD_DESCRIPTOR's RETURN reaches this SDEL address
                microcode_return_stack[microcode_sp] <= uc_addr;
                microcode_sp <= microcode_sp + 2'd1;
                gate_in_progress <= 1'b1;         // prevent re-detection
            end

        end

        // Suppress delay slot execution after fault triggers (must be outside if(uc_exec))
        fault_suppress_delay_slot <= any_fault || any_fault_r || (fault_suppress_delay_slot && stall);

        // z386x: a fault must dissolve a chain-armed entry (e.g. a store that
        // chained out at i_pop and then faulted at its WR issue) so the next
        // instruction is not popped into the fault redirect.
        if (any_fault)
            d2_valid_r <= 1'b0;

        // Normal macro entry starts D2; the decoded queue is not popped yet.
        if (i_entry)
            d2_valid_r <= 1'b1;

        // pop queue and load all instruction registers
        if (i_pop) begin
            uc_active <= 1'b1;
            d2_valid_r <= 1'b0;
            i_first <= 1'b1;
            stack_init_pending <= i_bus.stack_op;
            instr_eip_written <= 1'b0;
            // TMPeIP/TMPeSP writes moved to GPR block (single-driver)
            gate_in_progress <= 1'b0;
        end

        // z386x chained entry: the next decq entry pops in the next cycle
        // (d2_valid stays up through a FAST->FAST chain).
        if (fast_issue)
            d2_valid_r <= 1'b1;

        if (!stall) begin
            if (stack_init_pending && !i_pop) stack_init_pending <= 1'b0;
            if (i_first && !i_pop) i_first <= 1'b0;  // chained i_pop re-arms i_first
        end

        // Queue flush: clear wins over a normal D2 start.
        if (q_flush) begin
            d2_valid_r <= 1'b0;
            if (pe_mode_toggle_now)
                uc_active <= 1'b0;
        end

        // Interrupt recognition at instruction completion. MUST be last in the always_ff
        if (i_rni_delay && !stall && !page_fault) begin
            if (nmi_request_active && !single_step) begin
                uc_active <= 1'b1;
                interrupt_entry <= 1'b1;
                d2_valid_r <= 1'b0;
            end else if (intr_pending && EFLAGS[9] && !single_step && !inhibit_interrupts) begin
                uc_active <= 1'b1;
                interrupt_entry <= 1'b1;
                d2_valid_r <= 1'b0;
            end
        end
    end
end

// Microcode ROM address tracking. uc comes directly from the ROM output.
always_ff @(posedge clk) begin
    if (!reset_n) begin
        uc_addr_mem_r <= 12'h0;
        uc_addr <= 12'h0;
    end else begin
        if (microcode_rom_addr_ce)
            uc_addr_mem_r <= microcode_rom_addr;
        if (microcode_rom_ce)
            uc_addr <= uc_addr_mem_r;
    end
end

// synthesis translate_off
// v53 lifecycle oracle. This block observes the existing implementation; it
// must not alter execution. A chained successor may start on the same edge the
// predecessor fires, so pending state is replaced rather than cleared then.
reg        d2_oracle_pending_r;
reg [11:0] d2_oracle_entry_r;
reg        d2_fire_prev_r;
reg [11:0] d2_fire_entry_r;
reg        i_first_prev_r;
integer    d2_age_r;
longint    d2_stat_decode;
longint    d2_stat_start;
longint    d2_stat_fire;
longint    d2_stat_first;
longint    d2_stat_hold;
longint    d2_stat_cancel;
longint    d2_stat_chain;
longint    d2_stat_latency;
integer    d2_stat_latency_max;

wire d2_oracle_cancel = q_flush || any_fault_pop || interrupt_at_boundary ||
                        interrupt_entry;

always @(posedge clk) begin
    if (!reset_n) begin
        d2_oracle_pending_r <= 1'b0;
        d2_oracle_entry_r <= 12'h000;
        d2_fire_prev_r <= 1'b0;
        d2_fire_entry_r <= 12'h000;
        i_first_prev_r <= 1'b0;
        d2_age_r <= 0;
        d2_stat_decode <= 0;
        d2_stat_start <= 0;
        d2_stat_fire <= 0;
        d2_stat_first <= 0;
        d2_stat_hold <= 0;
        d2_stat_cancel <= 0;
        d2_stat_chain <= 0;
        d2_stat_latency <= 0;
        d2_stat_latency_max <= 0;
    end else begin
        d2_fire_prev_r <= d2_fire;
        i_first_prev_r <= i_first;

        if (dec_pop_now)
            d2_stat_decode <= d2_stat_decode + 1;
        if (d2_start)
            d2_stat_start <= d2_stat_start + 1;
        if (d2_fire)
            d2_stat_fire <= d2_stat_fire + 1;
        if (i_first && !i_first_prev_r)
            d2_stat_first <= d2_stat_first + 1;
        if (d2_valid && !d2_ready)
            d2_stat_hold <= d2_stat_hold + 1;
        if (d2_start && d2_fire)
            d2_stat_chain <= d2_stat_chain + 1;

        if (d2_fire_prev_r) begin
            if (!i_first)
                $fatal(1, "D2 lifecycle: fire was not followed by first EX");
            if (uc_addr !== d2_fire_entry_r)
                $fatal(1, "D2 lifecycle: EX uaddr=%03x expected=%03x",
                       uc_addr, d2_fire_entry_r);
            if (i.entry_point !== d2_fire_entry_r)
                $fatal(1, "D2 lifecycle: instruction entry=%03x expected=%03x",
                       i.entry_point, d2_fire_entry_r);
        end

        if (d2_oracle_cancel) begin
            if (d2_oracle_pending_r)
                d2_stat_cancel <= d2_stat_cancel + 1;
            d2_oracle_pending_r <= 1'b0;
            d2_age_r <= 0;
        end else begin
            if (d2_valid && !d2_fire)
                d2_age_r <= d2_age_r + 1;

            if (d2_fire) begin
                if (!d2_oracle_pending_r)
                    $fatal(1, "D2 lifecycle: fire without a launched entry");
                if ((!d2_rom_mem_resident && (uaddr !== d2_oracle_entry_r)) ||
                    (d2_rom_mem_resident && (uc_addr_mem_r !== d2_oracle_entry_r)))
                    $fatal(1, "D2 lifecycle: resident addr=%03x expected=%03x waited=%b",
                           d2_rom_mem_resident ? uc_addr_mem_r : uaddr,
                           d2_oracle_entry_r, d2_waited_r);
                d2_fire_entry_r <= d2_oracle_entry_r;
                d2_stat_latency <= d2_stat_latency + d2_age_r + 1;
                if ((d2_age_r + 1) > d2_stat_latency_max)
                    d2_stat_latency_max <= d2_age_r + 1;
                d2_oracle_pending_r <= 1'b0;
                d2_age_r <= 0;
            end

            if (d2_start) begin
                if (d2_oracle_pending_r && !d2_fire)
                    $fatal(1, "D2 lifecycle: start while prior entry is pending");
                d2_oracle_pending_r <= 1'b1;
                d2_oracle_entry_r <= d2_start_entry;
                d2_age_r <= 0;
            end
        end
    end
end

final if ($test$plusargs("d2_stats"))
    $display("d2: decoded=%0d start=%0d fire=%0d first=%0d chain=%0d hold=%0d cancel=%0d avg_latency=%.2f max_latency=%0d",
             d2_stat_decode, d2_stat_start, d2_stat_fire, d2_stat_first,
             d2_stat_chain, d2_stat_hold, d2_stat_cancel,
             d2_stat_fire ? (1.0 * d2_stat_latency / d2_stat_fire) : 0.0,
             d2_stat_latency_max);
// synthesis translate_on

// Interrupt State Machine:
always_ff @(posedge clk) begin
    if (!reset_n) begin
        nmi_prev <= 1'b0;
        nmi_pending <= 1'b0;
        nmi_blocked <= 1'b0;
        intr_pending <= 1'b0;
        intr_latch_inhibit <= 1'b0;
        inhibit_interrupts <= 1'b0;
    end else begin
        // NMI edge detection (every cycle)
        nmi_prev <= nmi;
        if (nmi_edge && !nmi_accept_boundary)
            nmi_pending <= 1'b1;

        // INTR is level-sensitive: latch when asserted with IF=1
        // intr_latch_inhibit prevents re-latching after CINTLA clears the latch
        if (!intr)
            intr_latch_inhibit <= 1'b0;
        if (intr && EFLAGS[9] && !intr_latch_inhibit)
            intr_pending <= 1'b1;

        // STI shadow: suppress interrupt recognition for one instruction after STI
        if (i_rni && i.opcode == 8'hFB)
            inhibit_interrupts <= 1'b1;
        else if (i_rni && inhibit_interrupts)
            inhibit_interrupts <= 1'b0;

        // Microcode interrupt ops (inside uc_exec)
        if (uc_exec) begin
            case (uc_aluop)
                ALUJMP_CLRNMI: nmi_blocked <= 1'b0;
                ALUJMP_SETNMI: nmi_blocked <= 1'b1;
                ALUJMP_CINTLA: begin
                    intr_pending <= 1'b0;
                    intr_latch_inhibit <= 1'b1;
                end
                default: ;
            endcase
        end

        // NMI entry: clear pending (matches completion handler in sequencer)
        if (nmi_accept_boundary)
            nmi_pending <= 1'b0;
    end
end

// Instruction Signals (latched at i_pop)
always_ff @(posedge clk) begin
    if (!reset_n) begin
        i <= '0;
        instr_is_cmp <= 1'b0;
        instr_is_shxd <= 1'b0;
        instr_cf <= 1'b0;
        instr_ind_is_ea <= 1'b0;
        jcc_active <= 1'b0;
        fast_i_r <= 1'b0;
        fast_multi_r <= 1'b0;
        fast_jcc_r <= 1'b0;
        fast_commit_sel_r <= FAST_COMMIT_NONE;
        fast_keep_slot_r <= 1'b0;
        branch_ustep_r <= 1'b0;
        branch_ustep_jcc_r <= 1'b0;
    end else if (i_pop) begin
        i <= i_bus;
        fast_i_r <= fast_pop_fast;
        fast_multi_r <= fast_pop_fast && fast_pop_fc.multi_word;
        fast_jcc_r <= fast_pop_fast && fast_pop_fc.jcc;
        fast_wf_r <= fast_pop_fc.writes_flags;
        fast_commit_sel_r <= fast_pop_fc.commit_sel;
        fast_keep_slot_r <= fast_pop_fc.keep_slot;
        branch_ustep_r <= fast_pop_fast &&
                          ((d2_kind == RECIPE_EARLY_BRANCH) || i_bus.opcode == 8'hE8) &&
                          i_bus.data32 && (!i_bus.stack_op || i_bus.opcode == 8'hE8) &&
                          (!fast_pop_fc.jcc || !jcc_pop_unsafe);
        branch_ustep_jcc_r <= fast_pop_fc.jcc;
        // LSS/LFS/LGS (0F B2/B4/B5): put opcode in i.immediate for microcode XOR trick
        if (i_bus.has_0f && (i_bus.opcode == 8'hB2 || i_bus.opcode == 8'hB4 || i_bus.opcode == 8'hB5))
            i.immediate <= {24'h0, i_bus.opcode};
        instr_is_shxd <= i_bus.has_0f && ((i_bus.opcode == 8'hA4) || (i_bus.opcode == 8'hA5) ||
                                          (i_bus.opcode == 8'hAC) || (i_bus.opcode == 8'hAD));
        instr_cf <= eflags_fwd[0];      // RCL/RCR carry-in: forward the committing CF (eflags_fwd)
        instr_is_cmp <= i_bus.opcode[7:2] == 6'b100000 || i_bus.opcode[7:3] == 5'b00111;
        instr_ind_is_ea <= i_bus.has_modrm || i_bus.stack_op || i_bus.has_moffs;
        // Pre-decode ALU group op: eliminates i.opcode/i.modrm muxes from ALU critical path
        alu_grp_op <= i_bus.opcode[7] ? i_bus.modrm[5:3] : i_bus.opcode[5:3];
        // Track Jcc instruction for alu_src_r substitution in BUSOP_IND_PLUS_ALU
        jcc_active <= (i_bus.opcode[7:4] == 4'b0111) ||
                      (i_bus.has_0f && i_bus.opcode[7:4] == 4'b1000);
        // Pre-decode: eliminates opcode comparisons from execution critical paths
        instr_is_loop <= (i_bus.opcode[7:1] == 7'b1110000);  // E0/E1
        // BT operation selector: immediate form (BA) uses modrm[4:3], register forms use opcode[4:3]
        instr_bt_sel <= (i_bus.opcode == 8'hBA) ? i_bus.modrm[4:3] : i_bus.opcode[4:3];
        // MOVZX/MOVSX/CBW pre-decode: opcode bits select sign/zero and byte/word source
        if (i_bus.opcode[0] || ~i_bus.opcode[5])              // 98, B7, BF
            instr_szext_op <= i_bus.opcode[3] ? ALU_SEXT : ALU_ZEXT;
        else                                                    // B6, BE
            instr_szext_op <= i_bus.opcode[3] ? ALU_SEXT_B : ALU_ZEXT_B;
    end
    if (interrupt_entry)
        jcc_active <= 1'b0;  // Clear on interrupt — prevent is_jcc from using stale displacement
    // z386x: fault/interrupt redirects run handler microcode while the popped
    // instruction's FAST state is stale — the handler's own RNI word must not
    // fire fast_last (sideband commit) or chain. Dissolve it.
    if (any_fault || any_fault_r || interrupt_entry) begin
        fast_i_r <= 1'b0;
        fast_multi_r <= 1'b0;
        fast_jcc_r <= 1'b0;
        fast_commit_sel_r <= FAST_COMMIT_NONE;
        branch_ustep_r <= 1'b0;
    end
end

// z386x fast path phases
assign fast_active = i_first && fast_i_r;
assign fast_last   = fast_i_r && uc_active && i_rni;

// synthesis translate_off
// Shape check: the RNI word a FAST class retires on must match its commit kind.
always @(posedge clk)
    if (reset_n && fast_last && uc_exec) begin
        automatic logic ok;
        case (fast_commit_sel_r)
            FAST_COMMIT_ALU:    ok = (uc[17:11] == ALUJMP_ALU) || (uc[17:11] == ALUJMP_INCDEC) ||
                                     (uc[17:11] == ALUJMP_PASS);
            FAST_COMMIT_SHIFT:  ok = (uc[17:11] == ALUJMP_SHIFT2);
            FAST_COMMIT_MEM:    ok = uc[38];                    // load/POP: the RNI word is bus/DLY
            FAST_COMMIT_SIGSRC: ok = (uc[17:11] == ALUJMP_BITS16) || (uc[17:11] == ALUJMP_BITS32);
            FAST_COMMIT_ESP:    ok = uc[40];                    // PUSH: the RNI word issues the WR
            default:            ok = 1'b1;                      // NONE: CMPTST/LEA/store shapes vary
        endcase
        if (!ok)
            $display("%0t FAST-CLASS MISMATCH: commit=%0d vs uc_addr=%03x aluop=%02x opcode=%02x modrm=%02x",
                     $time, fast_commit_sel_r, uc_addr, uc[17:11], i.opcode, i.modrm);
        if ((fast_commit_sel_r == FAST_COMMIT_ALU) &&
            (uc_dest != DEST_USTEP_ALU))
            $display("%0t USTEP-DEST MISMATCH: dest=%02x uc_addr=%03x opcode=%02x",
                     $time, uc_dest, uc_addr, i.opcode);
    end
// synthesis translate_on

always_ff @(posedge clk) begin
    if (!reset_n) begin
        fast_dead_slot <= 1'b0;
        fast_memc_pending <= 1'b0;
        fast_shc_pending <= 1'b0;
    end else if (!stall) begin
        // Memory classes keep their slot: the OPR_R writeback / write-DLY is
        // real work when unchained. Only chained-away slots are replaced.
        fast_dead_slot <= fast_last && uc_exec && !i_pop && !fast_keep_slot_r;
        // Deferred MEM commit: the load's slot was chained away; hardware
        // writes the (settled) OPR_R to the destination in that cycle.
        fast_memc_pending <= fast_last && uc_exec && i_pop && !any_fault &&
                             (fast_commit_sel_r == FAST_COMMIT_MEM);
        if (fast_last && uc_exec && i_pop && (fast_commit_sel_r == FAST_COMMIT_MEM)) begin
            fast_memc_dst  <= i.dst_reg_sel;
            fast_memc_size <= op_size;
        end
        // Deferred SHIFT commit: ALWAYS deferred (not just when chained) so
        // the barrel -> commit-mux -> GPR write arc disappears entirely; the
        // barrel output only reaches this data register at fast_last.
        fast_shc_pending <= fast_last && uc_exec && !any_fault &&
                            (fast_commit_sel_r == FAST_COMMIT_SHIFT);
        if (fast_last && uc_exec && (fast_commit_sel_r == FAST_COMMIT_SHIFT)) begin
            fast_shc_dst  <= i.dst_reg_sel;
            fast_shc_size <= op_size;
            fast_shc_data <= shift_result;
        end
    end
end

// synthesis translate_off
// Dead-slot cause breakdown (unchained fast_last cycles = reclaimable slot
// candidates). Classified by the state of the would-be successor (queue head)
// at the fast_last cycle. Printed at $finish via the final block.
int unsigned ds_total, ds_empty, ds_seq, ds_flags, ds_ea, ds_memc, ds_intr, ds_other, ds_other_1w, ds_keepslot;
always @(posedge clk) begin
    if (reset_n && !stall && fast_last && uc_exec && !i_pop && fast_keep_slot_r)
        ds_keepslot++;   // unchained store/RMW/POP boundary (slot is real work)
    if (reset_n && !stall && fast_last && uc_exec && !i_pop && !fast_keep_slot_r) begin
        ds_total++;
        if (decq_empty)                                        ds_empty++;
        else if (!fast_pop_fc.fast || fast_off)                ds_seq++;
        else if (interrupt_pending || single_step)             ds_intr++;
        else if (fast_pop_fc.reads_flags && fast_wf_r && !fast_pop_fc.jcc) ds_flags++;
        else if (fast_pop_fc.uses_ea && fast_ea2_conflict)     ds_ea++;
        else if (fast_memc_confN)                              ds_memc++;
        else if (!fast_multi_r)                                ds_other_1w++;  // chain1 missed at pop (decq_has2 lookahead?)
        else                                                   ds_other++;
    end
end
final begin
    if (ds_total > 0)
        $display("z386x dead-slot breakdown: total=%0d empty=%0d seq=%0d flags=%0d ea=%0d memc=%0d intr=%0d other1w=%0d other=%0d keepslot=%0d",
                 ds_total, ds_empty, ds_seq, ds_flags, ds_ea, ds_memc, ds_intr, ds_other_1w, ds_other, ds_keepslot);
end
// synthesis translate_on


//=============================================================================
// Unit 8: Write-back - deferred commits that retire one cycle after their producer: 
//         the two-cycle ALU flag retirement and the two-cycle shift.
//=============================================================================

// Two-cycle ALU flag retirement
// Details: doc/z386x/implementation_notes.md#src-24-z386x-z386-sv-2735
reg        flag2_eflags_p;     // commit to EFLAGS this cycle (producer had uc[37])
reg        flag2_ucflags_p;    // commit to uc_flags this cycle
reg [31:0] flag2_result_r;     // raw ALU result of the producer
reg        flag2_cf_r, flag2_af_r, flag2_of_r, flag2_zsp_r;
reg [1:0]  flag2_size_r;
wire flag2_class_uc = (uc_aluop == ALUJMP_ALU)    || (uc_aluop == ALUJMP_INCDEC) ||
                      (uc_aluop == ALUJMP_CMPTST) || (uc_aluop == ALUJMP_AND)    ||
                      (uc_aluop == ALUJMP_OR)     || (uc_aluop == ALUJMP_XOR)    ||
                      (uc_aluop == ALUJMP_ADD)    || (uc_aluop == ALUJMP_ADC)    ||
                      (uc_aluop == ALUJMP_SUB)    || (uc_aluop == ALUJMP_CMP)    ||
                      (uc_aluop == ALUJMP_AAAAAS) || (uc_aluop == ALUJMP_DAADAS);
always_ff @(posedge clk) begin
    if (!reset_n) begin
        flag2_eflags_p  <= 1'b0;
        flag2_ucflags_p <= 1'b0;
    end else begin
        flag2_eflags_p  <= uc_exec && alu_update_flags;
        flag2_ucflags_p <= uc_exec && flag2_class_uc;
        if (uc_exec && flag2_class_uc) begin
            flag2_result_r <= alu_result;
            flag2_cf_r     <= alu_flags[0];
            flag2_af_r     <= alu_flags[4];
            flag2_of_r     <= alu_flags[11];
            flag2_zsp_r    <= alu_zsp_update;
            flag2_size_r   <= op_size;
        end
    end
end
wire flag2_zf = (flag2_size_r == 2'd0) ? (flag2_result_r[7:0]  == 8'h0)  :
                (flag2_size_r == 2'd1) ? (flag2_result_r[15:0] == 16'h0) :
                                         (flag2_result_r       == 32'h0);
wire flag2_sf = (flag2_size_r == 2'd0) ? flag2_result_r[7]  :
                (flag2_size_r == 2'd1) ? flag2_result_r[15] :
                                         flag2_result_r[31];
wire flag2_pf = ~^flag2_result_r[7:0];

// Two-cycle shifter flag retirement
// Details: doc/z386x/implementation_notes.md#src-24-z386x-z386-sv-2777
reg sh2_commit_p, sh2_we_zsp, sh2_we_of;
reg sh2_cf, sh2_of, sh2_zf, sh2_sf, sh2_pf;
always_ff @(posedge clk) begin
    if (!reset_n)
        sh2_commit_p <= 1'b0;
    else begin
        sh2_commit_p <= uc_exec && (uc_aluop == ALUJMP_SHIFT2) && (shift_size != 5'd0);
        if (uc_exec && (uc_aluop == ALUJMP_SHIFT2) && (shift_size != 5'd0)) begin
            sh2_we_zsp <= shift_SET_Nzs;
            sh2_pf <= shift_pf;
            sh2_zf <= shift_zf;
            sh2_sf <= shift_sf;
            sh2_we_of <= 1'b0;
            if (instr_is_shxd) begin
                sh2_cf <= i.opcode[3] ? shift_last_out_lsb : shift_last_out_msb;
                if (shift_size == 5'd1) begin
                    sh2_we_of <= 1'b1;
                    sh2_of <= i.opcode[3] ? (shift_result[sh1_width_r-1] ^ shift_result[sh1_width_r-2]) :
                                            (shift_result[sh1_width_r-1] ^ shift_last_out_msb);
                end
            end else begin
                case (shift_op)
                    SHL,SAL: sh2_cf <= shift_overflow ? (shift_eq_width ? shift_eq_cf : 1'b0) : shift_last_out_msb;
                    RCL:     sh2_cf <= shift_last_out_msb;
                    SHR:     sh2_cf <= shift_overflow ? (shift_eq_width ? shift_eq_cf : 1'b0) : shift_last_out_lsb;
                    SAR:     sh2_cf <= shift_overflow ? shift_result[sh1_width_r-1] : shift_last_out_lsb;
                    RCR:     sh2_cf <= shift_last_out_lsb;
                    ROL:     sh2_cf <= shift_result[0];
                    ROR:     sh2_cf <= shift_result[sh1_width_r-1];
                endcase
                if (shift_size == 5'd1) begin
                    case (shift_op)
                        SHL:     begin sh2_we_of <= 1'b1; sh2_of <= shift_result[sh1_width_r-1] ^ shift_last_out_msb; end
                        SHR:     begin sh2_we_of <= 1'b1; sh2_of <= shift_lo[sh1_width_r-1]; end
                        SAR:     begin sh2_we_of <= 1'b1; sh2_of <= 1'b0; end
                        ROR,RCR: begin sh2_we_of <= 1'b1; sh2_of <= shift_result[sh1_width_r-1] ^ shift_result[sh1_width_r-2]; end
                        default: ;
                    endcase
                end
                // ROL/RCL: OF computed for ALL counts (real 386 behavior)
                if (shift_op == ROL) begin sh2_we_of <= 1'b1; sh2_of <= shift_result[sh1_width_r-1] ^ shift_result[0]; end
                if (shift_op == RCL) begin sh2_we_of <= 1'b1; sh2_of <= shift_result[sh1_width_r-1] ^ shift_last_out_msb; end
            end
        end
    end
end

// EFLAGS as it will be after this cycle's pending two-cycle flag commit
wire [31:0] eflags_fwd =
    sh2_commit_p ? { EFLAGS[31:12],
                     sh2_we_of  ? sh2_of : EFLAGS[11],
                     EFLAGS[10:8],
                     sh2_we_zsp ? sh2_sf : EFLAGS[7],
                     sh2_we_zsp ? sh2_zf : EFLAGS[6],
                     EFLAGS[5:3],
                     sh2_we_zsp ? sh2_pf : EFLAGS[2],
                     EFLAGS[1],
                     sh2_cf } :
    flag2_eflags_p ? { EFLAGS[31:12],
                       flag2_of_r,
                       EFLAGS[10:8],
                       flag2_zsp_r ? flag2_sf : EFLAGS[7],
                       flag2_zsp_r ? flag2_zf : EFLAGS[6],
                       EFLAGS[5],
                       flag2_af_r,
                       EFLAGS[3],
                       flag2_zsp_r ? flag2_pf : EFLAGS[2],
                       EFLAGS[1],
                       flag2_cf_r } :
    EFLAGS;


//=============================================================================
// Unit 9: Data Unit (ALU, register file, barrel shifter)
//=============================================================================

// SIGMA Update (ALU Accumulator)
always_ff @(posedge clk) begin
    if (!reset_n) begin
        SIGMA <= 32'h0;
    end else begin

    if (i_pop & i_bus.stack_op) begin
        automatic logic [31:0] stack_delta = i_bus.data32 ? 32'd4 : 32'd2;
        if (seg_cache[SEG_SS].D_B) begin
            // B=1: 32-bit stack - use full ESP
            SIGMA <= i_bus.stack_dir ? (forwarded_esp + stack_delta) : (forwarded_esp - stack_delta);
        end else begin
            // B=0: 16-bit stack - only lower 16 bits change, upper preserved
            automatic logic [15:0] sp = forwarded_esp[15:0];
            automatic logic [15:0] new_sp = i_bus.stack_dir ? (sp + stack_delta[15:0]) : (sp - stack_delta[15:0]);
            SIGMA <= {forwarded_esp[31:16], new_sp};
        end
    end else if (gate_detect_now) begin
        // Call gate: set SIGMA for CS write at 2F3 (gates jump to FARJUMP2 which skips PASS at 2ED)
        SIGMA <= {16'h0, TMPC[31:16]};
    end else if (uc_exec) begin
        case (uc_aluop)
            ALUJMP_ALU,
            ALUJMP_INCDEC,
            ALUJMP_IMCS,
            ALUJMP_SZ_EXT,
            ALUJMP_AND,
            ALUJMP_OR,
            ALUJMP_XOR,
            ALUJMP_SIGN,
            ALUJMP_ADD,
            ALUJMP_ADC,
            ALUJMP_SUB,
            ALUJMP_CMP,
            ALUJMP_PASS,
            ALUJMP_PASS2,
            ALUJMP_AAAAAS,
            ALUJMP_DAADAS,
            ALUJMP_SERECO:
            begin
                SIGMA <= alu_result;
            end

            ALUJMP_SHIFT1: begin
                SIGMA <= alu_dst;
                if (!instr_is_shxd) case (i.modrm[5:3])
                    RCL:         SIGMA <= (instr_cf << (width-1))  |
                                          ((alu_dst & shift_width_mask) >> 1);  // RCL
                    RCR:         SIGMA <= {alu_dst, instr_cf};            // RCL
                    SHL,SHR,SAL: SIGMA <= 0;                              // SHL/SHR/SAL
                    SAR:         SIGMA <= op_size == 2'd0 ? {32{alu_dst[7]}} :
                                        op_size == 2'd1 ? {32{alu_dst[15]}} :
                                        {32{alu_dst[31]}};                // SAR
                    default:     SIGMA <= alu_dst;                        // ROL, ROR
                endcase
            end

            ALUJMP_SHIFT,
            ALUJMP_SHIFT2,
            ALUJMP_BITTST:
            begin
                SIGMA <= shift_result;
            end

            ALUJMP_IMUL3: begin
                // Extract upper portion based on operand size
                SIGMA <= mul_upper;
            end

            ALUJMP_IMUL4: begin
                // Extract upper portion based on operand size (uncorrected)
                SIGMA <= mul_upper;
            end

            // Signed add-and-shift multiplication: subtract MULTMP from upper half if multiplier was negative
            ALUJMP_SZ_EX2: begin
                SIGMA <= 0;
            end

            ALUJMP_DIV5: begin
                // DIV5 final correction: if remainder was negative, add divisor
                if (!div_r_nonneg) begin
                    SIGMA <= div_op_by_size(DIV_OP_MASK, SIGMA + div_divisor_masked, op_size);
                end
            end
            ALUJMP_PREDIV: begin
                // PREDIV: Save signs, compute absolute value, and do first DIV7 iteration
                SIGMA <= div_iter_r_next;
            end
            ALUJMP_IDIV1: begin
                // IDIV1: Correct remainder sign (same sign as original dividend)
                if (idiv_dividend_neg)
                    SIGMA <= div_op_by_size(DIV_OP_NEGATE, SIGMA, op_size);
            end
            ALUJMP_IDIV2: begin
                // IDIV2: Correct quotient sign (negative if signs differed)
                SIGMA <= (idiv_dividend_neg ^ idiv_divisor_neg) ?
                         div_op_by_size(DIV_OP_NEGATE, RESULT, op_size) :
                         div_op_by_size(DIV_OP_MASK, RESULT, op_size);
            end
            ALUJMP_DIV7: begin
                // DIV7: one non-restoring iteration
                SIGMA <= div_iter_r_next;
            end
            default: begin
            end
        endcase
    end

    // Deferred fault override: clear SIGMA one cycle after fault fires.
    if (any_fault_r)
        SIGMA <= 32'h0;

    end
end

// Internal flags update (for microcode conditionals like JG, JNC)
always_ff @(posedge clk) begin
    if (!reset_n) begin
        uc_flags <= 32'h0000_0002;  // Bit 1 is always 1
    end else begin
        if (i_pop)
            uc_flags <= EFLAGS;
        // Two-cycle ALU flag commit
        if (flag2_ucflags_p) begin
            uc_flags[0]  <= flag2_cf_r;
            uc_flags[4]  <= flag2_af_r;
            uc_flags[11] <= flag2_of_r;
            if (flag2_zsp_r) begin
                uc_flags[2] <= flag2_pf;
                uc_flags[6] <= flag2_zf;
                uc_flags[7] <= flag2_sf;
            end
        end
        // Two-cycle shifter flag commit
        if (sh2_commit_p) begin
            uc_flags[0] <= sh2_cf;
            if (sh2_we_zsp) begin
                uc_flags[2] <= sh2_pf;
                uc_flags[6] <= sh2_zf;
                uc_flags[7] <= sh2_sf;
            end
            if (sh2_we_of)
                uc_flags[11] <= sh2_of;
        end
        // Shift/BITTST CF stays single-cycle
        if (!i_pop && uc_exec) begin
            case (uc_aluop)
                ALUJMP_BITTST:
                    uc_flags[0] <= shift_result[0];
                ALUJMP_SHIFT2:
                    if (shift_size != 5'd0)
                        uc_flags[0] <= shift_cf;
                default: ;  // ALU-class retires via flag2_* above
            endcase
        end
    end
end

// MUL/IMUL overflow: check if upper portion is sign-extension of result
function automatic logic mul_overflow_flag(
    input [31:0] upper, input sign, input [1:0] op_size);
    case (op_size)
        2'd0:    mul_overflow_flag = (upper[7:0]  != {8{sign}});
        2'd1:    mul_overflow_flag = (upper[15:0] != {16{sign}});
        default: mul_overflow_flag = (upper       != {32{sign}});
    endcase
endfunction

// EFLAGS update
always_ff @(posedge clk) begin
    if (!reset_n) begin
        EFLAGS <= 32'h0000_0002;  // Bit 1 is always 1
        clear_if_pending <= 1'b0;
        misc1_flag <= 1'b0;
        misc2_flag <= 1'b0;
        error_code_flag <= 1'b0;
        interrupt_hw <= 1'b0;
    end else begin
        if (i_pop && !halted) begin
            clear_if_pending <= 1'b0;
            misc1_flag <= 1'b0;
            misc2_flag <= 1'b0;
            error_code_flag <= 1'b0;
            interrupt_hw <= 1'b0;
            // jcc_active moved to instruction signals block (single-driver)
        end
        // Two-cycle ALU flag commit (producer ran last cycle)
        if (flag2_eflags_p) begin
            EFLAGS[0]  <= flag2_cf_r;
            EFLAGS[1]  <= 1'b1;
            EFLAGS[4]  <= flag2_af_r;
            EFLAGS[11] <= flag2_of_r;
            if (flag2_zsp_r) begin
                EFLAGS[2] <= flag2_pf;
                EFLAGS[6] <= flag2_zf;
                EFLAGS[7] <= flag2_sf;
            end
        end
        // Two-cycle shifter flag commit (SHIFT2 ran last cycle).
        if (sh2_commit_p) begin
            EFLAGS[0] <= sh2_cf;
            if (sh2_we_zsp) begin
                EFLAGS[2] <= sh2_pf;
                EFLAGS[6] <= sh2_zf;
                EFLAGS[7] <= sh2_sf;
            end
            if (sh2_we_of)
                EFLAGS[11] <= sh2_of;
        end
        if (uc_exec) begin
            case (uc_aluop)
                ALUJMP_FLGOPS: begin
                    // FLGOPS - CLC/STC/CMC/CLD/STD/CLI/STI
                    case (i.opcode[3:0])
                        4'h5: EFLAGS[0] <= ~EFLAGS[0];  // CMC: complement CF
                        4'h8: EFLAGS[0] <= 1'b0;        // CLC: clear CF
                        4'h9: EFLAGS[0] <= 1'b1;        // STC: set CF
                        4'hA: EFLAGS[9] <= 1'b0;        // CLI: clear IF
                        4'hB: EFLAGS[9] <= 1'b1;        // STI: set IF
                        4'hC: EFLAGS[10] <= 1'b0;       // CLD: clear DF
                        4'hD: EFLAGS[10] <= 1'b1;       // STD: set DF
                        default: ;
                    endcase
                end
                ALUJMP_BITTST: begin
                    // BITTST: BT/BTS/BTR/BTC - rotate data right, test bit 0
                    EFLAGS[0] <= shift_result[0];  // CF = bit 0 of the shifted value
                end
                ALUJMP_DIV5: begin
                    // AAM: ensure CF=0 before the ADC micro-op consumes it.
                    EFLAGS[0] <= 1'b0;
                end
                ALUJMP_CLZF: begin
                    // CLZF (BSR/BSF): Clear Zero Flag to indicate bit was found
                    EFLAGS[6] <= 1'b0;
                end
                ALUJMP_SEZF: begin
                    // SEZF (LAR/LSL/ARPL): Set Zero Flag to indicate success
                    EFLAGS[6] <= 1'b1;
                end
                ALUJMP_CLI: begin
                    // PRIMIF: Prime clearing of IF for INT instruction
                    // The actual clearing happens when CLRTFI executes
                    clear_if_pending <= 1'b1;
                end
                ALUJMP_SMISC1: begin
                    // SMISC1: Set MISC1 flag (used by INT handler to distinguish INT from call gate)
                    misc1_flag <= 1'b1;
                end
                ALUJMP_SMISC2: begin
                    // SMISC2: Set MISC2 flag (used by cross-privilege handler)
                    misc2_flag <= 1'b1;
                end
                ALUJMP_SERRCF: begin
                    // SERRCF: Set error code flag (fault handlers set this before INT dispatch)
                    error_code_flag <= 1'b1;
                end
                ALUJMP_SINTHW: begin
                    // SINTHW: Set interrupt_hw flag (exception/HW IRQ, not software INT n)
                    interrupt_hw <= 1'b1;
                end
                ALUJMP_CLT: begin
                    // CLRTFI: Clear TF (always) and IF (if primed by PRIMIF)
                    // Used by INT/exception handling to enter handler with interrupts disabled
                    EFLAGS[8] <= 1'b0;  // Always clear TF
                    if (clear_if_pending)
                        EFLAGS[9] <= 1'b0;  // Clear IF only if primed
                    clear_if_pending <= 1'b0;  // Reset the pending flag
                end
                // SHIFT2 architectural flags retire one cycle later via the
                // sh2_* commit (above)
                ALUJMP_SHIFT2: ;
                ALUJMP_SHIFT: begin
                    // AAD: SHIFT precedes ADC, clear CF so ADC behaves like ADD.
                    if (i.opcode == 8'hD5)
                        EFLAGS[0] <= 1'b0;
                end
                ALUJMP_SZ_EX2: begin
                    logic target_sign;
                    logic ovf;
                    target_sign = op_size == 2'd0 ? SIGMA[7] :
                                  op_size == 2'd1 ? SIGMA[15] :
                                  SIGMA[31];
                    if (!is_signed_mul) target_sign = 1'b0;
                    // MUL/IMUL finish - set CF/OF based on whether result fits
                    ovf = mul_overflow_flag(TMPD, target_sign, op_size);
                    EFLAGS[0] <= ovf;
                    EFLAGS[11] <= ovf;
                end
                ALUJMP_IMCS: begin
                    // IMCS: IMUL Correct Sign - set CF/OF for two/three operand IMUL overflow
                    // With DSP multiplier, the upper portion is already correct (no MULFIX correction needed)
                    logic target_sign;
                    logic [31:0] sigma_upper;
                    logic ovf;

                    target_sign = is_signed_mul ? (
                        (op_size == 2'd0 && SIGMA[7]) ||
                        (op_size == 2'd1 && SIGMA[15]) ||
                        (op_size == 2'd2 && SIGMA[31])
                    ) : 1'b0;

                    // Select upper portion based on operand size
                    case (op_size)
                        2'd0:    sigma_upper = {24'h0, SIGMA[15:8]};
                        2'd1:    sigma_upper = {16'h0, SIGMA[31:16]};
                        default: sigma_upper = TMPD;
                    endcase
                    ovf = mul_overflow_flag(sigma_upper, target_sign, op_size);
                    EFLAGS[0] <= ovf;
                    EFLAGS[11] <= ovf;
                end
                // ALU-class arithmetic flags retire one cycle later via the
                // flag2_* commit above (two-cycle flag retirement).
                default: ;
            endcase

            // SAHF and POPF - write destination to EFLAGS
            if (uc_dest == DEST_FLAGSL) begin
                EFLAGS[7:0] <= (dest_value[7:0] & 8'hD5) | 8'h02;
            end
            if (uc_dest == DEST_FLAGS) begin
                // In protected mode: IOPL only writable at CPL=0, IF only writable when CPL <= IOPL
                // Otherwise these bits are silently preserved (no fault)
                if (pe) begin
                    EFLAGS[7:0]   <= (dest_value[7:0] & 8'hD5) | 8'h02;
                    EFLAGS[8]     <= dest_value[8];                           // TF
                    EFLAGS[9]     <= (cpl <= EFLAGS[13:12]) ? dest_value[9] : EFLAGS[9];  // IF: only if CPL <= IOPL
                    EFLAGS[11:10] <= dest_value[11:10];                       // DF, OF
                    EFLAGS[13:12] <= (cpl == 2'b00) ? dest_value[13:12] : EFLAGS[13:12];  // IOPL: only if CPL=0
                    EFLAGS[14]    <= dest_value[14];                          // NT
                    // VM, RF (bits 17:16): writable at CPL=0 in 32-bit mode
                    // IRETD: is_dword=1 → writes VM/RF from stacked EFLAGS
                    // POPF:  BITS16 at uc=804 forces is_dword=0 → VM/RF preserved
                    if (is_dword && cpl == 2'b00) begin
                        EFLAGS[16] <= dest_value[16];   // RF
                        EFLAGS[17] <= dest_value[17];   // VM
                    end
                end else begin
                    EFLAGS[15:0] <= (dest_value[15:0] & 16'h7FD5) | 16'h0002;
                end
            end
            // DEST_EFLAGS: full 32-bit EFLAGS write (used by microcode to clear/set VM, RF)
            // Used at 633 to clear VM during V86→ring0 transition: SIGMA = EFLAGS & 0xFFFF → EFLAGS
            if (uc_dest == DEST_EFLAGS) begin
                EFLAGS <= (dest_value & 32'h00037FD5) | 32'h00000002;
            end
        end
        // RF (Resume Flag, bit 16): the 386 clears RF at the successfu -- doc/z386x/core_notes_v51.md #24
        if ((i_rni_delay && i.opcode != 8'hCF && i.opcode != 8'h9D) ||
            (fast_last && uc_exec))
            EFLAGS[16] <= 1'b0;
    end
end

// SEGREG (Segment Register Operand)
always_ff @(posedge clk) begin
    if (!reset_n) begin
        seg_reg_sel <= 3'b0;
    end else if (i_pop && !halted) begin
        // Instruction start: load SEGREG and seg_reg_sel for segment instructions
        // PUSH ES (06), PUSH CS (0E), PUSH SS (16), PUSH DS (1E), POP ES (07), POP SS (17), POP DS (1F)
        if (i_bus.opcode[7:5] == 3'b000 && i_bus.opcode[2:1] == 2'b11) begin
            case (i_bus.opcode[4:3])
                2'b00: begin seg_reg_sel <= 3'd0; end  // ES
                2'b01: begin seg_reg_sel <= 3'd1; end  // CS
                2'b10: begin seg_reg_sel <= 3'd2; end  // SS
                2'b11: begin seg_reg_sel <= 3'd3; end  // DS
            endcase
        end
        // 0F A0/A1/A8/A9: PUSH/POP FS/GS
        else if (i_bus.has_0f && (i_bus.opcode == 8'hA0 || i_bus.opcode == 8'hA1 ||
                                  i_bus.opcode == 8'hA8 || i_bus.opcode == 8'hA9)) begin
            seg_reg_sel <= i_bus.opcode[3] ? 3'd5 : 3'd4;  // GS=5, FS=4
        end
        // MOV r/m,Sreg (8C) and MOV Sreg,r/m (8E)
        else if (i_bus.opcode == 8'h8C || i_bus.opcode == 8'h8E) begin
            seg_reg_sel <= i_bus.modrm[5:3];  // i.modrm reg field is segment index
        end
        // LES (C4), LDS (C5)
        else if (i_bus.opcode == 8'hC4) seg_reg_sel <= 3'd0;  // ES
        else if (i_bus.opcode == 8'hC5) seg_reg_sel <= 3'd3;  // DS
        // LSS (0F B2), LFS (0F B4), LGS (0F B5)
        else if (i_bus.has_0f && i_bus.opcode == 8'hB2) seg_reg_sel <= 3'd2;  // SS
        else if (i_bus.has_0f && i_bus.opcode == 8'hB4) seg_reg_sel <= 3'd4;  // FS
        else if (i_bus.has_0f && i_bus.opcode == 8'hB5) seg_reg_sel <= 3'd5;  // GS
    end
end

// EIP destinations use only these four sources in the canonical ROM. Keep the
// full microcode source mux off this architectural write path.
function automatic [31:0] eip_source_mux(input [5:0] source);
    case (source)
        SRC_SIGMA:  eip_source_mux = SIGMA;
        SRC_TMPG:   eip_source_mux = TMPG;
        SRC_TMPeIP: eip_source_mux = TMPeIP;
        SRC_OPR_R:  eip_source_mux = OPR_R;
        default:    eip_source_mux = 32'h0;
    endcase
endfunction
wire [31:0] eip_source_value = eip_source_mux(uc_source_shift);

// synthesis translate_off
always @(posedge clk)
    if (reset_n && uc_exec &&
        (uc_dest == DEST_EIP || uc_dest == DEST_eIP || uc_dest == DEST_IP) &&
        (eip_source_value !== alu_result))
        $fatal(1, "EIP SOURCE MUX MISMATCH: uc_addr=%03x src=%02x narrow=%08x alu=%08x",
               uc_addr, uc_source, eip_source_value, alu_result);
// synthesis translate_on

// EIP (Instruction Pointer)
always_ff @(posedge clk) begin
    if (!reset_n) begin
        EIP <= 32'h0000FFF0;  // 386 reset vector offset
    end else if (branch_ustep_redirect) begin
        EIP <= ea_reg;
    end else if (i_pop && !halted /*&& (~uc_active || i_rni_delay)*/) begin
        // z386x: a chained i_pop can land on a control transfer's fina -- doc/z386x/core_notes_v51.md #25
        if (uc_exec && fast_last && (uc_dest == DEST_eIP)) begin
            automatic logic [31:0] tgt = is_dword
                                       ? eip_source_value
                                       : {16'h0, eip_source_value[15:0]};
            if (D)
                EIP <= tgt + {27'b0, i_bus.length};
            else
                EIP <= {16'h0, tgt[15:0] + {11'b0, i_bus.length}};
        end else if (D)
            EIP <= EIP + {27'b0, i_bus.length};
        else
            EIP <= {16'h0, EIP[15:0] + {11'b0, i_bus.length}};
    end else if (uc_exec && (uc_dest == DEST_EIP || uc_dest == DEST_eIP || uc_dest == DEST_IP)) begin
        // Microcode destination write to EIP -- doc/z386x/core_notes_v51.md #26
        if (uc_dest == DEST_EIP) begin
            if (D)
                EIP <= eip_source_value;
            else
                EIP <= {16'h0, eip_source_value[15:0]};
        end else if (uc_dest == DEST_eIP) begin
            if (is_dword)
                EIP <= eip_source_value;
            else
                EIP <= {16'h0, eip_source_value[15:0]};
        end else begin
            // DEST_IP: always 16-bit
            EIP <= {16'h0, eip_source_value[15:0]};
        end
    end
end

// op_size (Operand Size) and srcreg_size
// srcreg_size differs from op_size for MOVZX/MOVSX (source smaller than dest)
always_ff @(posedge clk) begin
    if (!reset_n) begin
        op_size <= 2'd1;  // Default to word size (16-bit real mode)
        srcreg_size <= 2'd1;
        op_size_src <= 2'd1;
        srcreg_size_src <= 2'd1;
    end else if (i_pop && !halted) begin
        // Instruction start: set op_size from decoded instruction
        automatic logic init_is_setcc = i_bus.has_0f && (i_bus.opcode[7:4] == 4'b1001);  // 0F 90-9F
        automatic logic init_is_movzx_movsx = i_bus.has_0f && (i_bus.opcode[7:4] == 4'b1011) && (i_bus.opcode[2:1] == 2'b11);  // 0F B6/B7/BE/BF
        automatic logic init_is_movzx_word = init_is_movzx_movsx && i_bus.opcode[0];  // B7/BF: word source, always dword dest
        automatic logic init_is_xlat = !i_bus.has_0f && (i_bus.opcode == 8'hD7);
        automatic logic init_byte = init_is_setcc ? 1'b1 :
                                    init_is_movzx_movsx ? 1'b0 :
                                    init_is_xlat ? 1'b1 :
                                    (i_bus.has_embedded_register && i_bus.has_w_bit) ? ~i_bus.opcode[3] :
                                    i_bus.has_w_bit ? ~i_bus.opcode[0] : 1'b0;
        // op_size: destination size. B7/BF always dword (ignore 66 prefix)
        automatic logic [1:0] init_op_size = init_byte ? 2'd0 :
                                             init_is_movzx_word ? 2'd2 :
                                             (i_bus.data32 ? 2'd2 : 2'd1);
        automatic logic [1:0] init_srcreg_size = init_is_movzx_movsx ? (i_bus.opcode[0] ? 2'd1 : 2'd0) : init_op_size;
        op_size <= init_op_size;
        op_size_decode <= init_op_size;
        op_size_src <= init_op_size;
        op_size_src_decode <= init_op_size;
        // srcreg_size: for MOVZX/MOVSX, source is byte (B6/BE) or word (B7/BF)
        srcreg_size <= init_srcreg_size;
        srcreg_size_decode <= init_srcreg_size;
        srcreg_size_src <= init_srcreg_size;
        srcreg_size_src_decode <= init_srcreg_size;
    end else if (uc_exec) begin
        // Microcode BITS operations
        case (uc_aluop)
            ALUJMP_BITS8:  begin op_size <= 2'd0; srcreg_size <= 2'd0; op_size_src <= 2'd0; srcreg_size_src <= 2'd0; end
            ALUJMP_BITS16: begin op_size <= 2'd1; srcreg_size <= 2'd1; op_size_src <= 2'd1; srcreg_size_src <= 2'd1; end
            ALUJMP_BITS32: begin op_size <= 2'd2; srcreg_size <= 2'd2; op_size_src <= 2'd2; srcreg_size_src <= 2'd2; end
            ALUJMP_BITSDE: begin
                op_size <= op_size_decode;
                srcreg_size <= srcreg_size_decode;
                op_size_src <= op_size_src_decode;
                srcreg_size_src <= srcreg_size_src_decode;
            end
            default: ;
        endcase
    end
end

// FLAGS backup active flag - set at instruction start, may be updated by FLGSBA
// FLAGSB always contains FLAGS from instruction start, valid for fault handling
always_ff @(posedge clk) begin
    if (!reset_n) begin
        flags_backup_active <= 1'b0;
        FLAGSB <= 32'h0;
    end else if (interrupt_entry) begin
        flags_backup_active <= 1'b0;
    end else if (i_pop && !halted) begin
        flags_backup_active <= 1'b1;
        FLAGSB <= eflags_fwd;       // init FLAGSB at instruction start
    end else if (uc_exec && uc_aluop == ALUJMP_FLGSBA) begin
        if (!flags_backup_active) begin
            flags_backup_active <= 1'b1;
            FLAGSB <= EFLAGS;
        end
    end else if (uc_exec && uc_dest == DEST_FLAGSB) begin
        // Microcode DEST_FLAGSB - only write if not already backed up
        if (!flags_backup_active) begin
            FLAGSB <= dest_value;
        end
    end
end

// GPR and internal registers
always_ff @(posedge clk) begin
    automatic logic [31:0] dest_value;
    automatic logic [15:0] cs_value;
    dest_value = alu_dst;
    cs_value = read_cs_source_fast(uc_source);
    if (!reset_n) begin
        EAX <= 32'h0;
        ECX <= 32'h0;
        EDX <= 32'h0;
        EBX <= 32'h0;
        ESP <= 32'h0;
        EBP <= 32'h0;
        ESI <= 32'h0;
        EDI <= 32'h0;

        CS <= 16'hF000;
        DS <= 16'h0000;
        ES <= 16'h0000;
        SS <= 16'h0000;
        FS <= 16'h0000;
        GS <= 16'h0000;
        LDTR <= 16'h0000;
        TR <= 16'h0000;
        SLCTR <= 32'h0;

        CR0 <= 32'h0;       // CR0 must clear PE and PG at reset
        CR2 <= 32'h0;

        TMPB <= 32'h0;
        TMPC <= 32'h0;
        TMPD <= 32'h0;
        TMPE <= 32'h0;
        TMPF <= 32'h0;
        CSOPCD <= 32'h0;
        FSVeIP <= 32'h0;
        OPROFF <= 32'h0;
        OPR_W <= 32'h0;
        div_r_nonneg <= 1'b1;
        idiv_dividend_neg <= 1'b0;
        idiv_divisor_neg <= 1'b0;
        div_first_cycle <= 1'b0;
    end else begin
        // Deferred SHIFT commit
        if (fast_shc_pending)
            write_gpr(fast_shc_dst, fast_shc_data, fast_shc_size);
        if (uc_exec) begin
        if (uc_source == SRC_IRF2) dest_value = IND;  // use combinational IRF2

        // z386x deferred MEM commit: a chained-away load slot's
        // OPR_R -> DSTREG write, executed in parallel with the successor's
        // first cycle. Placed before the case so any same-register write by
        // the (younger) current word wins.
        if (fast_memc_pending)
            write_gpr(fast_memc_dst, OPR_R, fast_memc_size);

        // Dead legs removed : the microcode never emits direct named-GPR dest codes
        // other than EAX/EDX/ESP/EBP/eSP/AX/BP/AL/AH (writes go via
        // DSTREG/SRCREG/IRF).  Restore a leg if the ROM ever changes.
        case (uc_dest)
            DEST_EAX: EAX <= dest_value;
            DEST_EDX: EDX <= dest_value;
            DEST_ESP: ESP <= dest_value;
            // eSP: stack-pointer-aware write. B bit (SS descriptor) controls
            // whether ESP (32-bit, B=1) or SP (16-bit, B=0) is the stack pointer.
            DEST_eSP: if (pe && seg_cache[SEG_SS].D_B)
                          ESP <= dest_value;
                      else
                          ESP[15:0] <= dest_value[15:0];
            DEST_EBP: EBP <= dest_value;

            DEST_DSTREG: write_gpr(i.dst_reg_sel, dest_value, op_size);
            DEST_SRCREG: write_gpr(i.src_reg_sel, dest_value, op_size);

            DEST_AX:     write_gpr(3'd0, dest_value, 2'd1);
            DEST_BP:     write_gpr(3'd5, dest_value, 2'd1);
            DEST_eAX_AL: write_gpr(3'd0, dest_value, op_size);
            DEST_eDX_AH: write_gpr(op_size == 2'd0 ? 3'd4 : 3'd2,
                                    dest_value, op_size);
            DEST_eCX:    write_gpr(3'd1, dest_value, i.addr32 ? 2'd2 : 2'd1);
            DEST_eSI:    write_gpr(3'd6, dest_value, i.addr32 ? 2'd2 : 2'd1);
            DEST_eDI:    write_gpr(3'd7, dest_value, i.addr32 ? 2'd2 : 2'd1);
            DEST_AL:     write_gpr(3'd0, dest_value, 2'd0);
            DEST_AH:     write_gpr(3'd4, dest_value, 2'd0);

            // Optimizer-owned FAST retire word. SEQ/fast_off ignores this
            // destination and reaches the original slot writeback unchanged.
            DEST_USTEP_ALU: begin
                if (fast_i_r && !fast_off && !any_fault)
                    write_gpr(i.dst_reg_sel, alu_result, op_size);
            end

            DEST_TMPB: TMPB <= dest_value;
            DEST_TMPC: TMPC <= dest_value;
            DEST_TMPD: TMPD <= dest_value;
            DEST_TMPE: TMPE <= dest_value;
            DEST_TMPF: TMPF <= dest_value;
            DEST_TMPG: TMPG <= dest_value;  // Used by far CALL/JMP to store IP
            DEST_TMPH: begin
                TMPH <= dest_value;         // encoding 0x11
            end
            DEST_TMP_TR: begin
                SLCTR <= dest_value;        // encoding 0x13 = SLCTR2, same register as SLCTR
            end
            DEST_TMPeIP: TMPeIP <= dest_value;
            DEST_TMPeSP: TMPeSP <= dest_value;
            DEST_CSOPCD: CSOPCD <= dest_value;
            DEST_FSVeIP: FSVeIP <= dest_value;
            DEST_OPROFF: OPROFF <= dest_value;
            DEST_OPR_W: OPR_W <= dest_value;

            DEST_MDTMP: MULTMP <= dest_value;
            DEST_MDTMP4: DIVTMP <= dest_value;

            DEST_CR0: begin
                CR0 <= dest_value;
                // Entering protected mode makes CPL 0 until a later control
                // transfer establishes a different visible CS RPL.
                if (dest_value[0] && !CR0[0])
                    CS[1:0] <= 2'b00;
            end
            DEST_CR2: begin
                CR2 <= dest_value;
            end

            DEST_DR6: DR6 <= dest_value;

            // Paging-related destinations (NOP for now)
            DEST_PAGER5: ; // Page cache register - paging-related, NOP

            // Direct segment register destinations (LDS/LES/LFS/LGS/LSS microcode)
            DEST_CS: begin
                // Workaround for call gates: FARJUMP2 (2F0-2F3) skips PASS at 2ED, so SIGMA isn't set from COUNTR
                // At 2F3, if SIGMA=0 but COUNTR!=0, use COUNTR (set by gate_detect for call gates)
                if (cs_value == 16'h0000 && COUNTR[15:0] != 16'h0000 && uc_addr == 12'h2F3)
                    cs_value = COUNTR[15:0];
                if (pe && !vm)
                    CS[15:2] <= cs_value[15:2];
                else
                    CS <= cs_value;
            end
            DEST_ES: ES <= dest_value[15:0];
            DEST_SS: SS <= dest_value[15:0];
            DEST_DS: DS <= dest_value[15:0];
            DEST_FS: FS <= dest_value[15:0];
            DEST_GS: GS <= dest_value[15:0];
            DEST_LDTR: LDTR <= dest_value[15:0];
            DEST_TR: TR <= dest_value[15:0];
            DEST_SLCTR: begin
                SLCTR <= dest_value;
            end

            DEST_IRF: begin
                if (COUNTR[5:3] != 3'b100)  // GPR
                    write_gpr(COUNTR[2:0], dest_value, is_dword ? 2'd2 : 2'd1);
                else case (COUNTR[5:0])
                    6'h20: if (uc_buscode != BUSOP_SAR && uc_buscode != BUSOP_SLIM) ES <= dest_value[15:0];
                    6'h22: if (uc_buscode != BUSOP_SAR && uc_buscode != BUSOP_SLIM) SS <= dest_value[15:0];
                    6'h23: if (uc_buscode != BUSOP_SAR && uc_buscode != BUSOP_SLIM) DS <= dest_value[15:0];
                    6'h24: if (uc_buscode != BUSOP_SAR && uc_buscode != BUSOP_SLIM) FS <= dest_value[15:0];
                    6'h25: if (uc_buscode != BUSOP_SAR && uc_buscode != BUSOP_SLIM) GS <= dest_value[15:0];
                    default: ;
                endcase
            end

            DEST_SEGREG: begin
                // Write to actual segment register using pre-decoded seg_reg_sel
                case (seg_reg_sel)
                    3'd0: ES <= dest_value[15:0];
                    3'd1: ; // CS - not writable
                    3'd2: SS <= dest_value[15:0];
                    3'd3: DS <= dest_value[15:0];
                    3'd4: FS <= dest_value[15:0];
                    3'd5: GS <= dest_value[15:0];
                    default: ;
                endcase
            end

            default: ; // No write
        endcase

        // SHIFT commits are DEFERRED one cycle (fast_shc_*)
        if (fast_last && !any_fault && fast_commit_sel_r == FAST_COMMIT_SIGSRC)
            write_gpr(i.src_reg_sel, SIGMA,
                      (uc_aluop == ALUJMP_BITS32) ? 2'd2 : 2'd1);
        // PUSH
        if (fast_last && !any_fault && fast_commit_sel_r == FAST_COMMIT_ESP)
            ESP <= SIGMA;

        if (uc_aluop == ALUJMP_PTSELE && !gate_detect_now) begin
            TMPH <= alu_dst;
        end

        // Call gate: set up TMPB, TMPH, TMPG, and COUNTR when gate_detect_now fires
        if (gate_detect_now) begin
            TMPB <= desc_raw_hi;                          // raw high DWORD for CALLGATE386
            TMPH <= {16'h0, TMPC[31:16]};                 // target CS selector (from gate low DWORD)
            TMPG <= {desc_raw_hi[31:16], TMPC[15:0]};     // target offset: hi DWORD[31:16] | lo DWORD[15:0]
        end


        if (uc_dest == DEST_MDTMP4) begin
            div_r_nonneg <= 1'b1;
            idiv_dividend_neg <= 1'b0;
            idiv_divisor_neg <= 1'b0;
            div_first_cycle <= 1'b1;  // Next DIV7/PREDIV is first cycle - check for overflow
        end

        if (uc_aluop == ALUJMP_DIV7) begin
            DIVTMP <= div_iter_q_next;
            div_r_nonneg <= div_iter_r_nonneg_next;
            div_first_cycle <= 1'b0;  // Clear after first DIV7 cycle
        end else if (uc_aluop == ALUJMP_PREDIV) begin
            DIVTMP <= div_iter_q_next;
            TMPB <= div_divisor_abs;
            div_r_nonneg <= div_iter_r_nonneg_next;
            idiv_dividend_neg <= div_dividend_neg;
            idiv_divisor_neg <= div_divisor_neg;
            div_first_cycle <= 1'b0;  // Clear - PREDIV includes first division iteration
        end else if (uc_aluop == ALUJMP_DIV5) begin
            div_r_nonneg <= 1'b1;
        end

        // CLZF (BSR only): commit the accumulated bit position (TMPC)
        if (uc_aluop == ALUJMP_CLZF && i.has_0f && i.opcode == 8'hBD) begin
            write_gpr(i.src_reg_sel, TMPC, op_size);
        end

        // COPY_STACK_DPL: commit DPL to CS[1:0] when cpl_transition is active
        if (copy_stack_dpl_s2 && cpl_transition)
            CS[1:0] <= copy_dpl_s2;

        // WRITE_RPL: write new CPL into SLCTR[1:0] from loaded CS descriptor's DPL
        if (write_rpl_s2)
            SLCTR[1:0] <= desc_raw_hi[14:13];

        end
    end

    // TMPeIP/TMPeSP: save EIP/ESP at instruction start and fault entry
    // Must be outside uc_exec gate because i_pop fires before uc_active is set
    if (i_pop) begin
        TMPeIP <= EIP;
        // TMPeSP <= forwarded_esp;
    end
    if (i_first)
        TMPeSP <= ESP;  // instruction-start ESP; fault frame (SRC_TMPeSP) is restartable,
                        // so it must use the START ESP even if the instruction already
                        // committed a stack push before faulting (e.g. ENTER's PUSH EBP).

    // Chained-store fault attribution: capture the restart IP at every demand WRITE issue
    if (mem_req_to_paging && mem_write_now && mem_accepted)
        wr_restart_eip <= TMPeIP;
    if (page_fault && pg_fault_code[1])
        TMPeIP <= wr_restart_eip;
end

// COUNTR
always_ff @(posedge clk) begin
    if (!reset_n) begin
        COUNTR <= 32'h0;
    end else if (interrupt_entry) begin
        // Clear COUNTR on interrupt entry
        COUNTR[4:0] <= 5'h0;
    end else if (gate_detect_now) begin
        // Call gate: update COUNTR with target CS selector
        COUNTR <= {16'h0, TMPC[31:16]};
    end else if (dsp_mul_early_exit) begin
        // DSP multiply done - force COUNTR to 0 to exit RPT loop early
        COUNTR[4:0] <= 5'h0;
    end else if (uc_exec) begin
        // Microcode-driven updates
        if (uc_aluop == ALUJMP_LDCNTR) begin
            // Register sources (alu_src[5]=0): load full 32-bit value
            COUNTR <= uc_alu_src[5] ? {26'b0, alu_src_data[5:0]} : alu_src_data;
        end
        else if (uc_aluop == ALUJMP_DECNTR)
            COUNTR <= COUNTR - 32'h1;
        else if (uc_dest == DEST_COUNT5) begin
            // DEST_COUNT5 (0x3A): Write 5-bit masked value for shift count (RCL/RCR m,CL)
            COUNTR <= {27'b0, dest_value[4:0]};
        end else if (uc_dest == DEST_COUNTR) begin
            COUNTR <= dest_value;
        end
        else if (repeat_active && (uc_aluop == ALUJMP_DIV7 || uc_aluop == ALUJMP_IMUL3 ||
                                    uc_aluop == ALUJMP_IMUL4 || uc_aluop == ALUJMP_PREDIV))
            COUNTR[4:0] <= COUNTR[4:0] - 1;
    end
end

// Condition code check for Jcc/SETcc/CMovcc -- doc/z386x/core_notes_v51.md #32
function automatic logic check_condition(input [3:0] cond, input [31:0] f);
    case (cond)
        4'b0000: check_condition = f[11];          // JO (OF=1)
        4'b0001: check_condition = !f[11];         // JNO (OF=0)
        4'b0010: check_condition = f[0];           // JB/JC/JNAE (CF=1)
        4'b0011: check_condition = !f[0];          // JNB/JNC/JAE (CF=0)
        4'b0100: check_condition = f[6];           // JE/JZ (ZF=1)
        4'b0101: check_condition = !f[6];          // JNE/JNZ (ZF=0)
        4'b0110: check_condition = f[0] | f[6];    // JBE/JNA (CF=1 or ZF=1)
        4'b0111: check_condition = !f[0] & !f[6];  // JNBE/JA (CF=0 and ZF=0)
        4'b1000: check_condition = f[7];           // JS (SF=1)
        4'b1001: check_condition = !f[7];          // JNS (SF=0)
        4'b1010: check_condition = f[2];           // JP/JPE (PF=1)
        4'b1011: check_condition = !f[2];          // JNP/JPO (PF=0)
        4'b1100: check_condition = f[7] != f[11];  // JL/JNGE (SF!=OF)
        4'b1101: check_condition = f[7] == f[11];  // JNL/JGE (SF=OF)
        4'b1110: check_condition = f[6] | (f[7] != f[11]);   // JLE/JNG (ZF=1 or SF!=OF)
        4'b1111: check_condition = !f[6] & (f[7] == f[11]);  // JNLE/JG (ZF=0 and SF=OF)
    endcase
endfunction

// IND register (address register) and ind_linear (relocated linear address of IND)
reg [31:0] IND_DELTA;
always_ff @(posedge clk) begin
    if (!reset_n) begin
        IND <= 32'h0;
        IND_DELTA <= 32'd4;
        ind_linear <= 32'h0;
        ind_linear_valid <= 1'b0;
    end else begin
        // Instruction start - initialize IND based on addressing mode
        if (i_pop) begin
            ind_linear_valid <= 1'b0;
            IND_DELTA <= !i_bus.stack_op ? 32'd2 :
                         !i_bus.stack_dir ? (i_bus.data32 ? -32'd4 : -32'd2) :
                                            (i_bus.data32 ? 32'd4 : 32'd2);
            if (i_bus.stack_op && i_bus.stack_dir) begin
                // Stack pop/ret: IND = ESP, relocate at i_pop
                automatic logic [31:0] stk =
                    seg_cache[SEG_SS].D_B ? forwarded_esp : {16'h0, forwarded_esp[15:0]};
                IND <= stk;
                ind_linear <= reloc(stk);
                ind_linear_valid <= 1'b1;
            end else if (i_bus.stack_op && !i_bus.stack_dir) begin
                // Stack push/call: IND = ESP-delta, relocate at i_pop.
                automatic logic [31:0] stk = seg_cache[SEG_SS].D_B
                    ? forwarded_esp - (i_bus.data32 ? 32'd4 : 32'd2)
                    : {16'h0, forwarded_esp[15:0] - (i_bus.data32 ? 16'd4 : 16'd2)};
                IND <= stk;
                ind_linear <= reloc(stk);
                ind_linear_valid <= 1'b1;
            end else if (i_bus.has_moffs) begin
                IND <= i_bus.addr32 ? i_bus.immediate : {16'h0, i_bus.immediate[15:0]};
                ind_linear <= reloc(i_bus.immediate);
                ind_linear_valid <= 1'b1;
            end else if (i_bus.has_modrm) begin
                IND <= ea_early;
                ind_linear <= pop_ind_linear;  // fused CSA+ternary (== reloc(ea_early))
                ind_linear_valid <= 1'b1;
            end
        end
        // BUSOP-related IND updates (only when uc_exec is active)
        else if (uc_exec) begin
            automatic logic [31:0] ind_reloc_src = IND;
            automatic logic        ind_lin_use3 = 1'b0;
            automatic logic [31:0] lin_a = IND;
            automatic logic [31:0] lin_b = 32'h0;
            automatic logic        ind_lin_mask16 = !eff_mask_pending;  // 16-bit mask
            case (uc_buscode)
                BUSOP_IND_PLUS_ALU: begin  // IN=+ - IND = source + ALU operand, IND_DELTA = ALU operand
                    automatic logic [31:0] ind_next;
                    automatic logic [31:0] alu1, alu2;
                    automatic logic is_jcc = jcc_active;
                    alu1 = uc_source == SRC_IRF2 ? IND : alu_dst;
                    alu2 = is_jcc ? alu_src_r : alu_src;
                    if (uc_alu_src != ALUSRC_ZERO)
                        IND_DELTA <= alu2;
                    if (uc_dest == DEST_DESSTK)
                        ind_lin_mask16 = !pe || !seg_cache[SEG_SS].D_B;
                    else if (uc_dest == DEST_DESCOD)
                        ind_lin_mask16 = !is_dword;
                    else if (uc_dest == DEST_DES_ES || uc_dest == DEST_DES_OS || uc_dest == DEST_DES_SR)
                        ind_lin_mask16 = !i.addr32;
                    ind_next = alu1 + alu2;
                    if (ind_lin_mask16 && (uc_dest == DEST_DESSTK || uc_dest == DEST_DESCOD ||
                        uc_dest == DEST_DES_ES || uc_dest == DEST_DES_OS || uc_dest == DEST_DES_SR))
                        ind_next = {16'h0, ind_next[15:0]};
                    // common EA: fuse alu1 + alu2 + seg_base into one add
                    ind_lin_use3 = 1'b1; lin_a = alu1; lin_b = alu2;
                    IND <= ind_next;
                    ind_reloc_src = ind_next;
                    ind_linear_valid <= 1'b1;
                end
                BUSOP_IND_ALU2: begin  // IN=2 - Set IND from ALU2 (alu_src)
                    IND <= alu_src;
                    ind_reloc_src = alu_src;
                    ind_linear_valid <= 1'b1;
                end
                BUSOP_IND_SRC: begin  // IND= - Set IND from source register
                    automatic logic [31:0] ind_val = alu_dst;
                    if (uc_dest == DEST_DESSTK && (!pe || !seg_cache[SEG_SS].D_B))
                        ind_val = {16'h0, ind_val[15:0]};
                    else if (uc_dest == DEST_DESCOD && !is_dword)
                        ind_val = {16'h0, ind_val[15:0]};
                    // DESIDT: IND = offset only; IDT base applied by the seg unit.
                    IND <= ind_val;
                    ind_reloc_src = ind_val;
                    ind_linear_valid <= 1'b1;
                end
                BUSOP_IND_PLUS: begin  // IN+= - IND = IND + alu_src, IND_DELTA = alu_src
                    // IN+= latches the delta like IN=+: PUSHA latches -1 at 089
                    // (IN=+) then +WORDSZ at 08B (IN+=) for the 08E IN+D loop.
                    automatic logic [31:0] ind_next = IND + alu_src;
                    if (!pe && !i.addr32)
                        ind_next = {16'h0, ind_next[15:0]};
                    IND <= ind_next;
                    ind_reloc_src = ind_next;
                    ind_lin_use3 = 1'b1; lin_a = IND; lin_b = alu_src;
                    ind_linear_valid <= 1'b1;
                    if (uc_alu_src != ALUSRC_ZERO)
                        IND_DELTA <= alu_src;
                end
                BUSOP_IN_PLUS_D: begin  // IN+D - IND += IND_DELTA (signed, latched by IN=+/IN+=)
                    automatic logic [31:0] ind_next;
                    ind_next = IND + IND_DELTA;
                    if (!pe ? !i.addr32 : !(descsw_mode ? seg_cache[SEG_CS].D_B : seg_cache[SEG_SS].D_B))
                        ind_next = {16'h0, ind_next[15:0]};
                    IND <= ind_next;
                    ind_reloc_src = ind_next;
                    ind_lin_use3 = 1'b1; lin_a = IND; lin_b = IND_DELTA;
                    ind_linear_valid <= 1'b1;
                end
                BUSOP_LAR: begin  // LAR result from segmentation unit
                    IND <= seg_lar_result;
                    ind_linear_valid <= 1'b0;  // IND loaded with descriptor data, not an address
                end
                BUSOP_LLIM: begin  // LLIM result from segmentation unit
                    IND <= seg_llim_result;
                    ind_linear_valid <= 1'b0;
                end
                BUSOP_LBAS: begin  // LBAS result from segmentation unit
                    IND <= seg_lbas_result;
                    ind_linear_valid <= 1'b0;
                end
                BUSOP_LPCR: begin  // LPCR (0x34) - Load Page Cache Register into IRF2 (IND)
                    case (uc_dest)
                        DEST_PFERRC: IND <= {29'h0, latched_pf_code};
                        DEST_LATTTF: IND <= latched_pf_addr;
                        DEST_PDBR:   IND <= CR3;
                        default:     IND <= IND;
                    endcase
                    ind_linear_valid <= 1'b0;
                end
                default: ;  // IND already holds the early EA from instruction start
            endcase
            if (ind_lin_use3)
                ind_linear <= reloc_add2(lin_a, lin_b, ind_lin_mask16); // Fused 3-input add
            else
                ind_linear <= reloc_uc(ind_reloc_src);
        end
    end
end

// Multiplier and division
reg [31:0] DIVTMP;
reg [31:0] MULTMP;
reg [31:0] RESULT;
reg        div_r_nonneg;
reg        idiv_dividend_neg;
reg        idiv_divisor_neg;
reg        div_first_cycle;      // Set when starting DIV, cleared after first DIV7 cycle

wire [63:0] mul_acc;             // DSP multiplier result
wire [31:0] mul_upper = op_size == 2'b00 ? {24'h0, mul_acc[39:32]} :
                        op_size == 2'b01 ? {16'h0, mul_acc[47:32]} :
                        mul_acc[63:32];
logic [31:0] div_iter_q_next;
logic [31:0] div_iter_r_next;
logic        div_iter_r_nonneg_next;
logic [31:0] div_divisor_masked;
logic [31:0] div_divisor_abs;
logic        div_dividend_neg;
logic        div_divisor_neg;
logic [31:0] prediv_q_in;
logic [31:0] prediv_r_in;
// IMUL: F6.5, F7.5, 0FAF, 69, 6B, MUL: F6.4 and F7.4
wire is_signed_mul = i.opcode[7:6] != 2'b11 || i.modrm[3];


// Start DSP multiply on first cycle of IMUL3/IMUL4 RPT loop
wire mul_is_imul = (uc_aluop == ALUJMP_IMUL3 || uc_aluop == ALUJMP_IMUL4);
wire dsp_mul_done;
wire dsp_mul_active;
reg  dsp_mul_completed;  // Stays true until instruction ends, prevents restart

// Only start a new multiply if not already completed for this instruction
// Also check !dsp_mul_done to prevent restart on the same cycle as completion
wire mul_start = uc_exec_mul_start && mul_is_imul && !dsp_mul_active && !dsp_mul_completed && !dsp_mul_done;

dsp_mul u_dsp_mul (
    .clk(clk),
    .reset_n(reset_n),
    .start(mul_start),
    .op_size(op_size),
    .is_signed(is_signed_mul),
    .multiplicand(MULTMP),
    .multiplier(TMPB),
    .product(mul_acc),
    .done(dsp_mul_done),
    .active(dsp_mul_active)
);

// Track multiply completion - set when done, cleared at instruction start
always_ff @(posedge clk) begin
    if (!reset_n)
        dsp_mul_completed <= 1'b0;
    else if (i_pop)
        dsp_mul_completed <= 1'b0;  // Clear at instruction start
    else if (dsp_mul_done)
        dsp_mul_completed <= 1'b1;  // Set when multiply completes
end

// Early exit: force COUNTR to 0 when DSP multiply completes during RPT loop
// This causes repeat_active to become false, exiting the loop early
wire dsp_mul_early_exit = dsp_mul_done && repeat_active && mul_is_imul;

// Division: prepare inputs for DIV7 and PREDIV steps
always_comb begin
    logic [15:0] full16;
    logic [31:0] full32;
    logic [63:0] full64;
    full16 = {16{1'bx}};
    full32 = {32{1'bx}};
    full64 = {64{1'bx}};

    div_dividend_neg = div_get_sign_bit(SIGMA, op_size);
    div_divisor_neg = div_get_sign_bit(TMPB, op_size);
    div_divisor_masked = div_op_by_size(DIV_OP_MASK, TMPB, op_size);
    div_divisor_abs = div_divisor_neg ? div_op_by_size(DIV_OP_NEGATE, TMPB, op_size) : div_divisor_masked;

    prediv_q_in = div_op_by_size(DIV_OP_MASK, DIVTMP, op_size);
    prediv_r_in = div_op_by_size(DIV_OP_MASK, SIGMA, op_size);

    case (op_size)
        2'd0: begin
            full16 = {SIGMA[7:0], DIVTMP[7:0]};
            if (div_dividend_neg)
                full16 = ~full16 + 16'h1;
            prediv_r_in = {24'h0, full16[15:8]};
            prediv_q_in = {24'h0, full16[7:0]};
        end
        2'd1: begin
            full32 = {SIGMA[15:0], DIVTMP[15:0]};
            if (div_dividend_neg)
                full32 = ~full32 + 32'h1;
            prediv_r_in = {16'h0, full32[31:16]};
            prediv_q_in = {16'h0, full32[15:0]};
        end
        default: begin
            full64 = {SIGMA, DIVTMP};
            if (div_dividend_neg)
                full64 = ~full64 + 64'h1;
            prediv_r_in = full64[63:32];
            prediv_q_in = full64[31:0];
        end
    endcase
end

// DIV7 and PREDIV are mutually exclusive microcode operations.  Select their
// inputs before the shared non-restoring iteration instead of synthesizing two
// 35-bit iteration datapaths.
wire div_iter_prediv = (uc_aluop == ALUJMP_PREDIV);
wire [31:0] div_iter_q_in = div_iter_prediv ? prediv_q_in : DIVTMP;
wire [31:0] div_iter_r_in = div_iter_prediv ? prediv_r_in : SIGMA;
wire [31:0] div_iter_d_in = div_iter_prediv ? div_divisor_abs : div_divisor_masked;
wire        div_iter_r_nonneg_in = div_iter_prediv ? 1'b1 : div_r_nonneg;

always_comb begin
    div7_calc(div_iter_q_in, div_iter_r_in, div_iter_d_in,
              div_iter_r_nonneg_in, op_size,
              div_iter_q_next, div_iter_r_next, div_iter_r_nonneg_next);
end

always_ff @(posedge clk) begin
    if (uc_exec_result && (uc_aluop == ALUJMP_IMUL3 || uc_aluop == ALUJMP_IMUL4)) begin
        RESULT <= op_size == 2'b00 ? mul_acc[31:24] :
                  op_size == 2'b01 ? mul_acc[31:16] :
                  mul_acc[31:0];
    end else if (uc_exec_result && uc_aluop == ALUJMP_DIV7) begin
        RESULT <= div_iter_q_next;
    end else if (uc_exec_result && uc_aluop == ALUJMP_PREDIV) begin
        RESULT <= div_iter_q_next;
    end else if (uc_exec_result && uc_dest == DEST_MDTMP4) begin
        RESULT <= dest_value;
    end
end

//
// ALU
//
wire [31:0] alu_flags;
wire        alu_zsp_update;

// Derive control signals from ALU opcode
// INC=11000, DEC=11001, INC2=11100, DEC2=11101: all have op[4:3]==11 && op[1]==0
wire alu_update_carry = !(alu_op5[4:3] == 2'b11 && !alu_op5[1]);
wire in_rpti_routine = (uaddr >= 12'h208) && (uaddr <= 12'h20e);   // TODO: Remove this special case

// Pre-computed ROM bits (eliminates wide combinational comparisons from hot paths)
wire alu_update_flags  = uc[37];
wire uc_bus_or_dly     = uc[38];
wire uc_is_mem_busop   = uc_mem_ctrl[0];
wire uc_is_write       = uc_mem_ctrl[1];
wire uc_is_check_write = uc_mem_ctrl[2];
wire uc_is_word_op     = uc_mem_ctrl[3];
wire uc_is_dword_op    = uc_mem_ctrl[4];
wire uc_jpereq_fwd     = uc_mem_ctrl[5];
wire uc_p_io_rd        = uc_mem_ctrl[6];   // IO-capable read buscode
wire uc_p_io_wr        = uc_mem_ctrl[7];   // IO-capable write buscode
wire uc_p_iack         = uc_mem_ctrl[8];   // IACK bus cycle
wire uc_p_pure_dly     = uc[48];   // DLY without a bus request of its own
wire uc_p_rpt          = uc[49];   // RPT opcode
wire uc_p_wio          = uc[50];   // WIO (RPT opcode + WIO subcode)

always_comb begin
    // Use the ROM-delay-cycle field registers for the hot ALU datapath. They
    // are aligned with uc, but avoid feeding ALU muxes from the wide ucode word.
    alu_dst = read_uc_source(uc_source_shift);        // NOPQRS
    alu_src = read_uc_alu_source(uc_alu_src_shift);   // ABCDEF
    alu_op5 = map_alu_op(uc_aluop_shift);             // TUVWXYZ
end

// Register alu_src for jump operations
always @(posedge clk) begin
    // Detect Jcc instructions at decode time
    automatic logic is_jcc_short = (i_bus.opcode[7:4] == 4'b0111);  // 70-7F
    automatic logic is_jcc_near = (i_bus.has_0f && i_bus.opcode[7:4] == 4'b1000);  // 0F 80-8F
    // Detect Jcc during execution (to avoid overwriting alu_src_r)
    automatic logic is_jcc_exec = (i.opcode[7:4] == 4'b0111) || (i.opcode[7:4] == 4'b1000 && i.has_0f);

    if (i_pop && !halted && (is_jcc_short || is_jcc_near)) begin
        if (is_jcc_short) begin
            // Short Jcc: sign-extend 8-bit i.displacement
            alu_src_r <= {{24{i_bus.displacement[7]}}, i_bus.displacement[7:0]};
        end else begin
            // Near Jcc: use full i.displacement (already sign-extended by decoder)
            alu_src_r <= i_bus.displacement;
        end
    end else if (!(is_jcc_exec && uc_active)) begin
        // Only update alu_src_r if NOT currently executing a Jcc
        alu_src_r <= alu_src[31:0];
    end
end

`ifdef Z386_ALTERA_ALU
alu_alt u_alu (
`else
alu u_alu (
`endif
    .op(alu_op5),
    .src(alu_src),
    .dst(alu_dst),
    .op_size(op_size),
    // eflags_fwd, not EFLAGS: under a FAST chain the ALU word runs
    .flags(eflags_fwd),
    .update_carry(alu_update_carry),
    .result(alu_result),
    .flags_out(alu_flags),
    .zsp_ahead(alu_zsp_ahead),
    .zsp_update(alu_zsp_update)
);

//
// Barrel Shifter Unit
//
logic        shift_swap;
logic [5:0]  shift_count;    // 64-bit shift count (6 bits: needs to hold value 32 for LDBSLU with count=0)
logic [4:0]  shift_size;     // original shift amount
logic        shift_overflow; // SHL/SHR/SAR count > width (result is 0 or sign-extended)
logic        shift_eq_width; // count == width (for SHL/SHR CF special case)
logic        shift_eq_cf;    // saved CF for count==width case

// Optimization: shift/bit-test execution microcode uses a smaller source-selector subset
logic [31:0] shift_src_value;
logic [31:0] shift_alu_value;
always_comb begin
    if (uc_aluop_shift == ALUJMP_SHIFT2) begin
        // The immutable ROM uses only these four sources for SHIFT2. Keeping
        // this timing-critical path separate avoids the general shift mux.
        case (uc_shift2_source)
            2'd0:    shift_src_value = TMPC;
            2'd1:    shift_src_value = TMPE;
            2'd2:    shift_src_value = SIGMA;
            2'd3:    shift_src_value = read_gpr(i_reg_src_reg_sel, op_size);
            default: shift_src_value = 32'd0;
        endcase
    end else begin
        case (uc_source_shift)
            SRC_SIGMA:   shift_src_value = SIGMA;
            SRC_DSTREG:  shift_src_value = read_gpr(i_reg_dst_reg_sel, srcreg_size);
            SRC_SRCREG:  shift_src_value = read_gpr(i_reg_src_reg_sel, op_size);
            SRC_IMM:     shift_src_value = i_reg_immediate;
            SRC_TMPB:    shift_src_value = TMPB;
            SRC_TMPC:    shift_src_value = TMPC;
            SRC_TMPD:    shift_src_value = TMPD;
            SRC_TMPE:    shift_src_value = TMPE;
            SRC_OPR_R:   shift_src_value = OPR_R;
            SRC_COUNTR:  shift_src_value = COUNTR;
            SRC_ZERO:    shift_src_value = 32'd0;
            SRC_NEG1:    shift_src_value = 32'hFFFF_FFFF;
            default:     shift_src_value = 32'd0;
        endcase
    end
end

// synthesis translate_off
always @(posedge clk)
    if (reset_n && uc_exec && (uc_aluop_shift == ALUJMP_SHIFT2) &&
        (uc_source_shift != SRC_TMPC) && (uc_source_shift != SRC_TMPE) &&
        (uc_source_shift != SRC_SIGMA) && (uc_source_shift != SRC_SRCREG))
        $fatal(1, "SHIFT2 source outside ROM predecode inventory: %02x", uc_source_shift);
// synthesis translate_on

always_comb begin
    case (uc_alu_src_shift)
        ALUSRC_CONST_0:   shift_alu_value = 32'd0;
        ALUSRC_TMPC:    shift_alu_value = TMPC;
        ALUSRC_TMPD:    shift_alu_value = TMPD;
        ALUSRC_TMPB:    shift_alu_value = TMPB;
        ALUSRC_DSTREG:  shift_alu_value = read_gpr(i_reg_dst_reg_sel, op_size);
        ALUSRC_SRCREG:  shift_alu_value = read_gpr(i_reg_src_reg_sel, op_size);
        ALUSRC_ECX:     shift_alu_value = ECX;
        ALUSRC_IMM:     shift_alu_value = i_reg_immediate;
        ALUSRC_BITS_V:  shift_alu_value = (op_size == 2'd0) ? 32'd7 : (op_size == 2'd2) ? 32'd31 : 32'd15;
        ALUSRC_CONST_1: shift_alu_value = 32'd1;
        ALUSRC_CONST_3: shift_alu_value = 32'd3;
        ALUSRC_CONST_7: shift_alu_value = 32'd7;
        ALUSRC_CONST_1FF: shift_alu_value = 32'h1FF;
        ALUSRC_CONST_4000: shift_alu_value = 32'h4000;
        ALUSRC_CONST_F0000: shift_alu_value = 32'h000F_0000;
        ALUSRC_MASK16: shift_alu_value = 32'h0000_FFFF;
        ALUSRC_CONST_FFFF0000: shift_alu_value = 32'hFFFF_0000;
        default:        shift_alu_value = 32'd0;
    endcase
end

// Combinational: use latched shift_swap with shift-specific operands
wire [31:0]  shift_hi = shift_swap ? shift_alu_value : shift_src_value;
wire [31:0]  shift_lo = shift_swap ? shift_src_value : shift_alu_value;
wire         is_sar = (i.modrm[5:3] == SAR) && !instr_is_shxd;
wire [63:0]  shift_in = op_size == 2'b00 ? {shift_hi, shift_lo[7:0]} :
                        op_size == 2'b01 ? {shift_hi, shift_lo[15:0]} :
                                           {shift_hi, shift_lo};
wire  [63:0] shifted = shift_in >> shift_count;

// For SHL/SHR with overflow (count >= width), result is 0
// For SAR with overflow, result is sign-extended (all 1s if negative, all 0s if positive)
wire [31:0]  sar_overflow_result = shift_lo[width-1] ? 32'hFFFFFFFF : 32'h0;
assign       shift_result = shift_overflow ? (is_sar ? sar_overflow_result : 32'h0) : shifted[31:0];
wire         shift_pf = ~^shift_result[7:0];

// ZF/SF taken from the raw barrel output (shifted), with the overflow special
// cases resolved separately
wire         shift_lo_sign = (op_size == 2'd0) ? shift_lo[7] :
                             (op_size == 2'd1) ? shift_lo[15] : shift_lo[31];
wire         shift_zf = shift_overflow ? (is_sar ? ~shift_lo_sign : 1'b1) :
                        (op_size == 2'd0) ? (shifted[7:0] == 8'h0) :
                        (op_size == 2'd1) ? (shifted[15:0] == 16'h0) :
                                            (shifted[31:0] == 32'h0);
wire         shift_sf = shift_overflow ? (is_sar ? shift_lo_sign : 1'b0) :
                        (op_size == 2'd0) ? shifted[7] :
                        (op_size == 2'd1) ? shifted[15] :
                                            shifted[31];

// flags related
reg          shift_SET_Nzs;
reg   [5:0]  sh1_width_r;  // Registered SHIFT1 width keeps op_size out of SHIFT2 flag selects.
reg   [2:0]  shift_op;     // ROL/ROR/RCL/RCR/SHL/SHR/SAR, for flag update
wire         shift_last_out_lsb = shift_in[shift_count-1];
// BSR can consume this on cycles where sh1_width_r is stale by one cycle.
wire         shift_last_out_msb = shifted[width];
// Simplified shift CF for uc_flags: left shift uses MSB, right shift uses LSB
wire         shift_cf = shift_swap ? shift_last_out_msb : shift_last_out_lsb;

wire  [5:0]  width = (op_size == 2'd0) ? 6'd8 : (op_size == 2'd1) ? 6'd16 : 6'd32;
wire  [31:0] shift_width_mask = (op_size == 2'd0) ? 32'h0000_00FF :
                                (op_size == 2'd1) ? 32'h0000_FFFF :
                                                    32'hFFFF_FFFF;

// Barrel shifter ops
always_ff @(posedge clk) begin
    if (uc_exec_shift) case (uc_aluop_shift)
    ALUJMP_SHIFT1: begin   // set up shifter parameters (count, swap, size)
        automatic logic [5:0] count_mod;
        automatic logic [5:0] count_raw;
        count_raw = alu_src[4:0];  // Count masked to 5 bits
        case (op_size)            // Reduce count modulo width (for ROL/ROR only)
            2'd0:    count_mod = {3'd0, count_raw[2:0]};  // mod 8
            2'd1:    count_mod = {2'd0, count_raw[3:0]};  // mod 16
            default: count_mod = count_raw;               // mod 32, count is already 0..31
        endcase
        shift_size = count_raw;  // Store original count for OF check (count==1)
        sh1_width_r <= width;

        if (instr_is_shxd) begin   // i.opcode[3], 1: SHRD, 0: SHLD
            shift_swap <= ~i.opcode[3];
            shift_count <= i.opcode[3] ? count_raw : (width - count_raw);
            shift_op <= (i.opcode[3] ? ROR : ROL);
            shift_SET_Nzs <= 1;
        end else begin
            shift_swap <= ~i.modrm[3];   // no swap for right shift/rotate
            shift_overflow <= (count_raw >= width) &&
                              ((i.modrm[5:3] == SHL) || (i.modrm[5:3] == SAL) ||
                               (i.modrm[5:3] == SHR) || (i.modrm[5:3] == SAR));
            shift_eq_width <= (count_raw == width) &&
                              ((i.modrm[5:3] == SHL) || (i.modrm[5:3] == SAL) ||
                               (i.modrm[5:3] == SHR) || (i.modrm[5:3] == SAR));
            // Left shift by width: last bit out = bit 0; Right shift by width: last bit out = bit (width-1)
            shift_eq_cf <= i.modrm[3] ? alu_dst[width-1] : alu_dst[0];
            case (i.modrm[5:3])
                ROL:     shift_count <= width - count_mod;               // ROL: use modulo
                ROR:     shift_count <= count_mod[4:0];                  // ROR: use modulo
                // RCL/RCR: don't use modulo - microcode already reduces count to [0, width] range
                // When count = width, we need shift_count = 0 (RCL) or width (RCR), not vice versa
                RCL:     shift_count <= width - count_raw[4:0];
                RCR:     shift_count <= count_raw[4:0];
                SHL,SAL: shift_count <= (count_raw >= width) ? 5'd31 :   // SHL/SAL: clamp to 31 if >= width (shifts all out)
                                        (width - count_raw);
                SHR:     shift_count <= (count_raw >= width) ? 5'd31 :   // SHR: clamp to 31 if >= width
                                        count_raw;
                default: shift_count <= (count_raw >= width) ? 5'd31 :   // SAR: clamp to 31 if >= width
                                                count_raw;
            endcase
            shift_op <= i.modrm[5:3];
            shift_SET_Nzs <= (i.modrm[5:3] == SHL) || (i.modrm[5:3] == SHR) ||
                             (i.modrm[5:3] == SAR) || (i.modrm[5:3] == SAL);
        end
    end

    ALUJMP_LDBSRM: begin   // set up right shift (for BITTST)
        shift_swap <= 0;
        // Mask bit offset to operand size: 16-bit uses bits [3:0], 32-bit uses bits [4:0]
        shift_count <= alu_src[4:0] & (width - 1);
    end
    ALUJMP_LDBSRU: begin   // set up right shift for BT/BTS/BTR/BTC byte offset calculation
        shift_swap <= 0;
        shift_count <= alu_src[4:0];
        shift_overflow <= 0;
    end
    ALUJMP_LDBSLM: begin   // set up left shift (for BITTST rotate back)
        shift_swap <= 1;
        // Mask bit offset to operand size, then compute complementary shift
        shift_count <= width - (alu_src[4:0] & (width - 1));
        shift_overflow <= 0;
    end
    ALUJMP_LDBSLU: begin   // set up left shift for BSR (shift left)
        shift_swap <= 1;
        shift_count <= width - alu_src[4:0];
        shift_size <= alu_src[4:0];  // Must be non-zero for SHIFT2 to update CF
        shift_SET_Nzs <= 0;   // BSR doesn't update SF/ZF/PF on each iteration
        shift_op <= SHL;
        shift_overflow <= 0;
    end

    ALUJMP_SHIFT, ALUJMP_SHIFT2: ;   // combinational, uses current alu_dst/alu_src with latched shift_swap

    endcase

end


assign dest_value = alu_dst;

// Debug tap (read by tb_z386 hierarchically; not used in the core).
wire use_shifter_result = (uc_aluop == ALUJMP_SHIFT2) || (uc_aluop == ALUJMP_SHIFT);


//=============================================================================
// Microcode Helper Functions
//=============================================================================

function automatic [31:0] read_gpr(input [2:0] sel, input [1:0] op_size);
    case (op_size)
        2'd0: begin
            case (sel)
                3'd0: read_gpr = {24'h0, EAX[7:0]};
                3'd1: read_gpr = {24'h0, ECX[7:0]};
                3'd2: read_gpr = {24'h0, EDX[7:0]};
                3'd3: read_gpr = {24'h0, EBX[7:0]};
                3'd4: read_gpr = {24'h0, EAX[15:8]};
                3'd5: read_gpr = {24'h0, ECX[15:8]};
                3'd6: read_gpr = {24'h0, EDX[15:8]};
                3'd7: read_gpr = {24'h0, EBX[15:8]};
            endcase
        end
        2'd1: begin
            case (sel)
                3'd0: read_gpr = {16'h0, EAX[15:0]};
                3'd1: read_gpr = {16'h0, ECX[15:0]};
                3'd2: read_gpr = {16'h0, EDX[15:0]};
                3'd3: read_gpr = {16'h0, EBX[15:0]};
                3'd4: read_gpr = {16'h0, ESP[15:0]};
                3'd5: read_gpr = {16'h0, EBP[15:0]};
                3'd6: read_gpr = {16'h0, ESI[15:0]};
                3'd7: read_gpr = {16'h0, EDI[15:0]};
            endcase
        end
        default: begin
            case (sel)
                3'd0: read_gpr = EAX;
                3'd1: read_gpr = ECX;
                3'd2: read_gpr = EDX;
                3'd3: read_gpr = EBX;
                3'd4: read_gpr = ESP;
                3'd5: read_gpr = EBP;
                3'd6: read_gpr = ESI;
                3'd7: read_gpr = EDI;
            endcase
        end
    endcase
endfunction

task automatic write_gpr(input [2:0] sel, input [31:0] value, input [1:0] op_size);
    case (op_size)
        2'd0: begin
            case (sel)
                3'd0: EAX[7:0]   <= value[7:0];
                3'd1: ECX[7:0]   <= value[7:0];
                3'd2: EDX[7:0]   <= value[7:0];
                3'd3: EBX[7:0]   <= value[7:0];
                3'd4: EAX[15:8]  <= value[7:0];
                3'd5: ECX[15:8]  <= value[7:0];
                3'd6: EDX[15:8]  <= value[7:0];
                3'd7: EBX[15:8]  <= value[7:0];
            endcase
        end
        2'd1: begin
            case (sel)
                3'b000: EAX[15:0] <= value[15:0];
                3'b001: ECX[15:0] <= value[15:0];
                3'b010: EDX[15:0] <= value[15:0];
                3'b011: EBX[15:0] <= value[15:0];
                3'b100: ESP[15:0] <= value[15:0];
                3'b101: EBP[15:0] <= value[15:0];
                3'b110: ESI[15:0] <= value[15:0];
                3'b111: EDI[15:0] <= value[15:0];
            endcase
        end
        2'd2: begin
            case (sel)
                3'b000: EAX <= value;
                3'b001: ECX <= value;
                3'b010: EDX <= value;
                3'b011: EBX <= value;
                3'b100: ESP <= value;
                3'b101: EBP <= value;
                3'b110: ESI <= value;
                3'b111: EDI <= value;
            endcase
        end
    endcase
endtask

function automatic [31:0] read_protun_source_fast(input [5:0] src_field);
    case (src_field)
        SRC_ZERO:    read_protun_source_fast = 32'd0;
        SRC_NEG1:    read_protun_source_fast = 32'hFFFF_FFFF;
        SRC_CR0:     read_protun_source_fast = CR0;
        SRC_TMPH:    read_protun_source_fast = TMPH;
        SRC_TMP_TR:  read_protun_source_fast = SLCTR;
        SRC_COUNTR,
        SRC_PROTUN:  read_protun_source_fast = PROTUN;
        SRC_SIGMA:   read_protun_source_fast = SIGMA;
        SRC_CS:      read_protun_source_fast = {16'h0, CS};
        SRC_OPR_R:   read_protun_source_fast = OPR_R;
        SRC_IRF2:    read_protun_source_fast = IND;
        SRC_TMPE:    read_protun_source_fast = TMPE;
        SRC_DSTREG:  read_protun_source_fast = read_gpr(i_reg_dst_reg_sel, srcreg_size);
        SRC_SRCREG:  read_protun_source_fast = read_gpr(i_reg_src_reg_sel, op_size);
        default:     read_protun_source_fast = read_uc_source(src_field);
    endcase
endfunction

function automatic [15:0] read_cs_source_fast(input [5:0] src_field);
    case (src_field)
        SRC_SIGMA:   read_cs_source_fast = SIGMA[15:0];
        SRC_TMPH:    read_cs_source_fast = TMPH[15:0];
        SRC_OPR_R:   read_cs_source_fast = OPR_R[15:0];
        SRC_PROTUN:  read_cs_source_fast = PROTUN[15:0];
        default:     read_cs_source_fast = read_uc_source(src_field);
    endcase
endfunction

function automatic [31:0] read_uc_alu_source(input [5:0] src_field);
    case (src_field)
        ALUSRC_EAX: read_uc_alu_source = EAX;
        ALUSRC_ECX: read_uc_alu_source = ECX;
        ALUSRC_EDX: read_uc_alu_source = EDX;
        ALUSRC_EBX: read_uc_alu_source = EBX;
        ALUSRC_ESP: read_uc_alu_source = ESP;  // Always full 32-bit; truncation at dest (eSP) or segmentation unit
        ALUSRC_EBP: read_uc_alu_source = EBP;
        ALUSRC_ESI: read_uc_alu_source = ESI;
        ALUSRC_EDI: read_uc_alu_source = EDI;
        // IMM8: For instructions with i.modrm, i.immediate (already sign-extended)
        // For others (E8/E9/0F8x/C8/9A/EA), decoder puts value in i.displacement
        ALUSRC_IMM8: read_uc_alu_source = i_has_modrm ? i_reg_immediate : i_reg_displacement;
        ALUSRC_IMM: read_uc_alu_source = i_reg_immediate;
        ALUSRC_TMPB: read_uc_alu_source = TMPB;
        ALUSRC_TMPC: read_uc_alu_source = TMPC;
        ALUSRC_TMPD: read_uc_alu_source = TMPD;
        ALUSRC_OPR_R: read_uc_alu_source = OPR_R;  // z386x F-ALUM fold (patched 029/02E)
        ALUSRC_TMPG: read_uc_alu_source = TMPG;  // Used by far CALL/JMP
        ALUSRC_TMPH: read_uc_alu_source = SLCTR;  // ALU source 0x13 = SLCTR2 = SLCTR
        ALUSRC_PROTUN: read_uc_alu_source = PROTUN;
        ALUSRC_ALLONES: read_uc_alu_source = 32'hFFFFFFFF;
        ALUSRC_FLAGS_MASK: read_uc_alu_source = 32'h0003_7fd7;  // FLAGS mask for INT stack push
        ALUSRC_CONST_4000: read_uc_alu_source = 32'h4000;
        ALUSRC_CONST_N200: read_uc_alu_source = 32'hFFFFFDFF;  // ~0x200
        ALUSRC_CONST_8: read_uc_alu_source = 32'd8;
        ALUSRC_CONST_40: read_uc_alu_source = 32'h40;
        ALUSRC_CONST_F0000: read_uc_alu_source = 32'h000F_0000;
        ALUSRC_CONST_0D: read_uc_alu_source = 32'h0D;
        ALUSRC_CONST_5D: read_uc_alu_source = 32'h5D;
        ALUSRC_SIGMA: read_uc_alu_source = SIGMA;
        ALUSRC_CONST_FC: read_uc_alu_source = 32'h800000FC;  // FPU port address
        ALUSRC_CONST_1: read_uc_alu_source = 32'd1;
        ALUSRC_CONST_2: read_uc_alu_source = 32'd2;
        ALUSRC_CONST_16: read_uc_alu_source = 32'd16;  // 0x10 for INT shift right by 16
        ALUSRC_CONST_3: read_uc_alu_source = 32'd3;
        ALUSRC_CONST_4: read_uc_alu_source = 32'd4;
        ALUSRC_CONST_6: read_uc_alu_source = 32'd6;
        ALUSRC_CONST_7: read_uc_alu_source = 32'd7;  // For AAD/AAM 8-bit mul
        ALUSRC_CONST_0F: read_uc_alu_source = 32'h0F;
        ALUSRC_CONST_65: read_uc_alu_source = 32'h65;
        ALUSRC_CONST_1F: read_uc_alu_source = 32'h1F;
        ALUSRC_CONST_FFFF0000: read_uc_alu_source = 32'hFFFF_0000;
        ALUSRC_CONST_60: read_uc_alu_source = 32'h60;
        ALUSRC_CONST_7FF: read_uc_alu_source = 32'h7FF;
        ALUSRC_CONST_9: read_uc_alu_source = 32'd9;
        ALUSRC_CONST_29: read_uc_alu_source = 32'h29;  // 41
        ALUSRC_CONST_70: read_uc_alu_source = 32'h70;
        ALUSRC_CONST_73: read_uc_alu_source = 32'h73;
        ALUSRC_CONST_1FF: read_uc_alu_source = 32'h1FF;
        ALUSRC_CONST_8200: read_uc_alu_source = 32'h8200;
        ALUSRC_CONST_71: read_uc_alu_source = 32'h47;  // For POPA LDCNTR
        ALUSRC_CONST_NEG1: read_uc_alu_source = 32'hFFFFFFFF;
        ALUSRC_CONST_NEG2: read_uc_alu_source = 32'hFFFFFFFE;
        ALUSRC_CONST_NEG4: read_uc_alu_source = 32'hFFFFFFFC;
        ALUSRC_MASK16: read_uc_alu_source = 32'h0000FFFF;
        ALUSRC_CONST_0: read_uc_alu_source = 32'd0;
        ALUSRC_WORDSZ: read_uc_alu_source = is_dword ? 32'd4 : (op_size == 2'd0 ? 32'd1 : 32'd2);
        ALUSRC_NEGWSZ: read_uc_alu_source = is_dword ? 32'hFFFFFFFC : (op_size == 2'd0 ? 32'hFFFFFFFF : 32'hFFFFFFFE);
        ALUSRC_INCREM: // String increment (±1/±2/±4 based on DF and op size)
            read_uc_alu_source = DF ?
                (op_size == 2'd0 ? 32'hFFFFFFFF :   // DF=1, byte: -1
                 op_size == 2'd1 ? 32'hFFFFFFFE :   // DF=1, word: -2
                                       32'hFFFFFFFC) : // DF=1, dword: -4
                (op_size == 2'd0 ? 32'd1 :         // DF=0, byte: +1
                 op_size == 2'd1 ? 32'd2 :         // DF=0, word: +2
                                       32'd4);         // DF=0, dword: +4
        ALUSRC_BITS_V: read_uc_alu_source = (op_size == 2'd0) ? 32'd7 : (op_size == 2'd2) ? 32'd31 : 32'd15;
        ALUSRC_DSTREG: read_uc_alu_source = read_gpr(i_reg_dst_reg_sel, op_size);
        ALUSRC_SRCREG: read_uc_alu_source = read_gpr(i_reg_src_reg_sel, op_size);
        ALUSRC_ZERO: read_uc_alu_source = 32'd0;
        default: read_uc_alu_source = 32'd0;
    endcase
endfunction

function automatic [31:0] read_uc_source(input [5:0] src_field);
    case (src_field)
        SRC_EAX: read_uc_source = EAX;
        SRC_ECX: read_uc_source = ECX;
        SRC_EDX: read_uc_source = EDX;
        SRC_ESP: read_uc_source = ESP;  // Always full 32-bit; truncation at dest (eSP) or segmentation unit
        SRC_EBP: read_uc_source = EBP;
        SRC_ESI: read_uc_source = ESI;
        SRC_EDI: read_uc_source = EDI;
        SRC_EIP: read_uc_source = EIP;  // IP of next instruction
        SRC_EFLAGS: read_uc_source = EFLAGS;  // Full 32-bit for PUSHFD, masked for PUSHF/LAHF
        SRC_CR0: read_uc_source = CR0;
        SRC_CR2: read_uc_source = CR2;
        SRC_TMPB: read_uc_source = TMPB;
        SRC_TMPC: read_uc_source = TMPC;
        SRC_TMPD: read_uc_source = TMPD;
        SRC_TMPE: read_uc_source = TMPE;
        SRC_TMPF: read_uc_source = TMPF;
        SRC_FLAGSB: read_uc_source = FLAGSB;  // FLAGS backup for INT
        SRC_TMPG: read_uc_source = TMPG;  // Used by far CALL/JMP for saved IP
        SRC_TMPH: read_uc_source = TMPH;     // encoding 0x11
        SRC_TMP_TR: read_uc_source = SLCTR;  // encoding 0x13 = SLCTR2 (32-bit: full descriptor hi for LAR/LSL)
        SRC_COUNTR: read_uc_source = COUNTR;
        SRC_PROTUN: read_uc_source = PROTUN;
        SRC_TMPeIP: read_uc_source = TMPeIP;  // Saved restart IP; destination/IND setup owns width truncation
        SRC_TMPeSP: read_uc_source = is_dword_src ? TMPeSP : {16'h0, TMPeSP[15:0]};  // Saved SP for fault
        SRC_DR6: read_uc_source = DR6;
        SRC_DR7: read_uc_source = DR7;
        SRC_CSOPCD: read_uc_source = CSOPCD;
        SRC_OPROFF: read_uc_source = OPROFF;
        SRC_MDTMP: read_uc_source = RESULT;
        SRC_SIGMA: read_uc_source = SIGMA;
        SRC_IMM: read_uc_source = i_reg_immediate;
        SRC_ES: read_uc_source = ES;
        SRC_CS: read_uc_source = CS;
        SRC_SS: read_uc_source = SS;
        SRC_DS: read_uc_source = DS;
        SRC_FS: read_uc_source = FS;
        SRC_GS: read_uc_source = GS;
        SRC_LDTR: read_uc_source = {16'h0, LDTR};
        SRC_TR: read_uc_source = {16'h0, TR};
        // SRC_SLCTR (0x35) is used ONLY by DESSDT
        SRC_SLCTR: read_uc_source = {16'h0, SLCTR[15:3], 3'b000};
        SRC_eAX_AL: read_uc_source = read_gpr(3'd0, op_size_src);
        SRC_eDX_AH: read_uc_source = read_gpr(
            op_size_src == 2'd0 ? 3'd4 : 3'd2, op_size_src);
        SRC_OPR_R: read_uc_source = OPR_R;
        SRC_IRF2: read_uc_source = IND;          // IRF2 is IND
        SRC_EA: read_uc_source = ea_reg;         // i_pop EA; valid all instruction (next pop is in the RNI delay slot, after all SRC_EA reads)

        SRC_eCX: read_uc_source = ECX; // i.addr32 ? ECX : {16'h0, ECX[15:0]}; Address-size-aware for LOOP/REP
        SRC_IRF: begin  // Indirect Register File read (PUSHA/PUSHAD)
            // COUNTR indexes: 7=EDI, 6=ESI, 5=EBP, 4=ESP, 3=EBX, 2=EDX, 1=ECX, 0=EAX
            // Uses is_dword_src so BITS16/BITS32 affects register width without
            // routing global op_size through this generic source mux.
            case (COUNTR[2:0])
                3'd0: read_uc_source = is_dword_src ? EAX : {16'h0, EAX[15:0]};
                3'd1: read_uc_source = is_dword_src ? ECX : {16'h0, ECX[15:0]};
                3'd2: read_uc_source = is_dword_src ? EDX : {16'h0, EDX[15:0]};
                3'd3: read_uc_source = is_dword_src ? EBX : {16'h0, EBX[15:0]};
                3'd4: read_uc_source = is_dword_src ? ESP : {16'h0, ESP[15:0]};  // Original ESP
                3'd5: read_uc_source = is_dword_src ? EBP : {16'h0, EBP[15:0]};
                3'd6: read_uc_source = is_dword_src ? ESI : {16'h0, ESI[15:0]};
                3'd7: read_uc_source = is_dword_src ? EDI : {16'h0, EDI[15:0]};
            endcase
        end
        SRC_SEGREG: begin
            case (seg_reg_sel)
                3'd0: read_uc_source = {16'h0, ES};
                3'd1: read_uc_source = {16'h0, CS};
                3'd2: read_uc_source = {16'h0, SS};
                3'd3: read_uc_source = {16'h0, DS};
                3'd4: read_uc_source = {16'h0, FS};
                3'd5: read_uc_source = {16'h0, GS};
                default: read_uc_source = 32'h0;
            endcase
        end
        SRC_DSTREG: read_uc_source = read_gpr(i_reg_dst_reg_sel, srcreg_size_src);
        SRC_SRCREG: read_uc_source = read_gpr(i_reg_src_reg_sel, op_size_src);
        SRC_NEG1: read_uc_source = 32'hFFFF_FFFF;
        default: read_uc_source = 32'd0;
    endcase
endfunction

function automatic [4:0] map_alu_op(input [6:0] uc_op);
begin
    casez (uc_op)
        ALUJMP_ALU:    map_alu_op = alu_grp_op; // Pre-decoded at i_pop from opcode/modrm
        ALUJMP_INCDEC: map_alu_op = i.opcode[7] ? {3'b110, alu_grp_op[1:0]}     // F6/F7/FE/FF: INC/DEC/NOT/NEG
                                                 : {4'b1100, alu_grp_op[0]};    // 40-4F: INC/DEC
        ALUJMP_SHIFT1: map_alu_op = ALU_PASS;   // <<>>? First pass - PASS Source to SIGMA
        ALUJMP_CMPTST: map_alu_op = instr_is_cmp ? ALU_CMP : ALU_AND;
        ALUJMP_SZ_EXT: map_alu_op = instr_szext_op;  // Pre-decoded at i_pop
        ALUJMP_AND: map_alu_op = ALU_AND;
        ALUJMP_OR:  map_alu_op = ALU_OR;
        ALUJMP_XOR: map_alu_op = ALU_XOR;
        ALUJMP_SIGN: map_alu_op = ALU_SIGN;     // Sign extension
        ALUJMP_ADD: map_alu_op = ALU_ADD;
        ALUJMP_ADC: map_alu_op = ALU_ADC;
        ALUJMP_SUB: map_alu_op = ALU_SUBT;
        ALUJMP_CMP: map_alu_op = ALU_CMP;
        ALUJMP_SHIFT:  map_alu_op = ALU_PASS;   // Shifter handles this
        ALUJMP_SHIFT2: map_alu_op = ALU_PASS;   // >><<? acts as PASS for shift result
        ALUJMP_PASS2:  map_alu_op = ALU_PASS2;  // Returns ABCDEF (alu_src)
        ALUJMP_AAAAAS: map_alu_op = i.opcode[3] ? ALU_AAS : ALU_AAA;
        ALUJMP_BITS16: map_alu_op = ALU_PASS;   // SIGMA update skipped
        ALUJMP_DAADAS: map_alu_op = i.opcode[3] ? ALU_DAS : ALU_DAA;
        ALUJMP_PASS, ALUJMP_JMP, ALUJMP_NOPMOVE: map_alu_op = ALU_PASS;
        ALUJMP_SERECO: begin                    // Set/Reset/Complement for BT/BTS/BTR/BTC
            // instr_bt_sel pre-decoded at i_pop: register forms use opcode[4:3], immediate (BA) uses modrm[4:3]
            case (instr_bt_sel)
                2'b00: map_alu_op = ALU_PASS;   // BT: just pass through (test only)
                2'b01: map_alu_op = ALU_OR;     // BTS: set bits (dst | src)
                2'b10: map_alu_op = ALU_ANDN;   // BTR: reset bits (dst & ~src)
                2'b11: map_alu_op = ALU_XOR;    // BTC: complement bits (dst ^ src)
            endcase
        end
        default: map_alu_op = ALU_PASS;
    endcase
end
endfunction

function automatic logic div_get_sign_bit(input [31:0] val, input [1:0] sz);
    case (sz)
        2'd0: return val[7];
        2'd1: return val[15];
        default: return val[31];
    endcase
endfunction

function automatic logic [31:0] div_op_by_size(
    input logic op,
    input logic [31:0] val,
    input logic [1:0] sz
);
    case (sz)
        2'd0: div_op_by_size = op == DIV_OP_NEGATE ?
            {24'h0, (~val[7:0]) + 8'h1} : {24'h0, val[7:0]};
        2'd1: div_op_by_size = op == DIV_OP_NEGATE ?
            {16'h0, (~val[15:0]) + 16'h1} : {16'h0, val[15:0]};
        default: div_op_by_size = op == DIV_OP_NEGATE ?
            (~val) + 32'h1 : val;
    endcase
endfunction

task automatic div7_calc(
    input        [31:0] q_in,
    input        [31:0] r_in,
    input        [31:0] d_in,
    input               r_nonneg_prev_in,
    input        [1:0]  op_size_in,
    output       [31:0] q_out,
    output       [31:0] r_out,
    output              r_nonneg_out
);
    int unsigned width;
    logic [31:0] q;
    logic [31:0] r;
    logic [31:0] d;
    logic        q_msb;
    logic [34:0] width_mask;
    logic [34:0] r_extended;
    logic [33:0] r_shifted;
    logic [34:0] r_alu_a;
    logic [34:0] r_next_full;
    logic        r_nonneg;
    logic [31:0] q_next;
    logic [34:0] sign_mask;

    case (op_size_in)
        2'd0: width = 8;
        2'd1: width = 16;
        default: width = 32;
    endcase

    q = div_op_by_size(DIV_OP_MASK, q_in, op_size_in);
    r = div_op_by_size(DIV_OP_MASK, r_in, op_size_in);
    d = div_op_by_size(DIV_OP_MASK, d_in, op_size_in);
    q_msb = q[width - 1];

    width_mask = (35'd1 << width) - 35'd1;
    r_extended = ({3'b0, r} & width_mask);
    if (!r_nonneg_prev_in)
        r_extended = r_extended | (35'd1 << width);
    r_shifted = (r_extended << 1) | q_msb;

    sign_mask = ~((35'd1 << (width + 2)) - 35'd1);
    if (r_shifted[width + 1])
        r_alu_a = {1'b0, r_shifted} | sign_mask;
    else
        r_alu_a = {1'b0, r_shifted};

    r_next_full = r_nonneg_prev_in ? (r_alu_a - {3'b0, d}) : (r_alu_a + {3'b0, d});
    r_nonneg = ~r_next_full[width + 2];
    q_next = q << 1;
    q_next[0] = r_nonneg;

    q_out = div_op_by_size(DIV_OP_MASK, q_next, op_size_in);
    r_out = div_op_by_size(DIV_OP_MASK, r_next_full[31:0], op_size_in);
    r_nonneg_out = r_nonneg;
endtask

// One-hot GPR mux: select register value from one-hot encoded selector
function automatic [31:0] onehot_gpr_mux(input [7:0] sel);
    case (sel)
        8'h01: onehot_gpr_mux = EAX;
        8'h02: onehot_gpr_mux = ECX;
        8'h04: onehot_gpr_mux = EDX;
        8'h08: onehot_gpr_mux = EBX;
        8'h10: onehot_gpr_mux = ESP;
        8'h20: onehot_gpr_mux = EBP;
        8'h40: onehot_gpr_mux = ESI;
        8'h80: onehot_gpr_mux = EDI;
        default: onehot_gpr_mux = 32'h0;
    endcase
endfunction

// Indexed GPR read with delay-slot write bypass for the early EA. The retained
// one-hot selectors are used by hazard logic, not decoded again on this path.
function automatic [31:0] fwd_ea_gpr(input sel_valid, input [2:0] idx);
    reg [31:0] cur, ovl_data;
    reg [1:0]  ovl_mode;
    reg        hit_dly, hit_shc, ovl_we;
    case (idx)
        3'd0: cur = EAX;
        3'd1: cur = ECX;
        3'd2: cur = EDX;
        3'd3: cur = EBX;
        3'd4: cur = ESP;
        3'd5: cur = EBP;
        3'd6: cur = ESI;
        default: cur = EDI;
    endcase
    if (!sel_valid)
        cur = 32'h0;
    hit_dly = dly_gpr_we && sel_valid && (dly_gpr_sel == idx);
    hit_shc = fast_shc_pending && sel_valid &&
              ((fast_shc_size != 2'd0 && fast_shc_dst == idx) ||
               (fast_shc_size == 2'd0 && {1'b0, fast_shc_dst[1:0]} == idx));
    ovl_we   = hit_dly || hit_shc;
    // dly_fwd_value, not dest_value: the dedicated 19-leaf register mux
    // (see its definition) keeps read_uc_source out of the EA adders.
    ovl_data = hit_dly ? dly_fwd_value : fast_shc_data;
    ovl_mode = hit_dly ? dly_gpr_mode :
               (fast_shc_size == 2'd0) ? (fast_shc_dst[2] ? FWD_BHI : FWD_BLO) :
               (fast_shc_size == 2'd1) ? FWD_W : FWD_D;
    if (ovl_we)
        case (ovl_mode)
            FWD_BLO: fwd_ea_gpr = {cur[31:8],  ovl_data[7:0]};
            FWD_BHI: fwd_ea_gpr = {cur[31:16], ovl_data[7:0], cur[7:0]};
            FWD_W:   fwd_ea_gpr = {cur[31:16], ovl_data[15:0]};
            default: fwd_ea_gpr = ovl_data;  // FWD_D
        endcase
    else
        fwd_ea_gpr = cur;
endfunction

// Operand prep shared by the i_pop offset and fused linear adders:
// returns {base_val, scaled_val} after scale / scale_to_base resolution.
function automatic [63:0] calc_ea_prep(
    input [31:0] base_in, input [31:0] index_in,
    input [1:0] scale, input scale_to_base);
    reg [31:0] base_val, scaled_val;
    begin
        if (scale_to_base) begin
            case (scale)
                2'b00: base_val = base_in;
                2'b01: base_val = base_in << 1;
                2'b10: base_val = base_in << 2;
                2'b11: base_val = base_in << 3;
            endcase
            scaled_val = 32'h0;   // Scale consumed into base; index consumed by s2b
        end else begin
            base_val = base_in;
            case (scale)
                2'b00: scaled_val = index_in;
                2'b01: scaled_val = index_in << 1;
                2'b10: scaled_val = index_in << 2;
                2'b11: scaled_val = index_in << 3;
            endcase
        end
        calc_ea_prep = {base_val, scaled_val};
    end
endfunction


endmodule
