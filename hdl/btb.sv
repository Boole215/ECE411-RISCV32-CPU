import rv32i_types::*;

module btb (
    input logic clk,
    input logic rst,
    input logic br_instr_fetch,
    input logic br_instr_exec,
    input logic [31:0] requested_pc,
    input logic [31:0] execute_pc_val,
    input logic [31:0] newly_calc_target,
    output logic [31:0] target_pc,
    output logic found
);

typedef struct packed {
    logic valid;
    logic tbd;
    logic [29:0] tag;
    logic [31:0] target_addr;
} btb_entry;

btb_entry btb [8];

// logic [2:0] btb_idx;
// assign btb_idx = requested_pc[4:2];
// assign found = 1'b0;

// btb[requested_pc[4:2]].tag
// btb[requested_pc[4:2]].tbd
// btb[requested_pc[4:2]].valid
// btb[requested_pc[4:2]].target_addr

always_ff @(posedge clk) begin
    if (rst) begin
      for (int i = 0; i < 8; ++i) btb[i] <= 64'b0;
    end
    // if branch instruction and valid and incoming pc matches tag
    // to find
    if (br_instr_fetch) begin
        if ( (btb[requested_pc[4:2]].valid) && (requested_pc[31:2] == (btb[requested_pc[4:2]].tag) ) && (btb[requested_pc[4:2]].tbd == 1'b0)) begin
            target_pc <= btb[requested_pc[4:2]].target_addr;
            found <= 1'b1;
            btb[requested_pc[4:2]].valid <= 1'b1;
            btb[requested_pc[4:2]].tbd <= 1'b0;
        end
        else begin
            btb[requested_pc[4:2]].tag <= requested_pc[31:2];
            btb[requested_pc[4:2]].tbd <= 1'b1;
            found <= 1'b0;
            btb[requested_pc[4:2]].valid <= 1'b0;
        end
    end
    // to update if this was the instruc we were waiting for 
    if ( br_instr_exec && (execute_pc_val[31:2] == btb[execute_pc_val[4:2]].tag) && (btb[execute_pc_val[4:2]].tbd) ) begin
        btb[execute_pc_val[4:2]].target_addr <= newly_calc_target;
        btb[requested_pc[4:2]].tbd <= 1'b1;
        btb[execute_pc_val[4:2]].valid <= 1'b1;
    end

end

endmodule


