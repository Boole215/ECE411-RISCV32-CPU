import rv32i_types::*;

module global_branch_history_table (
    input logic clk,
    input logic rst,
    input logic br_info_avail,
    input logic true_br_val,
    input logic[31:0] incoming_pc,
    output logic pred_br_result,
    output logic mispredict,
    input mem_stall
);

typedef enum logic[1:0] {  
    w_nt,
    s_nt,
    w_t,
    s_t
} fsm_states;

/* Creating the Branch History Table, and 3 bit wide global pattern register: */
fsm_states bht[32][16];
logic [3:0] gpr; // gpr determines which column to go to



/* Each pipelined register needs to store its own index for the pht, value of the pht (index into bht), current fsm prediction and a prediction of whether the branch was taken or not (denoted by first bit of the fsm state) */
// row
logic [4:0] fetch_bht_idx;
logic [4:0] decode_bht_idx;
logic [4:0] execute_bht_idx;
//gbhr
logic [3:0] fetch_gpr_col;
logic [3:0] decode_gpr_col;
logic [3:0] execute_gpr_col;
//state
fsm_states fetch_bht_entry;
fsm_states decode_bht_entry;
fsm_states execute_bht_entry;

logic [3:0] new_gpr; // in execute, when we find out the new branch direction, we update the gpr associated with it.  
fsm_states new_bht_entry; // in execute, when we find out the new branch direction, we update the bht entry associated with it with an fsm.  

assign pred_br_result = fetch_bht_entry[1];
always_comb begin
    // updating fetch's info as it will be giving result before new pc is loaded
    fetch_bht_idx = incoming_pc[7:3];
    fetch_gpr_col = gpr;
    fetch_bht_entry = bht[incoming_pc[7:3]][gpr];// ignore lowest 2 bits - always 0 for pc.
    new_gpr = {gpr[2:0], true_br_val};
    
    new_bht_entry = execute_bht_entry;
    
    // Need to update the BHT:
    // when the ex stage properly checks the result of the branch (taken, not taken)
    // we need to update the current state and FSM.
    mispredict = 1'b0;
    if (br_info_avail) begin
        unique case (execute_bht_entry) 
            s_t: begin
                if (true_br_val == 1'b0) begin
                    new_bht_entry = w_t;
                    mispredict = 1'b1;
                end
                else begin
                    new_bht_entry = s_t;
                    mispredict = 1'b0;
                end
            end
            w_t: begin
                if (true_br_val == 1'b0) begin
                    new_bht_entry = w_nt;
                    mispredict = 1'b1;
                end
                else begin
                    new_bht_entry = s_t;
                    mispredict = 1'b0;
                end
            end
            s_nt: begin
                if (true_br_val == 1'b0) begin
                    new_bht_entry = s_nt;
                    mispredict = 1'b0;
                end
                else begin
                    new_bht_entry = w_nt;
                    mispredict = 1'b1;
                end
            end
            w_nt: begin
                if (true_br_val == 1'b0) begin
                    new_bht_entry = s_nt;
                    mispredict = 1'b0;
                end
                else begin
                    new_bht_entry = w_t;
                    mispredict = 1'b1;
                end
            end
            default: ;
        endcase
    end

end

always_ff @(posedge clk) begin
    if (rst) begin
        gpr <= 4'b0;
        // clear pht and set bht entries to weakly not taken
        for (int i = 0; i < 32; i++) begin
            for (int j = 0; j < 16; j++) begin
                bht[i][j] <= w_nt;
            end
        end
    end
    else ;
    if (!rst && br_info_avail) begin
        bht[execute_bht_idx][execute_gpr_col] <= new_bht_entry;
        gpr <= new_gpr;
    end
    else ;

    if (mem_stall) begin
        
        decode_bht_idx <= decode_bht_idx;
        decode_gpr_col <= decode_gpr_col;
        decode_bht_entry <= decode_bht_entry;

        execute_bht_idx <= execute_bht_idx;
        execute_gpr_col <= execute_gpr_col;
        execute_bht_entry <= execute_bht_entry;

    end
    else begin

        decode_bht_idx <= fetch_bht_idx;
        decode_gpr_col <= fetch_gpr_col;
        decode_bht_entry <= fetch_bht_entry;

        execute_bht_idx <= decode_bht_idx;
        execute_gpr_col <= decode_gpr_col;
        execute_bht_entry <= decode_bht_entry;
    end
end
endmodule

/*

  logic [32:0] br_cnt;
  logic [32:0] mispred_cnt;
  logic predicted_br_result;
  logic mispredicted;
  global_branch_history_table(
    .clk(clk),
    .rst(rst),
    .update(EXE_MEM_controlword.opcode == op_br),
    .br_en(EXE_MEM_controlword.br_en),
    .addr(IFE_DEC_controlword.PC_val),
    .br_take(predicted_br_result), 
    .mispred(mispredicted), 
    .mem_stall(mem_stall)
  );
always_ff @(posedge clk) begin
    if (rst) begin
    br_cnt = 32'b0;
    mispred_cnt = 32'b0;
    end
    if (!rst && EXE_MEM_controlword.opcode == op_br)
      br_cnt <= br_cnt + 1;
    if (!rst && mispredicted)
      mispred_cnt <= mispred_cnt + 1;
end

*/
