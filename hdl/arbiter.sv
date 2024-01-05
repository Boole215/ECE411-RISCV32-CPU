module arbiter
  import rv32i_types::*;
(
    input logic         clk,
    input logic         rst,
    /*  Physical Memory Signals (arbiter <-> cacheline adapter) */
    input logic         pmem_resp,
    input logic [255:0] pmem_rdata,

    output logic              pmem_read,
    output logic              pmem_write,
    output rv32i_word         pmem_address,
    output logic      [255:0] pmem_wdata,


    /*  I-Cache Memory Signals  (arbiter <-> icache)*/

    // only need icache to send read signal: icache_pmem_read and address: icache_pmem_address
    input  logic         icache_pmem_read,
    input  logic [ 31:0] icache_pmem_address,
    output logic [255:0] icache_pmem_rdata,    // changed to output logic - sent to icache
    output logic         icache_pmem_resp,
    //did not use
    // output  logic [31:0]    icache_pmem_wdata,
    // input  logic            icache_pmem_write,

    /*  D-Cache Memory Signals  */


    input logic         dcache_pmem_read,
    input logic         dcache_pmem_write,
    input logic [255:0] dcache_pmem_wdata,
    input logic [ 31:0] dcache_pmem_address,

    output logic [255:0] dcache_pmem_rdata,  // changed to 255:0
    output logic         dcache_pmem_resp
);


  /*
    What this does:
    We have two separate caches:
    our i-cache, and our d-cache.
    But we only have one physical memory port.
    Therefore, we need to manage our cache requests.

    There are 3 cases I intend to write for this to handle:

    1 & 2: dcache / icache has a miss alone.

    3: dcache and icache miss together

    This doesn't handle cases where
    there is an active miss being handled,
    and then another miss comes in.
    This is because our design currently stalls on a cache miss, since inserting
    no-ops, would be barely more efficient, and much more complicated.
    i had dreams of parameterizing this, but it makes the code much harder to read,
    so for the sake of comprehension & readability, i will not. - Q
    */

  // 3-always logic
  enum int unsigned {
    /* List of states */
    hold,
    instruction_state,
    data_state
  }
      state, next_state;
  logic mem_stall = 1'b0;
  // Next state logic
  always_comb begin
    case (state)
      hold: begin
        if (icache_pmem_read) next_state = instruction_state;
        else if (dcache_pmem_read || dcache_pmem_write) next_state = data_state;
        else next_state = hold;
      end

      instruction_state: begin
        if (pmem_resp) //if physical memory is done with reading information to store in icache then go back to hold
          next_state = hold;
        else next_state = instruction_state;
      end

      data_state: begin
        if (pmem_resp) //if physical memory is done with reading information to store in icache then go back to hold
          next_state = hold;
        else next_state = data_state;
      end
      default: next_state = hold;
    endcase
  end

  // next state assignment
  always_ff @(posedge clk) begin
    if (rst) begin
      state <= hold;
    end else begin
      state <= next_state;
    end
  end

  always_comb begin
    pmem_read = 0;
    pmem_write = 0;
    pmem_address = 0;
    pmem_wdata = 0;
    icache_pmem_rdata = 0;
    icache_pmem_resp = 0;
    dcache_pmem_rdata = 0;
    dcache_pmem_resp = 0;

    case (state)
      hold: mem_stall = 1'b0;

      instruction_state: begin

        /* In this state, the physical memory needs to get the address required and give it back to icache */
        pmem_read = 1'b1;
        mem_stall = 1'b1;
        pmem_address = icache_pmem_address;
        icache_pmem_rdata = pmem_rdata;  //info read from pmem to be written into icache
        if (pmem_resp) begin
          icache_pmem_resp = 1'b1;
          mem_stall = 1'b0;
        end else begin
          icache_pmem_resp = 1'b0;
        end
      end

      data_state: begin
        /* In this state, the physical memory needs to get the address required write there or read from there and give to dcache */
        pmem_address = dcache_pmem_address;
        dcache_pmem_rdata = pmem_rdata;
        pmem_wdata = dcache_pmem_wdata;
        mem_stall = 1'b1;
        if (pmem_resp) begin
          dcache_pmem_resp = 1'b1;
          mem_stall = 1'b0;
        end else begin
          dcache_pmem_resp = 1'b0;
        end
        if (dcache_pmem_read) pmem_read = 1'b1;
        else pmem_read = 1'b0;

        if (dcache_pmem_write) pmem_write = 1'b1;
        else pmem_write = 1'b0;
      end
    endcase
  end
endmodule


// module mem_arbiter
// import rv32i_types::*;
// (
//     input logic clk,
//     input logic rst,
//     output logic mem_stall,
// /*  Physical Memory Signals  */
//     input logic             pmem_resp,

//     output logic            pmem_read,
//     output logic            pmem_write,
//     output rv32i_word       pmem_address,
//     output logic [255:0]    pmem_wdata,
//     output logic [255:0]    pmem_rdata,


// /*  I-Cache Memory Signals  */
//     output  logic [31:0]    icache_pmem_rdata,
//     output  logic           icache_pmem_resp,

//     input  logic [31:0]     icache_pmem_address,
//     input  logic [255:0]    icache_pmem_wdata,
//     input  logic            icache_pmem_read,
//     input  logic            icache_pmem_write,


// /*  D-Cache Memory Signals  */
//     output  logic [31:0]    dcache_pmem_rdata,
//     output  logic           dcache_pmem_resp, 

//     input  logic [31:0]     dcache_pmem_address,
//     input  logic [255:0]    dcache_pmem_wdata,
//     input  logic            dcache_pmem_read,
//     input  logic            dcache_pmem_write

// );

// alias dcache_case = 2'b1?;
// alias icache_case = 2'b01;

// logic           arbiter_idx;
// logic [1:0]     arbitration_bits;

// logic [31:0]     arbiter_pmem_address [1:0];
// logic [255:0]    arbiter_pmem_wdata   [1:0];
// logic            arbiter_pmem_read    [1:0];
// logic            arbiter_pmem_write   [1:0];

// always_comb begin
//     //dcache index = 1, icache index = 0.
//     arbitration_bits = {dcache_pmem_write || dcache_pmem_read, icache_pmem_write || icache_pmem_read};
//     case(arbitration_bits)
//         dcache_case: begin 
//             icache_pmem_resp = pmem_resp; 
//             dcache_pmem_resp = 1'b0;
//             arbiter_idx = 1'b1;
//             mem_stall = 1'b1;
//         end
//         icache_case: begin 
//             dcache_pmem_resp = pmem_resp;
//             icache_pmem_resp = 1'b0;
//             arbiter_idx = 1'b0;
//             mem_stall = 1'b1;
//         end
//         default:  
//             arbiter_idx = 1'b0;
//             icache_pmem_resp = 1'b0;
//             dcache_pmem_resp = 1'b0;
//             mem_stall = 1'b0;
//     endcase

//     arbiter_pmem_address = {dcache_pmem_address, icache_pmem_address};
//     arbiter_pmem_wdata   = {dcache_pmem_wdata  , icache_pmem_wdata  };
//     arbiter_pmem_read    = {dcache_pmem_read   , icache_pmem_read   };
//     arbiter_pmem_write   = {dcache_pmem_write  , icache_pmem_write  };


//     pmem_read     = arbiter_pmem_address[arbiter_idx];
//     pmem_write    = arbiter_pmem_wdata[arbiter_idx]  ;
//     pmem_address  = arbiter_pmem_read[arbiter_idx]   ;
//     pmem_wdata    = arbiter_pmem_write[arbiter_idx]  ;

//     icache_pmem_rdata = pmem_rdata;
//     dcache_pmem_rdata = pmem_rdata;
//     end
// endmodule
