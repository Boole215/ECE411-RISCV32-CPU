module cpu
  import rv32i_types::*;
(
    /* CP1 Signals*/
    // remove after cp1
    // input 					instr_mem_resp,
    // input rv32i_word 	    instr_mem_rdata,
    // input 					data_mem_resp,
    // input rv32i_word 	    data_mem_rdata,
    // output logic 			instr_read,
    // output rv32i_word 	    instr_mem_address,
    // output logic 			data_read,
    // output logic 			data_write,
    // output logic [3:0] 	    data_mbe,
    // output rv32i_word 	    data_mem_address,
    // output rv32i_word 	    data_mem_wdata,

    /*...Inputs...*/
    input clk,
    input rst,
    // For CP2

    //From physical memory
    input pmem_resp,
    input [63:0] pmem_rdata,

    //To physical memory
    output logic pmem_read,
    output logic pmem_write,
    output rv32i_word pmem_address,
    output [63:0] pmem_wdata


);
  /*
Theory:

We'll need 4 modules for our CPU:

1: datapath (duh)

2: icache

3: dcache

4: arbiter

*/
  /* CPU Datapath Signals */
  logic              instr_mem_resp;
  rv32i_word         instr_mem_rdata;
  logic              data_mem_resp;
  rv32i_word         data_mem_rdata;

  /*...Outputs...*/
  logic              instr_read;
  rv32i_word         instr_mem_address;
  logic              data_read;
  logic              data_write;
  logic      [  3:0] data_mbe;
  rv32i_word         data_mem_address;
  rv32i_word         data_mem_wdata;

  /* I-Cache and D-Cache to Arbiter Signals */
  logic      [255:0] dcache_pmem_rdata;
  logic              dcache_pmem_resp;
  logic      [ 31:0] dcache_pmem_address;
  logic      [255:0] dcache_pmem_wdata;
  logic              dcache_pmem_read;
  logic              dcache_pmem_write;

  logic      [255:0] icache_pmem_rdata;
  logic              icache_pmem_resp;
  logic      [ 31:0] icache_pmem_address;
  //   logic      [255:0] icache_pmem_wdata;
  logic              icache_pmem_read;
  //   logic              icache_pmem_write;
  /* Arbiter<--->Cacheline Signals */
  logic      [255:0] arbiter_wdata;
  logic      [255:0] arbiter_rdata;
  logic      [ 31:0] arbiter_address;
  logic              arbiter_read;
  logic              arbiter_write;
  logic              adaptor_resp;

  cpu_datapath cpu_datapath (.*);

  //cacheline adaptor between pmem <-> adapter <-> arbiter
  cacheline_adaptor cacheline_adaptor (

      .clk(clk),
      .reset_n(~rst),
      // Port to Arbiter
      .line_o(arbiter_rdata),
      .resp_o(adaptor_resp),

      // Port from Arbiter
      .line_i(arbiter_wdata),
      .address_i(arbiter_address),
      .read_i(arbiter_read),
      .write_i(arbiter_write),

      // Port to memory
      .burst_o(pmem_wdata),
      .address_o(pmem_address),
      .read_o(pmem_read),
      .write_o(pmem_write),

      // Port from memory
      .burst_i(pmem_rdata),
      .resp_i (pmem_resp)
  );

  arbiter arbiter (
      .clk,
      .rst,

      /*  Cacheline -> Arbiter  */
      .pmem_rdata(arbiter_rdata),
      .pmem_resp (adaptor_resp),

      /*  Arbiter -> Cacheline */
      .pmem_read(arbiter_read),
      .pmem_write(arbiter_write),
      .pmem_address(arbiter_address),
      .pmem_wdata(arbiter_wdata),

      /*  Arbiter -> I-Cache Memory Signals  */
      .icache_pmem_rdata,
      .icache_pmem_resp,

      /*  I-Cache Memory Signals -> Arbiter  */
      .icache_pmem_address,
      .icache_pmem_read,
      //   .icache_pmem_wdata,
      //   .icache_pmem_write,

      /*  Arbiter -> D-Cache Memory Signals  */
      .dcache_pmem_rdata,
      .dcache_pmem_resp,

      /*  D-Cache Memory Signals -> Arbiter  */
      .dcache_pmem_address,
      .dcache_pmem_wdata,
      .dcache_pmem_read,
      .dcache_pmem_write

  );


  /* icache module  */
  cache icache (
      .clk,
      .rst,
      /* Icache -> Arbiter */
      .pmem_address(icache_pmem_address),
      .pmem_read(icache_pmem_read),
      /* Arbiter -> Icache */
      .pmem_resp(icache_pmem_resp),
      .pmem_rdata(icache_pmem_rdata),


      /* Icache -> CPU */
      .mem_resp(instr_mem_resp),
      .mem_rdata_cpu(instr_mem_rdata),

      /* CPU -> Icache */
      .mem_read(instr_read),
      .mem_write(1'b0),
      .mem_byte_enable_cpu(4'b1111),
      .mem_address(instr_mem_address),
      .mem_wdata_cpu(32'b0)
  );


  /* dcache module  */
  cache dcache (
      .clk,
      .rst,


      /* Dcache -> Arbiter */
      .pmem_address(dcache_pmem_address),
      .pmem_read(dcache_pmem_read),
      .pmem_write(dcache_pmem_write),
      .pmem_wdata(dcache_pmem_wdata),
      /* Arbiter -> Dcache */
      .pmem_resp(dcache_pmem_resp),
      .pmem_rdata(dcache_pmem_rdata),

      /* Dcache -> CPU */
      .mem_read(data_read),
      .mem_write(data_write),
      .mem_byte_enable_cpu(data_mbe),
      .mem_address(data_mem_address),
      .mem_wdata_cpu(data_mem_wdata),

      /* CPU -> Dcache */
      .mem_resp(data_mem_resp),
      .mem_rdata_cpu(data_mem_rdata)
  );


endmodule : cpu
