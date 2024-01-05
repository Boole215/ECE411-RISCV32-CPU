module mp4_tb;
`timescale 1ns/10ps

/********************* Do not touch for proper compilation *******************/
// Instantiate Interfaces
tb_itf itf();
rvfi_itf rvfi(itf.clk, itf.rst);

// Instantiate Testbench
source_tb tb(
    .magic_mem_itf(itf),
    .mem_itf(itf),
    .sm_itf(itf),
    .tb_itf(itf),
    .rvfi(rvfi)
);

// Dump signals
initial begin
    $fsdbDumpfile("dump.fsdb");
    $fsdbDumpvars(0, mp4_tb, "+all");
end
/****************************** End do not touch *****************************/


/***************************** Spike Log Printer *****************************/
// Can be enabled for debugging
spike_log_printer printer(.itf(itf), .rvfi(rvfi));
/*************************** End Spike Log Printer ***************************/


/************************ Signals necessary for monitor **********************/
// This section not required until CP2

// TODO: The current bug with this lies with our cmp_out value when we reset our registers.
// The EXE_MEM CWord will begin branching because our rs1 data is == to cmp_mux_out.
assign rvfi.commit = dut.cpu.cpu_datapath.MEM_WRB_controlword.commit;

// Set high when target PC == Current PC for a branch

assign rvfi.halt = dut.cpu.cpu_datapath.MEM_WRB_controlword.PC_val == dut.cpu.cpu_datapath.MEM_WRB_controlword.PC_mux_val && rvfi.commit;

assign rvfi.inst = dut.cpu.cpu_datapath.MEM_WRB_controlword.instruction;
assign rvfi.trap = 1'b0;

assign rvfi.rs1_addr = dut.cpu.cpu_datapath.MEM_WRB_controlword.rs1;
assign rvfi.rs2_addr = dut.cpu.cpu_datapath.MEM_WRB_controlword.rs2;
assign rvfi.rs1_rdata = dut.cpu.cpu_datapath.MEM_WRB_controlword.rs1_data;
assign rvfi.rs2_rdata = dut.cpu.cpu_datapath.MEM_WRB_controlword.rs2_data;

assign rvfi.load_regfile = dut.cpu.cpu_datapath.MEM_WRB_controlword.load_regfile;
assign rvfi.rd_addr = dut.cpu.cpu_datapath.MEM_WRB_controlword.rd;
assign rvfi.rd_wdata = dut.cpu.cpu_datapath.regfilemux_out;

assign rvfi.pc_rdata = dut.cpu.cpu_datapath.MEM_WRB_controlword.PC_val;
assign rvfi.pc_wdata = dut.cpu.cpu_datapath.MEM_WRB_controlword.PC_mux_val;

assign rvfi.mem_addr = dut.cpu.cpu_datapath.MEM_WRB_controlword.alu_out;
assign rvfi.mem_rmask = dut.cpu.cpu_datapath.MEM_WRB_controlword.read_mask;
//assign rvfi.mem_rmask = 0;
assign rvfi.mem_wmask = dut.cpu.cpu_datapath.MEM_WRB_controlword.write_mask;
assign rvfi.mem_rdata = dut.cpu.cpu_datapath.mem_rdata_out_mem;
// TODO: There is definitely a better way of doing the thing below
assign rvfi.mem_wdata = dut.cpu.cpu_datapath.MEM_WRB_controlword.rs2_data << (8*dut.cpu.cpu_datapath.MEM_WRB_controlword.alu_out[1:0]);


// Set high when a valid instruction is modifying regfile or PC
initial rvfi.order = 0;
always @(posedge itf.clk iff rvfi.commit) rvfi.order <= rvfi.order + 1; // Modify for OoO

/*
Instruction and trap:
    rvfi.inst
    rvfi.trap

Regfile:
    rvfi.rs1_addr
    rvfi.rs2_addr
    rvfi.rs1_rdata
    rvfi.rs2_rdata
    rvfi.load_regfile
    rvfi.rd_addr
    rvfi.rd_wdata

PC:
    rvfi.pc_rdata
    rvfi.pc_wdata

Memory:
    rvfi.mem_addr
    rvfi.mem_rmask
    rvfi.mem_wmask
    rvfi.mem_rdata
    rvfi.mem_wdata

Please refer to rvfi_itf.sv for more information.
*/



/**************************** End RVFIMON signals ****************************/



/********************* Assign Shadow Memory Signals Here *********************/
// This section not required until CP2
/*
The following signals need to be set:
icache signals:
    itf.inst_read
    itf.inst_addr
    itf.inst_resp
    itf.inst_rdata

dcache signals:
    itf.data_read
    itf.data_write
    itf.data_mbe
    itf.data_addr
    itf.data_wdata
    itf.data_resp
    itf.data_rdata

Please refer to tb_itf.sv for more information.
*/

assign itf.inst_read = dut.cpu.instr_read;
assign itf.inst_addr = dut.cpu.instr_mem_address;
assign itf.inst_resp = dut.cpu.instr_mem_resp;
assign itf.inst_rdata = dut.cpu.instr_mem_rdata;

assign itf.data_read = dut.cpu.data_read;
assign itf.data_write = dut.cpu.data_write;
assign itf.data_mbe = dut.cpu.data_mbe;
assign itf.data_addr = dut.cpu.data_mem_address;
assign itf.data_wdata = dut.cpu.data_mem_wdata;
assign itf.data_resp = dut.cpu.data_mem_resp;
assign itf.data_rdata = dut.cpu.data_mem_rdata;


/*********************** End Shadow Memory Assignments ***********************/

// Set this to the proper value
assign itf.registers = '{default: '0};

/*********************** Instantiate your design here ************************/
/*
The following signals need to be connected to your top level for CP2:
Burst Memory Ports:
    itf.mem_read
    itf.mem_write
    itf.mem_wdata
    itf.mem_rdata
    itf.mem_addr
    itf.mem_resp

Please refer to tb_itf.sv for more information.
*/

mp4 dut(
    .clk(itf.clk),
    .rst(itf.rst),

    .pmem_read(itf.mem_read),
    .pmem_write(itf.mem_write),
    .pmem_wdata(itf.mem_wdata),
    .pmem_rdata(itf.mem_rdata),
    .pmem_address(itf.mem_addr),
    .pmem_resp(itf.mem_resp)
);
/***************************** End Instantiation *****************************/

endmodule
