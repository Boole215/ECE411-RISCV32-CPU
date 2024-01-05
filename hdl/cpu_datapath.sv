module cpu_datapath
  import rv32i_types::*;
(

    /*...Inputs...*/
    input clk,
    input rst,


    input            instr_mem_resp,
    input rv32i_word instr_mem_rdata,
    input            data_mem_resp,
    input rv32i_word data_mem_rdata,

    /*...Outputs...*/
    output logic            instr_read,
    output rv32i_word       instr_mem_address,
    output logic            data_read,
    output logic            data_write,
    output logic      [3:0] data_mbe,
    output rv32i_word       data_mem_address,
    output rv32i_word       data_mem_wdata
);

  /*****************************************************/
  /*.............Instruction Decode Signals ...........*/
  /*****************************************************/


  /*
Control Word name scheme:
part1_part2_controlword
part1 is the RECIEVING edge of the control word register.
part2 is the SENDING edge of the control word register.
*/
  /*.....Control words......*/
  rv32i_control_word control_rom_o;
  rv32i_control_word IFE_DEC_controlword;
  rv32i_control_word DEC_EXE_controlword;
  rv32i_control_word EXE_MEM_controlword;
  rv32i_control_word MEM_WRB_controlword;
  /*........................*/
  logic    mem_stall;
  logic load_regfile;
  alumux::alumux1_sel_t alumux1_sel;
  alumux::alumux2_sel_t alumux2_sel;
  regfilemux::regfilemux_sel_t regfilemux_sel;
  logic cmpmux_sel;
  alu_ops aluop;
  branch_funct3_t cmpop;
  rv32i_reg rs1;
  rv32i_reg rs2;
  rv32i_reg rd;

  /* Output of the muxes */
  rv32i_word pcmux_out;
  rv32i_word pc_out;
  rv32i_word alumux1_out;
  rv32i_word alumux2_out;
  rv32i_word cmp_mux_out;
  rv32i_word regfilemux_out;
  rv32i_word rs1_out;
  rv32i_word rs2_out;
  rv32i_word mem_rdata_out_mem;
  rv32i_word mem_rdata_out_wb;
  logic br_en;
  logic cmp_op_mux_out;
  /* regfile intermediate variables */
  rv32i_word regfile_rs1_data;
  rv32i_word regfile_rs2_data;
  /* ALU module intermediate variable */
  rv32i_word alu_data_out;
  /* Writeback intermediate variable */
  logic [1:0] bottom_two_bits;

  /* Forwarding unit Variables */
  //if bit 0, rs1 match exemem, if bit 1, rs2 match exemem, if bit 2, rs1 match memwrb, if bit 3, rs2 match memwrb, if bit 4, load case
  logic [4:0]forward_case;
  wire forward_load_case = forward_case[4] && (forward_case[0] || forward_case[1]);
  rv32i_word for_mem_data;
  rv32i_word for_wrb_data;
  rv32i_word exe_rs1_data;
  rv32i_word exe_rs2_data;


  /*********************************/
  /**********............***********/
  /*****.......................*****/
  /*........Forwarding Unit........*/
  /*****.......................*****/
  /**********............***********/
  /*********************************/

  always_comb begin
    forward_case = {
      EXE_MEM_controlword.opcode == op_load,

      (MEM_WRB_controlword.has_rd && MEM_WRB_controlword.rd != 0) && (MEM_WRB_controlword.rd == DEC_EXE_controlword.rs2),
      (MEM_WRB_controlword.has_rd && MEM_WRB_controlword.rd != 0) && (MEM_WRB_controlword.rd == DEC_EXE_controlword.rs1),

      (EXE_MEM_controlword.has_rd && EXE_MEM_controlword.rd != 0) && (EXE_MEM_controlword.rd == DEC_EXE_controlword.rs2),
      (EXE_MEM_controlword.has_rd && EXE_MEM_controlword.rd != 0) && (EXE_MEM_controlword.rd == DEC_EXE_controlword.rs1)
    };




    unique case (EXE_MEM_controlword.regfilemux_sel)
      regfilemux::alu_out: for_mem_data = EXE_MEM_controlword.alu_out;
      regfilemux::br_en:
      for_mem_data = {
        31'b0, EXE_MEM_controlword.branch
      };  // since we need to load 1 in, we zero extend branch enable so that the output of the mux is 32 bits storing val br_en
      regfilemux::u_imm: for_mem_data = EXE_MEM_controlword.u_imm;
      regfilemux::pc_plus4: for_mem_data = EXE_MEM_controlword.PC_val + 4;
      default: for_mem_data = EXE_MEM_controlword.alu_out;
    endcase

    for_wrb_data = regfilemux_out;

  end



  // code by geitanksha
  logic [31:0] br_cnt;
  logic [31:0] mispred_cnt;
  logic predicted_br_result;
  logic mispredicted;
  
  // local_branch_history_table(
  //   .clk(clk),
  //   .rst(rst),
  //   .br_info_avail(EXE_MEM_controlword.opcode == op_br),
  //   .true_br_val(EXE_MEM_controlword.br_en),
  //   .incoming_pc(IFE_DEC_controlword.PC_val),
  //   .pred_br_result(predicted_br_result), 
  //   .mispredict(mispredicted), 
  //   .mem_stall(mem_stall)
  // );
  
    global_branch_history_table(
    .clk(clk),
    .rst(rst),
    .br_info_avail(EXE_MEM_controlword.opcode == op_br),
    .true_br_val(EXE_MEM_controlword.br_en),
    .incoming_pc(IFE_DEC_controlword.PC_val),
    .pred_br_result(predicted_br_result), 
    .mispredict(mispredicted), 
    .mem_stall(mem_stall)
  );
  
  logic [31:0] new_targ_addr;
  logic found;

  btb (
    .clk(clk),
    .rst(rst),
    .br_instr_fetch(IFE_DEC_controlword.opcode == op_br),
    .br_instr_exec(EXE_MEM_controlword.opcode == op_br),
    .requested_pc(IFE_DEC_controlword.PC_val),
    .execute_pc_val(EXE_MEM_controlword.PC_val),
    .newly_calc_target(EXE_MEM_controlword.alu_out),
    .target_pc(new_targ_addr),
    .found(found)
);
// end of code by geitanksha



always_ff @(posedge clk) begin
    if (rst) begin
    br_cnt <= 32'b0;
    mispred_cnt <= 32'b0;
    end
    if (!rst && EXE_MEM_controlword.opcode == op_br && !mem_stall)
      br_cnt <= br_cnt + 1;
    if (!rst && mispredicted && !mem_stall)
      mispred_cnt <= mispred_cnt + 1;
end




  /*********************************/
  /**********............***********/
  /*****.......................*****/
  /*........Instruction Fetch......*/
  /*****.......................*****/
  /**********............***********/
  /*********************************/
  /* Control rom for instantiating the first control word.*/
  control_rom control_rom (
      //NOTE: using the raw instr_mem_rdata feels bad. I want to maybe put a register here for safety. Idk how necessary that is tho. -q
      .fetched_instruction_i(instr_mem_rdata),
      .PC_val_i(pc_out),
      .PC_mux_val_i(pcmux_out),
      .control_word_o(control_rom_o)
  );

  always_comb begin
    unique case (EXE_MEM_controlword.br_en)
      pcmux::pc_plus4: pcmux_out = pc_out + 4;
      pcmux::alu_out: begin
        if (DEC_EXE_controlword.opcode == op_jalr) pcmux_out = EXE_MEM_controlword.alu_out & ~1;
        else pcmux_out = EXE_MEM_controlword.alu_out;
      end
      default:         pcmux_out = pc_out + 4;  // Appendix D
    endcase  // unique case (br_en)
    mem_stall = (!instr_mem_resp) || ((data_write || data_read) && (!data_mem_resp));
    instr_mem_address = pc_out;

  end


  always_ff @(posedge clk) begin
    /* PC reg */
    //instr_mem_address <= pc_out;
    instr_read <= 1'b1;
    pc_out <= pcmux_out;

    // code by geitanksha
    if (IFE_DEC_controlword.br_en && found && predicted_br_result)  begin
    pc_out <= new_targ_addr;
    end
    // end of code by geitanksha


    /* stall condition */

    if (instr_mem_resp) IFE_DEC_controlword <= control_rom_o;

    if (forward_load_case || mem_stall) begin
      pc_out <= pc_out;
      IFE_DEC_controlword <= IFE_DEC_controlword;
    end


    /* Reset conditions */
    if (rst) begin
      pc_out <= 32'h80000000;  // Initial PC value.
      instr_read <= 1'b0;
    end


    if (rst || EXE_MEM_controlword.br_en) IFE_DEC_controlword <= {$bits(rv32i_control_word) {1'b0}};
    //Initial control word is done by control rom.
  end



  /*********************************/
  /**********............***********/
  /*****.......................*****/
  /*........Decode / Load ........ */
  /*****.......................*****/
  /**********............***********/
  /*********************************/

  regfile regfile (
      .clk  (clk),
      .rst  (rst),
      .load (MEM_WRB_controlword.load_regfile && MEM_WRB_controlword.commit),
      .in   (regfilemux_out),
      .src_a(IFE_DEC_controlword.rs1),
      .src_b(IFE_DEC_controlword.rs2),
      .dest (MEM_WRB_controlword.rd),
      .reg_a(regfile_rs1_data),
      .reg_b(regfile_rs2_data)
  );


  always_ff @(posedge clk) begin

    /* Things lower in this list have priority over things higher, so some parts of instruction controlword will be overwritten.*/
    DEC_EXE_controlword <= IFE_DEC_controlword;
    /*Overwrites to DEC_EXE_controlword go here ⬇️*/
    // overwrites from decode / load

    DEC_EXE_controlword.rs1_data <= regfile_rs1_data;
    DEC_EXE_controlword.rs2_data <= regfile_rs2_data;
    //Load forwarding case, stall dec_exe controlword.

    if (forward_load_case || mem_stall) begin
      DEC_EXE_controlword <= DEC_EXE_controlword;
    end

     if(forward_load_case && (forward_case[3] || forward_case[2])) begin
	if(forward_case[3] && !forward_case[2])
	  DEC_EXE_controlword.rs2_data <= for_wrb_data;
	else
	  DEC_EXE_controlword.rs1_data <= for_wrb_data;
     end

    if (rst || EXE_MEM_controlword.br_en) begin
      DEC_EXE_controlword <= {$bits(rv32i_control_word) {1'b0}};
    end

  end




  /*********************************/
  /**********............***********/
  /*****.......................*****/
  /*............Execute............*/
  /*****.......................*****/
  /**********............***********/
  /*********************************/
  alu alu (
      .aluop(DEC_EXE_controlword.aluop),
      .a(alumux1_out),
      .b(alumux2_out),
      .f(alu_data_out)
  );
  always_comb begin

    /* Muxes */

    /*Forward RS1 Mux*/
    case ({
      forward_case[2], forward_case[0]
    })
      2'b01:   exe_rs1_data = for_mem_data;
      2'b10:   exe_rs1_data = for_wrb_data;
      2'b11:   exe_rs1_data = for_mem_data;
      default: exe_rs1_data = DEC_EXE_controlword.rs1_data;
    endcase


    /*Forward RS2 Mux*/
    case ({
      forward_case[3], forward_case[1]
    })
      2'b01:   exe_rs2_data = for_mem_data;
      2'b10:   exe_rs2_data = for_wrb_data;
      2'b11:   exe_rs2_data = for_mem_data;
      default: exe_rs2_data = DEC_EXE_controlword.rs2_data;
    endcase

    /* ALU MUX 1 */
    unique case (DEC_EXE_controlword.alumux1_sel)
      alumux::rs1_out: alumux1_out = exe_rs1_data;
      alumux::pc_out:  alumux1_out = DEC_EXE_controlword.PC_val;
      default:         alumux1_out = exe_rs1_data;
    endcase
    /* ALU MUX 2 */
    unique case (DEC_EXE_controlword.alumux2_sel)
      alumux::rs2_out: alumux2_out = exe_rs2_data;
      alumux::i_imm:   alumux2_out = DEC_EXE_controlword.i_imm;
      alumux::u_imm:   alumux2_out = DEC_EXE_controlword.u_imm;
      alumux::b_imm:   alumux2_out = DEC_EXE_controlword.b_imm;
      alumux::s_imm:   alumux2_out = DEC_EXE_controlword.s_imm;
      alumux::j_imm:   alumux2_out = DEC_EXE_controlword.j_imm;
      default:         alumux2_out = DEC_EXE_controlword.i_imm;
    endcase
    /* CMP MUX */
    unique case (DEC_EXE_controlword.cmpmux_sel)
      cmpmux::rs2_out: cmp_mux_out = exe_rs2_data;
      cmpmux::i_imm:   cmp_mux_out = DEC_EXE_controlword.i_imm;
      default:         cmp_mux_out = exe_rs2_data;
    endcase

    //cmp_op_mux
    case (DEC_EXE_controlword.cmpop)
      beq:     cmp_op_mux_out = exe_rs1_data == cmp_mux_out;
      bne:     cmp_op_mux_out = exe_rs1_data != cmp_mux_out;
      blt:     cmp_op_mux_out = $signed(exe_rs1_data) < $signed(cmp_mux_out);
      bge:     cmp_op_mux_out = $signed(exe_rs1_data) >= $signed(cmp_mux_out);
      bltu:    cmp_op_mux_out = $unsigned(exe_rs1_data) < $unsigned(cmp_mux_out);
      bgeu:    cmp_op_mux_out = $unsigned(exe_rs1_data) >= $unsigned(cmp_mux_out);
      default: cmp_op_mux_out = 1'b0;
    endcase

    br_en = ((DEC_EXE_controlword.branch == 1'b1) && (cmp_op_mux_out)) || (DEC_EXE_controlword.opcode == op_jalr || DEC_EXE_controlword.opcode == op_jal);
  end

  always_ff @(posedge clk) begin
    EXE_MEM_controlword <= DEC_EXE_controlword;
    /*Overwrites to EXE_MEM_controlword go here ⬇️*/
    // control word EXE overwrites
    EXE_MEM_controlword.cmp_out <= cmp_op_mux_out;
    EXE_MEM_controlword.br_en <= br_en;
    EXE_MEM_controlword.alu_out <= alu_data_out;
    EXE_MEM_controlword.rs1_data <= exe_rs1_data;
    EXE_MEM_controlword.rs2_data <= exe_rs2_data;


    if (forward_load_case) begin
      EXE_MEM_controlword <= {$bits(rv32i_control_word) {1'b0}};
    end

    if (mem_stall) begin
      EXE_MEM_controlword <= EXE_MEM_controlword;
    end

    if (rst || (EXE_MEM_controlword.br_en && !mem_stall)) begin
      EXE_MEM_controlword <= {$bits(rv32i_control_word) {1'b0}};  // Initial PC value.
    end
  end


  /*********************************/
  /**********............***********/
  /*****.......................*****/
  /*.............Memory............*/
  /*****.......................*****/
  /**********............***********/
  /*********************************/


  always_comb begin
    /*
    * Consume: alu_out, mem_read, mem_write
    * Produce: data_memory_rdata
    */

    data_mem_address = ((EXE_MEM_controlword.alu_out) & (~3));
    data_read = EXE_MEM_controlword.data_mem_read;
    data_write = EXE_MEM_controlword.data_mem_write;

    unique case(EXE_MEM_controlword.alu_out[1:0])
      0:
	data_mem_wdata = EXE_MEM_controlword.rs2_data;
      1:
	data_mem_wdata = {EXE_MEM_controlword.rs2_data[23:0], 8'b0};
      2:
	data_mem_wdata = {EXE_MEM_controlword.rs2_data[15:0], 16'b0};
      3:
	data_mem_wdata = {EXE_MEM_controlword.rs2_data[7:0], 24'b0};
      default:
	data_mem_wdata = 32'bx;
    endcase // unique case (EXE_MEM_controlword.alu_out)


    if (EXE_MEM_controlword.data_mem_write)
      case (store_funct3_t'(EXE_MEM_controlword.instruction[14:12]))  // funct3
        // case (store_funct3_t'(EXE_MEM_controlword.funct3))
        sb: data_mbe = 4'b0001 << EXE_MEM_controlword.alu_out[1:0];
        sh: data_mbe = 4'b0011 << EXE_MEM_controlword.alu_out[1:0];
        sw: data_mbe = 4'b1111;
        default: data_mbe = 4'b0000;
      endcase
    else
      //   case (load_funct3_t'(EXE_MEM_controlword.funct3))
      case (load_funct3_t'(EXE_MEM_controlword.instruction[14:12]))  // funct3
        lb, lbu: data_mbe = 4'b0001 << EXE_MEM_controlword.alu_out[1:0];
        lh, lhu: data_mbe = 4'b0011 << EXE_MEM_controlword.alu_out[1:0];
        lw: data_mbe = 4'b1111;
        default: data_mbe = 4'b0000;
      endcase
  end





  always_ff @(posedge clk) begin
    MEM_WRB_controlword <= EXE_MEM_controlword;
    /*Overwrites to MEM_controlword go here ⬇️*/
    // TODO: Why isn't this part below part of the control wor
    if (EXE_MEM_controlword.br_en) begin
      if (DEC_EXE_controlword.opcode == op_jalr)
        MEM_WRB_controlword.PC_mux_val <= EXE_MEM_controlword.alu_out & ~1;

      else MEM_WRB_controlword.PC_mux_val <= EXE_MEM_controlword.alu_out;
    end

     if (data_mem_resp) begin
      mem_rdata_out_mem <= data_mem_rdata;
      MEM_WRB_controlword.read_data <= data_mem_rdata;
     end

    // If it's a write, additional logic is not necessary
    if (EXE_MEM_controlword.data_mem_write)
      MEM_WRB_controlword.write_mask <= data_mbe;
    else if (EXE_MEM_controlword.data_mem_read)
      MEM_WRB_controlword.read_mask <= data_mbe;

    if (mem_stall && (EXE_MEM_controlword == {$bits(rv32i_control_word) {1'b0}})) begin
      MEM_WRB_controlword <= {$bits(rv32i_control_word) {1'b0}};
    end

     if (mem_stall)
       MEM_WRB_controlword <= MEM_WRB_controlword;


    if (rst || mem_stall) begin
      MEM_WRB_controlword.commit <= 0;  // Initial PC value.
    end
  end

  /*********************************/
  /**********............***********/
  /*****.......................*****/
  /*...........Writeback...........*/
  /*****.......................*****/
  /**********............***********/
  /*********************************/

  always_comb begin

    bottom_two_bits = MEM_WRB_controlword.alu_out[1:0];

    /* REGFILE MUX */
    unique case (MEM_WRB_controlword.regfilemux_sel)
      regfilemux::alu_out: regfilemux_out = MEM_WRB_controlword.alu_out;
      regfilemux::br_en:
      regfilemux_out = {
        31'b0, MEM_WRB_controlword.cmp_out
      };  // since we need to load 1 in, we zero extend branch enable so that the output of the mux is 32 bits storing val br_en
      regfilemux::u_imm: regfilemux_out = MEM_WRB_controlword.u_imm;
      regfilemux::lw: regfilemux_out = MEM_WRB_controlword.read_data;
      regfilemux::pc_plus4: regfilemux_out = MEM_WRB_controlword.PC_val + 4;
      regfilemux::lb: begin
        case (bottom_two_bits)
          2'b00:
          regfilemux_out = {{24{MEM_WRB_controlword.read_data[7]}}, MEM_WRB_controlword.read_data[7:0]};  //sext 24 bits
          2'b01: regfilemux_out = {{24{MEM_WRB_controlword.read_data[15]}}, MEM_WRB_controlword.read_data[15:8]};
          2'b10: regfilemux_out = {{24{MEM_WRB_controlword.read_data[23]}}, MEM_WRB_controlword.read_data[23:16]};
          2'b11: regfilemux_out = {{24{MEM_WRB_controlword.read_data[31]}}, MEM_WRB_controlword.read_data[31:24]};
          default:
          regfilemux_out = {{24{MEM_WRB_controlword.read_data[7]}}, MEM_WRB_controlword.read_data[7:0]};  //sext 24 bits
        endcase
      end
      regfilemux::lbu: begin
        case (bottom_two_bits)
          2'b00:   regfilemux_out = {24'b0, MEM_WRB_controlword.read_data[7:0]};  //sext 24 bits
          2'b01:   regfilemux_out = {24'b0, MEM_WRB_controlword.read_data[15:8]};
          2'b10:   regfilemux_out = {24'b0, MEM_WRB_controlword.read_data[23:16]};
          2'b11:   regfilemux_out = {24'b0, MEM_WRB_controlword.read_data[31:24]};
          default: regfilemux_out = {24'b0, MEM_WRB_controlword.read_data[7:0]};  //sext 24 bits
        endcase
      end
      regfilemux::lh: begin
        case (bottom_two_bits)
          2'b00:   regfilemux_out = {{16{MEM_WRB_controlword.read_data[15]}}, MEM_WRB_controlword.read_data[15:0]};
          2'b10:   regfilemux_out = {{16{MEM_WRB_controlword.read_data[31]}}, MEM_WRB_controlword.read_data[31:16]};
          default: regfilemux_out = {{16{MEM_WRB_controlword.read_data[15]}}, MEM_WRB_controlword.read_data[15:0]};
        endcase
      end
      regfilemux::lhu: begin
        case (bottom_two_bits)
          2'b00:   regfilemux_out = {16'b0, MEM_WRB_controlword.read_data[15:0]};
          2'b10:   regfilemux_out = {16'b0, MEM_WRB_controlword.read_data[31:16]};
          default: regfilemux_out = {16'b0, MEM_WRB_controlword.read_data[15:0]};
        endcase
      end
      default: regfilemux_out = MEM_WRB_controlword.alu_out;  // Appendix D
    endcase
  end







endmodule : cpu_datapath
