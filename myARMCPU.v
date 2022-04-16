`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: XJTU
// Engineer: Liang Xu
// 
// Create Date: 2021/12/24 15:29:03
// Design Name: 
// Module Name: myARMCPU
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module ARM_CPU(
  input RESET,
  input CLOCK,
  input [31:0] IC,    //指令的二进制表示
  input [63:0] mem_data_in,
  output reg [63:0] PC,
  output [63:0] mem_address_out,
  output [63:0] mem_data_out,
  output control_memwrite_out,
  output control_memread_out
);
  wire PCSrc_wire;   //指令跳转信号线
  wire [63:0] jump_PC_wire;  //跳转目的地址

  always @(posedge CLOCK) begin
    if (PC === 64'bx) begin  //如果PC未定义，那么就从第0单元开始
    	PC <= 0;
   	end 
    else if (PCSrc_wire == 1'b1) begin  //信号为1则跳转到目的地址
      PC <= jump_PC_wire;
    end 
    else begin
      PC <= PC + 4;  //正常情况下，执行下一条指令，PC加4
  end

  //控制台调试
  $display("Current = %0d | Jump = %0d | Natural Next = %0d", PC, jump_PC_wire, (PC + 4));  
  end

  // Stage : Instruction Fetch 取指令阶段

  wire [63:0] IFID_PC;   //顺序程序寄存器
  wire [31:0] IFID_IR;   //指令寄存器
  IFID cache1 (CLOCK, PC, IC, IFID_PC, IFID_IR);  // FI/ID级间寄存器实例化为cache1


  // Stage : Instruction Decode 指令译码阶段

  // 先根据指令确定控制单元信号
  wire [1:0] CONTROL_aluop; // EX级
  wire CONTROL_alusrc; // EX级
  wire CONTROL_isZeroBranch; // Mem级
  wire CONTROL_isUnconBranch; // Mem级
  wire CONTROL_memRead; // Mem级
  wire CONTROL_memwrite; // Mem级
  wire CONTROL_regwrite; // WB级
  wire CONTROL_memToReg; // WB级
  ARM_Control unit1 (IFID_IR[31:21], CONTROL_aluop, CONTROL_alusrc, CONTROL_isZeroBranch, CONTROL_isUnconBranch, CONTROL_memRead, CONTROL_memwrite, CONTROL_regwrite, CONTROL_memToReg);

  //用多路选择器确定读寄存器2
  wire [4:0] reg2_wire;
  ID_Mux unit2(IFID_IR[20:16], IFID_IR[4:0], IFID_IR[28], reg2_wire);

  //实例化寄存器组
  wire [63:0] reg1_data, reg2_data;
  wire MEMWB_regwrite;  //寄存器写入信号
  wire [4:0] MEMWB_write_reg;  //写入寄存器号
  wire [63:0] write_reg_data;
  RegFile unit3(CLOCK, IFID_IR[9:5], reg2_wire, MEMWB_write_reg, write_reg_data, MEMWB_regwrite, reg1_data, reg2_data);

  //进行立即数扩展
  wire [63:0] sign_extend_wire;
  SignExtend unit4 (IFID_IR, sign_extend_wire);

  //级间保存并向后传递的控制信号
  wire [1:0] IDEX_aluop;
  wire IDEX_alusrc;
  wire IDEX_isZeroBranch;
  wire IDEX_isUnconBranch;
  wire IDEX_memRead;
  wire IDEX_memwrite;
  wire IDEX_regwrite;
  wire IDEX_memToReg;

  // ID/EX级间寄存器实例化为cache2
  wire [63:0] IDEX_reg1_data;
  wire [63:0] IDEX_reg2_data;
  wire [63:0] IDEX_PC;
  wire [63:0] IDEX_sign_extend;
  wire [10:0] IDEX_alu_control;
  wire [4:0] IDEX_write_reg;  // 不一定用到，但必须保存，写入的寄存器号
  IDEX cache2 (CLOCK, CONTROL_aluop, CONTROL_alusrc, CONTROL_isZeroBranch, CONTROL_isUnconBranch, CONTROL_memRead, CONTROL_memwrite, CONTROL_regwrite, CONTROL_memToReg, IFID_PC, reg1_data, reg2_data, sign_extend_wire, IFID_IR[31:21], IFID_IR[4:0], IDEX_aluop, IDEX_alusrc, IDEX_isZeroBranch, IDEX_isUnconBranch, IDEX_memRead, IDEX_memwrite, IDEX_regwrite, IDEX_memToReg, IDEX_PC, IDEX_reg1_data, IDEX_reg2_data, IDEX_sign_extend, IDEX_alu_control, IDEX_write_reg);

  // Stage : Execute  执行指令阶段

  wire [63:0] shift_left_wire;  //偏移量
  wire [63:0] PC_jump;  //基址
  wire jump_is_zero;   //此变量多余，但因为使用ALU单元，必须有该变量
  //算数左移两位
  Shift_Left unit5 (IDEX_sign_extend, shift_left_wire);
  //只使用ALU的加法功能，因此操作码为0010
  ALU unit6 (IDEX_PC, shift_left_wire, 4'b0010, PC_jump, jump_is_zero);

  wire [3:0] alu_main_control_wire;
  wire [63:0] alu_data2_wire;
  wire alu_main_is_zero;
  wire [63:0] alu_main_result;
  // ALU控制单元
  ALU_Control unit7(IDEX_aluop, IDEX_alu_control, alu_main_control_wire);
  // 选择ALU单元的B操作数
  ALU_Mux mux3(IDEX_reg2_data, IDEX_sign_extend, IDEX_alusrc, alu_data2_wire);
  // 实例化ALU单元进行运算
  ALU main_alu(IDEX_reg1_data, alu_data2_wire, alu_main_control_wire, alu_main_result, alu_main_is_zero);

  // EX/Mem级间寄存器实例化为cache3
  wire EXMEM_isZeroBranch;
  wire EXMEM_isUnconBranch;
  wire EXMEM_regwrite;
  wire EXMEM_memToReg;
  wire EXMEM_alu_zero;  //比较为0信号
  wire [4:0] EXMEM_write_reg;
  //control_memread_out, control_memwrite_out两个信号是CPU的输出，输出到数据存储器
  EXMEM cache3(CLOCK, IDEX_isZeroBranch, IDEX_isUnconBranch, IDEX_memRead, IDEX_memwrite, IDEX_regwrite, IDEX_memToReg, PC_jump, alu_main_is_zero, alu_main_result, IDEX_reg2_data, IDEX_write_reg, EXMEM_isZeroBranch, EXMEM_isUnconBranch, control_memread_out, control_memwrite_out, EXMEM_regwrite, EXMEM_memToReg, jump_PC_wire, EXMEM_alu_zero, mem_address_out, mem_data_out, EXMEM_write_reg);


  // Stage : Memory Access  存储器访问阶段  

  Branch unit8 (EXMEM_isUnconBranch, EXMEM_isZeroBranch, EXMEM_alu_zero, PCSrc_wire); //确定PC值

  // MemWB级间寄存器实例化为cache4
  wire MEMWB_memToReg;
  wire [63:0] MEMWB_address;
  wire [63:0] MEMWB_read_data;
  MEMWB cache4(CLOCK, mem_address_out, mem_data_in, EXMEM_write_reg, EXMEM_regwrite, EXMEM_memToReg, MEMWB_address, MEMWB_read_data, MEMWB_write_reg, MEMWB_regwrite, MEMWB_memToReg);


  // Stage : Writeback  数据写回阶段

  //选择写回的数据是ALU的结果还是存储器的数据
  WB_Mux unit9(MEMWB_address, MEMWB_read_data, MEMWB_memToReg, write_reg_data);
endmodule


module IFID    //  FI/ID级间寄存器
(
  input CLOCK,
  input [63:0] PC_in,   
  input [31:0] IC_in,
  output reg [63:0] PC_out,
  output reg [31:0] IC_out
);

  always @(negedge CLOCK) begin   //缓存PC和IC
    PC_out <= PC_in;
    IC_out <= IC_in;
  end
endmodule


module IDEX   // ID/EX级间寄存器
(
  input CLOCK,
  input [1:0] aluop_in, 	       // EX级
  input alusrc_in, 			         // EX级
  input isZeroBranch_in, 	       // Mem级
  input isUnconBranch_in, 	     // Mem级
  input memRead_in, 		         // Mem级
  input memwrite_in, 		         // Mem级
  input regwrite_in, 		         // WB级
  input memToReg_in, 		         // WB级
  input [63:0] PC_in,
  input [63:0] regdata1_in,
  input [63:0] regdata2_in,
  input [63:0] sign_extend_in,
  input [10:0] alu_control_in,
  input [4:0] write_reg_in,
  output reg [1:0] aluop_out, 	// EX级
  output reg alusrc_out, 		    // EX级
  output reg isZeroBranch_out, 	// Mem级
  output reg isUnconBranch_out, // Mem级
  output reg memRead_out, 		  // Mem级
  output reg memwrite_out, 		  // Mem级
  output reg regwrite_out,		  // WB级
  output reg memToReg_out,		    // WB级
  output reg [63:0] PC_out,
  output reg [63:0] regdata1_out,
  output reg [63:0] regdata2_out,
  output reg [63:0] sign_extend_out,
  output reg [10:0] alu_control_out,
  output reg [4:0] write_reg_out
);

  always @(negedge CLOCK) begin
    // EX级使用 
    aluop_out <= aluop_in;
    alusrc_out <= alusrc_in;
    // Mem级使用
  	isZeroBranch_out <= isZeroBranch_in;
    isUnconBranch_out <= isUnconBranch_in;
  	memRead_out <= memRead_in;
 	memwrite_out <= memwrite_in;
    //WB级使用
    regwrite_out <= regwrite_in;
  	memToReg_out <= memToReg_in;

    // 数据寄存器传递需要保存的数据
    PC_out <= PC_in;
    regdata1_out <= regdata1_in;
    regdata2_out <= regdata2_in;
    sign_extend_out <= sign_extend_in;
  	alu_control_out <= alu_control_in;
  	write_reg_out <= write_reg_in;
  end
endmodule


module EXMEM   // EX/Mem级间寄存器
(
  input CLOCK,
  input isZeroBranch_in, 	// Mem级
  input isUnconBranch_in, 	// Mem级
  input memRead_in, 		// Mem级
  input memwrite_in, 		// Mem级
  input regwrite_in, 		// WB级
  input memToReg_in, 		// WB级
  input [63:0] shifted_PC_in,   //已经左移2位并加上基址的PC值
  input alu_zero_in,  // CBZ指令的比较0判断
  input [63:0] alu_result_in,
  input [63:0] write_data_mem_in,
  input [4:0] write_reg_in,
  output reg isZeroBranch_out, 	// Mem级
  output reg isUnconBranch_out, // Mem级
  output reg memRead_out, 		// Mem级
  output reg memwrite_out, 		// Mem级
  output reg regwrite_out,		// WB级
  output reg memToReg_out,		// WB级
  output reg [63:0] shifted_PC_out,
  output reg alu_zero_out,
  output reg [63:0] alu_result_out,
  output reg [63:0] write_data_mem_out,
  output reg [4:0] write_reg_out
);

  always @(negedge CLOCK) begin
    // Mem级使用
  	isZeroBranch_out <= isZeroBranch_in;
    isUnconBranch_out <= isUnconBranch_in;
  	memRead_out <= memRead_in;
 	memwrite_out <= memwrite_in;
    // WB级使用
    regwrite_out <= regwrite_in;
  	memToReg_out <= memToReg_in;

    // 数据寄存器传递需要保存的数据
    shifted_PC_out <= shifted_PC_in;
    alu_zero_out <= alu_zero_in;
    alu_result_out <= alu_result_in;
    write_data_mem_out <= write_data_mem_in;
    write_reg_out <= write_reg_in;
  end
endmodule


module MEMWB   // Mem/WB级间寄存器
(
  input CLOCK,
  input [63:0] mem_address_in,
  input [63:0] mem_data_in,
  input [4:0] write_reg_in,
  input regwrite_in,   //WB级
  input memToReg_in,   //WB级
  output reg [63:0] mem_address_out,
  output reg [63:0] mem_data_out,
  output reg [4:0] write_reg_out,
  output reg regwrite_out,
  output reg memToReg_out
);

  always @(negedge CLOCK) begin
    // WB级使用
    regwrite_out <= regwrite_in;
    memToReg_out <= memToReg_in;
    mem_address_out <= mem_address_in;
    mem_data_out <= mem_data_in;
    write_reg_out <= write_reg_in;
  end
endmodule


module RegFile   //寄存器组
(
  input CLOCK,
  input [4:0] read1,
  input [4:0] read2,
  input [4:0] writeReg,
  input [63:0] writeData,
  input CONTROL_REGWRITE,   //寄存器写入信号
  output reg [63:0] data1,
  output reg [63:0] data2
);

  reg [63:0] Data[31:0];

  integer initCount;

  initial begin    //对寄存器组的内容初始化
    for (initCount = 0; initCount < 31; initCount = initCount + 1) begin
      Data[initCount] = initCount;
    end
    Data[31] = 64'h00000000;
  end

  always @(posedge CLOCK) begin

    data1 = Data[read1];  //读两个寄存器内容
    data2 = Data[read2];

    if (CONTROL_REGWRITE == 1'b1) begin
      Data[writeReg] = writeData;  //向寄存器写入数据
    end

    // 对寄存器组进行控制台调试
    for (initCount = 0; initCount < 32; initCount = initCount + 1) begin
      $display("REGISTER[%0d] = %0d", initCount, Data[initCount]);
    end
  end
endmodule

module ALU   //算术运算单元
(
  input [63:0] A,
  input [63:0] B,
  input [3:0] CONTROL,  //4位的ALU控制输入
  output reg [63:0] RESULT,
  output reg ZEROFLAG   //比较是否为0
);

  always @(*) begin
    case (CONTROL)
      4'b0000 : RESULT = A & B;
      4'b0001 : RESULT = A | B;
      4'b0010 : RESULT = A + B;
      4'b0110 : RESULT = A - B;
      4'b0111 : RESULT = B;  //传递输入B
      4'b1100 : RESULT = ~(A | B);
      default : RESULT = 64'hxxxxxxxx;
    endcase

    if (RESULT == 0) begin
      ZEROFLAG = 1'b1;
    end 
    else if (RESULT != 0) begin
      ZEROFLAG = 1'b0;
    end 
    else begin
      ZEROFLAG = 1'bx;
    end
  end
endmodule


module ALU_Control  // ALU控制单元通过ALUOp和opCode确定运算操作
(
  input [1:0] ALU_Op,
  input [10:0] ALU_INSTRUCTION,
  output reg [3:0] ALU_Out
);

  always @(ALU_Op or ALU_INSTRUCTION) begin
    case (ALU_Op)
      2'b00 : ALU_Out = 4'b0010;  //存数、取数指令，执行ADD
      2'b01 : ALU_Out = 4'b0111;  //CBZ指令，直接传递输入B
      2'b10 : begin

        case (ALU_INSTRUCTION)
          11'b10001011000 : ALU_Out = 4'b0010; // ADD
          11'b11001011000 : ALU_Out = 4'b0110; // SUB
          11'b10001010000 : ALU_Out = 4'b0000; // AND
          11'b10101010000 : ALU_Out = 4'b0001; // ORR
        endcase
      end
      default : ALU_Out = 4'bxxxx;
    endcase
  end
endmodule


module ALU_Mux   //选择ALU的操作数B的来源
(
  input [63:0] input1,
  input [63:0] input2,
  input CONTROL_ALUSRC,  //指明ALU的第二个操作数来源是寄存器组还是扩展地址
  output reg [63:0] out
);

  always @(input1, input2, CONTROL_ALUSRC, out) begin
    if (CONTROL_ALUSRC == 0) begin
      out <= input1;  //第二个操作数来自寄存器组
    end

    else begin
      out <= input2;  //第二个操作数来自符号扩展数
    end
  end
endmodule


module ID_Mux   //多路选择读寄存器2的来源是Rm[20:16]还是Rt[4:0]
(
  input [4:0] read1_in,  //IFID_IR[20:16],Rm字段
  input [4:0] read2_in,  //IFID_IR[4:0],Rt字段
  input reg2loc_in,   //通过IR[28]就可以分辨指令是不是R型指令
  output reg [4:0] reg_out  //输出
);

  always @(read1_in, read2_in, reg2loc_in) begin
    case (reg2loc_in)
        1'b0 : begin   //IR[28]为0，R指令
            reg_out <= read1_in;
        end
        1'b1 : begin  //IR[28]为1，不是R指令
            reg_out <= read2_in;
        end
        default : begin
            reg_out <= 1'bx;
        end
    endcase
  end
endmodule


module WB_Mux  //数据写回多路选择器
(
  input [63:0] input1,
  input [63:0] input2,
  input memToReg_control,
  output reg [63:0] out
);

  always @(*) begin
    if (memToReg_control == 0) begin
      out <= input1;  //写回ALU的运算结果
    end

    else begin
      out <= input2;  //写回存储器中的数据
    end
  end
endmodule


module Shift_Left   // 偏移地址左移两位
(
  input [63:0] data_in,
  output reg [63:0] data_out
);

  always @(data_in) begin
    data_out = data_in << 2;
  end
endmodule


module SignExtend   //立即数扩展模块
(
  input [31:0] inputInstruction,   //输入32位指令
  output reg [63:0] outImmediate   //输出64位立即数
);

  always @(inputInstruction) begin
    if (inputInstruction[31:26] == 6'b000101) begin // B指令，取26位数
      outImmediate[25:0] = inputInstruction[25:0];
      outImmediate[63:26] = {64{outImmediate[25]}};

    end else if (inputInstruction[31:24] == 8'b10110100) begin // CBZ指令，取19位数
      outImmediate[19:0] = inputInstruction[23:5];
      outImmediate[63:20] = {64{outImmediate[19]}};

    end else begin // 存数、取数指令，取9位数
      outImmediate[9:0] = inputInstruction[20:12];
      outImmediate[63:10] = {64{outImmediate[9]}};
    end
  end
endmodule


module Branch  //分支模块产生PC选择信号
(
  input unconditional_branch_in,
  input conditional_branch_in,
  input alu_main_is_zero,
  output reg PC_src_out
);

  reg conditional_branch_temp;

  always @(unconditional_branch_in, conditional_branch_in, alu_main_is_zero) begin
    conditional_branch_temp = conditional_branch_in & alu_main_is_zero;
    PC_src_out = unconditional_branch_in | conditional_branch_temp; // 无条件跳转或者比较为0跳转
  end
endmodule


module ARM_Control   //控制单元
(
  input [10:0] instruction,    //输入11位指令
  output reg [1:0] control_aluop,  //2位的ALUop
  output reg control_alusrc,  //指明ALU的第二个操作数来源是寄存器组还是扩展地址
  output reg control_isZeroBranch,  //条件分支信号
  output reg control_isUnconBranch,  //无条件分支信号
  output reg control_memRead,  //存储器读信号
  output reg control_memwrite,  //存储器写信号
  output reg control_regwrite,  //寄存器组写信号
  output reg control_memToReg  //写入寄存器的值来源是ALU还是数据存储器
);

  always @(instruction) begin
    /* B，无条件跳转指令 */
    if (instruction[10:5] == 6'b000101) begin
      control_memToReg <= 1'bx;
      control_memRead <= 1'b0;
      control_memwrite <= 1'b0;
      control_alusrc <= 1'b0;
      control_aluop <= 2'b01;   //ALUop为01代表跳转指令
      control_isZeroBranch <= 1'b0;
      control_isUnconBranch <= 1'b1;  //无条件跳转
      control_regwrite <= 1'b0;
    end

    /* CBZ，条件为零分支跳转指令 */
    else if (instruction[10:3] == 8'b10110100) begin
      control_memToReg <= 1'bx;
      control_memRead <= 1'b0;
      control_memwrite <= 1'b0;
      control_alusrc <= 1'b0;
      control_aluop <= 2'b01;  //ALUop为01代表跳转指令
      control_isZeroBranch <= 1'b1;  //比较为0跳转
      control_isUnconBranch <= 1'b0;
      control_regwrite <= 1'b0;
    end

    /* 其他指令 */
    else begin
      control_isZeroBranch <= 1'b0;
      control_isUnconBranch <= 1'b0;

      case (instruction[10:0])

        /* LDUR，取数指令 */
        11'b11111000010 : begin
          control_memToReg <= 1'b1;  //从存储器取数到寄存器
          control_memRead <= 1'b1;   //读存储器
          control_memwrite <= 1'b0;
          control_alusrc <= 1'b1;  //第二个操作数是符号扩展数
          control_aluop <= 2'b00;  //代表存数取数指令
          control_regwrite <= 1'b1;
        end

        /* STUR，存数指令 */
        11'b11111000000 : begin
          control_memToReg <= 1'bx;
          control_memRead <= 1'b0;
          control_memwrite <= 1'b1;  //写存储器
          control_alusrc <= 1'b1;  //第二个操作数是符号扩展数
          control_aluop <= 2'b00;  //代表存数取数指令
          control_regwrite <= 1'b0;
        end

        /* R类指令的控制信号都相同，只是ALU控制输入不同,所以下面完全一样 */
        /* ADD */
        11'b10001011000 : begin
          control_memToReg <= 1'b0;
          control_memRead <= 1'b0;
          control_memwrite <= 1'b0;
          control_alusrc <= 1'b0;
          control_aluop <= 2'b10;  //R类指令
          control_regwrite <= 1'b1;  //寄存器写
        end

        /* SUB */
        11'b11001011000 : begin
          control_memToReg <= 1'b0;
          control_memRead <= 1'b0;
          control_memwrite <= 1'b0;
          control_alusrc <= 1'b0;
          control_aluop <= 2'b10;
          control_regwrite <= 1'b1;
        end

        /* AND */
        11'b10001010000 : begin
          control_memToReg <= 1'b0;
          control_memRead <= 1'b0;
          control_memwrite <= 1'b0;
          control_alusrc <= 1'b0;
          control_aluop <= 2'b10;
          control_regwrite <= 1'b1;
        end

        /* ORR */
        11'b10101010000 : begin
          control_memToReg <= 1'b0;
          control_memRead <= 1'b0;
          control_memwrite <= 1'b0;
          control_alusrc <= 1'b0;
          control_aluop <= 2'b10;
          control_regwrite <= 1'b1;
        end

        /* 默认情况下，所有信号都未知，置为X */
        default : begin
          control_isZeroBranch <= 1'bx;
      	  control_isUnconBranch <= 1'bx;
          control_memToReg <= 1'bx;
          control_memRead <= 1'bx;
          control_memwrite <= 1'bx;
          control_alusrc <= 1'bx;
          control_aluop <= 2'bxx;
          control_regwrite <= 1'bx;
        end
      endcase
    end
  end
endmodule

module Instr_Memory  //指令存储器
(
  input [63:0] PC_in,   
  output reg [31:0] instruction_out
);

  reg [8:0] Data[71:0];

  initial begin
  	// LDUR x2, [x12]
    Data[0] = 8'hf8;
    Data[1] = 8'h40;
    Data[2] = 8'h01;
    Data[3] = 8'h82;
    // LDUR x3, [x13]
    Data[4] = 8'hf8;
    Data[5] = 8'h40;
    Data[6] = 8'h01;
    Data[7] = 8'ha3;
    // AND x6, x1, x30
    Data[8] = 8'h8a;
    Data[9] = 8'h1e;
    Data[10] = 8'h00;
    Data[11] = 8'h26;
    // ORR x5, x20, x1
    Data[12] = 8'haa;
    Data[13] = 8'h01;
    Data[14] = 8'h02;
    Data[15] = 8'h85;
    // NOP
    Data[16] = 8'h00;
    Data[17] = 8'h00;
    Data[18] = 8'h00;
    Data[19] = 8'h00;
    // ADD x9, x3, x2
    Data[20] = 8'h8b;
    Data[21] = 8'h02;
    Data[22] = 8'h00;
    Data[23] = 8'h69;
    // SUB x10, x3, x2
    Data[24] = 8'hcb;
    Data[25] = 8'h02;
    Data[26] = 8'h00;
    Data[27] = 8'h6a;
    // CBZ x6, #6
    Data[28] = 8'hb4;
    Data[29] = 8'h00;
    Data[30] = 8'h00;
    Data[31] = 8'hc6;
    // NOP
    Data[32] = 8'h00;
    Data[33] = 8'h00;
    Data[34] = 8'h00;
    Data[35] = 8'h00;
    // NOP
    Data[36] = 8'h00;
    Data[37] = 8'h00;
    Data[38] = 8'h00;
    Data[39] = 8'h00;
    // NOP
    Data[40] = 8'h00;
    Data[41] = 8'h00;
    Data[42] = 8'h00;
    Data[43] = 8'h00;
    // STUR x5, [x7, #1]
    Data[52] = 8'hf8;
    Data[53] = 8'h00;
    Data[54] = 8'h10;
    Data[55] = 8'he5;
    // STUR x9, [x8, #2]
    Data[56] = 8'hf8;
    Data[57] = 8'h00;
    Data[58] = 8'h21;
    Data[59] = 8'h09;
    // B #-8
    Data[60] = 8'h17;
    Data[61] = 8'hff;
    Data[62] = 8'hff;
    Data[63] = 8'hf8;
    // NOP
    Data[64] = 8'h00;
    Data[65] = 8'h00;
    Data[66] = 8'h00;
    Data[67] = 8'h00;
    // NOP
    Data[68] = 8'h00;
    Data[69] = 8'h00;
    Data[70] = 8'h00;
    Data[71] = 8'h00;
  end

  always @(PC_in) begin
    instruction_out[8:0] = Data[PC_in + 3];
    instruction_out[16:8] = Data[PC_in + 2];
    instruction_out[24:16] = Data[PC_in + 1];
    instruction_out[31:24] = Data[PC_in];
  end
endmodule


module Data_Memory   //数据存储器
(
    input [63:0] inputAddress,
    input [63:0] inputData,
    input CONTROL_MemWrite,  //存储器写信号
    input CONTROL_MemRead,   //存储器读信号
    output reg [63:0] outputData
);

    reg [63:0] Data[31:0];

    integer initCount;

    initial begin
        for (initCount = 0; initCount < 32; initCount = initCount + 1) begin
            Data[initCount] = initCount * 100;  //初始化存储器数据单元
        end
    end

    always @(*) begin
      if (CONTROL_MemWrite == 1'b1) begin
        Data[inputAddress] = inputData;   //写入数据
      end else if (CONTROL_MemRead == 1'b1) begin
        outputData = Data[inputAddress];  //读出数据
      end else begin
        outputData = 64'hxxxxxxxx;
      end

      // 对数据存储器进行控制台调试
      for (initCount = 0; initCount < 32; initCount = initCount + 1) begin
        $display("RAM[%0d] = %0d", initCount, Data[initCount]);
      end
    end
endmodule
