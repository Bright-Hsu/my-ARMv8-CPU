`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2021/12/25 14:40:31
// Design Name: 
// Module Name: cpuSim
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

module cpuSim;
    //CPU信号
    reg RESET;
    reg CLOCK;

    //指令存储器输入输出
    wire [63:0] PC_wire;
    wire [31:0] IC_wire;

    //数据存储器输入输出
    wire [63:0] mem_address;
    wire [63:0] mem_data_in;
    wire control_memwrite;
    wire control_memread;
    wire [63:0] mem_data_out;

    //连接两个存储器存储器和CPU
    ARM_CPU core(RESET, CLOCK, IC_wire, mem_data_out, PC_wire, mem_address, mem_data_in, control_memwrite, control_memread);
    Instr_Memory IM(PC_wire, IC_wire);
    Data_Memory DM(mem_address, mem_data_in, control_memwrite, control_memread, mem_data_out);

    //初始化信号
    initial begin
        CLOCK = 1'b0;
        RESET = 1'b1;
        #40 $finish;
    end
    //切换时钟信号
    always begin
        #1 CLOCK = ~CLOCK; RESET = 1'b0;
    end
endmodule