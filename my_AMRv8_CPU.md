# 实验六. 五级流水CPU(ARMv8)

> ***Author：bright***

-----

## 一. 实验目的

1. 综合运用Verilog进行复杂系统设计。

2. 深刻理解计算机系统硬件原理。

## 二. 实验内容

1. 设计一个基于ARM指令集的CPU。

2. CPU需要包含寄存器组、RAM模块、ALU模块、指令译码模块。

3. 该CPU能运行基本的汇编指令，包括`LDUR`,`STUR`,`ADD`,`SUB`,`ORR`,`AND`,`CBZ`,`B`, `NOP`。

4. 实现多核，SIMD或其他现代CPU的高级功能，参考教材*Computer Organization and Design: The Hardware Software Interface, ARM® Edition第6章*（可选，加分项）。

## 三. 实验要求

1. 分析各模块的的程序结构，画出其流程图。

2. 画出模块的电路图。

3. 分析电路的仿真波形，标出关键的数值。

4. 记录设计和调试过程。

## 四. 实验设计思路

### myARMCPU功能概述

参考***计算机组成与设计 硬件软件接口  ARM版*** 与教材 ***Computer Organization and Design: The Hardware Software Interface, ARM® Edition*** ，我实现了基于**ARMv8**核心指令集的五级流水CPU，支持四类指令，共8条，指令如下：

- 存储器访问指令，`LDUR`和`STUR`；
- 算术逻辑指令，`ADD`、`SUB`、`AND`、`ORR`；
- 比较为0分支指令，`CBZ`;
- 无条件分支指令，`B`。

这八条指令，配合空指令`NOP`即可基本实现五级流水CPU的运算与执行功能。

### 基本部件

**myARMCPU** 实现中的数据通路包括两种不同类型的逻辑单元：处理数据值得单元和存储状态的单元。处理数据值的单元都是组合逻辑，输出只取决于当前的输入。

但是存储状态的单元不是组合逻辑，它们包含状态 。如果一个单元带有内部存储， 那么该单元包含状态，这些单元称为状态单元 (state element) 。包含状态的逻辑部件又被称为时序逻辑，因为它们的输出由输入和内部状态共同决定。

建立数据通路，首先要分析每条指令执行所需要的部件。因此各部件是实现CPU的基础，设计模块时也需要依次为单元来设计，我认为主要可以分为以下几类：

- 数据存储单元。包括**指令存储器Instruction Memory、寄存器组Reg File、数据存储器Data Memory**；
- 运算单元。包括**主ALU、ADDer、Shift_Left、SignExtend、PC**；
- 多路选择器。包括**ALU_MUX、ID_MUX、WB_MUX、Branch**；
- 控制单元。包括**ARM_Control、ALU_Control**；
- 级间寄存器。四个，包括**IFID、IDEX、EXMEM、MEMWB**。

其中比较重要的部件，我将会在下面一一具体阐述并实现。

#### 指令存储器

虽然存储器不是CPU的部件，但他与CPU联系十分密切。任何指令的执行，都需要先从指令存储器中取出指令。指令存储器类似于制度存储器，可以把它当做一个组合逻辑单元，任意时刻的输出都反应了输入地址所指单元的内容。指令存储器示意图如下：

![IM](https://github.com/Bright-Hsu/my-ARMv8-CPU/blob/main/%E5%8E%9F%E7%90%86%E5%9B%BE/%E6%8C%87%E4%BB%A4%E5%AD%98%E5%82%A8%E5%99%A8.png)

可以看到其逻辑教简单，因此模块代码如下：

```verilog
module Instr_Memory  //指令存储器
(
  input [63:0] PC_in,   
  output reg [31:0] instruction_out
);
  reg [8:0] Data[71:0];

  initial begin
    //下面采用直接对单元赋值的方式初始化指令存储器
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
	// 或者用另一种方式，使用readmemh函数用.dat文件初始化
    // $readmemh("c:\\instructions.dat",Data);
  end

  always @(PC_in) begin
    instruction_out[8:0] = Data[PC_in + 3];
    instruction_out[16:8] = Data[PC_in + 2];
    instruction_out[24:16] = Data[PC_in + 1];
    instruction_out[31:24] = Data[PC_in];
  end
endmodule
```

#### 寄存器组

寄存器组是包含一系列寄存器的状态单元，可以通过提供的寄存器号进行读写访问。寄存器组也就是寄存器的集合，通过指定相应的寄存器号来读写具体的寄存器，寄存器组可以包含计算机的寄存器状态。

由与R型指令有3个寄存器操作数，因此每条指令都要从寄存器组中读出两个数据字，再写入1个数据字。要从寄存器中读出一个数据字，需要给寄存 器文件一个输入，以指明所要读的寄存器号，并且寄存器文件将产生一个输出，包含从寄存器组读出的值。写入一个数据字时，需要给寄存器组提供两个输入：一个指明要写的寄存器号，另一个提供要写的数据。因此寄存器组需要4个输入（3个寄存器号和1个数据）和2个输出（2个数据）。输入的寄存器号为5位，可以指定32个寄存器的某一个，数据的宽度为64位。寄存器组如下图所示：

![image-20220110131008888](C:\Users\DELL\AppData\Roaming\Typora\typora-user-images\image-20220110131008888.png)

寄存器组的模块代码如下：

```verilog
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
  end
endmodule
```

#### ALU运算单元

ALU有两个64位的数据输入，产生一个64位的运算结果，并且有1位输出信号指出结果是否为0。ALU需要有4位控制信号，指明ALU进行的运算操作，该4位信号由指令得出。ALU示意图如下：

![image-20220110131607168](C:\Users\DELL\AppData\Roaming\Typora\typora-user-images\image-20220110131607168.png)

为了方便使用，我这里的ALU运算单元都调用的是自带的运算符。模块代码如下：

```verilog
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
```

#### 数据存储器

作为计算机，必不可少的就是存储器，它可以存储大量数据。为CPU提供需要运算的数据并存储进去。LDUR和STUR指令就需要访问数据存储器。数据存储器是一个状态单元，两个输入为地址和待写入的 数据，一个输出为读出的结果。读、写控制信号都是独立的，但任意时钟只能激活其中—个。数据存储器的示意图如下：

![image-20220110132419367](C:\Users\DELL\AppData\Roaming\Typora\typora-user-images\image-20220110132419367.png)

数据存储器的模块代码如下：

```verilog
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
        
    end
endmodule
```

#### 多路选择器

本数据通路使用到了三个多路选择器，因为有三个地方有多个输入来源。第一个是ALU的第二个操作数，它可能来自寄存器组的第二个数据输出，也可能是指令中指明的立即数。第二个是寄存器组输入寄存器号2的来源，如果指令是R型，那么寄存器号为IR[20:16]（即Rm），如果指令不是R型，那么寄存器号为IR[4:0]（即Rt）。第三个选择器是数据写回的来源，如果是R型指令，需要写回ALU的运算结果，如果是存储器访问指令，需要写回存储器的读出数据。各个多路选择器模块代码如下：

```verilog
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

module ID_Mux   //选择读寄存器2的来源是Rm[20:16]还是Rt[4:0]
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
```

#### 其他部件

还有一些逻辑比较简单，但是都在某条指令的执行过程中不可或缺。比如将地址左移两位的部件**Shift_left2**，分支跳转指令用到的部件**Branch**，符号扩展单元**SignExtend**，程序计数器**PC**等。

这些单元的设计思路和模块代码就不再赘述，将会附在源代码当中。

### ALU控制单元

ALU的控制信号为4位，有六种有效的输入组合：

| ALU控制信号 | 功能      |
| ----------- | --------- |
| 0000        | 与AND     |
| 0001        | 或OR      |
| 0010        | 加ADD     |
| 0110        | 减SUB     |
| 0111        | 传递输入B |
| 1100        | 或非NOR   |

使用一个小的控制单元即可生成4位的ALU控制输入信号，该控制单元的输入为指令的操作字段OPCode和2位的ALUOP控制字段。

下图即为如何基于2位的控制信号和11位的操作码字段，生成ALU的控制信号。

![image-20220108214528276](C:\Users\DELL\AppData\Roaming\Typora\typora-user-images\image-20220108214528276.png)

可以看出ALU控制单元通过ALU控制单元通过ALUOp和opCode确定运算操作，其模块代码如下：

```verilog
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
```

### CPU主控制单元

myARMCPU需要用到的指令有R型、D型、B型、CB型，指令格式如下（B指令较简单，这里没有画出）：

![image-20220108214958125](C:\Users\DELL\AppData\Roaming\Typora\typora-user-images\image-20220108214958125.png)

指令格式中的信息如下：

- opcode字段,6~11位宽，位于指令中的31:26位到31:21位。
- 对于R型指令和 load/store指令的基址寄存器,第一寄存器操作数（Rn）总是位于9:5位。
- 另一个寄存器操作数有两个出处。R型指令是20:16位(Rm)，load指令的写入寄存器是4:0位(Rt)。该字段也用来指明CBZ 指令所需要测试的寄存器。因此，我们需要增加一个多路选择器，以选择指令中哪个字段指明了所要读的寄存器号。
- 另一个操作数可能是，比较为0分支指令的19位偏移或者是load/store指令的9位偏移。R型指令和 load指令的目的寄存器(分别是Rd和 Rt)由4:0位指出。

主控制单元产生几个控制信号如下：

![image-20220108215333951](C:\Users\DELL\AppData\Roaming\Typora\typora-user-images\image-20220108215333951.png)

通过opcode产生控制信号，输入输出组合如下：

![image-20220108215711605](C:\Users\DELL\AppData\Roaming\Typora\typora-user-images\image-20220108215711605.png)

根据上述原理，就可以通过枚举设计出主控制单元的模块代码，如下：

```verilog
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
```

### 级间寄存器

把指令划分为五个阶段，也就是五级流水线。因此，将数据通路分为五个部分，每一个部分用与之对应的指令执行阶段来命名。

1. **IF：取指令。**
2. **ID：指令译码及读寄存器文件。**
3. **EX：执行运算或者计算地址。**
4. **MEM：访问数据存储器。**
5. **WB：数据写回。**

各阶段分布如下图（英文版教材P298）：

![image-20220108220756238](C:\Users\DELL\AppData\Roaming\Typora\typora-user-images\image-20220108220756238.png)

由于在某一时刻，处于不同阶段执行的指令并不相同，因此为了使多条指令共享流水线数据通路，需要在每个阶段之间设计几件寄存器，保存当前阶段和以后阶段所需要的指令、数据、控制信息。如下图（英文版教材P300）;

![image-20220110140927466](C:\Users\DELL\AppData\Roaming\Typora\typora-user-images\image-20220110140927466.png)

控制信号也需要被级间寄存器保存，可以被分为四组，在指令的各个阶段使用：

![image-20220108221000673](C:\Users\DELL\AppData\Roaming\Typora\typora-user-images\image-20220108221000673.png)

每个信号只在特定阶段使用，详情如下：

1. 取指令：读指令存储器和写PC 的控制信号总是有效，因此这一级没有特别需要控制的部件。
2. 指令译码及读寄存器文件：这一级需要为第二个读寄存器单端口选择正确的寄存器号，因此需要设置Reg2Loc信号。该信号选择采用指令的20:16位(Rm)还是4:0位(Rt)。
3. 执行或计算地址：这一级将设置的控制信号为ALUOp和 ALUSrc。这些信号选择ALU的操作，并选择将寄存器文件读出的数据还是符号扩展的立即数作为ALU的输入。
4. 访问数据存储器：这一级设置的控制信号有Branch、MemRead 和 MemWrite，分别由比较为0分支、load和 store指令设置。直到Branch控制信号有效且ALU结果为0,图4-47中的PCSrc信号才选择下一个顺序的地址。
5. 数据写回：这一级的两个控制信号为MemtoReg和RegWrite，前者决定是将ALU结果还是将存储器数据传输到寄存器组，后者决定是否写入寄存器组。

四个级间寄存器的模块代码如下：

```verilog
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
```

### 流水线数据通路

将各个部件相连，可以画出流水数据通路如下（英文版教材P315）：

![image-20220108221128699](C:\Users\DELL\AppData\Roaming\Typora\typora-user-images\image-20220108221128699.png)

顶层模块为ARM_CPU，将各部件相连接，我这里把PC集成到了顶层模块里，模块代码如下：

```verilog
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
    if (PC === 64'bx) begin  //如果PC为定义，那么就从第0单元开始
    	PC <= 0;
   	end 
    else if (PCSrc_wire == 1'b1) begin  //信号为1则跳转到目的地址
      PC <= jump_PC_wire;
    end 
    else begin
      PC <= PC + 4;  //正常情况下，执行下一条指令，PC加4
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
```

## 五. 实验仿真及结果

### RTL仿真

进行RTL仿真，得到RTL电路图如下，与预期的连接相一致：

![image-20220110143625163](C:\Users\DELL\AppData\Roaming\Typora\typora-user-images\image-20220110143625163.png)

### 测试指令设计

在前面设计寄存器组、数据存储器时，我已经把他们里面的内容初始化了，寄存器组的各个寄存器内容如下：

```verilog
  initial begin    //对寄存器组的内容初始化
    for (initCount = 0; initCount < 31; initCount = initCount + 1) begin
      Data[initCount] = initCount;
    end
    Data[31] = 64'h00000000;
  end
```

数据存储器的数据单元初始化如下：

```verilog
    initial begin  //初始化存储器数据单元
        for (initCount = 0; initCount < 32; initCount = initCount + 1) begin
            Data[initCount] = initCount * 100;  
        end
    end
```

可以知道，最开始时，寄存器的内容等于它的寄存器号，存储单元的内容等于它的地址乘以100 。

测试指令自然应该涵盖CPU所能实现的所有指令，因此我设计了一系列指令来验证CPU能否运算正确以及正常工作。

- 首先，PC执行的两条指令是`LDUR`，两条指令分别把存储单元`#12`和`#13`的内容保存在寄存器x2和x3 ，因此当前寄存器x2和x3的内容分别是1200和1300 。
- 然后使用`AND`指令，进行与运算，可以知道寄存器x6的值变为1和30进行与运算，也就是0（后面将会用到）。
- 然后使用`ORR`指令，进行或运算，可以知道寄存器x5的值编程20和1进行或运算，结果是21。然后执行一条`NOP`指令，不进行操作。
- 接下来进行加减运算，执行`ADD x9, x3, x2`和`SUB x10, x3, x2`，可以知道寄存器x9的内容等于Reg[x3]+Reg[x2]=1300+1200=2500，寄存器x10的内容等于Reg[x3]-Reg[x2]=1300-1200=100 。
-  然后执行 `CBZ x6, #6`，从上面可以知道寄存器x6的内容已经变成了0，因此会发生跳转，将会跳转到PC=当前PC+(6*4)的指令，也就是下一条PC=52 。
- 然后执行`STUR`指令，将寄存器x5和x9的内容存入存储器的单元 Mem[[x7]+1]=Mem[7+1]=Mem[8]，Mem[[x8]+2]=Mem[8+2]=Mem[10]。
- 然后执行`B #-8`指令，无条件跳转到当前PC=当前PC-(8*4)的指令，因此下一条PC=28 。
- 跳转到PC=28以后，便开始循环执行28到60之间的指令。

指令存储器的内容如下：

|  PC   | ARMv8汇编指令       |              二进制机器码               | 十六进制码  |
| :---: | :------------------ | :-------------------------------------: | :---------: |
|   0   | `LDUR x2, [x12]`    | 1111 1000 0100 0000 0000 0001 1000 0010 |  f8400182   |
|   4   | `LDUR x3, [x13]`    | 1111 1000 0100 0000 0000 0001 1010 0011 |  f84001a3   |
|   8   | `AND x6, x1, x30`   | 1000 1010 0001 1110 0000 0000 0010 0110 |  8a1b0026   |
|  12   | `ORR x5, x20, x1`   | 1010 1010 0000 0001 0000 0010 1000 0101 |  aa010285   |
|  16   | `NOP`               | 0000 0000 0000 0000 0000 0000 0000 0000 |  00000000   |
|  20   | `ADD x9, x3, x2`    | 1000 1011 0000 0010 0000 0000 0110 1001 |  8b020069   |
|  24   | `SUB x10, x3, x2`   | 1100 1011 0000 0010 0000 0000 0110 1010 |  cb02006a   |
|  28   | `CBZ x6, #6`        | 1011 0100 0000 0000 0000 0000 1100 0110 |  b40000c6   |
| 32-48 | `NOP`               | 0000 0000 0000 0000 0000 0000 0000 0000 |  00000000   |
|  52   | `STUR x5, [x7, #1]` | 1111 1000 0000 0000 0001 0000 1110 0101 |  f80010e5   |
|  56   | `STUR x9, [x8, #2]` | 1111 1000 0000 0000 0010 0001 0000 1001 |  f8002109   |
|  60   | `B #-8`             | 0001 0111 1111 1111 1111 1111 1111 1000 | 17 ff ff f8 |

### Testbench测试文件

想要CPU执行指令，就必须先把指令存储器、数据存储器和CPU连接起来，然后给CPU一个始终信号，CPU就能从初始的PC开始执行指令。Testbench代码如下：

```verilog
module cpuSim();
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
        #40 $finish;  //40ns之后停止
    end
    //切换时钟信号
    always begin
        #1 CLOCK = ~CLOCK; RESET = 1'b0;
    end
endmodule
```

### 进行仿真测设

进行仿真，时间设置为40ns，波形图如下：

![image-20220110154331762](C:\Users\DELL\AppData\Roaming\Typora\typora-user-images\image-20220110154331762.png)

可以从波形图看到，PC的值从0开始，每次加4，即为执行下一条指令，执行到PC=28后，发生跳转，执行到PC=52，执行完PC=60的指令后，又跳转回了PC=28，发生循环。

然后观察寄存器组中数据的变化情况：

![image-20220110154945226](C:\Users\DELL\AppData\Roaming\Typora\typora-user-images\image-20220110154945226.png)

可以看到寄存器x2的内容从2变为1200，寄存器x3的内容从3变为1300，寄存器x6的内容从6变为0，寄存器x5的内容从5变为21，寄存器x9的内容从9变为2500，寄存器x10的内容从10变为100 ，且变化的时序与指令执行时序相符合。

观察数据存储器中数据的变化情况：

<img src="C:\Users\DELL\AppData\Roaming\Typora\typora-user-images\image-20220110155427888.png" alt="image-20220110155427888"  />

由于指令中只用了两条存数指令，`STUR x5, [x7, #1]`和`STUR x9, [x8, #2]`，因此只有存储单元#8和#10的内容变化，从图中看出，#8的内容从800变为21，#10的内容从1000变为2500 ，与预期相符合。

经过验证，myARMCPU实现的9个指令`LDUR`,`STUR`,`ADD`,`SUB`,`ORR`,`AND`,`CBZ`,`B`, `NOP`都能正确执行，符合预期。

## 六. 调试与心得体会

本次实验，我实现了基于ARMv8指令集的五级流水CPU，是目前为止我接触的最复杂的Verilog项目，主要是因为CPU的组成较为复杂，同时需要考虑到时序逻辑的影响和状态单元的保存。遇到这种复杂的工程项目，就更需以自顶向下的方式来设计，先将CPU分为几个模块，然后在一一实现这些模块的功能，最后再将其全部连接在一起，然后进行时序逻辑的设计，根据时钟信号一步步调试，最终即可构建成一个完整的CPU。

复杂数字逻辑电路和系统的层次化、结构化设计隐含着对系统硬件设计方案的逐次分解。在设计过程中的任意层次,至少得有一种形式来描述硬件。在集成电路设计的每一层次，硬件可以分为一些模块,该层次的硬件结构由这些模块的互联描述，该层次的硬件的行为由这些模块的行为描述。在不同的层次都可以进行仿真以对设计思想进行验证。最后利用Vivado管理错综复杂的层次，即可以很方便地查看某一层次某模块的源代码或电路图以改正仿真时发现的错误。

在设计流水线CPU的过程中，我先将数据通路分解为几种部件，即基本部件、控制单元、级间寄存器。然后基本单元又细化出了个部件，如下分类：

- 数据存储单元。包括**指令存储器Instruction Memory、寄存器组Reg File、数据存储器Data Memory**；
- 运算单元。包括**主ALU、ADDer、Shift_Left、SignExtend、PC**；
- 多路选择器。包括**ALU_MUX、ID_MUX、WB_MUX、Branch**；
- 控制单元。包括**ARM_Control、ALU_Control**；
- 级间寄存器。四个，包括**IFID、IDEX、EXMEM、MEMWB**。

最后对于每个模块，先分析其功能和结构，然后对其进行实现即可。

在连接各部件的过程中，我觉得尤其要注意时序信号对各部件的影响，要注意观察每个数据和信号什么时间打入部件当中，这是时序逻辑正确的关键。

设计完CPU之后，就需要对其进行仿真测试，验证CPU的功能正确性，因此指令的设计就显得尤为重要。所以我又设计了一系列指令，来逐一验证时序逻辑和指令的执行过程。

总而言之，本次实验使我学会了综合运用Verilog进行复杂系统设计。更为重要的是，它与计算机组成原理课程的知识精密结合，使我深刻理解计算机系统硬件原理，并能够对一个基本的基于ARM指令集的五级流水CPU进行实现，收获非常大，相信这对我今后的学习和工作的启发会是持续的帮助。

## 七. 源代码

模块代码`myARMCPU.v`如下：

```verilog
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
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
    if (PC === 64'bx) begin  //如果PC为定义，那么就从第0单元开始
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
```



Testbench代码`cpuSim.v`如下：

```verilog
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
```

