# my-ARMv8-CPU
an ARMv8-based five pipeline CPU

## 一. 实验目的

1. 综合运用Verilog进行复杂系统设计。

2. 深刻理解计算机系统硬件原理。

## 二. 实验内容

1. 设计一个基于ARM指令集的CPU。

2. CPU需要包含寄存器组、RAM模块、ALU模块、指令译码模块。

3. 该CPU能运行基本的汇编指令，包括`LDUR`,`STUR`,`ADD`,`SUB`,`ORR`,`AND`,`CBZ`,`B`, `NOP`。

4. 实现多核，SIMD或其他现代CPU的高级功能，参考教材*Computer Organization and Design: The Hardware Software Interface, ARM® Edition第6章*（可选，加分项）。

## 三. 设计原理详解

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
