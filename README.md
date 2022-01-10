# my-ARMv8-CPU
an ARMv8-based five pipeline CPU

参考***计算机组成与设计 硬件软件接口  ARM版*** 与教材 ***Computer Organization and Design: The Hardware Software Interface, ARM® Edition*** ，我实现了基于**ARMv8**核心指令集的五级流水CPU，支持四类指令，共8条，指令如下：

- 存储器访问指令，`LDUR`和`STUR`；
- 算术逻辑指令，`ADD`、`SUB`、`AND`、`ORR`；
- 比较为0分支指令，`CBZ`;
- 无条件分支指令，`B`。

这八条指令，配合空指令`NOP`即可基本实现五级流水CPU的运算与执行功能。
