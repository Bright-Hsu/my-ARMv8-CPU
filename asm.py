# 输入ARMv8汇编指令
raw_instruction = input('\n请输入ARMv8指令: ')
# 将汇编指令分割
formatted_instruction = raw_instruction.replace(' ', ',').replace(']', '').replace('[', '')
# 将指令用','分割成list
# 使用filter过滤
instruction_list = list(filter(None, formatted_instruction.split(',')))

# 指令与二进制的OPcode对应
OPCODES = {
    'LDUR' : ['11111000010'],
    'STUR' : ['11111000000'],
    'ADD'  : ['10001011000', '+'],
    'SUB'  : ['11001011000', '-'],
    'ORR'  : ['10101010000', '|'],
    'AND'  : ['10001010000', '&'],
    'CBZ'  : ['10110100'],
    'B'    : ['000101']
}

# 先加入已知的机器码
machine_code = OPCODES[instruction_list[0]][0]

print('指令解释为：')

if (instruction_list[0] == 'LDUR' or instruction_list[0] == 'STUR'): # 取数、存数指令

    dt_address = 0 if len(instruction_list) < 4 else int(''.join(filter(str.isdigit, instruction_list[3])))

    op = '00'  # ALUOp是00
    rn = int(''.join(filter(str.isdigit, instruction_list[2])))
    rt = int(''.join(filter(str.isdigit, instruction_list[1])))

    if (instruction_list[0] == 'LDUR'): # LDUR
        print('Register[' + str(rt) + '] = RAM[ Register[' + str(rn) + ']' + ('' if len(instruction_list) < 4 else (' + ' + str(dt_address))) + ' ]')
    else: # STUR
        print('RAM[ Register[' + str(rn) + ']' + ('' if len(instruction_list) < 4 else (' + ' + str(dt_address))) + ' ] = Register[' + str(rt) + ']')

    machine_code += str(bin(dt_address)[2:].zfill(9)) + op + str(bin(rn)[2:].zfill(5)) + str(bin(rt)[2:].zfill(5))

elif (instruction_list[0] == 'ADD' or
        instruction_list[0] == 'SUB' or
        instruction_list[0] == 'ORR' or
        instruction_list[0] == 'AND'): # R类指令

    rm = int(''.join(filter(str.isdigit, instruction_list[3])))
    shamt = '000000'   # 始终不移位
    rn = int(''.join(filter(str.isdigit, instruction_list[2])))
    rd = int(''.join(filter(str.isdigit, instruction_list[1])))
    print('Register[' + str(rd) + '] = Register[' + str(rn) + '] ' + OPCODES[instruction_list[0]][1] + ' Register[' + str(rm) + ']')

    machine_code += str(bin(rm)[2:].zfill(5)) + shamt + str(bin(rn)[2:].zfill(5)) + str(bin(rd)[2:].zfill(5))

elif (instruction_list[0] == 'B'): # B指令

    br_address = int(''.join(filter(str.isdigit, instruction_list[1])))
    print('PC = ' + str(br_address))

    machine_code += str(bin(br_address)[2:].zfill(26))

elif (instruction_list[0] == 'CBZ'): # CBZ指令

    cond_br_address = int(''.join(filter(str.isdigit, instruction_list[2])))
    rt = int(''.join(filter(str.isdigit, instruction_list[1])))
    print('if ( Register[' + str(rt) + '] == 0 ) { PC = ' + str(cond_br_address) + ' }')
    print('else { PC++ }')

    machine_code += str(bin(cond_br_address)[2:].zfill(19)) + str(bin(rt)[2:].zfill(5))

else:  # 不支持其他指令
    raise RuntimeError('OPCODE (' + instruction_list[0] + ') not supported')

# 输出机器码
print('------- Machine Code (' + str(len(machine_code)) + '-bits) -------')
print('BINARY : ' + machine_code)
print('HEX    : ' + str(hex(int(machine_code, 2)))[2:])
print('\n')
