#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define RAM_SIZE (16 * 1024)    // 16 KiB
#define REG_AMOUNT 32           // 32 registros

#define CS_POS 0x00000000       // Primer entrada
#define DS_POS 0x00010000       // Segunda entrada

//Define de registros
typedef enum {
    LAR = 0, MAR, MBR,
    IP = 3, OPC, OP1, OP2,
    EAX = 10, EBX, ECX, EDX, EEX, EFX,
    AC = 16,
    CC = 17,
    CS = 26, DS
} RegName;

//Define de Instrucciones
typedef enum {
    SYS = 0x00, JMP, JZ, JP, JN, JNZ, JNP, JNN, NOT,
    STOP = 0x0F,
    MOV = 0x10, ADD, SUB, MUL, DIV, CMP, SHL, SHR, SAR, AND, OR, XOR, SWAP, LDL, LDH, RND
} OpCode;

//Tamaños usados
typedef uint8_t Byte;
typedef uint32_t Register;

int main() {
    Byte Mem[RAM_SIZE];             // 16384 celdas de 1 byte cada una
    Register Reg[REG_AMOUNT];       // 32 registros de 4 bytes cada uno

    //TODO arranque de la lectura del archivo y su verificacion

    //Inicializacion
    Reg[CS] = CS_POS;
    Reg[DS] = DS_POS;
    Reg[IP] = Reg[CS];

    //TODO while
    //TODO fetch
    //TODO decode + execute

    return 0;
}
