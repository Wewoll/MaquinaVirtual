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

//Tama�os usados
typedef uint8_t Byte;
typedef uint32_t Register;

typedef struct {
    Byte     mem[RAM_SIZE];      // memoria RAM - 16384 celdas de 1 byte cada una
    Register reg[REG_AMOUNT];    // bancos de registros - 32 registros de 4 bytes cada uno
} MV;

static void inicializacion(MV *mv) {
    unsigned int i;

    for (i = 0; i < RAM_SIZE; i++) mv->mem[i] = 0;
    for (i = 0; i < REG_AMOUNT; i++) mv->reg[i] = 0;

    mv->reg[CS] = CS_POS;
    mv->reg[DS] = DS_POS;
    mv->reg[IP] = mv->reg[CS];
}

int main() {
    MV mv;
    inicializacion(&mv);
    //TODO arranque de la lectura del archivo y su verificacion

    //TODO while
    //TODO fetch
    //TODO decode + execute

    return 0;
}
