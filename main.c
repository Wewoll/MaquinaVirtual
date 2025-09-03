#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define RAM_SIZE (16 * 1024)    // 16 KiB
#define REG_AMOUNT 32           // 32 registros
#define SEG_AMOUNT 2            // 2 descriptores de segmentos

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

//Define de instrucciones
typedef enum {
    SYS = 0x00, JMP, JZ, JP, JN, JNZ, JNP, JNN, NOT,
    STOP = 0x0F,
    MOV = 0x10, ADD, SUB, MUL, DIV, CMP, SHL, SHR, SAR, AND, OR, XOR, SWAP, LDL, LDH, RND
} OpCode;

//Tamanos usados
typedef uint8_t Byte;
typedef uint16_t DosBytes;
typedef uint32_t Register;

//Tabla de descriptores de segmentos
typedef struct {
    DosBytes base;   // dirección lógica de inicio
    DosBytes limit;  // tamaño del segmento en bytes
} TableSeg;

//Maquina Virtual
typedef struct {
    Byte     mem[RAM_SIZE];       // 16 KiB de RAM
    Register reg[REG_AMOUNT];     // 32 registros de 4 bytes
    TableSeg seg[SEG_AMOUNT];     // tabla de segmentos: 0 = CS, 1 = DS
} TMV;

void inicializacion(TMV *mv) {
    unsigned int i;

    for (i = 0; i < RAM_SIZE; i++)
        mv->mem[i] = 0;
    for (i = 0; i < REG_AMOUNT; i++)
        mv->reg[i] = 0;

    mv->reg[CS] = CS_POS;
    mv->reg[DS] = DS_POS;
    mv->reg[IP] = mv->reg[CS];
}

int main() {
    TMV mv;
    inicializacion(&mv);
    //TODO arranque de la lectura del archivo y su verificacion

    //TODO while
    //TODO fetch
    //TODO decode + execute

    return 0;
}
