#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#define RAM_SIZE   (16 * 1024)  // 16 KiB
#define REG_AMOUNT 32           // 32 registros
#define SEG_AMOUNT 2            // 2 descriptores de segmentos

#define CS_POS 0x00000000       // Primer entrada
#define DS_POS 0x00010000       // Segunda entrada

#define MASK_SEG   0xFFFF0000   // Mascara para agarrar bits de segmento
#define MASK_UNSEG 0x0000FFFF   // Mascara para quitar bits de segmento
#define IN_CS      0            // Valor para chequear si se esta dentro de CS

#define MASKB_OPC 0b00011111
#define MASK_OPC  0x000000FF
#define MASKB_OP1 0b00110000
#define MASK_OP1  0x00FF0000
#define MASKB_OP2 0b11000000
#define MASK_OP2  0xFF000000

#define HEADER_RANGE 8          // Primeros bytes de cabecera del .vmx

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
typedef uint16_t TwoBytes;
typedef uint32_t Register;

//Tabla de descriptores de segmentos
typedef struct {
    TwoBytes base;   // dirección lógica de inicio
    TwoBytes size;   // tamaño del segmento en bytes
} TableSeg;

//Maquina Virtual
typedef struct {
    Byte     mem[RAM_SIZE];       // 16 KiB de RAM
    Register reg[REG_AMOUNT];     // 32 registros de 4 bytes
    TableSeg seg[SEG_AMOUNT];     // tabla de segmentos: 0 = CS, 1 = DS
} TMV;

void readFile(const char *filename, int *err, TMV *mv);
void initialization(TMV *mv, TwoBytes codeSize);
void fetchInstruction(TMV* mv);
void executeProgram(TMV* mv);


int main(int argc, char *argv[]) {
    TMV mv;
    int err = 0;

    if (argc < 2) {
        fprintf(stderr, "Uso: %s archivo.vmx [-d]\n", argv[0]);
        err++;
    }
    else {
        readFile(argv[1], &err, &mv);
        if (err == 0) {
            executeProgram(&mv);
        }
    }

    return err;
}

//Lee el archivo, habria que hacer control de errores con un errorHandler o algo asi
void readFile(const char *filename, int *err, TMV *mv) {
    FILE *arch;
    Byte header[HEADER_RANGE];
    TwoBytes codeSize;

    arch = fopen(filename, "rb");
    if (arch == NULL) {
        perror("No se pudo abrir el archivo");
        (*err)++;
    }
    else {
        if (fread(header, 1, 8, arch) != HEADER_RANGE) {
            fprintf(stderr, "Error: archivo muy corto\n");
            (*err)++;
        }
        else {
            if (memcmp(header, "VMX25", 5) != 0) {
                fprintf(stderr, "Error: firma incorrecta\n");
                (*err)++;
            }
            else {
                if (header[5] != 1) {
                    fprintf(stderr, "Error: version incorrecta (%d)\n", header[5]);
                    (*err)++;
                }
                else {
                    codeSize = ((TwoBytes)header[6] << 8) | header[7];
                    if (codeSize >= RAM_SIZE) {
                        fprintf(stderr, "Error: el codigo es demasiado grande (%u bytes)\n", codeSize);
                        (*err)++;
                    }
                    else {
                        initialization(mv, codeSize);

                        //Carga del codigo en la memoria principal
                        if (fread(mv->mem, 1, codeSize, arch) != codeSize) {
                            fprintf(stderr, "Error leyendo el codigo\n");
                            (*err)++;
                        }
                    }
                }
            }
        }

        fclose(arch);
    }
}

//Armar tabla de descriptores de segmentos e inicializar registros y memoria
void initialization(TMV *mv, TwoBytes codeSize) {
    unsigned int i;

    //Inicio de la tabla de descriptores de segmentos
    mv->seg[0].base = 0;
    mv->seg[0].size = codeSize;
    mv->seg[1].base = codeSize;
    mv->seg[1].size = RAM_SIZE - codeSize;

    //Inicio de la memoria y los registros
    for (i = 0; i < RAM_SIZE; i++)
        mv->mem[i] = 0;
    for (i = 0; i < REG_AMOUNT; i++)
        mv->reg[i] = 0;

    //Inicio de las pocisiones de CS, DS e IP
    mv->reg[CS] = CS_POS;
    mv->reg[DS] = DS_POS;
    mv->reg[IP] = mv->reg[CS];
}

//Agarra un byte de instruccion, temp no hace falta, pero hace todo mas claro
//Setea OPC, el tipo de OP1 y el tipo de OP2
void fetchInstruction(TMV* mv) {
    Byte temp;

    temp = mv->mem[mv->reg[IP] & MASK_UNSEG];
    mv->reg[OPC] = (Register)(temp & MASKB_OPC) & MASK_OPC;

    mv->reg[OP1] = ((Register)(temp & MASKB_OP2) << 18) & MASK_OP2;
    mv->reg[OP2] = 0;

    if ((temp & MASKB_OP1) != 0) {              //Si hay dos operandos, mueve la OP1 a OP2 y agarra el tipo de OP1
        mv->reg[OP2] = mv->reg[OP1];
        mv->reg[OP1] = ((Register)(temp & MASKB_OP1) << 20) & MASK_OP1;     //Aca deberia ir una excepcion de error en caso de que el tipo sea inmediato (porque no puede serlo)
    }
}

void executeProgram(TMV* mv) {
    while ((mv->reg[IP] & MASK_SEG) >> 16 == IN_CS) {
        fetchInstruction(mv);
        //fetchOperators(mv);
    }
}
