#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#define RAM_SIZE   (16 * 1024)  // 16 KiB
#define REG_AMOUNT 32           // 32 registros
#define SEG_AMOUNT 2            // 2 descriptores de segmentos

#define CS_POS 0x00000000       // Primer entrada
#define DS_POS 0x00010000       // Segunda entrada
#define CS_SEG 0                // Valor para chequear si se esta dentro de CS

#define MASK_SEG   0xFFFF0000   // Mascara para agarrar bits de segmento
#define MASK_UNSEG 0x0000FFFF   // Mascara para quitar bits de segmento

#define MASKB_OPC 0b00011111
#define MASKB_OP1 0b00110000
#define MASKB_OP2 0b11000000

#define HEADER_RANGE 8          // Primeros bytes de cabecera del .vmx

//Vector de mensajes de errores
static const char* errorMsgs[] = {
    NULL,                                       // índice 0 (sin error)
    "Uso: vmx archivo.vmx [-d]\n",              // 1
    "Error: No se pudo abrir el archivo\n",     // 2
    "Error: Archivo muy corto\n",               // 3
    "Error: Firma incorrecta\n",                // 4
    "Error: Version incorrecta\n",              // 5
    "Error: El codigo es demasiado grande\n",   // 6
    "Error: Hay menos codigo del indicado\n",   // 7
    "Error: IP se salio del CS\n",              // 8
    "Error: OP1 no puede ser inmediato\n"       // 9
};

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
    Register flag;                // error flag
} TMV;

void errorHandler(TMV* mv, int err);
void readFile(TMV *mv, const char *filename);
void initialization(TMV *mv, TwoBytes codeSize);
int isIPinCS(TMV* mv);
void fetchInstruction(TMV* mv);
void fetchOperators(TMV* mv);
void executeProgram(TMV* mv);

int main(int argc, char *argv[]) {
    TMV mv;

    mv.flag = 0;

    if (argc < 2) {
        errorHandler(&mv, 1);
    }
    else {
        readFile(&mv, argv[1]);
        if (mv.flag == 0) {
            executeProgram(&mv);
        }
    }

    return mv.flag;
}

//Manejo de errores
void errorHandler(TMV* mv, int err) {
    mv->flag = err;
    fprintf(stderr, "%s", errorMsgs[err]);
}

//Lee el archivo, habria que hacer control de errores con un errorHandler o algo asi
void readFile(TMV* mv, const char* filename) {
    FILE *arch;
    Byte header[HEADER_RANGE];
    TwoBytes codeSize;

    arch = fopen(filename, "rb");
    if (arch == NULL)
        errorHandler(mv, 2);
    else {
        if (fread(header, 1, 8, arch) != HEADER_RANGE)
            errorHandler(mv, 3);
        else if (memcmp(header, "VMX25", 5) != 0)
            errorHandler(mv, 4);
        else if (header[5] != 1)
            errorHandler(mv, 5);
        else {
            codeSize = ((TwoBytes) header[6] << 8) | header[7];
            if (codeSize >= RAM_SIZE)
                errorHandler(mv, 6);
            else {
                //Carga del codigo en la memoria principal
                if (fread(mv->mem, 1, codeSize, arch) != codeSize)
                    errorHandler(mv, 7);
                initialization(mv, codeSize);
            }
        }
        fclose(arch);
    }
}


//Armar tabla de descriptores de segmentos e inicializar registros y memoria
void initialization(TMV *mv, TwoBytes codeSize) {
    //Inicio de la tabla de descriptores de segmentos
    mv->seg[0].base = 0;
    mv->seg[0].size = codeSize;
    mv->seg[1].base = codeSize;
    mv->seg[1].size = RAM_SIZE - codeSize;

    //Inicio de las pocisiones de CS, DS e IP
    mv->reg[CS] = CS_POS;
    mv->reg[DS] = DS_POS;
    mv->reg[IP] = mv->reg[CS];
}

//Chequeo sobre la posicion del registro IP
int isIPinCS(TMV* mv) {
    return mv->seg[CS_SEG].base <= mv->reg[IP] && mv->reg[IP] < (mv->seg[CS_SEG].base + mv->seg[CS_SEG].size);
}

//Agarra un byte de instruccion, temp no hace falta, pero hace todo mas claro
//Setea OPC, el tipo de OP1 y el tipo de OP2
void fetchInstruction(TMV* mv) {
    Byte temp;

    temp = mv->mem[mv->reg[IP] & MASK_UNSEG];
    mv->reg[OPC] = 0;
    mv->reg[OPC] = (Register)(temp & MASKB_OPC);

    mv->reg[OP1] = 0;
    mv->reg[OP2] = 0;
    mv->reg[OP1] = (Register)(temp & MASKB_OP2) << 18;

    if ((temp & MASKB_OP1) != 0) {              //Si hay dos operandos, mueve la OP1 a OP2 y agarra el tipo de OP1
        mv->reg[OP2] = mv->reg[OP1];
        mv->reg[OP1] = (Register)(temp & MASKB_OP1) << 20;
    }
}

//Crea un registro y lo devuelve a los operandos
Register fetchOperand(TMV* mv, int bytes) {
    Register temp = 0;

    while (bytes > 0 && mv->flag == 0) {
        temp |= ((Register) mv->mem[mv->reg[IP] & MASK_UNSEG]) << (8 * bytes);
        mv->reg[IP]++;
        if (!(isIPinCS(mv)))
            errorHandler(mv, 8);
        bytes--;
    }

    return temp;
}

//Prepara a los operandos para que reciban su informacion
void fetchOperators(TMV* mv) {
    int op1Bytes, op2Bytes;

    mv->reg[IP]++;
    if (!(isIPinCS(mv)))
        errorHandler(mv, 8);
    else {
        op2Bytes = mv->reg[OP2] >> 24;
        op1Bytes = mv->reg[OP1] >> 24;
        if (op1Bytes == 2)
            errorHandler(mv, 9);
        else {
            mv->reg[OP2] |= fetchOperand(mv, op2Bytes);
            if (mv->flag == 0)
                mv->reg[OP1] |= fetchOperand(mv, op1Bytes);
        }
    }
}

void executeProgram(TMV* mv) {
    //Hay que agregar errores por si se salio sin stop capaz
    while (isIPinCS(mv)) {
        fetchInstruction(mv);
        fetchOperators(mv);
    }
}
