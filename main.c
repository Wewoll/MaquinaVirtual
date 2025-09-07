#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#define RAM_SIZE   (16 * 1024)  // 16 KiB
#define REG_AMOUNT 32           // 32 registros
#define SEG_AMOUNT 2            // 2 descriptores de segmentos

// Indice de segmentos de la tabla de descriptores
#define CS_SEG 0
#define DS_SEG 1

// Macros para construir la dirección logica inicial de cada segmento
#define SEG_POS(seg) ((seg) << 16)

#define CS_INI SEG_POS(CS_SEG)
#define DS_INI SEG_POS(DS_SEG)

#define MASKB_OPC 0b00011111
#define MASKB_OP1 0b00110000
#define MASKB_OP2 0b11000000

#define HEADER_RANGE 8          // Primeros bytes de cabecera del .vmx

#define STOP_VALUE 0xFFFFFFFF

#define CC_N 0x80000000
#define CC_Z 0x40000000

//Define de error
typedef enum {
    ERR_EXE = 1,
    ERR_FOPEN,
    ERR_ARCHSIZE,
    ERR_ARCHID,
    ERR_ARCHVER,
    ERR_CODSIZE,
    ERR_SEG,
    ERR_INS,
    ERR_OP1,
    ERR_DIV0,
} ErrNumber;

//Vector de mensajes de errores
static const char* errorMsgs[] = {
    NULL,                                           // indice 0 (sin error)
    "Uso: vmx archivo.vmx [-d]\n",                  // 1
    "Error: No se pudo abrir el archivo\n",         // 2
    "Error: Archivo muy corto\n",                   // 3
    "Error: Firma incorrecta\n",                    // 4
    "Error: Version incorrecta\n",                  // 5
    "Error: El codigo es demasiado grande\n",       // 6
    "Error: Fuera de los limites del segmento\n",   // 7 - pedido en el pdf
    "Error: Intruccion invalida\n",                 // 8 - pedido en el pdf
    "Error: OP1 no puede ser inmediato\n",          // 9
    "Error: No se puede dividir por 0\n"            // 10 - pedido en el pdf
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
Register decodeAddr(TMV* mv, Register logical);
int inCS(TMV* mv, Register logical);
void fetchInstruction(TMV* mv);
Register fetchOperand(TMV* mv, int bytes, int* offset);
void fetchOperators(TMV* mv);

Register getOP(TMV* mv, Register operand);
void setOP(TMV* mv, Register operandA, Register operandB);
void fmov(TMV* mv);
void fstop(TMV* mv);
void executeProgram(TMV* mv);



int main(int argc, char *argv[]) {
    TMV mv;

    mv.flag = 0;

    if (argc < 2) {
        errorHandler(&mv, ERR_EXE);
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

//Lee el archivo
void readFile(TMV* mv, const char* filename) {
    FILE *arch;
    Byte header[HEADER_RANGE];
    TwoBytes codeSize;

    arch = fopen(filename, "rb");
    if (arch == NULL)
        errorHandler(mv, ERR_FOPEN);
    else {
        if (fread(header, 1, 8, arch) != HEADER_RANGE)
            errorHandler(mv, ERR_ARCHSIZE);
        else if (memcmp(header, "VMX25", 5) != 0)
            errorHandler(mv, ERR_ARCHID);
        else if (header[5] != 1)
            errorHandler(mv, ERR_ARCHVER);
        else {
            codeSize = ((TwoBytes) header[6] << 8) | header[7];
            if (codeSize >= RAM_SIZE)
                errorHandler(mv, ERR_CODSIZE);
            else {
                //Carga del codigo en la memoria principal e inicializar
                fread(mv->mem, 1, codeSize, arch);
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
    mv->reg[CS] = CS_INI;
    mv->reg[DS] = DS_INI;
    mv->reg[IP] = mv->reg[CS];
}

//Decodificador de direccion logica a direccion fisica
Register decodeAddr(TMV* mv, Register logical) {
    TwoBytes segIndex, offset;
    TableSeg seg;

    segIndex = (TwoBytes) (logical >> 16);
    offset = (TwoBytes) logical;

    seg = mv->seg[segIndex];
    if  (offset >= seg.size)
        errorHandler(mv, ERR_SEG);

    return seg.base + offset;
}

//Chequeo sobre la posicion del registro IP
int inCS(TMV* mv, Register logical) {
    return mv->seg[CS_SEG].base <= logical && logical < (mv->seg[CS_SEG].base + mv->seg[CS_SEG].size);
}

//Agarra un byte de instruccion, temp no hace falta, pero hace todo mas claro
//Setea OPC, el tipo de OP1 y el tipo de OP2
void fetchInstruction(TMV* mv) {
    Byte temp;

    temp = mv->mem[decodeAddr(mv, mv->reg[IP])];
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
Register fetchOperand(TMV* mv, int bytes, int* offset) {
    Register temp = 0;

    while (bytes > 0 && mv->flag == 0) {
        temp |= ((Register) mv->mem[decodeAddr(mv, mv->reg[IP] + (*offset))]) << (8 * (bytes - 1));
        ++(*offset);
        if (!(inCS(mv, mv->reg[IP] + (*offset))) && mv->flag == 0)
            errorHandler(mv, ERR_SEG);
        bytes--;
    }

    return temp;
}

//Prepara a los operandos para que reciban su informacion
void fetchOperators(TMV* mv) {
    int op1Bytes, op2Bytes, offset = 1;

    op2Bytes = mv->reg[OP2] >> 24;
    op1Bytes = mv->reg[OP1] >> 24;
    if (op1Bytes == 2)
        errorHandler(mv, ERR_OP1);
    else {
        mv->reg[OP2] |= fetchOperand(mv, op2Bytes, &offset);
        if (mv->flag == 0)
            mv->reg[OP1] |= fetchOperand(mv, op1Bytes, &offset);
    }
}

Register getOP(TMV* mv, Register operand) {
    Byte tipo = operand >> 24;
    Register res;

    operand &= 0x00FFFFFF;

    switch (tipo) {
        case 1:
            res = mv->reg[operand];
            break;
        case 2:
            //falta extension de signo
            res = operand;
            break;
        case 3:
            //LAR, MAR, MBR
            // MBR = valor;
            // LAR = registro + offset
            // mar = decodeAdr(LAR)
            // mar = cargaAlta
            break;
    }

    return res;
}

void setOP(TMV* mv, Register operandA, Register operandB) {
    Byte tipo = operandA >> 24;

    operandA &= 0x00FFFFFF;
    switch (tipo) {
        case 1:
            mv->reg[operandA] = operandB;
            break;
        case 3:
            //Memoria
            break;
    }
}

void setCC(TMV* mv, Register valor) {
    if (valor < 0)
        mv->reg[CC] = CC_N;
    else if (valor == 0)
        mv->reg[CC] = CC_Z;
}

//Instruccion NOT bit a bit
void fnot(TMV* mv) {
    Register res;
    res = ~ getOP(mv, mv->reg[OP1]);
    setCC(mv, res);
    setOP(mv, mv->reg[OP1], res);
}

//Instruccion STOP
//Arreglar que error que tira
void fstop(TMV* mv) {
    mv->reg[IP] = STOP_VALUE;
}

//Instruccion MOV
void fmov(TMV* mv) {
    setOP(mv, mv->reg[OP1], getOP(mv, mv->reg[OP2]));
}

//Instruccion ADD
void fadd(TMV* mv) {
    Register res;
    res = getOP(mv, mv->reg[OP1]) + getOP(mv, mv->reg[OP2]);
    setCC(mv, res);
    setOP(mv, mv->reg[OP1], res);
}

//Instruccion SUB
void fsub(TMV* mv) {
    Register res;
    res = getOP(mv, mv->reg[OP1]) - getOP(mv, mv->reg[OP2]);
    setCC(mv, res);
    setOP(mv, mv->reg[OP1], res);
}

//Instruccion MUL
void fmul(TMV* mv) {
    Register res;
    res = getOP(mv, mv->reg[OP1]) * getOP(mv, mv->reg[OP2]);
    setCC(mv, res);
    setOP(mv, mv->reg[OP1], res);
}

//Instruccion DIV
void fdiv(TMV* mv) {
    Register res;
    if (getOP(mv, mv->reg[OP2]) == 0)
        errorHandler(mv, ERR_DIV0);
    else {
        res = getOP(mv, mv->reg[OP1]) / getOP(mv, mv->reg[OP2]);
        mv->reg[AC] = getOP(mv, mv->reg[OP1]) % getOP(mv, mv->reg[OP2]);
        setCC(mv, res);
        setOP(mv, mv->reg[OP1], res);
    }
}

//Instruccion AND bit a bit
void fand(TMV* mv) {
    Register res;
    res = getOP(mv, mv->reg[OP1]) & getOP(mv, mv->reg[OP2]);
    setCC(mv, res);
    setOP(mv, mv->reg[OP1], res);
}

//Instruccion OR bit a bit
void f_or(TMV* mv) {
    Register res;
    res = getOP(mv, mv->reg[OP1]) | getOP(mv, mv->reg[OP2]);
    setCC(mv, res);
    setOP(mv, mv->reg[OP1], res);
}

//Instruccion XOR bit a bit
void fxor(TMV* mv) {
    Register res;
    res = getOP(mv, mv->reg[OP1]) ^ getOP(mv, mv->reg[OP2]);
    setCC(mv, res);
    setOP(mv, mv->reg[OP1], res);
}

void executeProgram(TMV* mv) {
    //Hay que agregar errores por si se salio sin stop capaz
    //Hay que agregar que se salga si el flag != 0
    //Capaz no hace falta la funcion, esto podria ir en main

    while (mv->flag == 0 && inCS(mv, mv->reg[IP])) {
        fetchInstruction(mv);
        fetchOperators(mv);

        if(mv->flag == 0) {
            if (mv->reg[OPC] == STOP)
                fstop(mv);
            else {
                switch (mv->reg[OPC]) {
                    case NOT:
                        fnot(mv);
                        break;

                    case MOV:
                        fmov(mv);
                        break;

                    case ADD:
                        fadd(mv);
                        break;

                    case SUB:
                        fsub(mv);
                        break;

                    case MUL:
                        fmul(mv);
                        break;

                    case DIV:
                        fdiv(mv);
                        break;

                    case AND:
                        fand(mv);
                        break;

                    case OR:
                        f_or(mv);
                        break;

                    case XOR:
                        fxor(mv);
                        break;

                    // Falta que las instrucciones modifiquen al registro CC
                    // Faltan los errorHandler
                    /*
                    Instrucciones que faltan con...
                        2 operandos: CMP, JMP, JZ, SHL, SHR, SAR, SWAP, LDH, LDL, RND
                        1 op: SYS, JMP, JZ, JP, JN, JNZ, JNP, JNN
                    */
                    default:
                        errorHandler(mv, ERR_INS);
                        break;
                }
                mv->reg[IP] += 1 + (mv->reg[OP1] >> 24) + (mv->reg[OP2] >> 24);
            }
        }
    }

    if (!(inCS(mv, mv->reg[IP] && mv->reg[OPC] != STOP)))
        errorHandler(mv, ERR_SEG);
}
