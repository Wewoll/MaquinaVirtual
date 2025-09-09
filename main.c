#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#define RAM_SIZE   (16 * 1024)  // 16 KiB
#define REG_AMOUNT 32           // 32 registros
#define SEG_AMOUNT 2            // 2 descriptores de segmentos

#define HEADER_RANGE 8          // Primeros bytes de cabecera del .vmx

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

#define MASK_SEG    0xFFFF0000
#define MASK_UNTYPE 0x00FFFFFF
#define MASK_REG    0x00FF0000
#define MASK_OFFSET 0x0000FFFF
#define MASK_PHY    0x0000FFFF
#define MASK_LDL    0x0000FFFF
#define MASK_LDH    0xFFFF0000
#define MASK_SETMEM 0xFF

#define STOP_VALUE  0xFFFFFFFF

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
    "Error: No se puede dividir por 0\n"            // 9 - pedido en el pdf
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
typedef int8_t Byte;
typedef int16_t TwoBytes;
typedef int32_t Register;

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
    Register disassembler;        //
} TMV;

void errorHandler(TMV* mv, int err);
void readFile(TMV *mv, const char *filename);
void initialization(TMV *mv, TwoBytes codeSize);
void disASMOP(TMV* mv, Register operand, Byte type);
void disASM(TMV* mv);
Register decodeAddr(TMV* mv, Register logical);
int inSegment(TMV* mv, Register logical);
int inCS(TMV* mv, Register logical);
int inDS(TMV* mv, Register logical);
void fetchInstruction(TMV* mv);
Register fetchOperand(TMV* mv, int bytes, int* offset);
void fetchOperators(TMV* mv);

void setLAR(TMV* mv, Register operand);
void setMAR(TMV* mv, Register cantBytes, Register logical);
void getMemory(TMV* mv);
void setMemory(TMV* mv);
Register getOP(TMV* mv, Register operand);
void setOP(TMV* mv, Register operandA, Register operandB);
void setCC(TMV* mv, Register valor);
void fsysRead(TMV* mv);
void fsysWrite(TMV* mv);
void fsys(TMV* mv);
void fjmp(TMV* mv, int salto);
int fjz(TMV* mv);
int fjn(TMV* mv);
void fnot(TMV* mv);
void fstop(TMV* mv);
void fmov(TMV* mv);
void fadd(TMV* mv);
void fsub(TMV* mv);
void fmul(TMV* mv);
void fdiv(TMV* mv);
void fshl(TMV* mv);
void fshr(TMV* mv);
void fsar(TMV* mv);
void fand(TMV* mv);
void f_or(TMV* mv);
void fxor(TMV* mv);
void fswap(TMV* mv);
void fldl(TMV* mv);
void fldh(TMV* mv);
void frnd(TMV* mv);
void executeProgram(TMV* mv);



int main(int argc, char *argv[]) {
    TMV mv;

    mv.flag = 0;

    if (argc < 2) {
        errorHandler(&mv, ERR_EXE);
    }
    else {
        mv.disassembler = (argc == 3) && (strcmp(argv[2], "-d") == 0);
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


//Armar tabla de descriptores de segmentos e inicializar registros CS, DS, IP
void initialization(TMV *mv, TwoBytes codeSize) {
    //Inicio de la tabla de descriptores de segmentos
    mv->seg[CS_SEG].base = 0;
    mv->seg[CS_SEG].size = codeSize;
    mv->seg[DS_SEG].base = codeSize;
    mv->seg[DS_SEG].size = RAM_SIZE - codeSize;

    //Inicio de las pocisiones de CS, DS e IP
    mv->reg[CS] = CS_INI;
    mv->reg[DS] = DS_INI;
    mv->reg[IP] = mv->reg[CS];
}

void disASMOP(TMV* mv, Register operand, Byte type) {
    int i = type - 1;
    Byte aux = 0;

    for (; i >= 0; i--) {
        aux = (Byte) operand >> (8 * i);
        printf("%02X ", aux);
    }
}

void disASM(TMV* mv) {
    int i;
    Byte ins = 0, typA = 0, typB = 0;

    printf("[%04X] ", mv->mem[decodeAddr(mv, mv->reg[IP])]);

    typA = mv->reg[OP1] >> 18;
    typB = mv->reg[OP2] >> 18;
    if (typB != 0)
        typA >>= 2;
    ins = typB | typA | (Byte) mv->reg[OPC];
    printf("%02X ", ins);

    disASMOP(mv, mv->reg[OP2], typB);
    disASMOP(mv, mv->reg[OP1], typA);

    for (i = 6 - typA - typB; i > 0; i--)
        printf("   ");

    printf("|  \n"); //El \n esta de mas, porque tecnicamente falta la escritura del mnemonico
}

//Decodificador de direccion logica a direccion fisica
Register decodeAddr(TMV* mv, Register logical) {
    TwoBytes segIndex, offset;

    segIndex = (TwoBytes) (logical >> 16);
    offset = (TwoBytes) logical;

    if (offset >= mv->seg[segIndex].size)
        errorHandler(mv, ERR_SEG);

    return mv->seg[segIndex].base + offset;
}

// Verifica si una dirección logica esta dentro de su segmento
int inSegment(TMV* mv, Register logical) {
    TwoBytes segIndex, offset;

    segIndex = (TwoBytes) (logical >> 16);
    offset = (TwoBytes) logical;

    return offset < mv->seg[segIndex].size;
}

// Verifica si una dirección lógica está en CS
int inCS(TMV* mv, Register logical) {
    return ((logical >> 16) == CS_SEG) && inSegment(mv, logical);
}

// Verifica si una dirección lógica está en DS
int inDS(TMV* mv, Register logical) {
    return ((logical >> 16) == DS_SEG) && inSegment(mv, logical);
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
    Register logical, physical, temp = 0;

    while (bytes > 0 && mv->flag == 0) {
        logical = mv->reg[IP] + (*offset);
        if (!inCS(mv, logical))
            errorHandler(mv, ERR_SEG);
        else {
            physical = decodeAddr(mv, logical);
            temp |= ((Register)mv->mem[physical]) << (8 * (bytes - 1));
            ++(*offset);
            bytes--;
        }
    }

    return temp;
}

//Prepara a los operandos para que reciban su informacion
void fetchOperators(TMV* mv) {
    int op1Bytes, op2Bytes, offset = 1;

    op2Bytes = mv->reg[OP2] >> 24;
    op1Bytes = mv->reg[OP1] >> 24;
    mv->reg[OP2] |= fetchOperand(mv, op2Bytes, &offset);
    if (mv->flag == 0)
        mv->reg[OP1] |= fetchOperand(mv, op1Bytes, &offset);
}

//Seteo del valor del LAR cuando se trabaja con memoria
void setLAR(TMV* mv, Register operand) {
    Byte cod;
    TwoBytes offset;
    Register logical;

    cod = (Byte) ((operand & MASK_REG) >> 16);
    offset = (TwoBytes) (operand & MASK_OFFSET);
    logical = mv->reg[cod] + offset;
    printf("LAR - %x - %x - %x\n", cod, offset, logical);
    if (!inSegment(mv, logical))
        errorHandler(mv, ERR_SEG);
    mv->reg[LAR] = logical;
}

//Seteo del valor del MAR cuando se trabaja con memoria
void setMAR(TMV* mv, Register cantBytes, Register logical) {
    Register physical;

    if (mv->flag == 0) {
        printf("MAR\n");
        physical = decodeAddr(mv, logical);
        if (physical + cantBytes - 1 >= RAM_SIZE)
            errorHandler(mv, ERR_SEG);
        else
            mv->reg[MAR] = (cantBytes << 16) | physical;
    }
}

//Get del valor de memoria al MBR
void getMemory(TMV* mv) {
    Register cantBytes, physical, temp;
    int i;

    if (mv->flag == 0) {
        cantBytes = mv->reg[MAR] >> 16;
        physical = mv->reg[MAR] & MASK_PHY;
        temp = 0;

        for (i = 0; i < cantBytes; i++)
            temp |= (Register) mv->mem[physical + i] << (8 * (cantBytes - 1 - i));

        mv->reg[MBR] = temp;
    }
}

//Set del valor del MBR a la memoria
void setMemory(TMV* mv) {
    Register cantBytes, physical;
    int i;

    if (mv->flag == 0) {
        cantBytes = mv->reg[MAR] >> 16;
        physical = mv->reg[MAR] & MASK_PHY;

        for (i = 0; i < cantBytes; i++)
            mv->mem[physical + i] = mv->reg[MBR] >> (8 * (cantBytes - 1 - i)) & MASK_SETMEM;
    }
}

//Decodificar el operador y devolver su valor
Register getOP(TMV* mv, Register operand) {
    Byte tipo = operand >> 24;
    Register res;

    operand &= MASK_UNTYPE;

    switch (tipo) {
        case 1:
            res = mv->reg[operand];
            break;
        case 2:
            res = (Register)(TwoBytes)operand;
            break;
        case 3:
            setLAR(mv, operand);
            setMAR(mv, 4, mv->reg[LAR]);
            getMemory(mv);
            res = mv->reg[MBR];
            break;
    }

    return res;
}

//Setear el registro o memoria de operadorA con el valor de operadorB
void setOP(TMV* mv, Register operandA, Register operandB) {
    Byte tipo = operandA >> 24;

    operandA &= MASK_UNTYPE;
    switch (tipo) {
        case 1:
            mv->reg[operandA] = operandB;
            break;
        case 3:
            setLAR(mv, operandA);
            setMAR(mv, 4, mv->reg[LAR]);
            mv->reg[MBR] = operandB;
            setMemory(mv);
            break;
    }
}

//Setear el CC
void setCC(TMV* mv, Register valor) {
    if (valor < 0)
        mv->reg[CC] = CC_N;
    else if (valor == 0)
        mv->reg[CC] = CC_Z;
}

void fsysRead(TMV* mv) {
    int read;

    scanf("%d", &read);
    mv->reg[MBR] = read;
    setMemory(mv);
}

void fsysWrite(TMV* mv) {
    int write;

    getMemory(mv);
    write = mv->reg[MBR];
    printf("%d", write);
}

//Funcion SYS
void fsys(TMV* mv) {
    int cantBytes, tamCeldas;

    cantBytes = mv->reg[ECX] & MASK_LDL;
    tamCeldas = (mv->reg[ECX] & MASK_LDH) >> 16;

    setLAR(mv, mv->reg[EDX]);
    setMAR(mv, tamCeldas, mv->reg[LAR]);

    switch (getOP(mv, mv->reg[OP1])) {
        case 1:
            fsysRead(mv);
            break;
        case 2:
            fsysWrite(mv);
            break;
    }
}

//Salto con posibilidad de condicion
void fjmp(TMV* mv, int salto) {
    if (salto)
      mv->reg[IP] = getOP(mv, mv->reg[OP1]);
}

//Boolean para indicar si saltar por cero
int fjz(TMV* mv) {
    return mv->reg[CC] & CC_Z;
}

//Boolean para indicar si saltar por negativo
int fjn(TMV* mv) {
    return mv->reg[CC] & CC_N;
}

//Instruccion NOT bit a bit
void fnot(TMV* mv) {
    Register res;
    res = ~ getOP(mv, mv->reg[OP1]);
    setCC(mv, res);
    setOP(mv, mv->reg[OP1], res);
}

//Instruccion STOP
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

//Instruccion SUB (y CMP)
void fsub(TMV* mv) {
    Register res;
    res = getOP(mv, mv->reg[OP1]) - getOP(mv, mv->reg[OP2]);
    setCC(mv, res);
    if (mv->reg[OPC] == SUB)
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

//Desplazamientos de bits a izquierda
void fshl(TMV* mv) {
    Register res;
    res = getOP(mv, mv->reg[OP1]) << getOP(mv, mv->reg[OP2]);
    setCC(mv, res);
    setOP(mv, mv->reg[OP1], res);
}

//Desplazamientos de bits a derecha
void fshr(TMV* mv) {
    Register res;
    res = (Register) ((uint32_t) getOP(mv, mv->reg[OP1]) >> getOP(mv, mv->reg[OP2]));
    setCC(mv, res);
    setOP(mv, mv->reg[OP1], res);
}

//Desplazamientos de bits a derecha pero propaga signo
void fsar(TMV* mv) {
    Register res;
    res = getOP(mv, mv->reg[OP1]) >> getOP(mv, mv->reg[OP2]);
    setCC(mv, res);
    setOP(mv, mv->reg[OP1], res);
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

//Intruccion SWAP para cambiar de lugar operandos
void fswap(TMV* mv) {
    Register aux = 0;
    aux = mv->reg[OP1];
    mv->reg[OP1] = mv->reg[OP2];
    mv->reg[OP2] = aux;
}

void fldl(TMV* mv) {
    Register cargaBaja, cod;
    Byte tipo;

    cargaBaja = getOP(mv, mv->reg[OP2]) & MASK_LDL;
    tipo = mv->reg[OP1] >> 24;
    cod = mv->reg[OP1] & MASK_UNTYPE;

    switch (tipo) {
        case 1:
            mv->reg[cod] = (mv->reg[cod] & MASK_LDH) | cargaBaja;
            break;
        case 3:
            setLAR(mv, cod);
            setMAR(mv, 4, mv->reg[LAR]);
            mv->reg[MBR] = (mv->reg[MBR] & MASK_LDH) | cargaBaja;
            setMemory(mv);
            break;
    }
}

void fldh(TMV* mv) {
    Register cargaAlta, cod;
    Byte tipo;

    cargaAlta = (getOP(mv, mv->reg[OP2]) & MASK_LDL) << 16;
    tipo = mv->reg[OP1] >> 24;
    cod = mv->reg[OP1] & MASK_UNTYPE;

    switch (tipo) {
        case 1:
            mv->reg[cod] = (mv->reg[cod] & MASK_LDL) | cargaAlta;
            break;
        case 3:
            setLAR(mv, cod);
            setMAR(mv, 4, mv->reg[LAR]);
            mv->reg[MBR] = (mv->reg[MBR] & MASK_LDL) | cargaAlta;
            setMemory(mv);
            break;
    }
}

void frnd(TMV* mv) {
    Register randN;
    srand(time(NULL));
    randN = (Register) (rand() % getOP(mv, mv->reg[OP2]));
    setOP(mv, mv->reg[OP1], randN);
}

void executeProgram(TMV* mv) {
    //Hay que agregar errores por si se salio sin stop capaz
    //Hay que agregar que se salga si el flag != 0
    //Capaz no hace falta la funcion, esto podria ir en main
    int i = 0;
    while (mv->flag == 0 && inCS(mv, mv->reg[IP])) {
        fetchInstruction(mv);
        fetchOperators(mv);
        if (mv->disassembler)
            disASM(mv);
        printf("Instruccion %d - %x - %x\n", i, mv->reg[OPC], mv->reg[IP]);
        i++;
        if(mv->flag == 0) {
            if (mv->reg[OPC] == STOP)
                fstop(mv);
            else {
                switch (mv->reg[OPC]) {
                    case SYS:
                        fsys(mv);
                        break;

                    case JMP:
                        fjmp(mv, 1);
                        break;

                    case JZ:
                        fjmp(mv, fjz(mv));
                        break;

                    case JP:
                        fjmp(mv, !(fjz(mv)) && !(fjn(mv)));
                        break;

                    case JN:
                        fjmp(mv, fjn(mv));
                        break;

                    case JNZ:
                        fjmp(mv, !fjz(mv));
                        break;

                    case JNP:
                        fjmp(mv, fjz(mv) || fjn(mv));
                        break;

                    case JNN:
                        fjmp(mv, !fjn(mv));
                        break;

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

                    case CMP:
                        fsub(mv);
                        break;

                    case SHL:
                        fshl(mv);
                        break;

                    case SHR:
                        fshr(mv);
                        break;

                    case SAR:
                        fsar(mv);
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

                    case SWAP:
                        fswap(mv);
                        break;

                    case LDL:
                        fldl(mv);
                        break;

                    case LDH:
                        fldh(mv);
                        break;

                    case RND:
                        frnd(mv);
                        break;

                    /*
                    Instrucciones que faltan con...
                        2 operandos: LDH, LDL, RND
                        1 op: SYS
                    */
                    default:
                        errorHandler(mv, ERR_INS);
                        break;
                }
                mv->reg[IP] += 1 + (mv->reg[OP1] >> 24) + (mv->reg[OP2] >> 24);
                if (mv->reg[OPC] == SYS) {
                    printf("%x\n", mv->mem[77]);
                }
            }
        }
    }

    if (!(inCS(mv, mv->reg[IP])) && mv->reg[OPC] != STOP)
        errorHandler(mv, ERR_SEG);
}
