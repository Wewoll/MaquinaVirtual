#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#define RAM_SIZE   (16 * 1024)  // 16 KiB
#define REG_AMOUNT 32           // 32 registros
#define SEG_AMOUNT 2            // 2 descriptores de segmentos

#define HEADER_RANGE 8          // Primeros bytes de cabecera del .vmx

// Constantes para el Disassambler
#define MNEM_WIDTH 8
#define OP1_WIDTH  14
#define OP2_WIDTH  10

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

//Tamanos usados
typedef int8_t Byte;
typedef uint8_t UByte;
typedef int16_t TwoBytes;
typedef uint16_t UTwoBytes;
typedef int32_t Register;
typedef uint32_t URegister;

//Tabla de descriptores de segmentos
typedef struct {
    UTwoBytes base;   // dirección lógica de inicio
    UTwoBytes size;   // tamaño del segmento en bytes
} TableSeg;

//Maquina Virtual
typedef struct {
    UByte    mem[RAM_SIZE];       // 16 KiB de RAM
    Register reg[REG_AMOUNT];     // 32 registros de 4 bytes
    TableSeg seg[SEG_AMOUNT];     // tabla de segmentos: 0 = CS, 1 = DS
    UByte    flag;                // error flag
} TMV;

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

static const char* regStr[32] = {
    "LAR",      // 0
    "MAR",      // 1
    "MBR",      // 2
    "IP",       // 3
    "OPC",      // 4
    "OP1",      // 5
    "OP2",      // 6
    "INVALID",  // 7
    "INVALID",  // 8
    "INVALID",  // 9
    "EAX",      // 10
    "EBX",      // 11
    "ECX",      // 12
    "EDX",      // 13
    "EEX",      // 14
    "EFX",      // 15
    "AC",       // 16
    "CC",       // 17
    "INVALID",  // 18
    "INVALID",  // 19
    "INVALID",  // 20
    "INVALID",  // 21
    "INVALID",  // 22
    "INVALID",  // 23
    "INVALID",  // 24
    "INVALID",  // 25
    "CS",       // 26
    "DS",       // 27
    "INVALID",  // 28
    "INVALID",  // 29
    "INVALID",  // 30
    "INVALID"   // 31
};

//Define de instrucciones
typedef enum {
    SYS = 0x00, JMP, JZ, JP, JN, JNZ, JNP, JNN, NOT,
    STOP = 0x0F,
    MOV = 0x10, ADD, SUB, MUL, DIV, CMP, SHL, SHR, SAR, AND, OR, XOR, SWAP, LDL, LDH, RND
} OpCode;

static const char* opStr[32] = {
    "SYS",      // 0x00
    "JMP",      // 0x01
    "JZ",       // 0x02
    "JP",       // 0x03
    "JN",       // 0x04
    "JNZ",      // 0x05
    "JNP",      // 0x06
    "JNN",      // 0x07
    "NOT",      // 0x08
    "INVALID",  // 0x09
    "INVALID",  // 0x0A
    "INVALID",  // 0x0B
    "INVALID",  // 0x0C
    "INVALID",  // 0x0D
    "INVALID",  // 0x0E
    "STOP",     // 0x0F
    "MOV",      // 0x10
    "ADD",      // 0x11
    "SUB",      // 0x12
    "MUL",      // 0x13
    "DIV",      // 0x14
    "CMP",      // 0x15
    "SHL",      // 0x16
    "SHR",      // 0x17
    "SAR",      // 0x18
    "AND",      // 0x19
    "OR",       // 0x1A
    "XOR",      // 0x1B
    "SWAP",     // 0x1C
    "LDL",      // 0x1D
    "LDH",      // 0x1E
    "RND"       // 0x1F
};

//Prototipos
void executeProgram(TMV* mv);
void errorHandler(TMV* mv, int err);
void readFile(TMV *mv, const char *filename);
void initializationTable(TMV* mv, TwoBytes codeSize);
void initializationReg(TMV* mv);
void disASMOP(TMV* mv, Register operand, Byte type);
void disASMOPStr(TMV* mv, Register operand, Byte type, int* len);
void disASM(TMV* mv);
Register decodeAddr(TMV* mv, Register logical);
int inSegment(TMV* mv, Register logical);
int inCS(TMV* mv, Register logical);
int inDS(TMV* mv, Register logical);
void fetchInstruction(TMV* mv);
Register fetchOperand(TMV* mv, int bytes, int* offset);
void fetchOperators(TMV* mv);

//Prototipos centrados a operaciones
void setLAR(TMV* mv, Register operand);
void setMAR(TMV* mv, Register cantBytes, Register logical);
void getMemory(TMV* mv);
void setMemory(TMV* mv);
Register getOP(TMV* mv, Register operand);
void setOP(TMV* mv, Register operandA, Register operandB);
void setCC(TMV* mv, Register valor);
Register binToDecC2(char *binStr);
void fsysRead(TMV* mv);
void decToBinC2(Register value, char *binStr);
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

//Tipo puntero a Funcion
typedef void (*InstrFunc)(TMV*);

// Wrappers para instrucciones de salto y otras especiales
void instr_jmp(TMV* mv)      { fjmp(mv, 1); }
void instr_jz(TMV* mv)       { fjmp(mv, fjz(mv)); }
void instr_jp(TMV* mv)       { fjmp(mv, !(fjz(mv)) && !(fjn(mv))); }
void instr_jn(TMV* mv)       { fjmp(mv, fjn(mv)); }
void instr_jnz(TMV* mv)      { fjmp(mv, !fjz(mv)); }
void instr_jnp(TMV* mv)      { fjmp(mv, fjz(mv) || fjn(mv)); }
void instr_jnn(TMV* mv)      { fjmp(mv, !fjn(mv)); }
void instr_invalid(TMV* mv)  { errorHandler(mv, ERR_INS); }

//Tabla de punteros a funciones
InstrFunc instrTable[32] = {
    fsys,           // 0x00 SYS
    instr_jmp,      // 0x01 JMP
    instr_jz,       // 0x02 JZ
    instr_jp,       // 0x03 JP
    instr_jn,       // 0x04 JN
    instr_jnz,      // 0x05 JNZ
    instr_jnp,      // 0x06 JNP
    instr_jnn,      // 0x07 JNN
    fnot,           // 0x08 NOT
    instr_invalid,  // 0x09
    instr_invalid,  // 0x0A
    instr_invalid,  // 0x0B
    instr_invalid,  // 0x0C
    instr_invalid,  // 0x0D
    instr_invalid,  // 0x0E
    fstop,          // 0x0F STOP
    fmov,           // 0x10 MOV
    fadd,           // 0x11 ADD
    fsub,           // 0x12 SUB
    fmul,           // 0x13 MUL
    fdiv,           // 0x14 DIV
    fsub,           // 0x15 CMP (es sub, pero sin set)
    fshl,           // 0x16 SHL
    fshr,           // 0x17 SHR
    fsar,           // 0x18 SAR
    fand,           // 0x19 AND
    f_or,           // 0x1A OR
    fxor,           // 0x1B XOR
    fswap,          // 0x1C SWAP
    fldl,           // 0x1D LDL
    fldh,           // 0x1E LDH
    frnd            // 0x1F RND
};

//Lectura del archivo, disassambler y ejecucion
int main(int argc, char *argv[]) {
    TMV mv;

    mv.flag = 0;
    srand(time(NULL));

    if (argc < 2) {
        errorHandler(&mv, ERR_EXE);
    }
    else {
        readFile(&mv, argv[1]);
        if (mv.flag == 0) {
            if ((argc == 3) && (strcmp(argv[2], "-d") == 0)) {
                disASM(&mv);
            }
            executeProgram(&mv);
        }
    }

    return mv.flag;
}

//Flujo principal de ejecucion
void executeProgram(TMV* mv) {
    while (mv->flag == 0 && inCS(mv, mv->reg[IP])) {
        fetchInstruction(mv);
        fetchOperators(mv);
        mv->reg[IP] += 1 + (mv->reg[OP1] >> 24) + (mv->reg[OP2] >> 24);
        if(mv->flag == 0) {
            instrTable[mv->reg[OPC]](mv);
        }
    }

    if (!(inCS(mv, mv->reg[IP])) && mv->reg[OPC] != STOP)
        errorHandler(mv, ERR_SEG);
}

//Manejo de errores
void errorHandler(TMV* mv, int err) {
    mv->flag = err;
    fprintf(stderr, "%s", errorMsgs[err]);
}

//Lee el archivo
void readFile(TMV* mv, const char* filename) {
    FILE *arch;
    UByte header[HEADER_RANGE];
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
            codeSize = ((UTwoBytes) header[6] << 8) | header[7];
            if (codeSize >= RAM_SIZE)
                errorHandler(mv, ERR_CODSIZE);
            else {
                //Carga del codigo en la memoria principal e inicializar
                fread(mv->mem, 1, codeSize, arch);
                initializationTable(mv, codeSize);
                initializationReg(mv);
            }
        }
        fclose(arch);
    }
}


//Inicio de la tabla de descriptores de segmentos
void initializationTable(TMV* mv, TwoBytes codeSize) {
    mv->seg[CS_SEG].base = 0;
    mv->seg[CS_SEG].size = codeSize;
    mv->seg[DS_SEG].base = codeSize;
    mv->seg[DS_SEG].size = RAM_SIZE - codeSize;
}

//Inicio de las pocisiones de CS, DS e IP
void initializationReg(TMV* mv) {
    mv->reg[CS] = CS_INI;
    mv->reg[DS] = DS_INI;
    mv->reg[IP] = mv->reg[CS];
}

void disASMOP(TMV* mv, Register operand, Byte type) {
    int i = type - 1;
    UByte aux = 0;

    for (; i >= 0; i--) {
        aux = (UByte) (operand >> (8 * i));
        printf("%02X ", aux);
    }
}

void disASMOPStrToBuf(TMV* mv, Register operand, Byte type, char* buf, size_t buflen) {
    operand &= MASK_UNTYPE;
    switch (type) {
        case 1:
            snprintf(buf, buflen, "%s", regStr[operand]);
            break;
        case 2:
            if (1 <= mv->reg[OPC] && mv->reg[OPC] <= 7)
                snprintf(buf, buflen, "0x%04X", (TwoBytes) operand);
            else
                snprintf(buf, buflen, "%d", (TwoBytes) operand);
            break;
        case 3: {
            char tmp[32];
            snprintf(tmp, sizeof(tmp), "[%s", regStr[operand >> 16]);
            size_t len = strlen(tmp);
            operand = (Register)((TwoBytes)operand);
            if (operand > 0)
                strncat(tmp, "+", sizeof(tmp) - len - 1);
            if (operand != 0) {
                char num[16];
                snprintf(num, sizeof(num), "%d", operand);
                strncat(tmp, num, sizeof(tmp) - strlen(tmp) - 1);
            }
            strncat(tmp, "]", sizeof(tmp) - strlen(tmp) - 1);
            snprintf(buf, buflen, "%s", tmp);
            break;
        }
    }
}

void disASM(TMV* mv) {
    int i;
    UByte ins = 0, typA = 0, typB = 0, typInsA = 0, typInsB = 0;
    char op1Str[32], op2Str[32], mnemStr[16];

    while (inCS(mv, mv->reg[IP])) {
        fetchInstruction(mv);
        fetchOperators(mv);

        // --- Primer mitad: posicion de memoria y bytes en hexadecimal ---
        // Memoria
        printf("[%04X] ", decodeAddr(mv, mv->reg[IP]));

        // Formacion del byte de instruccion
        typA = (UByte)(mv->reg[OP1] >> 24);
        typB = (UByte)(mv->reg[OP2] >> 24);
        typInsA = typA << 6;
        typInsB = typB << 6;
        if (typB != 0)
            typInsA >>= 2;
        ins = (UByte) mv->reg[OPC] | typInsB | typInsA;

        // Bytes de instruccion
        printf("%02X ", ins);
        disASMOP(mv, mv->reg[OP2], typB);
        disASMOP(mv, mv->reg[OP1], typA);
        for (i = 6 - typA - typB; i > 0; i--)
            printf("   ");

        // --- Segunda mitad: mnemonico y operandos alineados ---
        snprintf(mnemStr, sizeof(mnemStr), "%s", opStr[mv->reg[OPC]]);
        op1Str[0] = 0;
        op2Str[0] = 0;
        disASMOPStrToBuf(mv, mv->reg[OP1], typA, op1Str, sizeof(op1Str));
        disASMOPStrToBuf(mv, mv->reg[OP2], typB, op2Str, sizeof(op2Str));

        if (typB != 0) {
            // OP1 alineado a la derecha, coma pegada, OP2 alineado a la derecha
            printf("|  %-*s%*s,%*s\n", MNEM_WIDTH, mnemStr, OP1_WIDTH, op1Str, OP2_WIDTH, op2Str);
        } else {
            // Solo OP1 alineado a la derecha
            printf("|  %-*s%*s\n", MNEM_WIDTH, mnemStr, OP1_WIDTH, op1Str);
        }

        mv->reg[IP] += 1 + (mv->reg[OP1] >> 24) + (mv->reg[OP2] >> 24);
    }

    initializationReg(mv);
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
    Register logical, physical, temp = 0, i = bytes;

    while (i > 0 && mv->flag == 0) {
        logical = mv->reg[IP] + (*offset);
        if (!inCS(mv, logical))
            errorHandler(mv, ERR_SEG);
        else {
            physical = decodeAddr(mv, logical);
            temp |= ((Register)mv->mem[physical]) << (8 * (i - 1));
            ++(*offset);
            i--;
        }
    }

    if (bytes == 2 && (temp & 0x00008000))
        temp |= 0x00FF0000;

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

    if (mv->reg[OPC] != SYS) {
        cod = (Byte) ((operand & MASK_REG) >> 16);
        offset = (TwoBytes) (operand & MASK_OFFSET);
        logical = mv->reg[cod] + offset;
    }
    else
        logical = operand;
    if (!inSegment(mv, logical))
        errorHandler(mv, ERR_SEG);
    mv->reg[LAR] = logical;
}

//Seteo del valor del MAR cuando se trabaja con memoria
void setMAR(TMV* mv, Register cantBytes, Register logical) {
    Register physical;

    if (mv->flag == 0) {
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

        for (i = 0; i < cantBytes; i++) {
            temp |= (Register) mv->mem[physical + i] << (8 * (cantBytes - 1 - i));
        }

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
            res = operand;
            if (res & 0x00800000)
                res |= 0xFF000000;
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
    if (valor == 0)
        mv->reg[CC] |= 0x40000000;
    else
        mv->reg[CC] &= 0xBFFFFFFF;

    if (valor < 0)
        mv->reg[CC] |= 0x80000000;
    else
        mv->reg[CC] &= 0x7FFFFFFF;
}

// Recibe un número binario en complemento a 2 en formato String, y devulve su valor decimal
Register binToDecC2(char *binStr) {
    int len = strlen(binStr);
    Register value = 0;

    for (int i = 0; i < len; i++) {
        value = (value << 1) | (binStr[i] - '0');
    }

    if (binStr[0] == '1') {
        value -= (1 << len);
    }

    return value;
}

//SYS Read
void fsysRead(TMV* mv) {
    Register read = 0;
    char car, binStr[33];
    int cantBytes, i;

    cantBytes = (mv->reg[ECX] & MASK_LDH) >> 16;

    printf("[%04X]: ", decodeAddr(mv, mv->reg[LAR]));
    switch (mv->reg[EAX]) {
        case 16:
            scanf("%32s", binStr);
            read = binToDecC2(binStr);
            break;
        case 8:
            scanf("%X", &read);
            break;
        case 4:
            scanf("%o", &read);
            break;
        case 2:
            // Leer varios caracteres y empaquetar en Register (de más significativo a menos)
            for (i = cantBytes - 1; i >= 0; i--) {
                scanf(" %c", &car);
                read |= ((Register)car) << (8 * i);
            }
            break;
        case 1:
            scanf("%d", &read);
            break;
        default: break;
    }

    mv->reg[MBR] = read;
    setMemory(mv);
}

// Recibe un número decimal y devulve su representación binaria en complemento a 2 en formato String
void decToBinC2(Register value, char *binStr) {
    URegister uValue;
    int bitIndex, stringPos, firstOneFound, bit;

    uValue = (URegister)value;
    stringPos = 0;
    firstOneFound = 0;
    bitIndex = 31;

    if (value < 0) {
        // Negativos, imprimir los 32 bits completos
        for (; bitIndex >= 0; bitIndex--) {
            bit = (uValue >> bitIndex) & 1u;
            binStr[stringPos] = bit ? '1' : '0';
            stringPos++;
        }
    } else {
        // Positivos, imprimir desde el primer '1' hasta el final
        for (; bitIndex >= 0; bitIndex--) {
            bit = (uValue >> bitIndex) & 1u;
            if (bit) {
                firstOneFound = 1;
            }
            if (firstOneFound) {
                binStr[stringPos] = bit ? '1' : '0';
                stringPos++;
            }
        }

        // Caso especial: value = 0
        if (!firstOneFound) {
            binStr[stringPos] = '0';
            stringPos++;
        }
    }

    binStr[stringPos] = '\0';
}

//SYS Write
void fsysWrite(TMV* mv) {
    Register write;
    char binStr[33];
    int cantBytes, i;

    getMemory(mv);
    write = mv->reg[MBR];
    cantBytes = (mv->reg[ECX] & MASK_LDH) >> 16;
    printf("[%04X]:", decodeAddr(mv, mv->reg[LAR]));

    if (mv->reg[EAX] & 0x10) {
        decToBinC2(write, binStr);
        printf(" 0b%s", binStr);
    }
    if (mv->reg[EAX] & 0x08)
        printf(" 0x%X", write);
    if (mv->reg[EAX] & 0x04)
        printf(" 0c%o", write);
    if (mv->reg[EAX] & 0x02) {
        printf(" ");
        // Imprime cada byte como caracter, de más significativo a menos
        for (i = cantBytes - 1; i >= 0; i--) {
            char c = (char)((write >> (8 * i)) & 0xFF);
            printf("%c", c);
        }
    }
    if (mv->reg[EAX] & 0x01)
        printf(" %d", write);

    printf("\n");
}

//Funcion SYS
void fsys(TMV* mv) {
    int cantCeldas, cantBytes, i = 0;

    cantCeldas = mv->reg[ECX] & MASK_LDL;
    cantBytes = (mv->reg[ECX] & MASK_LDH) >> 16;

    while (mv->flag == 0 && i < cantCeldas) {
        setLAR(mv, mv->reg[EDX] + cantBytes * i);
        setMAR(mv, cantBytes, mv->reg[LAR]);

        if (mv->flag == 0) {
            switch (getOP(mv, mv->reg[OP1])) {
                case 1:
                    fsysRead(mv);
                    break;
                case 2:
                    fsysWrite(mv);
                    break;
            }
            i++;
        }
    }
}

//Salto con posibilidad de condicion
void fjmp(TMV* mv, int salto) {
    if (salto) {
        mv->reg[IP] &= MASK_LDH;
        mv->reg[IP] |= (getOP(mv, mv->reg[OP1]) & MASK_LDL);
    }
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
    aux = getOP(mv, mv->reg[OP1]);
    setOP(mv, mv->reg[OP1], getOP(mv, mv->reg[OP2]));
    setOP(mv, mv->reg[OP2], aux);
}

//Instruccion LDL para hacer carga baja
void fldl(TMV* mv) {
    Register cargaBaja, cod;
    UByte tipo;

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

//Instruccion LDH para hacer carga alta
void fldh(TMV* mv) {
    Register cargaAlta, cod;
    UByte tipo;

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

//Instruccion RND para dar un numero random
void frnd(TMV* mv) {
    Register max, randN;
    int r, lim;

    max = getOP(mv, mv->reg[OP2]);
    if (max <= 1) {
        randN = 0;
    } else {
        lim = RAND_MAX - (RAND_MAX % max);
        do {
            r = rand();
        } while (r >= lim);
        randN = (Register) (r % max);
    }
    setOP(mv, mv->reg[OP1], randN);
}
