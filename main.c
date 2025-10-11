#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>                 // Necesario para definicion de tamanos
#include <string.h>                 // Necesario para manejo de strings
#include <stddef.h>                 // Necesario para el uso del macro offsetof, el mapeo de los segmentos
#include <time.h>                   // Necesario para funcion random

// Tamanos de la MV
#define RAM_KIB         16                   // Maximo de RAM en KiB
#define RAM_MAX         (RAM_KIB * 1024)     // Maximo de RAM en Bytes
#define REG_AMOUNT      32                   // 32 Registros
#define SEG_AMOUNT      8                    // 6 Descriptores de segmentos
#define SEG_MAXSIZE     65535                // 2^16 tamano maximo de segmento

// Cantidad de segmentos en base al mapa del layout de la memoria
#define NUM_SEGMENTS     (sizeof(segmentLayoutMap) / sizeof(SegmentLayout))

// Cantidades de bytes de header del .vmx
#define HEADER_P_RANGE 18
#define HEADER_P_COMUN 8
#define HEADER_P_V2    10

// Constantes para el Disassambler
#define MNEM_WIDTH 8
#define OP1_WIDTH  14
#define OP2_WIDTH  10

// Mascaras para extraer los OP
#define MASKB_OPC 0b00011111
#define MASKB_OP1 0b00110000
#define MASKB_OP2 0b11000000

// Mascaras para CC
#define CC_N 0x80000000
#define CC_Z 0x40000000

// Mascaras varias
#define MASK_SEG    0xFFFF0000
#define MASK_UNTYPE 0x00FFFFFF
#define MASK_REG    0x00FF0000
#define MASK_OFFSET 0x0000FFFF
#define MASK_PHY    0x0000FFFF
#define MASK_LDL    0x0000FFFF
#define MASK_LDH    0xFFFF0000
#define MASK_SETMEM 0xFF

// Valor que se asigna a IP cuando la instruccion es STOP
#define STOP_VALUE  0xFFFFFFFF

// Tamanos usados dentro del codigo
typedef int8_t Byte;
typedef uint8_t UByte;
typedef int16_t Word;
typedef uint16_t UWord;
typedef int32_t Long;
typedef uint32_t ULong;

// Tabla de descriptores de segmentos
typedef struct {
    UWord base;   // dirección lógica de inicio
    UWord size;   // tamaño del segmento en bytes
} TableSeg;

// Estructura para almacenar los parámetros de la línea de comandos
typedef struct {
    char*       vmxPath;
    char*       vmiPath;
    Long        memoryKiB;
    Byte        disassemblerFlag;
    Long        paramsArgc;          // Cantidad de parámetros para el programa
    char**      paramsArgv;          // Puntero a los parámetros en el argv original
} MVConfig;

// Maquina Virtual
typedef struct {
    UByte*     mem;                 // Memoria dinamica
    ULong      totalMemorySize;     // Tamaño total de la memoria en bytes
    Long       reg[REG_AMOUNT];     // 32 registros de 4 bytes
    TableSeg   seg[SEG_AMOUNT];     // Tabla de segmentos
    MVConfig   config;              // Configuracion de la MV
    UByte      flag;                // Error flag
} TMV;

// Almacena los tamaños de los segmentos leídos del header del .vmx
typedef struct {
    UWord codeSegSize;
    UWord dataSegSize;
    UWord extraSegSize;
    UWord stackSegSize;
    UWord constSegSize;
    UWord paramSegSize;
} SegmentSizes;

// Estructura que describe la metadata de un segmento
typedef struct {
    Long    reg;          // El enum del registro de segmento (ej: PS, KS, CS)
    ULong   size_offset;  // El offset del campo de tamaño dentro de la struct SegmentSizes
} SegmentLayout;

// El "mapa" que es la ÚNICA FUENTE DE LA VERDAD para el layout de la memoria.
// Define el orden físico y cómo encontrar el tamaño y el registro de cada segmento.
static const SegmentLayout segmentLayoutMap[] = {
    { PS, offsetof(SegmentSizes, paramSegSize)  },
    { KS, offsetof(SegmentSizes, constSegSize)  },
    { CS, offsetof(SegmentSizes, codeSegSize)   },
    { DS, offsetof(SegmentSizes, dataSegSize)   },
    { ES, offsetof(SegmentSizes, extraSegSize)  },
    { SS, offsetof(SegmentSizes, stackSegSize)  }
};

// Enumeracion de errores
typedef enum {
    ERR_EXE = 1,
    ERR_FOPEN,
    ERR_SEG,
    ERR_INS,
    ERR_DIV0,
    ERR_MEM
} ErrNumber;

// Vector de mensajes de errores
static const char* errorMsgs[] = {
    NULL,                                                                                   // indice 0 (sin error)
    "Uso: vmx [filename.vmx] [filename.vmi] [m=M] [-d] [-p param1 param2 ... paramN]\n",    // 1
    "Error: No se pudo abrir el archivo.\n",                                                // 2
    "Error: Fuera de los limites del segmento.\n",                                          // 3 - pedido en el pdf v1
    "Error: Intruccion invalida.\n",                                                        // 4 - pedido en el pdf v1
    "Error: No se puede dividir por 0.\n"                                                   // 5 - pedido en el pdf v1
    "Error: Memoria insuficiente.\n"                                                        // 6 - pedido en el pdf v2
};

// Enumeracion de registros
typedef enum {
    LAR = 0, MAR, MBR,
    IP = 3, OPC, OP1, OP2,
    SP = 7, BP,
    EAX = 10, EBX, ECX, EDX, EEX, EFX,
    AC = 16,
    CC = 17,
    CS = 26, DS, ES, SS, KS, PS
} RegName;

// Vector de nombres de registros
static const char* regStr[32] = {
    "LAR",      // 0
    "MAR",      // 1
    "MBR",      // 2
    "IP",       // 3
    "OPC",      // 4
    "OP1",      // 5
    "OP2",      // 6
    "SP",       // 7
    "BP",       // 8
    "RESERVED", // 9
    "EAX",      // 10
    "EBX",      // 11
    "ECX",      // 12
    "EDX",      // 13
    "EEX",      // 14
    "EFX",      // 15
    "AC",       // 16
    "CC",       // 17
    "RESERVED", // 18
    "RESERVED", // 19
    "RESERVED", // 20
    "RESERVED", // 21
    "RESERVED", // 22
    "RESERVED", // 23
    "RESERVED", // 24
    "RESERVED", // 25
    "CS",       // 26
    "DS",       // 27
    "ES",       // 28
    "SS",       // 29
    "KS",       // 30
    "PS"        // 31
};

// Enumeracion de instrucciones
typedef enum {
    SYS = 0x00, JMP, JZ, JP, JN, JNZ, JNP, JNN, NOT,
    PUSH = 0x0B, POP, CALL,
    RET = 0x0E, STOP,
    MOV = 0x10, ADD, SUB, MUL, DIV, CMP, SHL, SHR, SAR, AND, OR, XOR, SWAP, LDL, LDH, RND
} OpCode;

// Vector de mnemonicos de instrucciones
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
    "PUSH",     // 0x0B
    "POP",      // 0x0C
    "CALL",     // 0x0D
    "RET",      // 0x0E
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

// --- Prototipos ---
void executeProgram(TMV* mv);
void errorHandler(TMV* mv, int err);
void parseCommandLine(int argc, char* argv[], TMV* mv);
void readProgramHeader(TMV* mv, SegmentSizes* segSizes, UWord* offsetEP)
void loadProgramData(TMV* mv, SegmentSizes* segSizes);
ULong sumSegSizes(SegmentSizes* segSizes);
ULong calculatePSSize(TMV* mv);
void createParamSegment(TMV* mv);
void initializeTable(TMV* mv, SegmentSizes* segSizes);
void initializeRegisters(TMV* mv, SegmentSizes* segSizes, UWord offsetEP);

// Prototipos del disassambler
void disASMOP(TMV* mv, Long operand, Byte type);
void disASMOPStr(TMV* mv, Long operand, Byte type, int* len);
void disASM(TMV* mv);

// Prototipos de ubicacion fisica
Long decodeAddr(TMV* mv, Long logical);
int inSegment(TMV* mv, Long logical);
int inCS(TMV* mv, Long logical);
int inDS(TMV* mv, Long logical);

// Prototipos de fetch
void fetchInstruction(TMV* mv);
Long fetchOperand(TMV* mv, int bytes, int* offset);
void fetchOperators(TMV* mv);
void addIP(TMV* mv);

// Prototipos de memoria
void setLAR(TMV* mv, Long operand);
void setMAR(TMV* mv, Long cantBytes, Long logical);
void getMemory(TMV* mv);
void setMemory(TMV* mv);

// Prototipos centrados a operadores
Long getOP(TMV* mv, Long operand);
void setOP(TMV* mv, Long operandA, Long operandB);
void setCC(TMV* mv, Long valor);

// Prototipos de SYS
void fsys(TMV* mv, Long* value1, Long* value2);
void fsysRead(TMV* mv);
void decToBinC2(Long value, char *binStr);
void fsysWrite(TMV* mv);
void fsysStrread(TMV* mv);
void fsysStrwrite(TMV* mv);
void fsysClrscr(TMV* mv);
void fsysBreak(TMV* mv);

// Prototipos de las instrucciones
void fmsl(TMV* mv, Long* value1, Long* value2);
Long fjz(TMV* mv);
Long fjn(TMV* mv);
void fnot(TMV* mv, Long* value1, Long* value2);
void fstop(TMV* mv, Long* value1, Long* value2);
void fmov(TMV* mv, Long* value1, Long* value2);
void fadd(TMV* mv, Long* value1, Long* value2);
void fsub(TMV* mv, Long* value1, Long* value2);
void fmul(TMV* mv, Long* value1, Long* value2);
void fdiv(TMV* mv, Long* value1, Long* value2);
void fshl(TMV* mv, Long* value1, Long* value2);
void fshr(TMV* mv, Long* value1, Long* value2);
void fsar(TMV* mv, Long* value1, Long* value2);
void fand(TMV* mv, Long* value1, Long* value2);
void f_or(TMV* mv, Long* value1, Long* value2);
void fxor(TMV* mv, Long* value1, Long* value2);
void fswap(TMV* mv, Long* value1, Long* value2);
void fldl(TMV* mv, Long* value1, Long* value2);
void fldh(TMV* mv, Long* value1, Long* value2);
void frnd(TMV* mv, Long* value1, Long* value2);

// Puntero a funciones de instruccion
typedef void (*InstrFunc)(TMV*, Long*, Long*);

typedef struct {
    UByte       fetchValue;     // Opcional: getOperand
    InstrFunc   execute;        // La función principal
    UByte       setearCC;       // Opcional: setCC
    UByte       setValue;       // Opcional: setOperand
} Instruction;

// Wrapper para error "Intruccion invalida"
void finvalid(TMV* mv, Long* value1, Long* value2)  { errorHandler(mv, ERR_INS); }

static const Instruction instrTable[32] = {
    [SYS] =  { .fetchValue = 1, .execute = fsys, .setearCC = 0, .setValue = 0 },     // 0x00
    [JMP] =  { .fetchValue = 1, .execute = fmsl, .setearCC = 0, .setValue = 0 },     // 0x01
    [JZ] =   { .fetchValue = 1, .execute = fmsl, .setearCC = 0, .setValue = 0 },     // 0x02
    [JP] =   { .fetchValue = 1, .execute = fmsl, .setearCC = 0, .setValue = 0 },     // 0x03
    [JN] =   { .fetchValue = 1, .execute = fmsl, .setearCC = 0, .setValue = 0 },     // 0x04
    [JNZ] =  { .fetchValue = 1, .execute = fmsl, .setearCC = 0, .setValue = 0 },     // 0x05
    [JNP] =  { .fetchValue = 1, .execute = fmsl, .setearCC = 0, .setValue = 0 },     // 0x06
    [JNN] =  { .fetchValue = 1, .execute = fmsl, .setearCC = 0, .setValue = 0 },     // 0x07
    [NOT] =  { .fetchValue = 1, .execute = fnot, .setearCC = 1, .setValue = 1 },     // 0x08

    [9]   =  { .fetchValue = 0, .execute = finvalid, .setearCC = 0, .setValue = 0 }, // 0x09
    [10]  =  { .fetchValue = 0, .execute = finvalid, .setearCC = 0, .setValue = 0 }, // 0x0A

    //[PUSH] = { .fetchValue = 1, .execute = fpush, .setearCC = 0, .setValue = 0 },    // 0x0B
    //[POP] =  { .fetchValue = 1, .execute = fpop, .setearCC = 0, .setValue = 0 },     // 0x0C
    //[CALL] = { .fetchValue = 1, .execute = fcall, .setearCC = 0, .setValue = 0 },    // 0x0D

    //[RET] =  { .fetchValue = 0, .execute = fret, .setearCC = 0, .setValue = 0 },     // 0x0E
    [STOP] = { .fetchValue = 0, .execute = fstop, .setearCC = 0, .setValue = 0 },    // 0x0F

    [MOV] =  { .fetchValue = 2, .execute = fmov, .setearCC = 0, .setValue = 1 },     // 0x10
    [ADD] =  { .fetchValue = 2, .execute = fadd, .setearCC = 1, .setValue = 1 },     // 0x11
    [SUB] =  { .fetchValue = 2, .execute = fsub, .setearCC = 1, .setValue = 1 },     // 0x12
    [MUL] =  { .fetchValue = 2, .execute = fmul, .setearCC = 1, .setValue = 1 },     // 0x13
    [DIV] =  { .fetchValue = 2, .execute = fdiv, .setearCC = 1, .setValue = 1 },     // 0x14
    [CMP] =  { .fetchValue = 2, .execute = fsub, .setearCC = 1, .setValue = 0 },     // 0x15
    [SHL] =  { .fetchValue = 2, .execute = fshl, .setearCC = 1, .setValue = 1 },     // 0x16
    [SHR] =  { .fetchValue = 2, .execute = fshr, .setearCC = 1, .setValue = 1 },     // 0x17
    [SAR] =  { .fetchValue = 2, .execute = fsar, .setearCC = 1, .setValue = 1 },     // 0x18
    [AND] =  { .fetchValue = 2, .execute = fand, .setearCC = 1, .setValue = 1 },     // 0x19
    [OR] =   { .fetchValue = 2, .execute = f_or, .setearCC = 1, .setValue = 1 },     // 0x1A
    [XOR] =  { .fetchValue = 2, .execute = fxor, .setearCC = 1, .setValue = 1 },     // 0x1B
    [SWAP] = { .fetchValue = 2, .execute = fswap, .setearCC = 0, .setValue = 2 },    // 0x1C
    [LDL] =  { .fetchValue = 2, .execute = fldl, .setearCC = 0, .setValue = 1 },     // 0x1D
    [LDH] =  { .fetchValue = 2, .execute = fldh, .setearCC = 0, .setValue = 1 },     // 0x1E
    [RND] =  { .fetchValue = 2, .execute = frnd, .setearCC = 0, .setValue = 1 },     // 0x1F
};

// Puntero a funciones del SYS
typedef void (*SysFunc)(TMV*);

static const SysFunc sysTable[16] = {
    [0x01] = fsysRead,
    [0x02] = fsysWrite,
    //[0x03] = fsysStrread,
    //[0x04] = fsysStrwrite,
    //[0x07] = fsysClrscr,
    [0x0F] = fsysBreak,
};

// Puntero a funciones de condicion
typedef Long (*CondFunc)(TMV*);

// Wrappers de salto
Long condTrue(TMV* mv) { return 1; }
Long condZ(TMV* mv)    { return fjz(mv); }
Long condP(TMV* mv)    { return !fjz(mv) && !fjn(mv); }
Long condN(TMV* mv)    { return fjn(mv); }
Long condNZ(TMV* mv)   { return !fjz(mv); }
Long condNP(TMV* mv)   { return fjz(mv) || fjn(mv); }
Long condNN(TMV* mv)   { return !fjn(mv); }

static const CondFunc condVector[8] = {
    [JMP] = condTrue,
    [JZ]  = condZ,
    [JP]  = condP,
    [JN]  = condN,
    [JNZ] = condNZ,
    [JNP] = condNP,
    [JNN] = condNN,
};



/// --- CODIGO ---
/**
 * @brief
 */
int main(int argc, char *argv[]) {
    TMV mv;
    SegmentSizes segSizes;
    UWord offsetEP;
    ULong psSize;

    mv.flag = 0;
    srand(time(NULL));      // Para la instruccion RND

    // Parsear los argumentos y llenar la estructura config
    parseCommandLine(argc, argv, &mv);

    if (mv.flag == 0) {
        // --- RESERVA DE MEMORIA DINÁMICA ---
        mv.totalMemorySize = mv.config.memoryKiB * 1024;
        mv.mem = (UByte*) malloc(mv.totalMemorySize);

        if (mv.mem == NULL)
            errorHandler(&mv, ERR_MEM);
        else {
            // --- Lógica de Carga ---
            // Calcular el size de PS
            psSize = calculatePSSize(&mv);
            if (psSize > SEG_MAXSIZE)
                errorHandler(&mv, ERR_MEM);
            else {
                segSizes.paramSegSize = psSize;
                createParamSegment(&mv);

                // Si hay un .vmx, se carga el programa
                if (mv.config.vmxPath != NULL) {
                    // Inicializacion en 0 de tamanos de segmentos
                    segSizes.extraSegSize = 0;
                    segSizes.stackSegSize = 0;
                    segSizes.constSegSize = 0;

                    // Inicializacion en 0 del Entry Point
                    offsetEP = 0;

                    // Lectura del header del .vmx
                    readProgramHeader(&mv, &segSizes, &offsetEP);

                    // Suma de los sizes para ver si sobrepasan el total de memoria
                    if (sumSegSizes(&segSizes) > mv.totalMemorySize)
                        errorHandler(mv, ERR_MEM);
                    else {
                        // Inicializaciones de tabla de segmentos y registros
                        initializeTable(&mv, &segSizes);
                        initializeRegisters(&mv, &segSizes, offsetEP);

                        // Carga de memoria
                        loadProgramData(&mv, &segSizes);
                    }
                }

                // Si no hay .vmx pero sí .vmi, se carga la imagen
                else if (mv.config.vmiPath != NULL) {
                    readImage(&mv);
                }
            }

            // --- Lógica de Ejecución ---
            if (mv.flag == 0) {
                // Disassambler
                if (mv.config.disassemblerFlag) {
                    disASM(&mv);
                }

                // Ejecucion del programa
                executeProgram(&mv);
            }
        }
    }

    // Liberamos la memoria
    if (mv.mem != NULL) {
        free(mv.mem);
    }

    return mv.flag;
}

// Flujo principal de ejecucion
void executeProgram(TMV* mv) {
    //fpush
    //fpush(mv, mv->config.paramArgc, 0);
    //fpush(mv, -1, 0);
    while (mv->flag == 0 && inCS(mv, mv->reg[IP])) {
        executeOneInstruction(mv);
    }
}

void executeOneInstruction(TMV* mv) {
    Instruction instr;
    Long value1, value2;

    fetchInstruction(mv);
    fetchOperators(mv);
    addIP(mv);
    instr = instrTable[mv->reg[OPC]];

    if (mv->flag == 0) {
        if (instr.fetchValue > 0) {
            value1 = getOP(mv, mv->reg[OP1]);
            if (instr.fetchValue == 2)
                value2 = getOP(mv, mv->reg[OP2]);
        }

        instr.execute(mv, &value1, &value2);
    }

    if (mv->flag == 0) {
        if (instr.setearCC == 1)
            setCC(mv, value1);
        if (instr.setValue > 0) {
            setOP(mv, mv->reg[OP1], value1);
            if (instr.setValue == 2)
                setOP(mv, mv->reg[OP2], value2);
        }
    }
}

 /**
 * @brief Recibe un int corresponsinte al error y lo comunica el mensaje adecuado de acuerdo a un vector de errores
 */
void errorHandler(TMV* mv, int err) {
    mv->flag = err;
    fprintf(stderr, "%s", errorMsgs[err]);
}

 /**
 * @brief Parsea la informacion recibida por parametros en config
 */
void parseCommandLine(int argc, char* argv[], TMV* mv) {
    int i;

    // Inicializar con valores por defecto
    mv->config.vmxPath = NULL;
    mv->config.vmiPath = NULL;
    mv->config.memoryKiB = RAM_KIB;
    mv->config.disassemblerFlag = 0;
    mv->config.paramsArgc = 0;
    mv->config.paramsArgv = NULL;

    // Iterar y parsear
    for (i = 1; i < argc; i++) {
        if (strstr(argv[i], ".vmx")) {
            mv->config.vmxPath = argv[i];
        }
        else if (strstr(argv[i], ".vmi")) {
            mv->config.vmiPath = argv[i];
        }
        else if (strncmp(argv[i], "m=", 2) == 0) {
            sscanf(argv[i], "m=%d", &mv->config.memoryKiB);
        }
        else if (strcmp(argv[i], "-d") == 0) {
            mv->config.disassemblerFlag = 1;
        }
        else if (strcmp(argv[i], "-p") == 0) {
            mv->config.paramsArgc = argc - (i + 1);
            mv->config.paramsArgv = &argv[i + 1];
        }
    }


    // Validar
    if (mv->config.vmxPath == NULL && mv->config.vmiPath == NULL) {
        errorHandler(mv, ERR_EXE);
    }
    else if (mv->config.vmxPath == NULL) {
        mv->config.paramsArgc = 0; // Se ignoran los parámetros -p si no hay .vmx
        mv->config.paramsArgv = NULL;
    }
}

 /**
 * @brief Lee solo el HEADER de un archivo .vmx para obtener los tamaños de los segmentos (y el offsetEP).
 */
void readProgramHeader(TMV* mv, SegmentSizes* segSizes, UWord* offsetEP) {
    FILE* arch;
    UByte header[HEADER_P_RANGE];

    arch = fopen(mv->config.vmxPath, "rb");
    if (arch == NULL)
        errorHandler(mv, ERR_FOPEN);
    else {
        fread(header, 1, HEADER_P_COMUN, arch);
        segSizes->codeSegSize = ((UWord) header[6] << 8) | header[7];

        if (header[5] == 1)
            segSizes->dataSegSize = mv->totalMemorySize - segSizes->codeSegSize;
        else if (header[5] == 2) {
            fread(header + HEADER_P_COMUN, 1, HEADER_P_V2, arch);
            segSizes->dataSegSize = ((UWord) header[8] << 8) | header[9];
            segSizes->extraSegSize = ((UWord) header[10] << 8) | header[11];
            segSizes->stackSegSize = ((UWord) header[12] << 8) | header[13];
            segSizes->constSegSize = ((UWord) header[14] << 8) | header[15];
            *offsetEP = ((UWord) header[16] << 8) | header[17];
        }

        fclose(arch);
    }
}

/**
 * @brief Carga el código y las constantes desde el archivo .vmx a la memoria de la MV.
 * Debe ser llamada DESPUÉS de initializeTable.
 */
void loadProgramData(TMV* mv, SegmentSizes* segSizes) {
    FILE* arch;
    UByte version;

    arch = fopen(mv->config.vmxPath, "rb");
    if (arch == NULL)
        errorHandler(mv, ERR_FOPEN);
    else {
        fseek(arch, 5, SEEK_SET);
        version = fgetc(arch);

        fseek(arch, (version == 1 ? HEADER_P_COMUN : HEADER_P_RANGE), SEEK_SET);
        fread(mv->mem + mv->seg[decodeSeg(mv, CS)].base, 1, segSizes->codeSegSize, arch);
        if (version == 2 && segSizes->constSegSize > 0)
            fread(mv->mem + mv->seg[decodeSeg(mv, KS)].base, 1, segSizes->constSegSize, arch);

        fclose(arch);
    }
}

void writeImage(TMV* mv) {
    FILE* arch;
    char* id = "VMI25";
    UByte version = 1, i;

    arch = fopen(mv->config.vmiPath, "wb");
    if (arch == NULL)
        errorHandler(mv, ERR_FOPEN);
    else {
        // Header
        fwrite(id, 1, 5, arch);
        fwrite(version, 1, 1, arch);
        fwrite(mv->totalMemorySize, 1, 2, arch);

        // Registros
        fwrite(mv->reg, 1, 128, arch);

        // Tabla de descriptores de segmentos
        for(i = 0, i < SEG_AMOUNT, i++) {
            fwrite(mv->seg[i].base, 1, 2, arch);
            fwrite(mv->seg[i].size, 1, 2, arch);
        }

        // Memoria
        fwrite(mv->mem, 1, mv->totalMemorySize, arch);

        fclose(arch);
    }
}

void readImage(TMV* mv) {
    FILE* arch;

    arch = fopen(mv->config.vmiPath, "rb");
    if (arch == NULL)
        errorHandler(mv, ERR_FOPEN);
    else {
        fseek(arch, 6, SEEK_SET);

        // Total de memoria
        fread(&mv->totalMemorySize, 1, 2, arch);

        // Registros
        fread(mv->reg, 1, 128, arch),

        // Tabla de descriptores de segmentos
        for(i = 0, i < SEG_AMOUNT, i++) {
            fread(&mv->seg[i].base, 1, 2, arch);
            fread(&mv->seg[i].size, 1, 2, arch);
        }

        // Memoria
        fread(mv->mem, 1, mv->totalMemorySize, arch);

        fclose(arch);
    }
}

/**
 * @brief Suma los tamaños de todos los segmentos.
 * @return La suma total de los tamaños en bytes, como un ULong.
 */
ULong sumSegSizes(SegmentSizes* segSizes) {
    ULong total = 0;

    total += segSizes->codeSegSize;
    total += segSizes->dataSegSize;
    total += segSizes->extraSegSize;
    total += segSizes->stackSegSize;
    total += segSizes->constSegSize;
    total += segSizes->paramSegSize;

    return total;
}

/**
 * @brief Suma los tamaños de todos los parametros y anade el tamano del puntero que los direcciona.
 * @return El size de PS en bytes, como un ULong.
 */
ULong calculatePSSize(TMV* mv) {
    ULong psSize = 0;
    int i;

    for (i = 0; i < mv->config.paramsArgc; i++) {
        psSize += strlen(mv->config.paramsArgv[i]) + 1;
    }
    psSize += (ULong) mv->config.paramsArgc * sizeof(Long);

    return psSize;
}

void createParamSegment(TMV* mv) {
    UWord offsetStr = 0, offsetPtr = 0, cantBytes = 4, i;
    ULong ptrStr;

    if (mv->config.paramsArgc > 0) {
        for (i = 0, i < mv->config.paramsArgc, i++) {
            strcpy(&mv->mem[offsetStr], mv->config.paramsArgv[i]);
            offsetStr += strlen(mv->config.paramsArgv[i]) + 1;
        }

        offsetPtr = offsetStr;
        offsetStr = 0;
        for (i = 0, i < mv->config.paramsArgc, i++) {
            ptrStr = 0x00000000 | offsetStr;
            for (j = 0; j < cantBytes; j++)
                mv->mem[offsetStr + j] = (UByte) (ptrStr >> (8 * (cantBytes - 1 - j)) & MASK_SETMEM);
            offsetStr += strlen(mv->config.paramsArgv[i]) + 1;
            offsetPtr += cantBytes;
        }
    }
}

/**
 * @brief Construye la tabla de descriptores de segmentos en memoria.
 *
 * Su única responsabilidad es crear el "mapa" físico de la memoria
 * llenando la estructura mv->seg. No modifica ningún registro.
 * Utiliza el mapa de configuración para determinar el orden y tamaño.
 *
 * Inicializacion de la tabla de descriptores de segmentos.
 */
void initializeTable(TMV* mv, SegmentSizes* segSizes) {
    Long physicalOffset = 0, tableIndex = 0, i = 0;
    UWord currentSize;

    // Iteramos sobre el mapa que define el orden y la metadata.
    for (; i < NUM_SEGMENTS; i++) {
        // Obtenemos el tamaño del segmento actual de forma automática.
        currentSize = *(UWord*)((Byte*)segSizes + segmentLayoutMap[i].size_offset);

        if (currentSize > 0) {
            // Si el segmento tiene tamaño, le creamos una entrada en la tabla.
            mv->seg[tableIndex].base = physicalOffset;
            mv->seg[tableIndex].size = currentSize;

            physicalOffset += currentSize;
            tableIndex++;
        }
    }
}

/**
 * @brief Inicializa todos los registros de la MV.
 *
 * Configura los punteros de segmento (CS, DS, etc.) basándose en la
 * existencia de cada segmento y luego inicializa los punteros de ejecución (IP, SP).
 */
void initializeRegisters(TMV* mv, SegmentSizes* segSizes, UWord offsetEP) {
    Long i = 0, tableIndex = 0, currentReg = 0;
    UWord currentSize = 0;

    // Iteramos sobre el mapa para asignar los registros de segmento.
    for (i = 0; i < NUM_SEGMENTS; i++) {
        currentReg = segmentLayoutMap[i].reg;

        if (mv->seg[i].size > 0) {
            mv->reg[currentReg] = tableIndex << 16;
            tableIndex++;
        } else {
            mv->reg[currentReg] = -1;
        }
    }

    // Finalmente, inicializamos los punteros de ejecución (IP y SP).
    mv->reg[IP] = mv->reg[CS] | offsetEP;

    if (mv->reg[SS] != -1)
        mv->reg[SP] = mv->reg[SS] + mv->seg[mv->reg[SS] >> 16].size;
    else
        mv->reg[SP] = -1;
}

// Funcion para transformar un operando a bytes para el disassembler
void disASMOP(TMV* mv, Long operand, Byte type) {
    int i = type - 1;
    UByte aux = 0;

    for (; i >= 0; i--) {
        aux = (UByte) (operand >> (8 * i));
        printf("%02X ", aux);
    }
}

// Funcion para transformar un operando a string para el disassembler
void disASMOPStrToBuf(TMV* mv, Long operand, Byte type, char* buf, size_t buflen) {
    operand &= MASK_UNTYPE;
    switch (type) {
        case 1:
            snprintf(buf, buflen, "%s", regStr[operand]);
            break;
        case 2:
            if (1 <= mv->reg[OPC] && mv->reg[OPC] <= 7)
                snprintf(buf, buflen, "0x%04X", (Word) operand);
            else
                snprintf(buf, buflen, "%d", (Word) operand);
            break;
        case 3: {
            char tmp[32];
            snprintf(tmp, sizeof(tmp), "[%s", regStr[operand >> 16]);
            size_t len = strlen(tmp);
            operand = (Long)((Word)operand);
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

// Funcion principal del disassembler
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

Long decodeSeg(TMV* mv, Long cod) {
    return mv->reg[cod] >> 16;
}

//Decodificador de direccion logica a direccion fisica
Long decodeAddr(TMV* mv, Long logical) {
    Word segIndex, offset;

    segIndex = (Word) (logical >> 16);
    offset = (Word) logical;

    if (offset >= mv->seg[segIndex].size)
        errorHandler(mv, ERR_SEG);

    return mv->seg[segIndex].base + offset;
}

// Verifica si una dirección logica esta dentro de su segmento
int inSegment(TMV* mv, Long logical) {
    Word segIndex, offset;

    segIndex = (Word) (logical >> 16);
    offset = (Word) logical;

    return offset < mv->seg[segIndex].size;
}

// Verifica si una dirección lógica está en CS
int inCS(TMV* mv, Long logical) {
    return ((logical >> 16) == CS_SEG) && inSegment(mv, logical);
}

// Verifica si una dirección lógica está en DS
int inDS(TMV* mv, Long logical) {
    return ((logical >> 16) == DS_SEG) && inSegment(mv, logical);
}

void addIP(TMV* mv) {
    mv->reg[IP] += (Long) (1 + (mv->reg[OP1] >> 24) + (mv->reg[OP2] >> 24));
}

// Agarra un byte de instruccion. Setea OPC, el tipo de OP1 y el tipo de OP2
void fetchInstruction(TMV* mv) {
    Byte temp;

    temp = mv->mem[decodeAddr(mv, mv->reg[IP])];
    mv->reg[OPC] = 0;
    mv->reg[OPC] = (Long)(temp & MASKB_OPC);

    mv->reg[OP1] = 0;
    mv->reg[OP2] = 0;
    mv->reg[OP1] = (Long)(temp & MASKB_OP2) << 18;

    if ((temp & MASKB_OP1) != 0) {              //Si hay dos operandos, mueve la OP1 a OP2 y agarra el tipo de OP1
        mv->reg[OP2] = mv->reg[OP1];
        mv->reg[OP1] = (Long)(temp & MASKB_OP1) << 20;
    }
}

// Crea un registro para un operando y lo retorna
Long fetchOperand(TMV* mv, int bytes, int* offset) {
    Long logical, physical, temp = 0, i = bytes;

    while (i > 0 && mv->flag == 0) {
        logical = mv->reg[IP] + (*offset);
        if (!inCS(mv, logical))
            errorHandler(mv, ERR_SEG);
        else {
            physical = decodeAddr(mv, logical);
            temp |= ((Long)mv->mem[physical]) << (8 * (i - 1));
            ++(*offset);
            i--;
        }
    }

    if (bytes == 2 && (temp & 0x00008000))
        temp |= 0x00FF0000;

    return temp;
}

// Prepara a los operandos para que reciban su informacion
void fetchOperators(TMV* mv) {
    int op1Bytes, op2Bytes, offset = 1;

    op2Bytes = mv->reg[OP2] >> 24;
    op1Bytes = mv->reg[OP1] >> 24;
    mv->reg[OP2] |= fetchOperand(mv, op2Bytes, &offset);
    if (mv->flag == 0)
        mv->reg[OP1] |= fetchOperand(mv, op1Bytes, &offset);
}

// Seteo del valor del LAR cuando se trabaja con memoria
void setLAR(TMV* mv, Long operand) {
    Byte cod;
    Word offset;
    Long logical;

    if (mv->reg[OPC] != SYS) {
        cod = (Byte) ((operand & MASK_REG) >> 16);
        offset = (Word) (operand & MASK_OFFSET);
        logical = mv->reg[cod] + offset;
    }
    else
        logical = operand;
    if (!inSegment(mv, logical))
        errorHandler(mv, ERR_SEG);
    mv->reg[LAR] = logical;
}

// Seteo del valor del MAR cuando se trabaja con memoria
void setMAR(TMV* mv, Long cantBytes, Long logical) {
    Long physical;

    if (mv->flag == 0) {
        physical = decodeAddr(mv, logical);
        if (physical + cantBytes - 1 >= RAM_SIZE)
            errorHandler(mv, ERR_SEG);
        else
            mv->reg[MAR] = (cantBytes << 16) | physical;
    }
}

// Get del valor de memoria al MBR
void getMemory(TMV* mv) {
    Long cantBytes, physical, temp;
    int i;

    if (mv->flag == 0) {
        cantBytes = mv->reg[MAR] >> 16;
        physical = mv->reg[MAR] & MASK_PHY;
        temp = 0;

        for (i = 0; i < cantBytes; i++) {
            temp |= (Long) mv->mem[physical + i] << (8 * (cantBytes - 1 - i));
        }

        mv->reg[MBR] = temp;
    }
}

// Set del valor del MBR a la memoria
void setMemory(TMV* mv) {
    Long cantBytes, physical;
    int i;

    if (mv->flag == 0) {
        cantBytes = mv->reg[MAR] >> 16;
        physical = mv->reg[MAR] & MASK_PHY;

        for (i = 0; i < cantBytes; i++)
            mv->mem[physical + i] = mv->reg[MBR] >> (8 * (cantBytes - 1 - i)) & MASK_SETMEM;
    }
}

// Decodificar el operador y devolver su valor
Long getOP(TMV* mv, Long operand) {
    Byte tipo = operand >> 24;
    Long res;

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

// Setear el registro o memoria de operadorA con el valor de operadorB
void setOP(TMV* mv, Long operandA, Long operandB) {
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

// Setear el CC
void setCC(TMV* mv, Long valor) {
    if (valor == 0)
        mv->reg[CC] |= 0x40000000;
    else
        mv->reg[CC] &= 0xBFFFFFFF;

    if (valor < 0)
        mv->reg[CC] |= 0x80000000;
    else
        mv->reg[CC] &= 0x7FFFFFFF;
}

// SYS Read
void fsysRead(TMV* mv) {
    Long read = 0;
    char car, binStr[33];
    int cantCeldas, cantBytes, i = 0, j = 0;

    cantCeldas = mv->reg[ECX] & MASK_LDL;
    cantBytes = (mv->reg[ECX] & MASK_LDH) >> 16;

    while (mv->flag == 0 && i < cantCeldas) {
        setLAR(mv, mv->reg[EDX] + cantBytes * i);
        setMAR(mv, cantBytes, mv->reg[LAR]);

        if (mv->flag == 0) {
            printf("[%04X]: ", decodeAddr(mv, mv->reg[LAR]));

            switch (mv->reg[EAX]) {
                case 16:
                    scanf("%32s", binStr);
                    // Leer binario como string, solo cantBytes bits
                    read = 0;
                    for (j = 0; j < cantBytes * 8 && binStr[j]; j++) {
                        read = (read << 1) | (binStr[j] - '0');
                    }
                    // Si el bit más alto está en 1, hacer complemento a 2 para ese tamaño
                    if (binStr[0] == '1') {
                        read -= (1 << (cantBytes * 8));
                    }
                    break;
                case 8:
                    scanf("%X", &read);
                    break;
                case 4:
                    scanf("%o", &read);
                    break;
                case 2:
                    read = 0;
                    for (j = cantBytes - 1; j >= 0; j--) {
                        scanf(" %c", &car);
                        read |= ((Long)car) << (8 * j);
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

        i++;
    }
}

// Recibe un número decimal y devulve su representación binaria en complemento a 2 en formato String
void decToBinC2(Long value, char *binStr) {
    ULong uValue;
    int bitIndex, stringPos, firstOneFound, bit;

    uValue = (ULong)value;
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
    Long write;
    char c, binStr[33];
    int cantCeldas, cantBytes, i = 0, j = 0;

    cantCeldas = mv->reg[ECX] & MASK_LDL;
    cantBytes = (mv->reg[ECX] & MASK_LDH) >> 16;

    while (mv->flag == 0 && i < cantCeldas) {
        setLAR(mv, mv->reg[EDX] + cantBytes * i);
        setMAR(mv, cantBytes, mv->reg[LAR]);
        getMemory(mv);
        write = mv->reg[MBR];

        if (mv->flag == 0) {
            printf("[%04X]:", decodeAddr(mv, mv->reg[LAR]));

            if (mv->reg[EAX] & 0x10) {
                decToBinC2(write, binStr);
                printf(" 0b%s", binStr);
            }
            if (mv->reg[EAX] & 0x08)
                printf(" 0x%X", write);
            if (mv->reg[EAX] & 0x04)
                printf(" 0o%o", write);
            if (mv->reg[EAX] & 0x02) {
                printf(" ");
                // Imprime cada byte como caracter, de más significativo a menos
                for (j = cantBytes - 1; j >= 0; j--) {
                    c = (char)((write >> (8 * j)) & 0xFF);
                    if (32 <= c && c <= 126)
                        printf("%c", c);
                    else
                        printf(".");
                }
            }
            if (mv->reg[EAX] & 0x01)
                printf(" %d", write);

            printf("\n");
        }

        i++;
    }
}

void fsysBreak(TMV* mv) {
    char car;

    writeImage(mv);
    do {
        scanf("%c", &car);
        if (car == 'q')
            mv->reg[IP] = -1;
        else if (car == CR)
            executeOneInstruction(mv);
    } while (car != 'g' && car != 'q');
}

// Funcion SYS - Llamadas al sistema
void fsys(TMV* mv, Long* value1, Long* value2) {
    if (sysTable[*value1]) {
        sysTable[*value1](mv);
    }
}

// Funcion de centralizada de logica de saltos
void fmsl(TMV* mv, Long* value1, Long* value2) {
    if (condVector[mv->reg[OPC]](mv)) {
        mv->reg[IP] &= MASK_LDH;
        mv->reg[IP] |= (*value1 & MASK_LDL);
    }
}

// Boolean para indicar si saltar por cero
Long fjz(TMV* mv) {
    return mv->reg[CC] & CC_Z;
}

// Boolean para indicar si saltar por negativo
Long fjn(TMV* mv) {
    return mv->reg[CC] & CC_N;
}

// Instruccion NOT - Niega bit a bit
void fnot(TMV* mv, Long* value1, Long* value2) {
    *value1 = ~ *value1;
}

// Instruccion STOP - Setea IP a STOP_VALUE
void fstop(TMV* mv, Long* value1, Long* value2) {
    mv->reg[IP] = STOP_VALUE;
}

// Instruccion MOV - Paso el valor de value2 a value1
void fmov(TMV* mv, Long* value1, Long* value2) {
    *value1 = *value2;
}

// Instruccion ADD - Suma los valores de value1 y value2
void fadd(TMV* mv, Long* value1, Long* value2) {
    *value1 = *value1 + *value2;
}

// Instruccion SUB - Resta los valores de value1 y value2
void fsub(TMV* mv, Long* value1, Long* value2) {
    *value1 = *value1 - *value2;
}

// Instruccion MUL - Multiplicas los valores de value1 y value2C
void fmul(TMV* mv, Long* value1, Long* value2) {
    *value1 = *value1 * *value2;
}

// Instruccion DIV - Divide los valores de value1 y value2. Setea el resto en AC.
void fdiv(TMV* mv, Long* value1, Long* value2) {
    Long resto;

    if (*value2 == 0)
        errorHandler(mv, ERR_DIV0);
    else {
        resto = *value1 % *value2;
        *value1 = *value1 / *value2;

        if (resto < 0) {
            if (*value2 > 0) {
                *value1 -= 1;
                resto += *value2;
            }
            else {
                *value1 += 1;
                resto -= *value2;
            }
        }

        mv->reg[AC] = resto;
    }
}

// Instruccion SHL - Desplazamientos de bits a izquierda.
void fshl(TMV* mv, Long* value1, Long* value2) {
    *value1 = *value1 << *value2;
}

// Instruccion SHR - Desplazamientos de bits a derecha.
void fshr(TMV* mv, Long* value1, Long* value2) {
    *value1 = (Long) ((ULong) *value1 >> *value2);
}

// Instruccion SAR - Desplazamientos de bits a derecha pero propaga signo.
void fsar(TMV* mv, Long* value1, Long* value2) {
    *value1 = *value1 >> *value2;
}

// Instruccion AND - Operacion logica AND bit a bit.
void fand(TMV* mv, Long* value1, Long* value2) {
    *value1 = *value1 & *value2;
}

// Instruccion OR - Operacion logica OR bit a bit.
void f_or(TMV* mv, Long* value1, Long* value2) {
    *value1 = *value1 | *value2;
}

// Instruccion XOR - Operacion logica XOR bit a bit.
void fxor(TMV* mv, Long* value1, Long* value2) {
    *value1 = *value1 ^ *value2;
}

// Intruccion SWAP - Intercambia los valores de los value
void fswap(TMV* mv, Long* value1, Long* value2) {
    Long aux = 0;
    aux = *value1;
    *value1 = *value2;
    *value2 = aux;
}

// Instruccion LDL - Carga los 2 bytes menos significatidos del value1 con los dos bytes menos significativos del value2
void fldl(TMV* mv, Long* value1, Long* value2) {
    *value2 = *value2 & MASK_LDL;
    *value1 = (*value1 & MASK_LDH) | *value2;
}


// Instruccion LDH - Carga los 2 bytes mas significatidos del value1 con los dos bytes menos significativos del value2
void fldh(TMV* mv, Long* value1, Long* value2) {
    *value2 = (*value2 & MASK_LDL) << 16;
    *value1 = (*value1 & MASK_LDL) | *value2;
}

// Instruccion RND - Setea en value1 con un numero random entre 0 y value2
void frnd(TMV* mv, Long* value1, Long* value2) {
    int r, lim;

    if (*value2 <= 1) {
        *value1 = 0;
    }
    else {
        lim = RAND_MAX - (RAND_MAX % *value2);
        do {
            r = rand();
        } while (r >= lim);
        *value1 = (Long) (r % *value2);
    }
}
