/*******************************************************************************
#  main.c
#  Máquina Virtual - Implementación de la VM usada en el trabajo práctico
#
#  Descripción:
#    Implementación en C de una máquina virtual educativa. Contiene:
#      - Definiciones de memoria y registros
#      - Parsers para archivos .vmx/.vmi
#      - Ciclo de fetch-decode-execute de instrucciones
#      - Manejadores de sistema (SYS) y utilidades de I/O
#
#  Autores:
#    - Paniagua, Lucas
#    - Cherepakhin, Victoria
#    - Zuviria, Nicolas
#
#  Nota:
#    Los comentarios y la documentación dentro de este archivo están en
#    español y siguen un estilo Doxygen/consistente para facilitar la lectura
#    y la generación de documentación futura.
*******************************************************************************/

#include <stdio.h>
#include <stdlib.h>   /* Necesario para malloc/free y system (syscalls de consola) */
#include <stdint.h>   /* Tipos con tamaños explícitos (int8_t, uint32_t, ...) */
#include <string.h>   /* Manejo de cadenas */
#include <stddef.h>   /* offsetof() usado para layout de segmentos */
#include <time.h>     /* srand/rand para la instrucción RND */

/* ------------------------------------------------------------------------- */
/* Constantes y configuraciones globales (definiciones de la VM)             */
/* ------------------------------------------------------------------------- */

/* Valor sentinel para punteros lógicos inválidos (equivalente a NULL lógico) */
#define NIL -1

/* Tamaños de la máquina virtual (valores por defecto) */
#define RAM_KIB     16                  /* Máximo de RAM en KiB */
#define RAM_MAX     (RAM_KIB * 1024)    /* Máximo de RAM en bytes */
#define REG_AMOUNT  32                  /* Cantidad de registros (32) */
#define SEG_AMOUNT  8                   /* Cantidad de entradas en la tabla de segmentos */
#define SEG_MAXSIZE 65535               /* Tamaño máximo por segmento (2^16 - 1) */

/* Número de segmentos válidos (derivado del mapa de layout) */
#define NUM_SEGMENTS        (sizeof(segmentLayoutMap) / sizeof(SegmentLayout))
#define DEFAULT_SEG_SIZE    1024        // 1 KiB

/* Constantes para el parseo de la línea de comandos */
#define ARG_VMX     ".vmx"
#define ARG_VMI     ".vmi"
#define ARG_MEM     "m="
#define ARG_DISASM  "-d"
#define ARG_PARAMS  "-p"

/* Tamaños de cabeceras/headers en bytes para archivos .vmx/.vmi */
#define HEADER_P_RANGE 18
#define HEADER_P_COMUN 8
#define HEADER_P_V2    10
#define HEADER_I       8

/* Formato de presentación para el disassembler */
#define MNEM_WIDTH 4
#define OP_WIDTH   12

/* Máscaras para decodificación de operandos (bits dentro del byte de instrucción) */
#define MASKB_OPC 0b00011111
#define MASKB_OP1 0b00110000
#define MASKB_OP2 0b11000000

/* Flags del registro CC */
#define CC_N 0x80000000
#define CC_Z 0x40000000

/* Máscaras auxiliares usadas en manipulación de operandos y direcciones */
#define MASK_SEG    0xFFFF0000
#define MASK_UNTYPE 0x00FFFFFF
#define MASK_SIZE   0x00000003
#define MASK_REG    0x00FF0000
#define MASK_OFFSET 0x0000FFFF
#define MASK_PHY    0x0000FFFF
#define MASK_LDL    0x0000FFFF
#define MASK_LDH    0xFFFF0000
#define MASK_SETMEM 0xFF

/* Tamanos usados dentro del codigo */
typedef int8_t Byte;
typedef uint8_t UByte;
typedef int16_t Word;
typedef uint16_t UWord;
typedef int32_t Long;
typedef uint32_t ULong;

/* Tabla de descriptores de segmentos */
typedef struct {
    UWord base;   /* dirección lógica de inicio */
    UWord size;   /* tamaño del segmento en bytes */
} TableSeg;

/* Estructura para almacenar los parámetros de la línea de comandos */
typedef struct {
    char*       vmxPath;
    char*       vmiPath;
    ULong       memoryKiB;
    Byte        disassemblerFlag;
    Long        paramsArgc;          /* Cantidad de parámetros para el programa */
    char**      paramsArgv;          /* Puntero a los parámetros en el argv original */
} MVConfig;

/* Maquina Virtual */
typedef struct {
    UByte*     mem;                 /* Memoria dinamica */
    ULong      totalMemorySize;     /* Tamaño total de la memoria en bytes */
    Long       reg[REG_AMOUNT];     /* 32 registros de 4 bytes */
    TableSeg   seg[SEG_AMOUNT];     /* Tabla de segmentos */
    UWord      offsetEP;            /* Offset del Entry Point */
    MVConfig   config;              /* Configuracion de la MV */
    UByte      errorFlag;           /* Error flag */
    UByte      breakFlag;           /* Breakpoint flag */
} TMV;

/* Almacena los tamaños de los segmentos leídos del header del .vmx */
typedef struct {
    UWord codeSegSize;
    UWord dataSegSize;
    UWord extraSegSize;
    UWord stackSegSize;
    UWord constSegSize;
    UWord paramSegSize;
} SegmentSizes;

/* Enumeracion de errores */
typedef enum {
    ERR_EXE = 1,
    ERR_FOPEN,
    ERR_SEG,
    ERR_INS,
    ERR_DIV0,
    ERR_MEM,
    ERR_SOF,
    ERR_SUF
} ErrNumber;

/* Vector de mensajes de errores */
static const char* errorMsgs[] = {
    NULL,                                                                                   /* indice 0 (sin error) */
    "Uso: vmx [filename.vmx] [filename.vmi] [m=M] [-d] [-p param1 param2 ... paramN]\n",    /* 1 */
    "Error: No se pudo abrir el archivo.\n",                                                /* 2 */
    "Error: Fuera de los limites del segmento.\n",                                          /* 3 - pedido en el pdf v1 */
    "Error: Intruccion invalida.\n",                                                        /* 4 - pedido en el pdf v1 */
    "Error: No se puede dividir por 0.\n",                                                  /* 5 - pedido en el pdf v1 */
    "Error: Memoria insuficiente.\n",                                                       /* 6 - pedido en el pdf v2 */
    "Error: Stack Overflow.\n",                                                             /* 7 - pedido en el pdf v2 */
    "Error: Stack Underflow.\n"                                                             /* 8 - pedido en el pdf v2 */
};

/* Enumeracion de registros */
typedef enum {
    LAR = 0, MAR, MBR,
    IP = 3, OPC, OP1, OP2,
    SP = 7, BP,
    EAX = 10, EBX, ECX, EDX, EEX, EFX,
    AC = 16,
    CC = 17,
    CS = 26, DS, ES, SS, KS, PS
} RegName;

/* Vector de nombres de registros */
static const char* regStr[32] = {
    "LAR",      /* 0 */
    "MAR",      /* 1 */
    "MBR",      /* 2 */
    "IP",       /* 3 */
    "OPC",      /* 4 */
    "OP1",      /* 5 */
    "OP2",      /* 6 */
    "SP",       /* 7 */
    "BP",       /* 8 */
    "RESERVED", /* 9 */
    "EAX",      /* 10 */
    "EBX",      /* 11 */
    "ECX",      /* 12 */
    "EDX",      /* 13 */
    "EEX",      /* 14 */
    "EFX",      /* 15 */
    "AC",       /* 16 */
    "CC",       /* 17 */
    "RESERVED", /* 18 */
    "RESERVED", /* 19 */
    "RESERVED", /* 20 */
    "RESERVED", /* 21 */
    "RESERVED", /* 22 */
    "RESERVED", /* 23 */
    "RESERVED", /* 24 */
    "RESERVED", /* 25 */
    "CS",       /* 26 */
    "DS",       /* 27 */
    "ES",       /* 28 */
    "SS",       /* 29 */
    "KS",       /* 30 */
    "PS"        /* 31 */
};

/* Estructura que describe la metadata de un segmento */
typedef struct {
    Long    reg;          /* El enum del registro de segmento (ej: PS, KS, CS) */
    ULong   size_offset;  /* El offset del campo de tamaño dentro de la struct SegmentSizes */
} SegmentLayout;

/* El "mapa" que es la ÚNICA FUENTE DE LA VERDAD para el layout de la memoria.
    Define el orden físico y cómo encontrar el tamaño y el registro de cada segmento. */
static const SegmentLayout segmentLayoutMap[] = {
    { PS, offsetof(SegmentSizes, paramSegSize)  },
    { KS, offsetof(SegmentSizes, constSegSize)  },
    { CS, offsetof(SegmentSizes, codeSegSize)   },
    { DS, offsetof(SegmentSizes, dataSegSize)   },
    { ES, offsetof(SegmentSizes, extraSegSize)  },
    { SS, offsetof(SegmentSizes, stackSegSize)  }
};

/* Enumeracion de instrucciones */
typedef enum {
    SYS = 0x00, JMP, JZ, JP, JN, JNZ, JNP, JNN, NOT,
    PUSH = 0x0B, POP, CALL,
    RET = 0x0E, STOP,
    MOV = 0x10, ADD, SUB, MUL, DIV, CMP, SHL, SHR, SAR, AND, OR, XOR, SWAP, LDL, LDH, RND
} OpCode;

/* Vector de mnemonicos de instrucciones */
static const char* opStr[32] = {
    "SYS",      /* 0x00 */
    "JMP",      /* 0x01 */
    "JZ",       /* 0x02 */
    "JP",       /* 0x03 */
    "JN",       /* 0x04 */
    "JNZ",      /* 0x05 */
    "JNP",      /* 0x06 */
    "JNN",      /* 0x07 */
    "NOT",      /* 0x08 */
    "INVALID",  /* 0x09 */
    "INVALID",  /* 0x0A */
    "PUSH",     /* 0x0B */
    "POP",      /* 0x0C */
    "CALL",     /* 0x0D */
    "RET",      /* 0x0E */
    "STOP",     /* 0x0F */
    "MOV",      /* 0x10 */
    "ADD",      /* 0x11 */
    "SUB",      /* 0x12 */
    "MUL",      /* 0x13 */
    "DIV",      /* 0x14 */
    "CMP",      /* 0x15 */
    "SHL",      /* 0x16 */
    "SHR",      /* 0x17 */
    "SAR",      /* 0x18 */
    "AND",      /* 0x19 */
    "OR",       /* 0x1A */
    "XOR",      /* 0x1B */
    "SWAP",     /* 0x1C */
    "LDL",      /* 0x1D */
    "LDH",      /* 0x1E */
    "RND"       /* 0x1F */
};

/* --- PROTOTIPOS (declaraciones de funciones) --- */

/** Inicialiazacion: configuración, carga del programa, manejo de imágenes y inicio de la MV */
void errorHandler(TMV* mv, int err);
void parseCommandLine(int argc, char* argv[], TMV* mv);
void mallocMemory(TMV* mv);
void readProgramHeader(TMV* mv, SegmentSizes* segSizes);
void loadProgramData(TMV* mv, SegmentSizes* segSizes);
void readImage(TMV* mv);
UWord readUWordBE(FILE* arch);
Long readLongBE(FILE* arch);
void writeImage(TMV* mv);
void writeUWordBE(FILE* arch, UWord data);
void writeLongBE(FILE* arch, Long data);
UWord calculatePSSize(TMV* mv);
ULong sumSegSizes(SegmentSizes* segSizes);
void createParamSegment(TMV* mv, UWord psSize);
void initializeTable(TMV* mv, SegmentSizes* segSizes);
void initializeRegisters(TMV* mv, SegmentSizes* segSizes);
void initIP(TMV* mv);

/** Disassembler: convertir memoria a texto legible */
void disASM(TMV* mv);
void disASMConst(TMV* mv);
void disASMCode(TMV* mv);
void disASMOP(TMV* mv, Long operand, Byte type);
void disASMOPStrToBuf(TMV* mv, Long operand, UByte type, char* buf, size_t buflen);

/** Ejecutador: ciclo principal y ejecución de instrucciones */
void executeProgram(TMV* mv);
void executeOneInstruction(TMV* mv);
void initializeMainStack(TMV* mv);

/** Decodificadores y utilidades de direccionamiento */
Long decodeSeg(TMV* mv, Long cod);
Long decodeAddr(TMV* mv, Long logical);
int inSegment(TMV* mv, Long logical);
int inCS(TMV* mv, Long logical);

/** Fetch/decodificación de instrucción */
void fetchInstruction(TMV* mv);
void fetchOperators(TMV* mv);
Long fetchOperand(TMV* mv, int bytes, int* offset);
void addIP(TMV* mv);

/** Memoria: helpers relacionados con LAR/MAR/MBR */
void setLARFromOperand(TMV* mv, Long operand);
void setLARFromAddress(TMV* mv, Long logicalAddress);
void setMAR(TMV* mv, Long cantBytes, Long logical);
void getMemory(TMV* mv);
void setMemory(TMV* mv);

/** Operandos: lectura y escritura (get/set) */
Long getOP(TMV* mv, Long operand);
void setOP(TMV* mv, Long operandA, Long operandB);
void setCC(TMV* mv, Long valor);

/** SYS: llamadas al sistema/servicios */
void fsys(TMV* mv, Long* value1, Long* value2);
void fsysRead(TMV* mv);
void decToBinC2(Long value, char *binStr);
void fsysWrite(TMV* mv);
void fsysStrRead(TMV* mv);
void fsysStrWrite(TMV* mv);
void fsysClrScr(TMV* mv);
void fsysBreakpoint(TMV* mv);

/** Saltos / condiciones */
void fmsl(TMV* mv, Long* value1, Long* value2);
Long fjz(TMV* mv);
Long fjn(TMV* mv);

/** Instrucciones (ALU / control / pila / etc.) */
void fnot(TMV* mv, Long* value1, Long* value2);
void fpush(TMV* mv, Long* value1, Long* value2);
void fpop(TMV* mv, Long* value1, Long* value2);
void fcall(TMV* mv, Long* value1, Long* value2);
void fret(TMV* mv, Long* value1, Long* value2);
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

/* Puntero a función que implementa una instrucción.
 * Firma: (TMV* mv, Long* value1, Long* value2)
 * - mv: contexto de la máquina
 * - value1/value2: punteros a los operandos ya cargados por fetch (pueden ser NULL)
 */
typedef void (*InstrFunc)(TMV*, Long*, Long*);

/* Descripción de la entrada de la tabla de instrucciones.
 * - fetchValue: cuántos operandos deben ser leídos antes de ejecutar (0, 1 o 2).
 * - execute: puntero a la rutina que realiza la semántica de la instrucción.
 * - setearCC: si es 1, se invoca setCC(mv, value1) tras la ejecución.
 * - setValue: cuántos operandos deben escribirse de vuelta (0, 1 o 2).
 */
typedef struct {
    UByte       fetchValue;     /* 0..2: número de operandos a obtener */
    InstrFunc   execute;        /* implementación de la instrucción */
    UByte       setearCC;       /* 0/1: actualizar flags CC con value1 */
    UByte       setValue;       /* 0..2: número de operandos a escribir */
} Instruction;

/* Wrapper simple para instrucción inválida: marca el error correspondiente. */
void finvalid(TMV* mv, Long* value1, Long* value2)  { errorHandler(mv, ERR_INS); }

/* Tabla que describe el comportamiento de cada opcode (0x00..0x1F).
 * Índices corresponden con la enumeración OpCode.
 * Para cada opcode definimos:
 *  - cuántos operandos traer (fetchValue)
 *  - qué función ejecuta (execute)
 *  - si actualiza condiciones (setearCC)
 *  - cuántos operandos escribe de vuelta (setValue)
 */
static const Instruction instrTable[32] = {
    [SYS] =  { .fetchValue = 1, .execute = fsys,   .setearCC = 0, .setValue = 0 },  /* 0x00 SYS */
    [JMP] =  { .fetchValue = 1, .execute = fmsl,   .setearCC = 0, .setValue = 0 },  /* 0x01 JMP */
    [JZ] =   { .fetchValue = 1, .execute = fmsl,   .setearCC = 0, .setValue = 0 },  /* 0x02 JZ  */
    [JP] =   { .fetchValue = 1, .execute = fmsl,   .setearCC = 0, .setValue = 0 },  /* 0x03 JP  */
    [JN] =   { .fetchValue = 1, .execute = fmsl,   .setearCC = 0, .setValue = 0 },  /* 0x04 JN  */
    [JNZ] =  { .fetchValue = 1, .execute = fmsl,   .setearCC = 0, .setValue = 0 },  /* 0x05 JNZ */
    [JNP] =  { .fetchValue = 1, .execute = fmsl,   .setearCC = 0, .setValue = 0 },  /* 0x06 JNP */
    [JNN] =  { .fetchValue = 1, .execute = fmsl,   .setearCC = 0, .setValue = 0 },  /* 0x07 JNN */
    [NOT] =  { .fetchValue = 1, .execute = fnot,   .setearCC = 1, .setValue = 1 },  /* 0x08 NOT */

    [9]   =  { .fetchValue = 0, .execute = finvalid,.setearCC = 0, .setValue = 0 }, /* 0x09 invalid */
    [10]  =  { .fetchValue = 0, .execute = finvalid,.setearCC = 0, .setValue = 0 }, /* 0x0A invalid */

    [PUSH] = { .fetchValue = 1, .execute = fpush,  .setearCC = 0, .setValue = 0 },  /* 0x0B PUSH */
    [POP] =  { .fetchValue = 1, .execute = fpop,   .setearCC = 0, .setValue = 1 },  /* 0x0C POP  */
    [CALL] = { .fetchValue = 1, .execute = fcall,  .setearCC = 0, .setValue = 0 },  /* 0x0D CALL */

    [RET] =  { .fetchValue = 0, .execute = fret,   .setearCC = 0, .setValue = 0 },  /* 0x0E RET  */
    [STOP] = { .fetchValue = 0, .execute = fstop,  .setearCC = 0, .setValue = 0 },  /* 0x0F STOP */

    [MOV] =  { .fetchValue = 2, .execute = fmov,   .setearCC = 0, .setValue = 1 },  /* 0x10 MOV  */
    [ADD] =  { .fetchValue = 2, .execute = fadd,   .setearCC = 1, .setValue = 1 },  /* 0x11 ADD  */
    [SUB] =  { .fetchValue = 2, .execute = fsub,   .setearCC = 1, .setValue = 1 },  /* 0x12 SUB  */
    [MUL] =  { .fetchValue = 2, .execute = fmul,   .setearCC = 1, .setValue = 1 },  /* 0x13 MUL  */
    [DIV] =  { .fetchValue = 2, .execute = fdiv,   .setearCC = 1, .setValue = 1 },  /* 0x14 DIV  */
    [CMP] =  { .fetchValue = 2, .execute = fsub,   .setearCC = 1, .setValue = 0 },  /* 0x15 CMP  */
    [SHL] =  { .fetchValue = 2, .execute = fshl,   .setearCC = 1, .setValue = 1 },  /* 0x16 SHL  */
    [SHR] =  { .fetchValue = 2, .execute = fshr,   .setearCC = 1, .setValue = 1 },  /* 0x17 SHR  */
    [SAR] =  { .fetchValue = 2, .execute = fsar,   .setearCC = 1, .setValue = 1 },  /* 0x18 SAR  */
    [AND] =  { .fetchValue = 2, .execute = fand,   .setearCC = 1, .setValue = 1 },  /* 0x19 AND  */
    [OR] =   { .fetchValue = 2, .execute = f_or,   .setearCC = 1, .setValue = 1 },  /* 0x1A OR   */
    [XOR] =  { .fetchValue = 2, .execute = fxor,   .setearCC = 1, .setValue = 1 },  /* 0x1B XOR  */
    [SWAP] = { .fetchValue = 2, .execute = fswap,  .setearCC = 0, .setValue = 2 },  /* 0x1C SWAP */
    [LDL] =  { .fetchValue = 2, .execute = fldl,   .setearCC = 0, .setValue = 1 },  /* 0x1D LDL  */
    [LDH] =  { .fetchValue = 2, .execute = fldh,   .setearCC = 0, .setValue = 1 },  /* 0x1E LDH  */
    [RND] =  { .fetchValue = 2, .execute = frnd,   .setearCC = 0, .setValue = 1 },  /* 0x1F RND  */
};

/* Tabla de funciones del servicio SYS (sub-códigos dentro de la instrucción SYS).
 * Solo se definen los índices que existen; entradas no inicializadas son NULL.
 * Los índices son valores de 0x00..0x0F que provienen del primer operando de SYS.
 */
typedef void (*SysFunc)(TMV*);

static const SysFunc sysTable[16] = {
    [0x01] = fsysRead,
    [0x02] = fsysWrite,
    [0x03] = fsysStrRead,
    [0x04] = fsysStrWrite,
    [0x07] = fsysClrScr,
    [0x0F] = fsysBreakpoint,
};

/* Tipo para funciones de condición usadas por las instrucciones de salto.
 * Cada función consulta flags del MV y devuelve distinto de cero si la
 * condición se cumple.
 */
typedef Long (*CondFunc)(TMV*);

/* Wrappers de condiciones para mapear el opcode de salto a la comprobación.
 * Se usan en condVector para simplificar la lógica centralizada de saltos.
 */
Long condTrue(TMV* mv) { return 1; }
Long condZ(TMV* mv)    { return fjz(mv); }
Long condP(TMV* mv)    { return !fjz(mv) && !fjn(mv); }
Long condN(TMV* mv)    { return fjn(mv); }
Long condNZ(TMV* mv)   { return !fjz(mv); }
Long condNP(TMV* mv)   { return fjz(mv) || fjn(mv); }
Long condNN(TMV* mv)   { return !fjn(mv); }

/* Vector que mapea cada tipo de salto (JMP/JZ/...) a su función de condición.
 * El índice está alineado con el enum OpCode para que condVector[OPC] sea la
 * función correcta a evaluar antes de ejecutar fmsl.
 */
static const CondFunc condVector[8] = {
    [JMP] = condTrue,
    [JZ]  = condZ,
    [JP]  = condP,
    [JN]  = condN,
    [JNZ] = condNZ,
    [JNP] = condNP,
    [JNN] = condNN,
};



/* --- CÓDIGO: implementación de la VM --- */
/**
 * @brief Punto de entrada de la VM.
 *
 * Esta función realiza la orquestación completa del ciclo de vida de la
 * máquina virtual:
 *  - Parseo de argumentos y configuración (archivos .vmx/.vmi, memoria, flags).
 *  - Reserva de memoria dinámica según el parámetro m= (o el valor por defecto).
 *  - Carga del programa (.vmx) o restauración de imagen (.vmi).
 *  - Inicialización de tablas de segmentos y registros.
 *  - Opcional: desensamblado si se pasó -d.
 *  - Ejecución del programa hasta fin de segmento o instrucción STOP.
 *
 * Parámetros:
 *  - argc/argv: argumentos de línea de comandos (ver README.md para uso).
 *
 * Valores de retorno / códigos de error:
 *  - 0: ejecución correcta.
 *  - 1..8: distintos errores documentados en errorMsgs (uso, fopen, segfault lógico,
 *    instrucción inválida, división por cero, memoria insuficiente, stack overflow/underflow).
 *
 * Efectos secundarios:
 *  - Reserva y liberación de memoria dinámica (mv.mem).
 *  - Posible escritura de archivos .vmi si se ejecuta SYS breakpoint.
 */
int main(int argc, char *argv[]) {
    TMV mv;
    SegmentSizes segSizes;

    mv.errorFlag = 0;
    mv.breakFlag = 0;
    srand(time(NULL));      // Para la instruccion RND

    // Parsear los argumentos y llenar la estructura config
    parseCommandLine(argc, argv, &mv);

    if (mv.errorFlag == 0) {
        // --- RESERVA DE MEMORIA DINÁMICA ---
        mallocMemory(&mv);

        if (mv.mem == NULL)
            errorHandler(&mv, ERR_MEM);
        else {
            // --- Lógica de Carga ---
            // Calcular el size de PS
            segSizes.paramSegSize = calculatePSSize(&mv);
            // Si hay un .vmx, se carga el programa
            if (mv.config.vmxPath != NULL) {
                if (segSizes.paramSegSize > mv.totalMemorySize)
                    errorHandler(&mv, ERR_MEM);
                else {
                    // Creacion del Param Segment
                    createParamSegment(&mv, segSizes.paramSegSize);

                    // Lectura del header del .vmx
                    readProgramHeader(&mv, &segSizes);

                    // Suma de los sizes para ver si sobrepasan el total de memoria
                    if (sumSegSizes(&segSizes) > mv.totalMemorySize)
                        errorHandler(&mv, ERR_MEM);
                    else {
                        // Inicializaciones de tabla de segmentos y registros
                        initializeTable(&mv, &segSizes);
                        initializeRegisters(&mv, &segSizes);

                        // Carga de memoria
                        loadProgramData(&mv, &segSizes);
                    }
                }
            }
            // Si no hay .vmx pero sí .vmi, se carga la imagen
            else if (mv.config.vmiPath != NULL) {
                readImage(&mv);
            }

            // --- Lógica de Ejecución ---
            if (mv.errorFlag == 0) {
                // Disassambler
                if (mv.config.disassemblerFlag) {
                    disASM(&mv);
                    initIP(&mv);
                }

                // Ejecucion del programa
                executeProgram(&mv);
            }

            // Liberamos la memoria
            if (mv.mem != NULL) {
                free(mv.mem);
            }
        }
    }

    return mv.errorFlag;
}

/**
 * @brief Maneja los errores, establece el flag de error e imprime el mensaje correspondiente.
 */
void errorHandler(TMV* mv, int err) {
    mv->errorFlag = err;
    fprintf(stderr, "%s", errorMsgs[err]);
}

/**
 * @brief Parsea los argumentos de la línea de comandos y configura la estructura MVConfig.
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
        if (strstr(argv[i], ARG_VMX)) {
            mv->config.vmxPath = argv[i];
        }
        else if (strstr(argv[i], ARG_VMI)) {
            mv->config.vmiPath = argv[i];
        }
        else if (strncmp(argv[i], ARG_MEM, 2) == 0) {
            sscanf(argv[i], "m=%d", &mv->config.memoryKiB);
        }
        else if (strcmp(argv[i], ARG_DISASM) == 0) {
            mv->config.disassemblerFlag = 1;
        }
        else if (strcmp(argv[i], ARG_PARAMS) == 0) {
            if (i + 1 < argc) { // Solo procesar si hay algo después de -p
                mv->config.paramsArgc = argc - (i + 1);
                mv->config.paramsArgv = &argv[i + 1];
            }
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
 * @brief Reserva memoria dinámica para la MV según mv->config.memoryKiB.
 *
 * - mv->totalMemorySize se actualiza en bytes.
 * - mv->mem apunta al bloque reservado (o NULL si falla la reserva).
 */
void mallocMemory(TMV* mv) {
    mv->totalMemorySize = mv->config.memoryKiB * 1024;
    mv->mem = (UByte*) malloc(mv->totalMemorySize);
}

/**
 * @brief Lee únicamente el encabezado (header) de un archivo .vmx para obtener los tamaños de los segmentos y el punto de entrada.
 */
void readProgramHeader(TMV* mv, SegmentSizes* segSizes) {
    FILE* arch;
    UByte header[HEADER_P_RANGE];

    arch = fopen(mv->config.vmxPath, "rb");
    if (arch == NULL)
        errorHandler(mv, ERR_FOPEN);
    else {
        fread(header, 1, HEADER_P_COMUN, arch);
        segSizes->codeSegSize = ((UWord) header[6] << 8) | header[7];

        if (header[5] == 1) {
            segSizes->dataSegSize = DEFAULT_SEG_SIZE;
            segSizes->extraSegSize = 0;
            segSizes->stackSegSize = DEFAULT_SEG_SIZE * 8;
            segSizes->constSegSize = 0;
            mv->offsetEP = 0;
        }
        else if (header[5] == 2) {
            fread(header + HEADER_P_COMUN, 1, HEADER_P_V2, arch);
            segSizes->dataSegSize = ((UWord) header[8] << 8) | header[9];
            segSizes->extraSegSize = ((UWord) header[10] << 8) | header[11];
            segSizes->stackSegSize = ((UWord) header[12] << 8) | header[13];
            segSizes->constSegSize = ((UWord) header[14] << 8) | header[15];
            mv->offsetEP = ((UWord) header[16] << 8) | header[17];
        }

        fclose(arch);
    }
}

/**
 * @brief Carga el código y las constantes desde el archivo .vmx a sus posiciones finales en la memoria de la MV.
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

/**
 * @brief Carga el estado completo de la MV (memoria, registros, tabla) desde un archivo de imagen .vmi.
 */
void readImage(TMV* mv) {
    FILE* arch;
    UByte header[HEADER_I];
    int i;

    arch = fopen(mv->config.vmiPath, "rb");
    if (arch == NULL)
        errorHandler(mv, ERR_FOPEN);
    else {
        // Header
        fread(header, sizeof(UByte), HEADER_I, arch);

        // Extraer tamaño de memoria y RESERVARLA
        mv->config.memoryKiB = ((UWord)header[6] << 8) | header[7];
        mallocMemory(mv);

        if (mv->mem == NULL)
            errorHandler(mv, ERR_MEM);
        else {
            // Registros (leídos uno por uno)
            for (i = 0; i < REG_AMOUNT; i++) {
                mv->reg[i] = readLongBE(arch);
            }

            // Tabla de Segmentos (leída campo por campo)
            for (i = 0; i < SEG_AMOUNT; i++) {
                mv->seg[i].base = readUWordBE(arch);
                mv->seg[i].size = readUWordBE(arch);
            }

            // Memoria (leída en bloque)
            fread(mv->mem, sizeof(UByte), mv->totalMemorySize, arch);
        }

        fclose(arch);
        mv->offsetEP = (UWord) mv->reg[IP];
    }
}

/**
 * @brief Lee un UWord (2 bytes) en formato Big Endian desde un archivo.
 */
UWord readUWordBE(FILE* arch) {
    UWord data = 0;
    data |= (fgetc(arch) << 8);
    data |= fgetc(arch);
    return data;
}

/**
 * @brief Lee un Long (4 bytes) en formato Big Endian desde un archivo.
 */
Long readLongBE(FILE* arch) {
    Long data = 0;
    data |= (fgetc(arch) << 24);
    data |= (fgetc(arch) << 16);
    data |= (fgetc(arch) << 8);
    data |= fgetc(arch);
    return data;
}

/**
 * @brief Guarda el estado actual de la MV (memoria, registros, tabla) en un archivo de imagen .vmi en formato Big Endian.
 */
void writeImage(TMV* mv) {
    FILE* arch;
    UByte version = 1, i;
    UWord memKiB = mv->totalMemorySize / 1024;

    arch = fopen(mv->config.vmiPath, "wb");
    if (arch == NULL)
        errorHandler(mv, ERR_FOPEN);
    else {
        // Header
        fwrite("VMI25", sizeof(char), 5, arch);
        fputc(version, arch);
        writeUWordBE(arch, memKiB);

        // Registros
        for (i = 0; i < REG_AMOUNT; i++) {
            writeLongBE(arch, mv->reg[i]);
        }

        // Tabla de Descriptores
        for (i = 0; i < SEG_AMOUNT; i++) {
            writeUWordBE(arch, mv->seg[i].base);
            writeUWordBE(arch, mv->seg[i].size);
        }

        // Memoria Principal
        // Esto es un array de bytes, así que un fwrite directo está bien
        fwrite(mv->mem, sizeof(UByte), mv->totalMemorySize, arch);

        fclose(arch);
    }
}

/**
 * @brief Escribe un UWord (2 bytes) en formato Big Endian en un archivo.
 */
void writeUWordBE(FILE* arch, UWord data) {
    fputc((data >> 8) & MASK_SETMEM, arch);
    fputc(data & MASK_SETMEM, arch);
}

/**
 * @brief Escribe un Long (4 bytes) en formato Big Endian en un archivo.
 */
void writeLongBE(FILE* arch, Long data) {
    fputc((data >> 24) & MASK_SETMEM, arch);
    fputc((data >> 16) & MASK_SETMEM, arch);
    fputc((data >> 8) & MASK_SETMEM, arch);
    fputc(data & MASK_SETMEM, arch);
}

/**
 * @brief Calcula el tamaño requerido en bytes para el Param Segment basándose en los argumentos de entrada.
 */
UWord calculatePSSize(TMV* mv) {
    ULong psSize = 0;
    int i;

    for (i = 0; i < mv->config.paramsArgc; i++) {
        psSize += strlen(mv->config.paramsArgv[i]) + 1;
    }
    psSize += mv->config.paramsArgc * sizeof(Long);

    return psSize;
}

/**
 * @brief Calcula la suma total en bytes de todos los segmentos definidos.
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
 * @brief Escribe los strings y el arreglo de punteros argv en el espacio de memoria del Param Segment.
 */
void createParamSegment(TMV* mv, UWord psSize) {
    Long argvArraySize, argvPhysicalStart, currentStringOffset, psLogicalPtr, stringLogicalPtr, destAddr, cantBytes, i, j;

    argvArraySize = mv->config.paramsArgc * sizeof(Long);
    argvPhysicalStart = psSize - argvArraySize;
    currentStringOffset = 0;
    /*
     * psLogicalPtr: dirección lógica base dentro del Param Segment.
     * Por convención, si existe el Param Segment su base lógica es 0.
     * currentStringOffset es el offset físico dentro del PS donde se van
     * escribiendo las cadenas, mientras que argvPhysicalStart apunta al
     * array de punteros físicos que va al final del segmento.
     */
    psLogicalPtr = 0;   /* Si existe, siempre se ubica en 0 */
    /* Cantidad de bytes usados para almacenar cada puntero lógico (4 bytes). */
    cantBytes = 4;

    if (mv->config.paramsArgc > 0) {
        for (i = 0; i < mv->config.paramsArgc; i++) {
            /* Escribir la cadena en el inicio del PS (posición física currentStringOffset).
             * Observación: usamos strcpy porque sabemos que hay suficiente espacio
             * (psSize fue calculado previamente sumando longitudes y espacio para
             * el array de punteros).
             */
            strcpy((char*)&mv->mem[currentStringOffset], mv->config.paramsArgv[i]);

            /* Construcción del puntero lógico: la parte alta identifica el
             * segmento (psLogicalPtr) y la parte baja es el offset dentro del
             * segmento (currentStringOffset). Se escribe el puntero en formato
             * big-endian en el área argvPhysicalStart + i*sizeof(Long).
             */
            stringLogicalPtr = psLogicalPtr | currentStringOffset;
            destAddr = argvPhysicalStart + i * sizeof(Long);
            for (j = 0; j < cantBytes; j++)
                mv->mem[destAddr + j] = (UByte) (stringLogicalPtr >> (8 * (cantBytes - 1 - j)) & MASK_SETMEM);

            /* Avanzar al siguiente espacio libre para la siguiente cadena. */
            currentStringOffset += strlen(mv->config.paramsArgv[i]) + 1;
        }
    }
}

/**
 * @brief Inicializa la tabla de descriptores de segmentos, estableciendo los valores por defecto y luego colocando los segmentos válidos.
 */
void initializeTable(TMV* mv, SegmentSizes* segSizes) {
    Long physicalOffset = 0, tableIndex = 0, i = 0;
    UWord currentSize;

    // Inicializamos toda la tabla con los valores por defecto.
    // Iteramos sobre SEG_AMOUNT (8) para cubrir todos los "slots" físicos de la tabla.

    for (i = 0; i < SEG_AMOUNT; i++) {
        mv->seg[i].base = NIL;
        mv->seg[i].size = 0;
    }


    // Iteramos sobre el mapa que define el orden y la metadata.
    /* Recorremos segmentLayoutMap —que define el orden físico de los
     * segmentos— y, para cada segmento con tamaño > 0, añadimos una
     * entrada en la tabla física mv->seg[]. physicalOffset acumula la
     * dirección física (offset dentro de mv->mem) actual.
     */
    for (i = 0; i < NUM_SEGMENTS; i++) {
        currentSize = *(UWord*)((UByte*)segSizes + segmentLayoutMap[i].size_offset);

        if (currentSize > 0) {
            /* tableIndex es el "slot" físico donde almacenamos el descriptor.
             * La base almacenada en la tabla es la dirección física dentro de mv->mem.
             */
            mv->seg[tableIndex].base = physicalOffset;
            mv->seg[tableIndex].size = currentSize;

            physicalOffset += currentSize;
            tableIndex++;
        }
    }
}

/**
 * @brief Inicializa todos los registros de la MV, incluyendo los punteros de segmento (CS, DS, etc.) y los punteros de ejecución (IP, SP).
 */
void initializeRegisters(TMV* mv, SegmentSizes* segSizes) {
    Long i = 0, tableIndex = 0, currentReg = 0;
    UWord currentSize = 0;

    // Iteramos sobre el mapa para asignar los registros de segmento.
    for (i = 0; i < NUM_SEGMENTS; i++) {
        currentSize = *(UWord*)((UByte*)segSizes + segmentLayoutMap[i].size_offset);
        currentReg = segmentLayoutMap[i].reg;

        if (currentSize > 0) {
            /* Guardamos en el registro de segmento la combinación
             * (índice_tabla << 16). De esta forma el registro contiene
             * tanto el índice físico al descriptor como la parte alta del
             * puntero lógico. Más tarde decodeSeg/ decodeAddr esperan este
             * formato.
             */
            mv->reg[currentReg] = tableIndex << 16;
            tableIndex++;
        } else {
            /* Si el segmento no existe, dejamos el registro en NIL. */
            mv->reg[currentReg] = NIL;
        }
    }

    // Finalmente, inicializamos los punteros de ejecución (IP y SP).
    initIP(mv);

    if (mv->reg[SS] != NIL)
        /* SP se inicializa en la "parte alta" del stack: base física del
         * descriptor (mv->seg[index].base) + tamaño del segmento. Como
         * mv->reg[SS] contiene (index<<16), accedemos con (mv->reg[SS] >> 16).
         */
        mv->reg[SP] = mv->reg[SS] + mv->seg[mv->reg[SS] >> 16].size;
    else
        mv->reg[SP] = NIL; // Si no hay pila, SP también es inválido
}

/**
 * @brief Inicializa IP usando el segmento CS y el offset del Entry Point.
 */
void initIP(TMV* mv) {
    mv->reg[IP] = mv->reg[CS] | mv->offsetEP;
}

/* --- DISASSEMBLER --- */
/**
 * @brief Orquesta el proceso de desensamblado.
 * Imprime primero el contenido del Const Segment y luego el del Code Segment.
 */
void disASM(TMV* mv) {
    disASMConst(mv);
    disASMCode(mv);
}

/**
 * @brief Desensambla y muestra el contenido del Const Segment como cadenas de caracteres.
 */
void disASMConst(TMV* mv) {
    Long currentAddr, physicalLimitKS, len, maxHexBytes, i;
    UByte car;
    char stringFull[256]; // Buffer para la string completa

    if (decodeSeg(mv, KS) != NIL) {
        physicalLimitKS = mv->seg[decodeSeg(mv, KS)].base + mv->seg[decodeSeg(mv, KS)].size;
        currentAddr = mv->seg[decodeSeg(mv, KS)].base;

        while (currentAddr < physicalLimitKS) {
            len = 0;

            // Imprimir la dirección de memoria
            printf(" [%04X] ", (UWord) currentAddr);

            // Leer la string completa para mostrarla
            while (mv->mem[currentAddr + len] != '\0') {
                car = mv->mem[currentAddr + len];
                // Reemplazar no imprimibles por '.'
                stringFull[len] = (32 <= car && car <= 126) ? car : '.';
                len++;
            }
            stringFull[len] = '\0'; // Terminar la string

            // Determinar cuántos bytes hexadecimales mostrar
            maxHexBytes = (len + 1 > 7) ? 6 : len + 1;

            // Imprimir los bytes en hexadecimal
            for (i = 0; i < maxHexBytes; i++) {
                printf("%02X ", mv->mem[currentAddr + i]);
            }

            // Indicar que la cadena continúa
            if (len + 1 > 7) {
                printf(".. ");
            }
            else {
                // Relleno para alinear
                for (i = 7 - maxHexBytes; i > 0; i--) {
                    printf("   ");
                }
            }

            // Imprimir la cadena de caracteres
            printf("| \"%s\"\n", stringFull);

            // Mover el addr a la siguiente cadena
            currentAddr += len + 1;
        }
    }
}

/**
 * @brief Desensambla y muestra las instrucciones del Code Segment.
 */
void disASMCode(TMV* mv) {
    Byte i;
    UByte ins = 0, typA = 0, typB = 0, typInsA = 0, typInsB = 0;
    UWord currentAddr, offsetEP;
    char mnemStr[MNEM_WIDTH + 1], op1Str[OP_WIDTH + 1], op2Str[OP_WIDTH + 1];

    // Poner IP en la base del CS para imprimir todo el CS
    mv->reg[IP] = mv->reg[CS];
    offsetEP = mv->seg[decodeSeg(mv, CS)].base + mv->offsetEP;

    while (inCS(mv, mv->reg[IP])) {
        fetchInstruction(mv);
        fetchOperators(mv);

        // --- Primer mitad: posicion de memoria y bytes en hexadecimal ---
        // Memoria
        currentAddr = (UWord) decodeAddr(mv, mv->reg[IP]);
        printf("%c", (currentAddr == offsetEP) ? '>' : ' ');
        printf("[%04X] ", currentAddr);

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
        printf("| %*s%*s%s%*s\n", MNEM_WIDTH, mnemStr, OP_WIDTH, op1Str, (typB != 0 ? "," : ""), OP_WIDTH, op2Str);

        addIP(mv);
    }
}

/**
 * @brief Funcion auxiliar para transformar un operando a bytes para el disassembler.
 */
void disASMOP(TMV* mv, Long operand, Byte type) {
    Byte i = type - 1;
    UByte aux = 0;

    for (; i >= 0; i--) {
        aux = (UByte) (operand >> (8 * i));
        printf("%02X ", aux);
    }
}

/**
 * @brief Convierte un operando codificado en una cadena de texto legible para el disassembler.
 * Maneja los pseudónimos de registro (AX, AL, AH) y los modificadores de memoria (b[], w[]).
 */
void disASMOPStrToBuf(TMV* mv, Long operand, UByte type, char* buf, size_t buflen) {
    UByte size, hex;
    Word offset;
    char car;

    operand &= MASK_UNTYPE;

    switch (type) {
        case 0:         // NULL
            break;
        case 1: {       // REGISTRO
            size = (operand >> 6) & MASK_SIZE;
            hex = operand & 0x0000000F;

            switch (size) {
                case 0: // 4 bytes
                    snprintf(buf, buflen, "%s", regStr[operand]);
                    break;
                case 1: // Cuarto byte
                    snprintf(buf, buflen, "%XL", hex);
                    break;
                case 2: // Tercer byte
                    snprintf(buf, buflen, "%XH", hex);
                    break;
                case 3: // 2 bytes
                    snprintf(buf, buflen, "%XX", hex);
                    break;
            }
            break;
        }
        case 2: {       // INMEDIATO
            if ((JMP <= mv->reg[OPC] && mv->reg[OPC] <= JNN) || mv->reg[OPC] == CALL)
                snprintf(buf, buflen, "0x%04X", (Word) operand + mv->seg[decodeSeg(mv, CS)].base);
            else
                snprintf(buf, buflen, "%d", (Word) operand);
            break;
        }
        case 3: {       // MEMORIA
            size = (operand >> 22) & MASK_SIZE;
            switch (size) {
                case 0: // long
                    car = 'l';
                    break;
                case 2: // word
                    car = 'w';
                    break;
                case 3: // byte
                    car = 'b';
                    break;
            }

            offset = (Word) (operand & 0xFFFF);
            snprintf(buf, buflen, "%c[%s%s%d]", car, regStr[(operand >> 16) & 0x1F], (offset >= 0 ? "+" : ""), offset);
            break;
        }
    }
}

/* --- EJECUCIÓN DEL PROGRAMA --- */
/**
 * @brief Bucle principal de ejecución. Ejecuta instrucciones una por una mientras el IP esté en el Code Segment.
 */
void executeProgram(TMV* mv) {
    initializeMainStack(mv);

    while (mv->errorFlag == 0 && inCS(mv, mv->reg[IP])) {
        executeOneInstruction(mv);
    }
}

/**
 * @brief Realiza un ciclo completo de instrucción (fetch, decode, execute).
 */
void executeOneInstruction(TMV* mv) {
    Instruction instr;
    Long value1, value2;

    fetchInstruction(mv);
    fetchOperators(mv);
    addIP(mv);
    instr = instrTable[mv->reg[OPC]];

    if (mv->errorFlag == 0) {
        if (instr.fetchValue > 0) {
            value1 = getOP(mv, mv->reg[OP1]);
            if (instr.fetchValue == 2)
                value2 = getOP(mv, mv->reg[OP2]);
        }

        instr.execute(mv, &value1, &value2);

        if (mv->errorFlag == 0) {
            if (instr.setearCC == 1)
                setCC(mv, value1);
            if (instr.setValue > 0) {
                setOP(mv, mv->reg[OP1], value1);
                if (instr.setValue == 2)
                    setOP(mv, mv->reg[OP2], value2);
            }
        }
    }
}

/**
 * @brief Prepara la pila con los valores iniciales para la subrutina main, reutilizando fpush.
 */
void initializeMainStack(TMV* mv) {
    UWord psSize;
    Long valueToPush, argvArraySize;

    // PUSH del puntero a argv
    valueToPush = NIL; // Valor por defecto si no hay parámetros
    if (mv->config.paramsArgc > 0) {
        psSize = calculatePSSize(mv);
        argvArraySize = mv->config.paramsArgc * sizeof(Long);
        valueToPush = mv->reg[PS] | (psSize - argvArraySize);
    }
    fpush(mv, &valueToPush, NULL);

    // PUSH de argc
    valueToPush = mv->config.paramsArgc;
    fpush(mv, &valueToPush, NULL);

    // PUSH de la dirección de retorno (-1)
    valueToPush = NIL;
    fpush(mv, &valueToPush, NULL);
}

/* --- DECODIFICADORES --- */
/**
 * @brief Obtiene el índice de la tabla de segmentos a partir de un puntero lógico de segmento.
 */
Long decodeSeg(TMV* mv, Long cod) {
    return mv->reg[cod] >> 16;
}

/**
 * @brief Decodifica una dirección lógica (segmento:offset) a dirección física.
 *
 * Valida que el offset esté dentro de los límites del segmento y devuelve
 * la dirección física (base del segmento + offset). Si el offset está fuera
 * del segmento, setea el error ERR_SEG mediante errorHandler.
 */
Long decodeAddr(TMV* mv, Long logical) {
    Word segIndex, offset;

    segIndex = (Word) (logical >> 16);
    offset = (Word) logical;

    if (offset >= mv->seg[segIndex].size)
        errorHandler(mv, ERR_SEG);

    return mv->seg[segIndex].base + offset;
}

/**
 * @brief Indica si una dirección lógica está dentro del tamaño del segmento.
 *
 * Devuelve distinto de cero si offset < mv->seg[segIndex].size.
 */
int inSegment(TMV* mv, Long logical) {
    Word segIndex, offset;

    segIndex = (Word) (logical >> 16);
    offset = (Word) logical;

    return offset < mv->seg[segIndex].size;
}

/**
 * @brief Verifica si una dirección lógica pertenece al Code Segment.
 */
int inCS(TMV* mv, Long logical) {
    return ((logical >> 16) == (mv->reg[CS] >> 16)) && inSegment(mv, logical);
}

/* --- FETCH / DECODIFICACIÓN DE INSTRUCCIÓN --- */
/**
 * @brief Lee el byte de instrucción apuntado por IP y extrae OPC/OP1/OP2.
 *
 * Formato del byte:
 *  - bits 0..4: opcode (OPC)
 *  - bits 5..6: OP1 (puede no existir)
 *  - bits 6..7: OP2 o OP1 (dependiendo de la existencia del anterior)
 *
 * Efecto:
 *  - mv->reg[OPC], mv->reg[OP1], mv->reg[OP2] quedan cargados con los
 *    metadatos (tipo/tamaño/valor parcial).
 */
void fetchInstruction(TMV* mv) {
    Byte temp;

    temp = mv->mem[decodeAddr(mv, mv->reg[IP])];
    mv->reg[OPC] = 0;
    mv->reg[OPC] = (Long)(temp & MASKB_OPC);

    mv->reg[OP1] = 0;
    mv->reg[OP2] = 0;
    /*
     * Decodificación rápida del byte de instrucción:
     * - Los campos de tipo/tamaño de operandos se colocan en los 8 bits
     *   más significativos de mv->reg[OP1]/mv->reg[OP2] tras desplazar.
     * - Para simplificar la lectura posterior se almacenan los metadatos
     *   ya alineados en los registros OP1/OP2; los bytes reales del
     *   operando se OR-ean más tarde (fetchOperators -> fetchOperand).
     *
     * Notas sobre los shifts:
     * - MASKB_OP2 contiene los bits de tipo para el segundo operando
     *   (o el primero si no hay dos operandos). Se desplaza 18 para
     *   posicionar su tamaño/tipo en el byte más alto del Long.
     * - Si existe el campo OP1, movemos lo previamente puesto en OP1
     *   a OP2 y colocamos el nuevo tipo de OP1 desplazado 20 para
     *   mantener consistencia de alineación.
     */
    mv->reg[OP1] = (Long)(temp & MASKB_OP2) << 18;

    if ((temp & MASKB_OP1) != 0) {
        /* Si hay dos operandos: rotamos y colocamos los tipos en los
         * bytes altos correspondientes para que fetchOperators pueda
         * extraer el número de bytes a leer con (reg >> 24).
         */
        mv->reg[OP2] = mv->reg[OP1];
        mv->reg[OP1] = (Long)(temp & MASKB_OP1) << 20;
    }
}

/**
 * @brief Completa la información de los operandos leyendo los bytes siguientes.
 *
 * Usa los campos de tipo previamente cargados en OP1/OP2 para determinar
 * cuántos bytes leer de la instrucción y construye los valores parciales.
 */
void fetchOperators(TMV* mv) {
    int op1Bytes, op2Bytes, offset = 1;

    op2Bytes = mv->reg[OP2] >> 24;
    op1Bytes = mv->reg[OP1] >> 24;
    mv->reg[OP2] |= fetchOperand(mv, op2Bytes, &offset);
    if (mv->errorFlag == 0)
        mv->reg[OP1] |= fetchOperand(mv, op1Bytes, &offset);
}

/**
 * @brief Lee 'bytes' desde la secuencia de instrucciones y construye el operando.
 *
 * Parsea los bytes a partir de mv->reg[IP] + *offset y devuelve un valor
 * Long que contiene el contenido bruto del operando (sin interpretación).
 *
 * Si el operando es un inmediato (ocupa 2 bytes) y el MSB indica signo, se hace 
 * sign-extension manual para que el valor devuelto esté correctamente extendido a 24 bits.
 */
Long fetchOperand(TMV* mv, int bytes, int* offset) {
    Long logical, physical, temp = 0, i = bytes;

    while (i > 0 && mv->errorFlag == 0) {
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

/**
 * @brief Avanza el registro IP en función del tamaño de la instrucción actual.
 *
 * Suma 1 (byte de la instruccion) más los bytes de OP1 y OP2 (almacenados en los
 * 8 bits más significativos de OP1/OP2) para posicionar IP en la siguiente
 * instrucción.
 */
void addIP(TMV* mv) {
    mv->reg[IP] += (Long) (1 + (mv->reg[OP1] >> 24) + (mv->reg[OP2] >> 24));
}

/* --- SUBSISTEMA DE MEMORIA --- */
/**
 * @brief Calcula una dirección lógica a partir de un operando de memoria [reg+off] y la guarda en LAR.
 */
void setLARFromOperand(TMV* mv, Long operand) {
    UByte baseRegIndex;
    Word offset;
    Long logicalAddress;

    baseRegIndex = (operand >> 16) & 0x1F;
    offset = (Word) operand;

    // Obtiene el valor del registro base y le suma el offset
    logicalAddress = mv->reg[baseRegIndex] + offset;

    if (!inSegment(mv, logicalAddress))
        errorHandler(mv, ERR_SEG);
    else
        mv->reg[LAR] = logicalAddress;
}

/**
 * @brief Valida una dirección lógica que ya está calculada y la guarda en LAR.
 */
void setLARFromAddress(TMV* mv, Long logicalAddress) {
    if (!inSegment(mv, logicalAddress))
        errorHandler(mv, ERR_SEG);
    else
        mv->reg[LAR] = logicalAddress;
}

/**
 * @brief Calcula y setea el registro MAR a partir de una dirección lógica.
 *
 * MAR almacena en sus 2 bytes altos la cantidad de bytes y en los 2 bajos
 * la dirección física. Esta función valida límites y actualiza mv->reg[MAR].
 */
void setMAR(TMV* mv, Long cantBytes, Long logical) {
    Long physical;

    if (mv->errorFlag == 0) {
        physical = decodeAddr(mv, logical);
        if (physical + cantBytes - 1 >= mv->totalMemorySize)
            errorHandler(mv, ERR_SEG);
        else
            /* MAR encoding: los 16 bits altos almacenan la cantidad de
             * bytes a transferir, los 16 bits bajos contienen la
             * dirección física. Ej: MAR = (cantBytes << 16) | physical
             */
            mv->reg[MAR] = (cantBytes << 16) | physical;
    }
}

/**
 * @brief Lee desde memoria física al registro MBR usando el contenido de MAR.
 *
 * Interpreta mv->reg[MAR] para determinar cantidad de bytes y dirección
 * física y coloca el valor resultante en mv->reg[MBR].
 */
void getMemory(TMV* mv) {
    Long cantBytes, physical, temp;
    int i;

    if (mv->errorFlag == 0) {
        cantBytes = mv->reg[MAR] >> 16;
        physical = mv->reg[MAR] & MASK_PHY;
        temp = 0;

        for (i = 0; i < cantBytes; i++) {
            /* Leemos bytes en orden de mayor a menor significancia y los
             * concatenamos en temp. Si cantBytes < 4, el valor queda en
             * las posiciones menos significativas del Long y se corresponde
             * con la representación big-endian del dato leído.
             */
            temp |= (Long) mv->mem[physical + i] << (8 * (cantBytes - 1 - i));
        }

        mv->reg[MBR] = temp;
    }
}

/**
 * @brief Escribe el contenido de MBR en memoria física según MAR.
 *
 * Trunca el valor si el tamaño a escribir es menor que 4 bytes.
 */
void setMemory(TMV* mv) {
    Long cantBytes, physical;
    int i;

    if (mv->errorFlag == 0) {
        cantBytes = mv->reg[MAR] >> 16;
        physical = mv->reg[MAR] & MASK_PHY;

        /* Escribimos desde el byte más significativo del campo MBR hacia
         * el menos significativo según cantBytes; si cantBytes < 4, se
         * truncará el valor superior automáticamente (operación de shift).
         */
        for (i = 0; i < cantBytes; i++)
            mv->mem[physical + i] = mv->reg[MBR] >> (8 * (cantBytes - 1 - i)) & MASK_SETMEM;
    }
}

/* --- OPERANDOS (lectura/escritura) --- */
/**
 * @brief Obtiene el valor de un operando, considerando su tipo y tamaño variable.
 * Siempre devuelve un Long (32 bits) con el signo extendido.
 */
Long getOP(TMV* mv, Long operand) {
    UByte tipo, size, cantBytes, regIndex;
    Long res = 0, shift;

    tipo = operand >> 24;                    // Los 2 bits de tipo

    switch (tipo) {
        case 1: { // REGISTRO (extendido, sin prefijo, sufijo L, sufijo H)
            regIndex = operand & 0x0000001F;                    // Los 5 bits del registro
            size = (operand >> 6) & MASK_SIZE;                  // Los 2 bits de tamano

            switch (size) {
                case 0: // extendido (4 bytes)
                    res = mv->reg[regIndex];
                    break;
                case 1: // sufijo L (cuarto byte)
                    res = mv->reg[regIndex] & 0xFF;
                    if (res & 0x80) res |= 0xFFFFFF00; // Extensión de signo
                    break;
                case 2: // sufijo H (tercer byte)
                    res = (mv->reg[regIndex] >> 8) & 0xFF;
                    if (res & 0x80) res |= 0xFFFFFF00; // Extensión de signo
                    break;
                case 3: // sin prefijo (2 bytes)
                    res = mv->reg[regIndex] & 0xFFFF;
                    if (res & 0x8000) res |= 0xFFFF0000; // Extensión de signo
                    break;
            }
            break;
        }
        case 2: { // INMEDIATO
            res = (Word) operand;
            break;
        }
        case 3: { // MEMORIA (b, w, l)
            size = (operand >> 22) & MASK_SIZE;                 // Los 2 bits de tamano
            cantBytes = 4 - size;

            setLARFromOperand(mv, operand & 0x001FFFFF); // Pasamos el operando sin los bits de tamaño
            setMAR(mv, cantBytes, mv->reg[LAR]);
            getMemory(mv); // getMemory ahora lee 'cantBytes' en MBR
            res = mv->reg[MBR];

            // Extensión de signo manual después de la lectura
            if (cantBytes < 4) {
                shift = (4 - cantBytes) * 8;
                // Movemos el valor a la izquierda y luego de vuelta a la derecha
                // para que el bit de signo se propague.
                res = (res << shift) >> shift;
            }
            break;
        }
    }

    return res;
}

/**
 * @brief Asigna un valor a un operando, considerando su tipo y tamaño variable.
 */
void setOP(TMV* mv, Long operandA, Long operandB) {
    UByte tipo, size, regIndex, cantBytes;

    tipo = operandA >> 24;

    switch (tipo) {
        case 1: { // REGISTRO
            regIndex = operandA & 0x0000001F;                    // Los 5 bits del registro
            size = (operandA >> 6) & MASK_SIZE;                  // Los 2 bits de tamano

            switch (size) {
                case 0: // Extendido: se reemplaza todo el registro
                    mv->reg[regIndex] = operandB;
                    break;
                case 1: // Cuarto byte: solo se modifica el byte bajo
                    mv->reg[regIndex] = (mv->reg[regIndex] & 0xFFFFFF00) | (operandB & 0xFF);
                    break;
                case 2: // Tercer byte: solo se modifica el segundo byte bajo
                    mv->reg[regIndex] = (mv->reg[regIndex] & 0xFFFF00FF) | ((operandB & 0xFF) << 8);
                    break;
                case 3: // Dos bytes: solo se modifican los 2 bytes bajos
                    mv->reg[regIndex] = (mv->reg[regIndex] & 0xFFFF0000) | (operandB & 0xFFFF);
                    break;
            }
            break;
        }
        case 3: { // MEMORIA
            size = (operandA >> 22) & MASK_SIZE;                 // Los 2 bits de tamano
            cantBytes = 4 - size;

            setLARFromOperand(mv, operandA & 0x001FFFFF);
            setMAR(mv, cantBytes, mv->reg[LAR]);
            mv->reg[MBR] = operandB; // setMemory truncará el valor a 'cantBytes'
            setMemory(mv);
            break;
        }
    }
}


/**
 * @brief Actualiza el registro CC (flags) según el valor proporcionado.
 *
 * - Bit Z (0x40000000) se setea si valor == 0.
 * - Bit N (0x80000000) se setea si valor < 0.
 */
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

/* --- SERVICIOS DEL SISTEMA (SYS) --- */
/**
 * @brief SYS: Dispone las llamadas al sistema.
 *
 * El primer operando indica la función SYS a ejecutar (índice en sysTable).
 */
void fsys(TMV* mv, Long* value1, Long* value2) {
    if (sysTable[*value1]) {
        sysTable[*value1](mv);
    }
}

/**
 * @brief SYS read: lee valores desde stdin y los escribe en memoria.
 *
 * Interpreta EAX/ECX/EDX para conocer formato, cantidad y dirección.
 */
void fsysRead(TMV* mv) {
    Long read = 0;
    char car, binStr[33];
    int cantCeldas, cantBytes, i = 0, j = 0;

    cantCeldas = mv->reg[ECX] & MASK_LDL;
    cantBytes = (mv->reg[ECX] & MASK_LDH) >> 16;

    while (mv->errorFlag == 0 && i < cantCeldas) {
        setLARFromAddress(mv, mv->reg[EDX] + cantBytes * i);
        setMAR(mv, cantBytes, mv->reg[LAR]);

        if (mv->errorFlag == 0) {
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

    while ((car = getchar()) != '\n' && car != EOF);
}

/**
 * @brief Convierte un entero (Long) a su representación binaria en C2.
 *
 * Rellena el buffer binStr con la representación en complemento a 2. El
 * llamador debe asegurar que binStr tenga suficiente espacio (>=33 bytes
 * para 32 bits + terminador).
 */
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

/**
 * @brief SYS write: lee memoria y volca su representación por stdout.
 *
 * Interpreta EAX/ECX/EDX para conocer formato, cantidad y dirección.
 */
void fsysWrite(TMV* mv) {
    Long write;
    char c, binStr[33];
    int cantCeldas, cantBytes, i = 0, j = 0;

    cantCeldas = mv->reg[ECX] & MASK_LDL;
    cantBytes = (mv->reg[ECX] & MASK_LDH) >> 16;

    while (mv->errorFlag == 0 && i < cantCeldas) {
        setLARFromAddress(mv, mv->reg[EDX] + cantBytes * i);
        setMAR(mv, cantBytes, mv->reg[LAR]);
        getMemory(mv);
        write = mv->reg[MBR];

        if (mv->errorFlag == 0) {
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
                /* Imprime cada byte como caracter, de más significativo a menos.
                 * Se protegen los no imprimibles con '.' para mantener salida
                 * alineada y legible en consola.
                 */
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

/**
 * @brief SYS string read: lee una línea desde stdin y la escribe en memoria.
 *
 * Lee hasta ENTER o hasta el límite indicado en ECX (si ECX == NIL se
 * considera ilimitado). Escribe el terminador '\0' al final.
 */
void fsysStrRead(TMV* mv) {
    UWord cantMaxCar, i = 0;
    char car;

    cantMaxCar = (UWord) mv->reg[ECX];

    // Leemos caracter por caracter hasta Enter, fin de archivo, o el límite
    while ((cantMaxCar == NIL || i < cantMaxCar) && (car = getchar()) != '\n' && car != EOF) {
        setLARFromAddress(mv, mv->reg[EDX] + i);
        setMAR(mv, 1, mv->reg[LAR]);
        mv->reg[MBR] = (Long)car;
        setMemory(mv);
        i++;
    }

    // Escribimos el terminador nulo '\0' al final
    setLARFromAddress(mv, mv->reg[EDX] + i);
    setMAR(mv, 1, mv->reg[LAR]);
    mv->reg[MBR] = '\0';
    setMemory(mv);

    if (car != '\n' && car != EOF) {
        while ((car = getchar()) != '\n' && car != EOF);
    }
}

/**
 * @brief SYS string write: imprime una cadena almacenada en memoria.
 *
 * Lee byte a byte desde la dirección en EDX hasta encontrar '\0'.
 */
void fsysStrWrite(TMV* mv) {
    Long currentAddr;
    char car;

    currentAddr = mv->reg[EDX];
    car = ' ';

    while (car != '\0' && mv->errorFlag == 0) {
        setLARFromAddress(mv, currentAddr);
        setMAR(mv, 1, mv->reg[LAR]);
        getMemory(mv);

        car = (char) mv->reg[MBR];
        if (car != '\0') {
            printf("%c", car);
        }
        currentAddr++;
    }
}

/**
 * @brief Limpia la pantalla de la consola (SYS 0x07) llamando a un comando del sistema.
 */
void fsysClrScr(TMV* mv) {
    // The #ifdef _WIN32 directive checks if the code is being compiled on Windows.
    #ifdef _WIN32
        system("cls");
    // #else runs for any other operating system (like Linux or macOS).
    #else
        system("clear");
    #endif
}

/**
 * @brief Maneja la lógica del BREAKPOINT (SYS 0x0F), guardando la imagen de la MV y esperando comandos del usuario.
 */
void fsysBreakpoint(TMV* mv) {
    char car = 0;

    // Condición de entrada: hay un path .vmi y NO estamos ya en un breakpoint
    if (mv->config.vmiPath != NULL && mv->breakFlag == 0) {
        mv->breakFlag = 1; // Activamos la bandera para prevenir anidacion
        writeImage(mv);

        do {
            scanf("%c", &car);

            if (car == 10 || car == 13) { // Enter
                executeOneInstruction(mv);
                if (mv->errorFlag == 0)
                    writeImage(mv);     // Si la instrucción no causó un error, guardamos la nueva imagen
            }
        // Condición de salida: 'g', 'q', error de la VM, o fin del programa
        } while (car != 'g' && car != 'q' && mv->errorFlag == 0 && inCS(mv, mv->reg[IP]));

        // Si salimos del bucle por presionar 'q', marcamos el fin del programa
        if (car == 'q') {
            mv->reg[IP] = NIL;
        }

        mv->breakFlag = 0; // Desactivamos la bandera al salir del breakpoint
    }
}

/* --- LÓGICA DE MICROSECUENCIAS / SALTOS --- */
/**
 * @brief Lógica centralizada de saltos (JMP/JZ/JP/...).
 *
 * Si la condición correspondiente al opcode actual (mv->reg[OPC]) se cumple,
 * reemplaza el offset bajo de IP por value1 (manteniendo la parte alta).
 */
void fmsl(TMV* mv, Long* value1, Long* value2) {
    if (condVector[mv->reg[OPC]](mv)) {
        mv->reg[IP] &= MASK_LDH;
        mv->reg[IP] |= (*value1 & MASK_LDL);
    }
}

/**
 * @brief Condición: Z flag (cero) activa.
 *
 * Devuelve distinto de cero si Z está seteado.
 */
Long fjz(TMV* mv) {
    return mv->reg[CC] & CC_Z;
}

/**
 * @brief Condición: N flag (negativo) activa.
 *
 * Devuelve distinto de cero si N está seteado.
 */
Long fjn(TMV* mv) {
    return mv->reg[CC] & CC_N;
}

/* --- INSTRUCCIONES ARITMÉTICAS Y LÓGICAS --- */
/**
 * @brief NOT: niega bit a bit el operando value1.
 */
void fnot(TMV* mv, Long* value1, Long* value2) {
    *value1 = ~ *value1;
}

/**
 * @brief PUSH: Empuja un valor en la pila.
 *
 * Disminuye SP en 4 y escribe el valor en la nueva posición de pila. Se
 * valida overflow de stack comparando direcciones físicas.
 */
void fpush(TMV* mv, Long* value1, Long* value2) {
    mv->reg[SP] -= 4;
    if (decodeAddr(mv, mv->reg[SP]) < decodeAddr(mv, mv->reg[SS]))
        errorHandler(mv, ERR_SOF);
    else {
        setLARFromAddress(mv, mv->reg[SP]);
        setMAR(mv, 4, mv->reg[LAR]);
        mv->reg[MBR] = *value1;
        setMemory(mv);
    }
}

/**
 * @brief POP: Saca un valor de la pila y lo devuelve en value1.
 *
 * Verifica underflow, lee 4 bytes desde la dirección apuntada por SP y
 * luego incrementa SP.
 */
void fpop(TMV* mv, Long* value1, Long* value2) {
    Long stackEndAddress;

    stackEndAddress = mv->reg[SS] + mv->seg[decodeSeg(mv, SS)].size;
    if (mv->reg[SP] >= stackEndAddress)
        errorHandler(mv, ERR_SUF);
    else {
        setLARFromAddress(mv, mv->reg[SP]);
        setMAR(mv, 4, mv->reg[LAR]);
        getMemory(mv);
        *value1 = mv->reg[MBR];
        mv->reg[SP] += 4;
    }
}

/**
 * @brief CALL: guarda la dirección de retorno en la pila y salta a value1.
 */
void fcall(TMV* mv, Long* value1, Long* value2) {
    Long returnAddr;

    returnAddr = mv->reg[IP];
    fpush(mv, &returnAddr, NULL);
    mv->reg[IP] &= MASK_LDH;
    mv->reg[IP] |= (*value1 & MASK_LDL);
}

/**
 * @brief RET: recupera la dirección de retorno (POP) y la carga en IP.
 */
void fret(TMV* mv, Long* value1, Long* value2) {
    fpop(mv, value1, value2);
    mv->reg[IP] = *value1;
}

/**
 * @brief STOP: finaliza la ejecución fijando IP a NIL.
 */
void fstop(TMV* mv, Long* value1, Long* value2) {
    mv->reg[IP] = NIL;
}

/**
 * @brief MOV: copia el valor de value2 en value1, modificando value1.
 */
void fmov(TMV* mv, Long* value1, Long* value2) {
    *value1 = *value2;
}

/**
 * @brief ADD: suma value2 a value1, modificando value1.
 */
void fadd(TMV* mv, Long* value1, Long* value2) {
    *value1 = *value1 + *value2;
}

/**
 * @brief SUB: resta value2 de value1, modificando value1.
 */
void fsub(TMV* mv, Long* value1, Long* value2) {
    *value1 = *value1 - *value2;
}

/**
 * @brief MUL: multiplica value1 por value2, modificando value1.
 */
void fmul(TMV* mv, Long* value1, Long* value2) {
    *value1 = *value1 * *value2;
}

/**
 * @brief DIV: divide value1 entre value2, guarda el cociente en value1 y el resto en AC.
 */
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

/**
 * @brief SHL: desplaza value1 a la izquierda por value2 bits.
 */
void fshl(TMV* mv, Long* value1, Long* value2) {
    *value1 = *value1 << *value2;
}

/**
 * @brief SHR: desplaza lógicamente (no propaga signo) value1 a la derecha por value2 bits.
 */
void fshr(TMV* mv, Long* value1, Long* value2) {
    *value1 = (Long) ((ULong) *value1 >> *value2);
}

/**
 * @brief SAR: desplaza aritméticamente (propaga signo) value1 a la derecha por value2 bits.
 */
void fsar(TMV* mv, Long* value1, Long* value2) {
    *value1 = *value1 >> *value2;
}

/**
 * @brief AND: operación bit a bit AND entre value2 y value1, modificando value1.
 */
void fand(TMV* mv, Long* value1, Long* value2) {
    *value1 = *value1 & *value2;
}

/**
 * @brief OR: operación bit a bit OR entre value2 y value1, modificando value1.
 */
void f_or(TMV* mv, Long* value1, Long* value2) {
    *value1 = *value1 | *value2;
}

/**
 * @brief XOR: operación bit a bit XOR entre value2 y value1, modificando value1.
 */
void fxor(TMV* mv, Long* value1, Long* value2) {
    *value1 = *value1 ^ *value2;
}

/**
 * @brief SWAP: intercambia value1 y value2.
 */
void fswap(TMV* mv, Long* value1, Long* value2) {
    Long aux = 0;
    aux = *value1;
    *value1 = *value2;
    *value2 = aux;
}

/**
 * @brief LDL: carga los 2 bytes menos significativos de value1 con los 2 bytes
 * menos significativos de value2.
 */
void fldl(TMV* mv, Long* value1, Long* value2) {
    *value2 = *value2 & MASK_LDL;
    *value1 = (*value1 & MASK_LDH) | *value2;
}

/**
 * @brief LDH: carga los 2 bytes más significativos de value1 con los 2 bytes
 * menos significativos de value2.
 */
void fldh(TMV* mv, Long* value1, Long* value2) {
    *value2 = (*value2 & MASK_LDL) << 16;
    *value1 = (*value1 & MASK_LDL) | *value2;
}

/**
 * @brief RND: genera un entero aleatorio en [0, value2) y lo escribe en value1.
 *
 * Si value2 <= 1, el resultado es 0. Se usa rand() con rechazo para evitar
 * sesgos cuando RAND_MAX no es múltiplo de value2.
 */
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
