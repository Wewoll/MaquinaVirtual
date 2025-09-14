# 🖥️ Máquina Virtual

**Trabajo Práctico - Fundamentos de la Arquitectura de Computadoras**

---

## Autores

- Lucas Paniagua
- Victoria Cherepakhin
- Nicolas Zuviria

---

## Índice

- [Descripción](#descripción)
- [Objetivos](#objetivos)
- [Componentes de la Máquina Virtual](#componentes-de-la-máquina-virtual)
- [Ejecución](#ejecución)
- [Uso del Traductor (`vmt`)](#uso-del-traductor-vmt)
- [Uso de la Máquina Virtual (`vmx`)](#uso-de-la-máquina-virtual-vmx)
- [Créditos](#créditos)

---

## Descripción

Este trabajo práctico consiste en desarrollar una aplicación, en un lenguaje de programación a elección, que **emule la ejecución de un programa en el lenguaje máquina** de una computadora descripta en el documento de la cátedra.

El programa a ejecutar debe estar previamente escrito en el lenguaje **Assembler** de la máquina virtual y traducido a lenguaje máquina utilizando el programa traductor (`vmt`) provisto por la cátedra.

---

## Objetivos

- Simular el funcionamiento de una máquina virtual.
- Ejecutar instrucciones en lenguaje máquina generadas a partir de código Assembler.
- Profundizar en los conceptos de arquitectura de computadoras y emulación.

---

## Componentes de la Máquina Virtual

La máquina virtual implementada cuenta con los siguientes componentes principales:

- **Memoria principal (RAM):** 16 KiB.
- **Tabla de descriptores de segmentos.**
- **32 registros de 4 bytes** (en esta primera parte se utilizan 17).
- **Procesador** capaz de:
  - Decodificar instrucciones en lenguaje máquina.
  - Direccionar cada byte de la memoria principal.
  - Realizar operaciones aritméticas y lógicas en 32 bits.

---

## Ejecución

1. **Carga inicial:**  
   La máquina virtual lee el encabezado del programa para verificar si puede ejecutarlo. Si es así, carga el código en la memoria principal, arma la tabla de descriptores de segmentos e inicializa los registros.

2. **Inicialización de registros:**
   - **CS** y **DS** se cargan con punteros al comienzo del segmento de código y de datos, respectivamente.  
     - En **esta primera parte**, los valores iniciales son:
       - CS = `0x00000000`
       - DS = `0x00010000`
     - (**Nota:** En futuras entregas, estos valores pueden variar según la implementación de los segmentos.)
   - **IP** se inicializa apuntando a la primera instrucción del código (igual que CS).

3. **Ciclo de ejecución:**
   - Leer la instrucción apuntada por el registro **IP**.
   - Almacenar el código de operación en el registro **OPC**.
   - Guardar en los registros **OP1** y **OP2** los operandos A y B, respectivamente:
     - El byte más significativo indica el tipo de operando.
     - Los otros tres bytes contienen el valor del operando.
     - Si el operando no existe, el registro se llena con 0.
   - Actualizar **IP** a la próxima instrucción.
   - Ejecutar la operación correspondiente.

   La ejecución se repite hasta que **IP** apunte fuera del segmento de código. Si se ejecuta una instrucción **STOP**, la máquina virtual asigna `-1` (`0xFFFFFFFF`) al registro IP.

4. **Operaciones con memoria:**
   - El registro **LAR** almacena la dirección lógica a acceder.
   - El registro **MAR**: los 2 bytes altos indican la cantidad de bytes, los 2 bajos la dirección física tras la traducción.
   - El registro **MBR** contiene el valor leído o a escribir.
   - La lectura de instrucciones no modifica estos registros.

5. **Registro de condición (CC):**
   - Bit más significativo (**N**): 1 si el resultado es negativo, 0 en otro caso.
   - Segundo bit más significativo (**Z**): 1 si el resultado es cero, 0 en otro caso.

---

## Uso del Traductor (`vmt`)

El traductor, provisto por la cátedra, se utiliza desde una consola del siguiente modo:

```sh
vmt archivo.asm [archivo.vmx] [-o]
```

- **vmt**: ejecutable del traductor.
- **archivo.asm** (obligatorio): ruta y nombre del archivo de texto con el código fuente en Assembler.
- **archivo.vmx** (opcional): ruta y nombre del archivo generado en lenguaje máquina. Si se omite, se crea uno con el mismo nombre que el `.asm` pero con extensión `.vmx`.
- **-o** (opcional): flag para omitir la salida por pantalla de la traducción (no omite mensajes de error).

---

## Uso de la Máquina Virtual (`vmx`)

Se debe entregar el código fuente y el ejecutable compilado de la máquina virtual, la cual debe poder utilizarse desde una consola del siguiente modo:

```sh
vmx archivo.vmx [-d]
```

- **vmx**: ejecutable de la máquina virtual.
- **archivo.vmx** (obligatorio): ruta y nombre del archivo con el programa en lenguaje máquina.
- **-d** (opcional): flag que muestra el código desensamblado (assembler correspondiente al código máquina cargado en memoria).

---

## Créditos

Trabajo realizado para la asignatura **Fundamentos de la Arquitectura de Computadoras**.

**Docentes:**
- Lic. Pablo A. Montini
- Ing. Juan I. Iturriaga
- Ing. Franco Lanzillotta

**Ayudantes:**
- Erik Borgnia Giannini
- Francisco Verón