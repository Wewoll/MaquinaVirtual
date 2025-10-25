# 🖥️ Máquina Virtual

**Trabajo Práctico - Fundamentos de la Arquitectura de Computadoras**

---

## Autores

- Paniagua, Lucas
- Cherepakhin, Victoria
- Zuviria, Nicolas

---

## Índice

- [Descripción](#descripción)
- [Objetivos](#objetivos)
- [Uso de la Máquina Virtual](#uso-de-la-máquina-virtual)
- [Descripción de la Máquina Virtual](#descripción-de-la-máquina-virtual)
- [Ejecución](#ejecución)
- [Breakpoint](#breakpoint)
- [Créditos](#créditos)

---

## Descripción

Este trabajo práctico consiste en desarrollar una aplicación, en un lenguaje de programación a elección, que **emule la ejecución de un programa en el lenguaje máquina** de una computadora descripta en el documento de la cátedra.

El programa a ejecutar debe estar previamente escrito en el lenguaje **Assembler** de la máquina virtual y traducido a lenguaje máquina utilizando el programa traductor (`vmt`) provisto por la cátedra. Tambien, se debera poder complementar con el debugger (`vmg`) provisto por la cátedra para poder visualizar en tiempo real el estado de la máquina.

---

## Objetivos

- Simular el funcionamiento de una máquina virtual.
- Ejecutar instrucciones en lenguaje máquina generadas a partir de código Assembler.
- Profundizar en los conceptos de arquitectura de computadoras y emulación.

---

## Uso de la Máquina Virtual

El programa se ejecuta desde la línea de comandos con la siguiente sintaxis:

```sh
vmx [filename.vmx] [filename.vmi] [m=M] [-d] [-p param1 param2 ...]
```

- **vmx** es el programa ejecutable del proceso Ejecutor o Máquina Virtual.
- **filename.vmx** (opcional*) es la ruta y nombre del archivo con el programa en lenguaje máquina (puede ser cualquier nombre con extensión **.vmx**).
- **filename.vmi** (opcional*) es la ruta y nombre del archivo donde se almacenará la imagen de la máquina virtual (puede ser cualquier nombre con extensión **.vmi**).
- **m=M** (opcional) permite indicar el tamaño de la memoria principal, donde **M** es un valor expresado en KiB. Si se omite, el valor por defecto sigue siendo 16 KiB.
- **-d** (opcional) es un flag que fuerza a la máquina virtual a mostrar el código Assembler correspondiente al código máquina cargado en la memoria principal.
- **-p** (opcional) es un flag que sirve para indicar los parámetros (**param1 param2 ... paramN**) que se le deben pasar a la subrutina principal del proceso. Siempre se debe escribir al final del comando.

**Nota:** para la ejecución es obligatorio al menos uno de los dos archivos: .vmx y/o .vmi. En caso de no especificarse un archivo .vmx, se ignoran los parámetros -p.

---

## Descripción de la Máquina Virtual

La máquina virtual implementada cuenta con los siguientes componentes principales:

- Memoria principal (RAM) de tamaño variable.
- Tabla de descriptores de segmentos.
- 32 registros de 4 bytes.
- Procesador con capacidad para:
  - Decodificar instrucciones en lenguaje máquina.
  - Direccionar a cada byte de la memoria principal.
  - Realizar operaciones aritméticas y lógicas en 32 bits.

---

## Ejecución

La máquina virtual (vmx) gestiona el ciclo de vida completo de un proceso, desde su carga inicial hasta su finalización. El flujo se adapta según los archivos de entrada proporcionados (.vmx y/o .vmi).

1. **Escenarios de Carga:**
Existen tres modos principales de ejecución:

   - ***Ejecutar un Programa Nuevo (.vmx):*** Si recibe como entrada un archivo .vmx, crea la memoria según el tamaño definido, crea y ubica los segmentos según el header del archivo y parámetros, luego configura la tabla de segmentos e inicializa los sus respectivos registros. Además, debe inicializar el registro IP con el entry point y actualizar el Stack Segment y el Param Segment.

   - ***Continuar una Ejecución (.vmi):*** Si no recibe un .vmx y recibe solo un archivo de imagen .vmi, debe cargar la memoria principal, la tabla de segmentos y los registros tal como están en el archivo y continuar la ejecución.

   - ***Ejecución con Breakpoints (.vmx y .vmi):*** En caso de recibir ambos archivos, se ejecuta el archivo .vmx (como en el primer caso) y se utiliza el .vmi para generar la imagen en cada breakpoint.

2. **Ciclo de Ejecución:**
Una vez inicializada, la VM entra en el ciclo principal de fetch-decode-execute. Este ciclo se repite hasta que se cumple una de las condiciones de finalización.

   - ***Fetch (Búsqueda):*** Lee el byte de instrucción apuntado por IP y los bytes de sus operandos.

   - ***Decode (Decodificación):*** Interpreta la instrucción, los tipos de operando y sus tamaños, guardando esta información en los registros *OPC*, *OP1* y *OP2*.

   - ***Update IP (Sumar el tamaño de la instrucción actual):*** El IP se actualiza para apuntar a la siguiente instrucción.

   - ***Execute (Ejecución):*** Realiza la operación correspondiente a la instrucción, ya sea una operación aritmética, un salto, una llamada al sistema o una operación de pila.

3. **Condiciones de Finalización:**
La ejecución del programa termina si ocurre cualquiera de las siguientes situaciones:

   - El registro *IP* apunta a una dirección fuera de los límites del Code Segment.

   - Se ejecuta una instrucción *STOP*, que asigna *-1* (*NIL*) al registro *IP*.

   - La subrutina main ejecuta una instrucción *RET*, lo que hace un *POP* de la dirección de retorno inicial (*-1*) y la carga en el *IP*.

   - El usuario ingresa el carácter *'q'* (quit) durante un *breakpoint* para abortar la ejecución.

---

## Breakpoint
Los breakpoints son un tipo de llamada al sistema especial que permiten pausar o detener la ejecución para observar el estado actual de la máquina virtual. Estas llamadas al sistema deben ser ignoradas si al momento de ejecutar la máquina virtual no se ha incluido el parámetro que indica el archivo de imagen.

Cuando se ejecuta un breakpoint, la máquina virtual debe pausar su ejecución y generar un archivo de imagen. Luego, debe quedar a la espera de que el usuario realice una de las siguientes acciones:

- Si se ingresa el carácter 'g' (go), la máquina virtual continúa su ejecución hasta el próximo
breakpoint o hasta finalizar la ejecución.
- Si se ingresa el carácter 'q' (quit), la máquina virtual aborta la ejecución, dejando intacto el
archivo .vmi de modo que se pueda retomar la ejecución del mismo.
- Si únicamente se presiona la tecla Enter, la máquina virtual debe ejecutar la siguiente instrucción
y luego volver a realizar un nuevo breakpoint, sin importar qué haga dicha instrucción. Esto
posibilita que el código pueda ser ejecutado paso a paso.

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
