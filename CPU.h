/* 
 * File:   CPU.h
 * Author: USUARIO
 *
 * Created on 18 de mayo de 2018, 22:16
 */

#ifndef CPU_H
#define	CPU_H

#include <stdint.h>

enum ADDR_MODE {    
    ACCUMULATOR = 0,
    ABS = 1,
    ABS_X = 2,
    ABS_Y = 3,
    IMMEDIATE = 4, //#
    IMPL = 5, //implied
    IND = 6, //indirect
    X_IND = 7, //x-indexed indirect
    IND_Y = 8, //indirect y-indexed
    REL = 9, //relative
    ZPG = 10, //zero page
    ZPG_X = 11,
    ZPG_Y = 12
};
const unsigned int addr_mode_length[] = {
//ACCUM  //ABS   //ABS_X    //ABS_Y
    1,      3,      3,      3,
//#       //impl  //ind    //X_IND
    2,      1,      3,      2,
//IND_Y   //rel  //zpg    //zpg_x
    2,      2,      2,      2,
//zpg_y
    2
};

struct Instruction {
    const char* label; //Ins in Assembly
    uint8_t opcode; //hex value of the instruction
    unsigned cycles; //duration in cycles
    ADDR_MODE addr_mode; //addressing mode 
    bool keep_pc;   //if true, pc wont be incremented
}; //Note that length in bytes is implicit by the addr_mode

enum CPU_REG{
    REG_A = 0,
    REG_X = 1,
    REG_Y = 2,
};
enum P_flags {
    C_flag=1,
    Z_flag=2,
    I_flag=3,
    D_flag=4,
    V_flag=7,
    N_flag=8
};
class CPU {
    protected:
        //TEMP VARS
        uint8_t RAM[0xFFFF];
        uint8_t Stack[ 0xFF ];
        
        uint16_t PC;
        uint8_t  regs[ 3 ]; //A - 0, X - 1, Y - 2
        uint8_t  P;
        //NVssDIZC
        uint8_t  SP;
        
        uint8_t WB( uint16_t addr, uint8_t v );
        uint8_t RB( uint16_t addr );
        unsigned calcOperand( ADDR_MODE mode );
        void print_state();
        void push( uint8_t v );
        uint8_t pull();
    public:
        CPU( unsigned char* data, unsigned nbytes );
        unsigned run();
        Instruction evaluate( uint8_t opcode );
};

#endif	/* CPU_H */

