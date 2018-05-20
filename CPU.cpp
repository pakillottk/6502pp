#include "CPU.h"
#include <cstdlib>
#include <string.h>
#include <iostream>
#include <bitset>
#include "lib\PGMBinaryOps.h"

#define DEBUG_CPU true

void print_inst( Instruction* i ) {
    std::cout << i->label << " (0x" << std::hex << (int)i->opcode << ") cycles: " << i->cycles << std::endl;
}

CPU::CPU( unsigned char* data, unsigned nbytes ) {
    //TEMP CODE
    memset( RAM, 0, sizeof RAM );
    memcpy( &RAM[0xC000], data, nbytes );
    
    memset( regs, 0, sizeof regs );
    SP = 0xFD;
    P = 0x24;
    //TMP
    PC=0xC000;
    //PC=0xC000;
}

uint8_t CPU::RB(uint16_t addr) {
    std::cout << "Reading at " << std::hex << (int)addr <<std::endl;
    return RAM[addr];
}

uint8_t CPU::WB(uint16_t addr, uint8_t v) {
    std::cout << "Store at " << std::hex << (int)addr << ": " << (int)v << std::endl;
    return RAM[addr] = v;
}

unsigned CPU::run() {
    #if DEBUG_CPU
        std::cout << "Fetching at: " << std::hex << (int)PC << std::endl;
    #endif

    uint8_t opcode = RB(PC);
    #if DEBUG_CPU
        std::cout << "To decode: " << std::hex << (int)opcode << std::endl;
    #endif

    Instruction decoded = evaluate( opcode );
    if( !decoded.keep_pc ) {
       PC = PC + addr_mode_length[ decoded.addr_mode ]; 
    }   
    
    #if DEBUG_CPU
        print_inst(&decoded);
        print_state();
        
        std::cout << "02: " << std::hex << (int)RAM[2] << std::endl;
        std::cout << "03: " << std::hex << (int)RAM[3] << std::endl;
    #endif     

    return decoded.cycles;
}

void CPU::print_state() {
    std::cout << "  CPU STATE  " << std::endl;
    std::cout << std::hex 
              << "A:" <<(int)regs[REG_A] 
              << " X:" <<(int)regs[REG_X] 
              << " Y:" <<(int)regs[REG_Y] 
              << " P:" <<(int)P
              << " SP:" <<(int)SP << std::endl;
    
    std::cout << "bin P: " << std::bitset<8>(P) << std::endl;
    std::cout << "       NVssDIZC" << std::endl;
}
/*
 *  Returns the ins parameter (rel, immediate),
 *  the REG_A value (accumulator)
 *  or the effective address for mode
*/
unsigned CPU::calcOperand(ADDR_MODE mode) {
    const uint8_t t = RB(PC+1), s = RB(PC+2);
    uint16_t c = pgmbin::combineLittleEndian(t,s);
    uint8_t truncNextT = (t+1)&0xFF;
    uint16_t truncNextC = pgmbin::combineLittleEndian(truncNextT,s);
    switch( mode ) {
        case ACCUMULATOR:
            return regs[REG_A];
        case REL:
        case IMMEDIATE: {
            return t;
        }
        case IND: {
            return pgmbin::combineLittleEndian( RB(c), RB(truncNextC));
        }
        case ABS: {
            return c;
        }
        case ABS_X: {
            return c + regs[REG_X];
        }
        case ABS_Y: {
            return c + regs[REG_Y];
        }
        case ZPG: {
            return 0x00FF & t;
        }        
        case ZPG_X:{
            return 0x00FF & ( t + regs[REG_X] );
        }
        case ZPG_Y:{
            return 0x00FF & ( t + regs[REG_Y] );
        }
        case X_IND: {
            return pgmbin::combineLittleEndian(RB(t+regs[REG_X]&0xFF),RB((t+regs[REG_X]+1)&0xFF));
        }
        case IND_Y: {  
            return pgmbin::combineLittleEndian(RB(t&0xFF), RB((t + 1)&0xFF)) + regs[REG_Y];
        }
        case IMPL: {
            return RB(PC);
        }        
    }
}

void CPU::push( uint8_t v ) {
    RAM[ 0x100 + (SP--) ] = v;
}

uint8_t CPU::pull() {
    return RAM[0x100+(++SP)];
}

/*
 *       MACROS: OP TASKS
 *      =======
 *  Set little operations, which later combined
 *  produced the completion of the instruction.
 *  
 */
//All variables necessary to run the tasks
#define EVALUATOR_HEADER()\
    const char* label; bool keep_pc = false;\
    unsigned cycles = 1, operand, temp;\
    uint8_t tmp8b; uint16_t addr; ADDR_MODE mode;

//Sets the PC using ins operand as offset
#define OFFSET_PC() PC = pgmbin::combine(PC>>8, (PC + operand + 2)&0xFF) ; keep_pc = true;

//Sets PC to ins operand
#define SET_PC() PC = operand; keep_pc = true;

//PUSH 1Byte INTO STACK
#define STACK_PUSH( VAL ) push( VAL );
//PUSH 2Bytes INTO STACK (LSB stored at top) 
#define STACK_PUSH16( VAL16 ) STACK_PUSH( VAL16 >> 8 ) STACK_PUSH( VAL16&0x00FF )

//PULL from STACK into TO
#define STACK_PULL( TO ) TO = pull();
//PULL 2bytes from STACK into TO
#define STACK_PULL16( TO ) tmp8b = pull(); temp = pull(); TO = pgmbin::combineLittleEndian(tmp8b, temp);

//Set the PC from the Stack
#define RESTORE_PC() STACK_PULL16(PC) PC++; keep_pc = true;

//Sets N to 1 if VAL MSB is set
#define UPDATE_N( VAL ) P = pgmbin::setBitAt<N_flag>( pgmbin::getBitAt<8>(VAL), P );

//Sets Z to 1 if VAL is zero
#define UPDATE_Z( VAL ) P = pgmbin::setBitAt<Z_flag>( VAL == 0, P );

//UPDATE ZN flaggs with VAL
#define UPDATE_NZFLAGS( VAL ) UPDATE_N( VAL ) UPDATE_Z( VAL )

//Set REG to VAL and Updates NZ
#define SET_REG( REG, VAL ) regs[REG] = VAL; UPDATE_NZFLAGS( VAL )

//Store VAL into mem ADDR
#define STORE( ADDR, VAL ) WB( ADDR, VAL );

//Reads mem at ins operand (effective address calculated)
//sets operand as value in memory. Address is kept at addr
#define READ() addr = operand; operand = RB( operand );

//If COND OFFSET_PC is called
#define COND_BRANCH( COND )if( COND ){ OFFSET_PC() }

//Sets the C flag if VAL > 0xFF
#define UPDATE_C( VAL ) P = pgmbin::setBitAt<C_flag>( VAL > 0xFF, P ); 

//Sets the V flag in ADC SUB where VAL is set to operation result
#define UPDATE_V( VAL ) P = pgmbin::setBitAt<V_flag>(~(regs[REG_A] ^ operand) & (regs[REG_A] ^ VAL) & 0x80, P);

//Adds with carry into A and uppdates flags
#define ADC() temp = regs[REG_A] + operand + pgmbin::getBitAt<C_flag>( P ); UPDATE_C( temp ) UPDATE_V(temp)\
              tmp8b=temp; SET_REG(REG_A, tmp8b)
//Subs with borrow into A and update flags
#define SBC() operand=~operand&0xFF; ADC()

//Performs a bitwise operation between REG_A and operand (updates flags)
#define LOGIC_OP( OPERATOR ) tmp8b = regs[REG_A] OPERATOR operand; SET_REG( REG_A, tmp8b ) 

//Shift operand 1 bit to the left and update C flag
#define SHIFT_L() temp = ( operand << 1 ); UPDATE_C( temp ) temp&=0xFF;
//Shift operand 1 bit to the right and update C flag
#define SHIFT_R() P = pgmbin::setBitAt<C_flag>(pgmbin::getBitAt<1>(operand), P); temp = pgmbin::swapBits<8,1>(operand); temp = ( operand >> 1 ); 

//Shift operand 1 bit to left (stores in temp8b) set bit 1 to Carry. Set carry to original operand bit 8.
#define ROL() tmp8b = ( operand << 1 );  tmp8b = pgmbin::setBitAt<1>( pgmbin::getBitAt<C_flag>(P), tmp8b); P = pgmbin::setBitAt<C_flag>( pgmbin::getBitAt<8>(operand), P); 
//Shift operand 1 bit to right (stores in temp8b) set bit 8 to Carry. Set carry to original operand bit 1.
#define ROR() tmp8b = ( operand >> 1 );  tmp8b = pgmbin::setBitAt<8>( pgmbin::getBitAt<C_flag>(P), tmp8b); P = pgmbin::setBitAt<C_flag>( pgmbin::getBitAt<1>(operand), P);

//bits 8 and 7(1-based) to flags N,V. AND A with operand and update Z
#define BITS() tmp8b=regs[REG_A]&operand; P=pgmbin::setBitAt<Z_flag>(tmp8b==0, P); P = pgmbin::setBitAt<N_flag>( pgmbin::getBitAt<8>(operand), P); P = pgmbin::setBitAt<V_flag>( pgmbin::getBitAt<7>(operand), P);
//LOGIC_OP( &= ) P = pgmbin::setBitAt<N_flag>( pgmbin::getBitAt<8>(operand), P ); P = pgmbin::setBitAt<7>( pgmbin::getBitAt<7>(operand), P );
               
//Reads reg and sets Z = reg==operand C = reg >= operand and updates NZ with reg-operand
#define CMP(REG) tmp8b = regs[REG]; P = pgmbin::setBitAt<Z_flag>(tmp8b == operand, P);\
                 P = pgmbin::setBitAt<C_flag>(tmp8b >= operand, P); UPDATE_NZFLAGS(tmp8b-operand)

//Decrements VAL by 1, store in tmp8b
#define DEC(VAL) tmp8b = VAL-1;
//Increments VAL by 1, store in tmp8b
#define INC(VAL) tmp8b = VAL+1;

Instruction CPU::evaluate(uint8_t opcode) {
    EVALUATOR_HEADER()
    
    Instruction I;
    I.label = "nop";    
    I.opcode = 0xEA;    
    I.cycles = cycles;   
    I.addr_mode = IMPL;  
    I.keep_pc = false;
    
    //Defines a case for evaluate the OPCODE
    //==========================================
    #define OP( opcode, LABEL, CYCLES, ADDR, TASKS )\
    case opcode: { label = LABEL; cycles=CYCLES; mode = ADDR; operand = calcOperand(ADDR); TASKS; break; }
    //==========================================    
    switch( opcode ) {
       //  OPCODE      LABEL         CYCLES     ADDR_MODE                         TASKS
       //====================================================================================================================                      
       // ADC
        OP( 0x69,         "adc #",        2,    IMMEDIATE,                                                               ADC())
        OP( 0x65,       "adc zpg",        3,          ZPG,                                                        READ() ADC())
        OP( 0x75,     "adc zpg,X",        4,        ZPG_X,                                                        READ() ADC())
        OP( 0x6d,       "adc abs",        4,          ABS,                                                        READ() ADC())
        OP( 0x7d,     "adc abs,X",        4,        ABS_X,                                                        READ() ADC())
        OP( 0x79,     "adc abs,Y",        4,        ABS_Y,                                                        READ() ADC())
        OP( 0x61,     "adc ind,X",        6,        X_IND,                                                        READ() ADC())        
        OP( 0x71,     "adc ind,Y",        5,        IND_Y,                                                        READ() ADC())                 
        //AND
        OP( 0x29,         "and #",        2,    IMMEDIATE,                                                         LOGIC_OP(&))
        OP( 0x25,       "and zpg",        3,          ZPG,                                                  READ() LOGIC_OP(&))
        OP( 0x35,     "and zpg,x",        4,        ZPG_X,                                                  READ() LOGIC_OP(&))
        OP( 0x2d,       "and abs",        4,          ABS,                                                  READ() LOGIC_OP(&))
        OP( 0x3d,     "and abs,x",        4,        ABS_X,                                                  READ() LOGIC_OP(&))
        OP( 0x39,     "and abs,y",        4,        ABS_Y,                                                  READ() LOGIC_OP(&))
        OP( 0x21,     "and ind,x",        6,        X_IND,                                                  READ() LOGIC_OP(&))
        OP( 0x31,     "and ind,y",        5,        IND_Y,                                                  READ() LOGIC_OP(&))       
        //ASL
        OP( 0x0a,         "ASL A",        2,  ACCUMULATOR,                                      SHIFT_L() SET_REG(REG_A, temp))
        OP( 0x06,       "ASL zpg",        5,          ZPG,           READ() SHIFT_L() STORE(addr, temp) UPDATE_NZFLAGS( temp ))
        OP( 0x16,     "ASL zpg,x",        6,        ZPG_X,           READ() SHIFT_L() STORE(addr, temp) UPDATE_NZFLAGS( temp ))
        OP( 0x0e,       "ASL abs",        6,          ABS,           READ() SHIFT_L() STORE(addr, temp) UPDATE_NZFLAGS( temp ))
        OP( 0x1e,     "ASL abs,x",        7,        ABS_X,           READ() SHIFT_L() STORE(addr, temp) UPDATE_NZFLAGS( temp ))        
        //CONDITIONAL JUMPS
        OP( 0xb0,       "BCS rel",        2,          REL,                           COND_BRANCH( pgmbin::getBitAt<C_flag>(P)))
        OP( 0x90,       "BCC rel",        2,          REL,                           COND_BRANCH(!pgmbin::getBitAt<C_flag>(P)))        
        OP( 0xf0,       "BEQ rel",        2,          REL,                           COND_BRANCH( pgmbin::getBitAt<Z_flag>(P))) 
        OP( 0xd0,       "BNE rel",        2,          REL,                           COND_BRANCH(!pgmbin::getBitAt<Z_flag>(P)))
        OP( 0x30,       "BMI rel",        2,          REL,                           COND_BRANCH( pgmbin::getBitAt<N_flag>(P)))            
        OP( 0x10,       "BPL rel",        2,          REL,                           COND_BRANCH(!pgmbin::getBitAt<N_flag>(P))) 
        OP( 0x70,       "BVC rel",        2,          REL,                           COND_BRANCH( pgmbin::getBitAt<V_flag>(P)))
        OP( 0x50,       "BVC rel",        2,          REL,                           COND_BRANCH(!pgmbin::getBitAt<V_flag>(P)))                
        //BITs
        OP( 0x24,       "bit zpg",        3,          ZPG,                                                       READ() BITS())
        OP( 0x2c,       "bit abs",        4,          ABS,                                                       READ() BITS())        
        //FLAG CLEARS
        OP( 0x18,           "CLC",        2,         IMPL,                                  P = pgmbin::setBitAt<C_flag>(0, P))
        OP( 0xd8,           "CLD",        2,         IMPL,                                  P = pgmbin::setBitAt<D_flag>(0, P))
        OP( 0x58,           "CLI",        2,         IMPL,                                  P = pgmbin::setBitAt<I_flag>(0, P))
        OP( 0xb8,           "CLV",        2,         IMPL,                                  P = pgmbin::setBitAt<V_flag>(0, P))        
        //CMPs
        OP( 0xc9,         "cmp #",        2,     IMMEDIATE,                                                         CMP(REG_A))
        OP( 0xc5,       "cmp zpg",        3,          ZPG,                                                   READ() CMP(REG_A))
        OP( 0xd5,     "cmp zpg,x",        4,        ZPG_X,                                                   READ() CMP(REG_A))
        OP( 0xcd,       "cmp abs",        4,          ABS,                                                   READ() CMP(REG_A))
        OP( 0xdd,     "cmp abs,x",        4,        ABS_X,                                                   READ() CMP(REG_A))
        OP( 0xd9,     "cmp abs,y",        4,        ABS_Y,                                                   READ() CMP(REG_A))
        OP( 0xc1,     "cmp ind,x",        6,        X_IND,                                                   READ() CMP(REG_A))
        OP( 0xd1,     "cmp ind,y",        5,        IND_Y,                                                   READ() CMP(REG_A))        
        //CPXs
        OP( 0xe0,         "cpx #",        2,     IMMEDIATE,                                                         CMP(REG_X))
        OP( 0xe4,       "cpx zpg",        3,          ZPG,                                                   READ() CMP(REG_X))
        OP( 0xec,       "cpx abs",        4,          ABS,                                                   READ() CMP(REG_X))              
        //CPYs
        OP( 0xc0,         "cpy #",        2,     IMMEDIATE,                                                         CMP(REG_Y))
        OP( 0xc4,       "cpy zpg",        3,          ZPG,                                                   READ() CMP(REG_Y))
        OP( 0xcc,       "cpy abs",        4,          ABS,                                                   READ() CMP(REG_Y))        
        //DECs
        OP( 0xc6,       "DEC zpg",        5,          ZPG,        READ() DEC(operand) UPDATE_NZFLAGS(tmp8b) STORE(addr, tmp8b))
        OP( 0xd6,     "DEC zpg,x",        6,        ZPG_X,        READ() DEC(operand) UPDATE_NZFLAGS(tmp8b) STORE(addr, tmp8b))
        OP( 0xce,       "DEC abs",        3,          ABS,        READ() DEC(operand) UPDATE_NZFLAGS(tmp8b) STORE(addr, tmp8b))
        OP( 0xde,     "DEC abs,x",        7,        ABS_X,        READ() DEC(operand) UPDATE_NZFLAGS(tmp8b) STORE(addr, tmp8b))        
        //DEX
        OP( 0xca,      "DEX impl",        2,         IMPL,                              DEC(regs[REG_X]) SET_REG(REG_X, tmp8b))
        //DEY
        OP( 0x88,      "DEY impl",        2,         IMPL,                              DEC(regs[REG_Y]) SET_REG(REG_Y, tmp8b))  
        //EOR
        OP( 0x49,         "eor #",        2,    IMMEDIATE,                                                         LOGIC_OP(^))
        OP( 0x45,       "eor zpg",        3,          ZPG,                                                  READ() LOGIC_OP(^))
        OP( 0x55,     "eor zpg,x",        4,        ZPG_X,                                                  READ() LOGIC_OP(^))
        OP( 0x4d,       "eor abs",        4,          ABS,                                                  READ() LOGIC_OP(^))
        OP( 0x5d,     "eor abs,x",        4,        ABS_X,                                                  READ() LOGIC_OP(^))
        OP( 0x59,     "eor abs,y",        4,        ABS_Y,                                                  READ() LOGIC_OP(^))
        OP( 0x41,     "eor ind,x",        6,        X_IND,                                                  READ() LOGIC_OP(^))
        OP( 0x51,     "eor ind,y",        5,        IND_Y,                                                  READ() LOGIC_OP(^))
        //INCs
        OP( 0xe6,       "INC zpg",        5,          ZPG,        READ() INC(operand) UPDATE_NZFLAGS(tmp8b) STORE(addr, tmp8b))
        OP( 0xf6,     "INC zpg,x",        6,        ZPG_X,        READ() INC(operand) UPDATE_NZFLAGS(tmp8b) STORE(addr, tmp8b))
        OP( 0xee,       "INC abs",        6,          ABS,        READ() INC(operand) UPDATE_NZFLAGS(tmp8b) STORE(addr, tmp8b))
        OP( 0xfe,     "INC abs,x",        7,        ABS_X,        READ() INC(operand) UPDATE_NZFLAGS(tmp8b) STORE(addr, tmp8b)) 
        //INX
        OP( 0xe8,      "INX impl",        2,         IMPL,                              INC(regs[REG_X]) SET_REG(REG_X, tmp8b))
        //INY
        OP( 0xc8,      "INY impl",        2,         IMPL,                              INC(regs[REG_Y]) SET_REG(REG_Y, tmp8b))  
        //JMPs
        OP( 0x4c,       "jmp abs",        3,          ABS,                                                            SET_PC())
        OP( 0x6c,       "jmp ind",        5,          IND,                                                            SET_PC())
        //JSR
        OP( 0x20,       "JSR abs",        6,          ABS,                                        STACK_PUSH16((PC+2)) SET_PC())
        //LDAs
        OP( 0xa9,         "LDA #",        2,     IMMEDIATE,                                           SET_REG( REG_A, operand ))
        OP( 0xa5,       "LDA zpg",        3,          ZPG,                                     READ() SET_REG( REG_A, operand ))
        OP( 0xb5,     "LDA zpg,x",        4,        ZPG_X,                                     READ() SET_REG( REG_A, operand ))
        OP( 0xad,       "LDA abs",        4,          ABS,                                     READ() SET_REG( REG_A, operand ))
        OP( 0xbd,     "LDA abs,x",        4,        ABS_X,                                     READ() SET_REG( REG_A, operand ))
        OP( 0xb9,     "LDA abs,y",        4,        ABS_Y,                                     READ() SET_REG( REG_A, operand ))
        OP( 0xa1,     "LDA ind,x",        6,        X_IND,                                     READ() SET_REG( REG_A, operand ))
        OP( 0xb1,     "LDA ind,y",        5,        IND_Y,                                     READ() SET_REG( REG_A, operand ))
        //LDXs
        OP( 0xa2,         "LDX #",        2,     IMMEDIATE,                                           SET_REG( REG_X, operand ))
        OP( 0xa6,       "LDX zpg",        3,          ZPG,                                     READ() SET_REG( REG_X, operand ))
        OP( 0xb6,     "LDX zpg,y",        4,        ZPG_Y,                                     READ() SET_REG( REG_X, operand ))
        OP( 0xae,       "LDX abs",        4,          ABS,                                     READ() SET_REG( REG_X, operand ))
        OP( 0xbe,     "LDX abs,y",        4,        ABS_Y,                                     READ() SET_REG( REG_X, operand ))
        //LDYs
        OP( 0xa0,         "LDY #",        2,     IMMEDIATE,                                           SET_REG( REG_Y, operand ))
        OP( 0xa4,       "LDY zpg",        3,          ZPG,                                     READ() SET_REG( REG_Y, operand ))
        OP( 0xb4,     "LDY zpg,x",        4,        ZPG_X,                                     READ() SET_REG( REG_Y, operand ))
        OP( 0xac,       "LDY abs",        4,          ABS,                                     READ() SET_REG( REG_Y, operand ))
        OP( 0xbc,     "LDY abs,x",        4,        ABS_X,                                     READ() SET_REG( REG_Y, operand ))
        //LSR
        OP( 0x4a,         "LSR A",        2,  ACCUMULATOR,                                       SHIFT_R() SET_REG(REG_A, temp))
        OP( 0x46,       "LSR zpg",        5,          ZPG,            READ() SHIFT_R() STORE(addr, temp) UPDATE_NZFLAGS( temp ))
        OP( 0x56,     "LSR zpg,x",        6,        ZPG_X,            READ() SHIFT_R() STORE(addr, temp) UPDATE_NZFLAGS( temp ))
        OP( 0x4e,       "LSR abs",        6,          ABS,            READ() SHIFT_R() STORE(addr, temp) UPDATE_NZFLAGS( temp ))
        OP( 0x5e,     "LSR abs,x",        7,        ABS_X,            READ() SHIFT_R() STORE(addr, temp) UPDATE_NZFLAGS( temp ))    
        //ORA
        OP( 0x09,         "ORA #",        2,    IMMEDIATE,                                                         LOGIC_OP(|))
        OP( 0x05,       "ORA zpg",        3,          ZPG,                                                  READ() LOGIC_OP(|))
        OP( 0x15,     "ORA zpg,x",        4,        ZPG_X,                                                  READ() LOGIC_OP(|))
        OP( 0x0d,       "ORA abs",        4,          ABS,                                                  READ() LOGIC_OP(|))
        OP( 0x1d,     "ORA abs,x",        4,        ABS_X,                                                  READ() LOGIC_OP(|))
        OP( 0x19,     "ORA abs,y",        4,        ABS_Y,                                                  READ() LOGIC_OP(|))
        OP( 0x01,     "ORA ind,x",        6,        X_IND,                                                  READ() LOGIC_OP(|))
        OP( 0x11,     "ORA ind,y",        5,        IND_Y,                                                  READ() LOGIC_OP(|))
        //PHA       
        OP( 0x48,           "PHA",        3,         IMPL,                                             STACK_PUSH(regs[REG_A]))
        //PHP       
        OP( 0x08,           "PHP",        3,         IMPL,                                                       STACK_PUSH(P))
        //PLA       
        OP( 0x68,           "PLA",        4,         IMPL,                             STACK_PULL(tmp8b) SET_REG(REG_A, tmp8b))
        //PLP       
        OP( 0x28,           "PLP",        4,         IMPL,                                                       STACK_PULL(P))
        //ROL
        OP( 0x2a,         "ROL A",        2,  ACCUMULATOR,                                         ROL() SET_REG(REG_A, tmp8b))
        OP( 0x26,       "ROL zpg",        5,          ZPG,             READ() ROL() STORE(addr, tmp8b) UPDATE_NZFLAGS( tmp8b ))
        OP( 0x36,     "ROL zpg,x",        6,        ZPG_X,             READ() ROL() STORE(addr, tmp8b) UPDATE_NZFLAGS( tmp8b ))
        OP( 0x2e,       "ROL abs",        6,          ABS,             READ() ROL() STORE(addr, tmp8b) UPDATE_NZFLAGS( tmp8b ))
        OP( 0x3e,     "ROL abs,x",        7,        ABS_X,             READ() ROL() STORE(addr, tmp8b) UPDATE_NZFLAGS( tmp8b ))  
        //ROR
        OP( 0x6a,         "ROR A",        2,  ACCUMULATOR,                                         ROR() SET_REG(REG_A, tmp8b))
        OP( 0x66,       "ROR zpg",        5,          ZPG,             READ() ROR() STORE(addr, tmp8b) UPDATE_NZFLAGS( tmp8b ))
        OP( 0x76,     "ROR zpg,x",        6,        ZPG_X,             READ() ROR() STORE(addr, tmp8b) UPDATE_NZFLAGS( tmp8b ))
        OP( 0x6e,       "ROR abs",        6,          ABS,             READ() ROR() STORE(addr, tmp8b) UPDATE_NZFLAGS( tmp8b ))
        OP( 0x7e,     "ROR abs,x",        7,        ABS_X,             READ() ROR() STORE(addr, tmp8b) UPDATE_NZFLAGS( tmp8b ))  
        //RETURNS
        OP( 0x40,           "RTI",        6,         IMPL,                                    STACK_PULL(P) RESTORE_PC() PC--;)
        OP( 0x60,           "RTS",        6,         IMPL,                                                        RESTORE_PC())      
        //BRK
        OP( 0x00,           "brk",        1,         IMPL,   if(pgmbin::getBitAt<I_flag>(P)){STACK_PUSH16(PC+1) STACK_PUSH(P)})  
        //SBCs
        OP( 0xe9,         "SBC #",        2,    IMMEDIATE,                                                               SBC())
        OP( 0xe5,       "SBC zpg",        3,          ZPG,                                                        READ() SBC())
        OP( 0xf5,     "SBC zpg,X",        4,        ZPG_X,                                                        READ() SBC())
        OP( 0xed,       "SBC abs",        4,          ABS,                                                        READ() SBC())
        OP( 0xfd,     "SBC abs,X",        4,        ABS_X,                                                        READ() SBC())
        OP( 0xf9,     "SBC abs,Y",        4,        ABS_Y,                                                        READ() SBC())
        OP( 0xe1,     "SBC ind,X",        6,        X_IND,                                                        READ() SBC())        
        OP( 0xf1,     "SBC ind,Y",        5,        IND_Y,                                                        READ() SBC())
        //FLAG SETS
        OP( 0x38,           "SEC",        2,         IMPL,                                  P = pgmbin::setBitAt<C_flag>(1, P))
        OP( 0xf8,           "SED",        2,         IMPL,                                  P = pgmbin::setBitAt<D_flag>(1, P))
        OP( 0x78,           "SEI",        2,         IMPL,                                  P = pgmbin::setBitAt<I_flag>(1, P))
        //STAs
        OP( 0x85,       "STA zpg",        3,          ZPG,                                         STORE( operand, regs[REG_A]))
        OP( 0x95,     "STA zpg,x",        4,        ZPG_X,                                         STORE( operand, regs[REG_A]))
        OP( 0x8d,       "STA abs",        4,          ABS,                                         STORE( operand, regs[REG_A]))
        OP( 0x9d,     "STA abs,x",        5,        ABS_X,                                         STORE( operand, regs[REG_A]))
        OP( 0x99,     "STA abs,y",        5,        ABS_Y,                                         STORE( operand, regs[REG_A]))       
        OP( 0x81,     "STA ind,x",        6,        X_IND,                                         STORE( operand, regs[REG_A]))
        OP( 0x91,     "STA ind,y",        6,        IND_Y,                                         STORE( operand, regs[REG_A]))
        //STXs
        OP( 0x86,       "STX zpg",        3,          ZPG,                                         STORE( operand, regs[REG_X])) 
        OP( 0x96,     "STX zpg,x",        4,        ZPG_Y,                                         STORE( operand, regs[REG_X]))        
        OP( 0x8e,     "STX zpg,x",        4,          ABS,                                         STORE( operand, regs[REG_X]))
        //STYs
        OP( 0x84,       "STY zpg",        3,          ZPG,                                         STORE( operand, regs[REG_Y])) 
        OP( 0x94,     "STY zpg,x",        4,        ZPG_X,                                         STORE( operand, regs[REG_Y]))        
        OP( 0x8c,     "STY zpg,x",        4,          ABS,                                         STORE( operand, regs[REG_Y]))
        //TAX
        OP( 0xaa,           "TAX",        2,         IMPL,                 regs[REG_X]=regs[REG_A]; UPDATE_NZFLAGS(regs[REG_X]))
        //TAY
        OP( 0xa8,           "TAY",        2,         IMPL,                 regs[REG_Y]=regs[REG_A]; UPDATE_NZFLAGS(regs[REG_Y]))     
        //TSX
        OP( 0xba,           "TSX",        2,         IMPL,                          regs[REG_X]=SP; UPDATE_NZFLAGS(regs[REG_X]))
        //TXA
        OP( 0x8a,           "TXA",        2,         IMPL,                 regs[REG_A]=regs[REG_X]; UPDATE_NZFLAGS(regs[REG_A]))
        //TXS
        OP( 0x9a,           "TXS",        2,         IMPL,                                                     SP = regs[REG_X])
        //TYA
        OP( 0x98,           "TYA",        2,         IMPL,                  regs[REG_A]=regs[REG_Y]; UPDATE_NZFLAGS(regs[REG_A]))
        case 0xEA:  
            return I;
        default:            
            std::cout << "Unknown OPCODE" << std::endl;
            exit(1);
    }
    
    #if DEBUG_CPU
        std::cout << "ins operand: " << std::hex << (int)operand << std::endl;
    #endif
    
    I.label = label;    
    I.opcode = opcode;    
    I.cycles = cycles;   
    I.addr_mode = mode;  
    I.keep_pc = keep_pc; 
    
    return I;
}
