
#ifndef PGMBINARYOPS_H
#define	PGMBINARYOPS_H

#include <stdint.h>
/*
 *      Lightweight module for typical bin/hex operations.
 * 
 *      Written by: Fco. Gázquez (pmgmsc@gmail.com)
 *      Github: github.com/pakillottk
 *      
 */
namespace pgmbin {
    /*
     *  Given - Byte ba and Byte cd
     *  Return - 2 bytes of the form: 0xbacd                  
     */
    template<uint8_t ba, uint8_t cd>
    inline uint16_t combine() {
        return (ba << 8) | cd;
    }
    inline uint16_t combine(uint8_t ba, uint8_t cd) {
        return (ba << 8) | cd;
    }    
    /*
     *  Given - Byte lb (low byte) and Byte hb (high byte)
     *  Return - result of combine( hb. lb )                         
     */
    template<uint8_t lb, uint8_t hb>
    inline uint16_t combineLittleEndian() {
        return combine<hb,lb>();
    }  
    inline uint16_t combineLittleEndian(uint8_t lb, uint8_t hb) {
        return combine(hb,lb);
    }    
    /*
     *  Given -  Byte hb (high byte) and Byte lb (low byte) 
     *  Return - result of combine( hb. lb )                            
     */
    template<uint8_t hb, uint8_t lb>
    inline uint16_t combineBigEndian() {
        return combine<hb,lb>();
    }    
    inline uint16_t combineBigEndian( uint8_t hb, uint8_t lb ) {
        return combine(hb,lb);
    }    
    /*
     *  Given -  n (1-based) and Byte byte 
     *  Return - value of nibble n in byte                                 
     */
    template <short n>
    inline uint8_t getNibble( uint8_t byte ) {
        return (byte >> (n-1)) & 0xF;
    }  
    inline uint8_t getNibble( short n, uint8_t byte ) {
        return (byte >> (n-1)) & 0xF;
    }
    /*
     *  Given -  bit (1-based), set (0 or 1) and Byte byte 
     *  Return - Changes bit of byte to set and returns new byte                               
     */
    template <short bit>
    inline char setBitAt( char set, uint8_t byte ) {
        return set ? byte |= (0x1 << (bit-1)) : byte &= ~(0x1 << (bit-1));
    } 
    inline char setBitAt( short bit, char set, uint8_t byte ) {
        return set ? byte |= (0x1 << (bit-1)) : byte &= ~(0x1 << (bit-1));
    } 
    /*
     *  Given -  bit (bit 1-based) and Byte byte 
     *  Return - value of bit n in byte                                
     */
    template <short bit>
    inline char getBitAt( uint8_t byte ) {
        return (byte >> (bit-1)) & 0x1;
    }
    inline char getBitAt( short bit, uint8_t byte ) {
        return (byte >> (bit-1)) & 0x1;
    }
    /*
     *  Given -  bitf (bit 1-based), bitt (bit 1-based) and Byte byte 
     *  Return - Sets the bit at bitt to value at bit bitf, returns new byte                                
     */
    template <short bitf, short bitt>
    inline uint8_t swapBits( uint8_t byte ) {
        return setBitAt<bitt>( getBitAt<bitf>(byte), byte );
    }
    inline uint8_t swapBits( short bitf, short bitt, uint8_t byte ) {
        return setBitAt( bitt, getBitAt(bitf, byte), byte );
    }
};

#endif	/* PGMBINARYOPS_H */

