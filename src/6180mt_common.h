
/**
 * @file 6180mt_common.h
 * @author Feibit 
 * @version v1.0.0
 * @brief 通用功能接口声明文件。
 * 
 * @copyright 
 * 
 * @defgroup COMMON 通用功能模块
 * @{
 */

#ifndef __6180MT_COMMON_H__
#define __6180MT_COMMON_H__

#include <stdint.h>
#include "log.h"

/** @brief 定义寄存器的BIT位掩码 */
#define BIT0     ((uint32_t)1 << 0)
#define BIT1     ((uint32_t)1 << 1)
#define BIT2     ((uint32_t)1 << 2)
#define BIT3     ((uint32_t)1 << 3)
#define BIT4     ((uint32_t)1 << 4)
#define BIT5     ((uint32_t)1 << 5)
#define BIT6     ((uint32_t)1 << 6)
#define BIT7     ((uint32_t)1 << 7)
#define BIT8     ((uint32_t)1 << 8)
#define BIT9     ((uint32_t)1 << 9)
#define BIT10    ((uint32_t)1 << 10)
#define BIT11    ((uint32_t)1 << 11)
#define BIT12    ((uint32_t)1 << 12)
#define BIT13    ((uint32_t)1 << 13)
#define BIT14    ((uint32_t)1 << 14)
#define BIT15    ((uint32_t)1 << 15)
#define BIT16    ((uint32_t)1 << 16)
#define BIT17    ((uint32_t)1 << 17)
#define BIT18    ((uint32_t)1 << 18)
#define BIT19    ((uint32_t)1 << 19)
#define BIT20    ((uint32_t)1 << 20)
#define BIT21    ((uint32_t)1 << 21)
#define BIT22    ((uint32_t)1 << 22)
#define BIT23    ((uint32_t)1 << 23)
#define BIT24    ((uint32_t)1 << 24)
#define BIT25    ((uint32_t)1 << 25)
#define BIT26    ((uint32_t)1 << 26)
#define BIT27    ((uint32_t)1 << 27)
#define BIT28    ((uint32_t)1 << 28)
#define BIT29    ((uint32_t)1 << 29)
#define BIT30    ((uint32_t)1 << 30)
#define BIT31    ((uint32_t)1 << 31)


#define SET_REG_VAL(reg, val)           ((reg) = (val))     ///< 设置寄存器的值

#define GET_REG_VAL(reg)                (reg)               ///< 取得寄存器的值

/**
 * @brief 设置寄存器的某些位的值为1
 * 
 * @param reg 寄存器
 * @param bit 只有`bit`中值为1的位，才会被设置到寄存器中，`bit`为0的位在寄存器中值不变
 */
#define SET_REG_BIT(reg, bit)           ((reg) |= (bit))

/**
 * @brief 设置寄存器的某些位的值
 * 
 * @param reg 寄存器
 * @param bit 要设置的值
 * @param msk `msk`中为1的位对应的`bit`参数中的值会被设置到寄存器中，0或1都会被设置
 */
#define SET_REG_BIT_MSK(reg, bit, msk)  ((reg) = ((reg) & ~(msk) | (bit)))

/**
 * @brief 清除寄存器的某些位的值为0
 * 
 * @param reg 寄存器
 * @param bit 只有`bit`中值为1的位，才会在寄存器中清0，`bit`为0的位在寄存器中值不变
 * @return  
 */
#define CLR_REG_BIT(reg, bit)           ((reg) &= ~(bit))

/**
 * @brief 取得寄存器中某些位的值
 * 
 * @param reg 寄存器
 * @param bit 只有`bit`中值为1的位，才会返回寄存器中的值，`bit`为0的位总是返回0
 * @return  
 */
#define GET_REG_BIT(reg, bit)           ((reg) & (bit))

typedef unsigned char		uint8;
typedef signed char			int8;
typedef	unsigned short		uint16;	
typedef	signed short		int16;
typedef unsigned int		uint32;
typedef int					int32;

typedef unsigned char		UINT8;
typedef	unsigned short		UINT16;	
typedef unsigned int		UINT32;

#ifndef FALSE
	#define FALSE	0
#endif

#ifndef TRUE
	#define TRUE	( !FALSE )
#endif

#define BREAK_UINT32( var, ByteNum ) \
          (uint8)((uint32)(((var) >>((ByteNum) * 8)) & 0x00FF))

#define BUILD_UINT32(Byte0, Byte1, Byte2, Byte3) \
          ((uint32)((uint32)((Byte0) & 0x00FF) \
          + ((uint32)((Byte1) & 0x00FF) << 8) \
          + ((uint32)((Byte2) & 0x00FF) << 16) \
          + ((uint32)((Byte3) & 0x00FF) << 24)))

#define BUILD_UINT16(loByte, hiByte) \
          ((uint16)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))

#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)



#endif // !defined __N32G020XX_COMMON_H__

/** @} */
