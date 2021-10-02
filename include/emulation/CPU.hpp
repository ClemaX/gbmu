
# pragma once

# define GBMU_NAMESPACE gbmu

# include <cstdint>
# include <exception>

# define PREFIX_XMASK 0b11000000 
# define PREFIX_YMASK 0b00111000
# define PREFIX_ZMASK 0b00000111

# define OPCODE_ERROR_INIT "Ilegal opcode prefix"

namespace GBMU_NAMESPACE {

    class Opcode
    {
        Opcode();

        typedef int32_t opcode_t;
		typedef int8_t	opcode_byte_t;

		enum {
			UNPREFIXED	= (1 << 0),
			PREFIX_CB	= (UNPREFIXED << 1),
			PREFIX_ED	= (PREFIX_CB << 1),
			PREFIX_DD	= (PREFIX_ED << 1),
			PREFIX_FD	= (PREFIX_DD << 1),
		};

		opcode_t		prefixedValue;
		opcode_byte_t	unprefixedValue;
		int8_t			prefixFamily;

		struct OpcodeInitializerException
		: public std::exception
		{ const char* what() { return OPCODE_ERROR_INIT; } };

        public:

		opcode_byte_t 	getX() const { return this->unprefixedValue & PREFIX_XMASK; }
		opcode_byte_t 	getY() const  { return this->unprefixedValue & PREFIX_YMASK; }
		opcode_byte_t 	getZ() const  { return this->unprefixedValue & PREFIX_ZMASK; }
		int8_t			getPrefixFamily() const { return this->prefixFamily; } 
		
		Opcode(opcode_t opcode)
		: prefixedValue(opcode),
		unprefixedValue(static_cast<opcode_byte_t>(opcode))
		{
			// TODO: Check if i shift the right amount
			switch (static_cast<uint16_t>(opcode >> 16))
			{
				case 0XCB:
					this->prefixFamily = PREFIX_CB;
					break ;
				case 0XED:
					this->prefixFamily = PREFIX_ED;
					break ;
				case 0XDD:
					this->prefixFamily = PREFIX_DD;
					break ;
				case 0XFD:
					this->prefixFamily = PREFIX_FD;
					break ;
				case (0XDD << 8) | 0XCB:
					this->prefixFamily = PREFIX_DD | PREFIX_CB;
					break ;
				case (0XFD << 8) | 0xDB:
					this->prefixFamily = PREFIX_FD | PREFIX_CB;
					break ;
				case 0x0:
					this->prefixFamily = UNPREFIXED;
					break ;

				default:
					throw OpcodeInitializerException();
			}
		}
    };

	template <class Memory>
    class CPU
    {
		typedef int8_t		reg8_t;
		typedef uint8_t 	ureg8_t;
		typedef int16_t 	reg16_t;
		typedef uint16_t	ureg16_t;

		struct Registers
		{
			// | 15-8| 0-7 |
			// +-+-+-+-+-+-+
			// |  A  |  F  |
			// +-+-+-+-+-+-+
			// |  B  |  C  |
			// +-+-+-+-+-+-+
			// |  D  |  E  |
			// +-+-+-+-+-+-+
			// |  H  |  L  |
			// +-+-+-+-+-+-+
			// |     SP    |
			// +-+-+-+-+-+-+
			// |     PC    |
			// +-+-+-+-+-+-+

			union
			{
				ureg16_t af;
				struct
				{
					ureg8_t a;
					ureg8_t f;

				};
				
			};
			
			union
			{
				ureg16_t bc;
				struct
				{
					ureg8_t b;
					ureg8_t c;
				};
			};

			union
			{
				ureg16_t de;
				struct
				{
					ureg8_t d;
					ureg8_t e;
				};
			};

			union
			{
				ureg16_t hl;
				struct
				{
					ureg8_t h;
					ureg8_t l;
				};
			};

			reg16_t sp;
			reg16_t pc;

			Registers() : af(0), bc(0), de(0), hl(0), sp(0 /* to do */), pc(0X100) { }
		};

		typedef Registers reg_t;

		typedef int8_t flags_t;

		enum {
			FLAG_BIT0		= (1 << 0),
			FLAG_BIT1		= (FLAG_BIT0 << 1),
			FLAG_BIT2		= (FLAG_BIT1 << 1),
			FLAG_BIT3		= (FLAG_BIT2 << 1),
			FLAG_ZERO		= (FLAG_BIT3 << 0),
			FLAG_SUBSTRACT	= (FLAG_ZERO << 1),
			FLAG_HALF_CARRY	= (FLAG_SUBSTRACT << 1),
			FLAG_CARRY		= (FLAG_HALF_CARRY << 1)
		};

		reg_t		regs;
		flags_t		regFlags; // TODO: defines/function members for fast modify flags

		typedef Memory mem_t;

		mem_t	memory;

		// TODO: Array of mapped instructions indexing function pointers

		public:

		CPU(mem_t memory_type) : memory(memory_type), regFlags()
		{
			/* ... */
		}
    };
}