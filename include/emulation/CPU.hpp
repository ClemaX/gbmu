
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

		~Opcode() { }
    };

	struct Z80Registers
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

		typedef int8_t		word_t;
		typedef uint8_t 	uword_t;
		typedef int16_t 	dword_t;
		typedef uint16_t	udword_t;

		union
		{
			udword_t af;
			struct
			{
				uword_t a;
				uword_t f;

			};

		};

		union
		{
			udword_t bc;
			struct
			{
				uword_t b;
				uword_t c;
			};
		};

		union
		{
			udword_t de;
			struct
			{
				uword_t d;
				uword_t e;
			};
		};

		union
		{
			udword_t hl;
			struct
			{
				uword_t h;
				uword_t l;
			};
		};

		udword_t sp;
		udword_t pc;

		Z80Registers(ptrdiff_t stackstart, ptrdiff_t entrypoint = 0X100)
		: af(0), bc(0), de(0), hl(0), sp(stackstart), pc(entrypoint)
		{ }
	};

	template <class Memory, class Registers, typename Flags>
	class CPU
	{
		protected:

		typedef Memory		mem_t;
		typedef Registers	reg_t;
		typedef Flags		flags_t;

		typedef CPU<Memory, Registers, Flags> cpu_t;

		mem_t	memory;
		reg_t	regs;
		flags_t	flags;

		CPU(const mem_t& mem, const reg_t& registers, flags_t flagsCt)
		: memory(mem), regs(regs), flags(flagsCt)
		{ }

		CPU(const mem_t&& mem, const reg_t&& registers, flags_t flagsCt)
		: memory(mem), regs(regs), flags(flagsCt)
		{ }

		CPU(const CPU& other)
		: memory(other.mem), regs(other.regs), flags(other.flagsCt)
		{ }

		CPU(const CPU&& other)
		: memory(other.mem), regs(other.regs), flags(other.flagsCt)
		{ }

		CPU&
		operator=(const CPU& other)
		{
			if (this != &other)
			{
				this->memory = other.memory;
				this->regs = other.regs;
				this->flags = other.flags;
			}
			return *this;
		}

		~CPU()
		{ }
	};

	template <class Memory>
    class CPUZ80
	: protected CPU<Memory, Z80Registers, uint8_t>
    {
		// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
		// | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
		// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
		// | Z | N | H | C | 0 | 0 | 0 | 0 |
		// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
		enum
		{
			FLAG_ZERO		= (1 << 0),
			FLAG_SUBSTRACT	= (FLAG_ZERO << 1),
			FLAG_HALF_CARRY	= (FLAG_SUBSTRACT << 1),
			FLAG_CARRY		= (FLAG_HALF_CARRY << 1),
			FLAG_BIT0		= (FLAG_CARRY << 1),
			FLAG_BIT1		= (FLAG_BIT0 << 1),
			FLAG_BIT2		= (FLAG_BIT1 << 1),
			FLAG_BIT3		= (FLAG_BIT2 << 1),
		};

		typedef void (const *exeOpcode_t)(const cpu_t& core);

		// TODO: Array of mapped instructions indexing function pointers type exeOpcode_t

		public:

		CPUZ80(const mem_t& memory_type)
		: CPU(memory_type, Z80Registers(0/*TODO*/), (flags_t)0)
		{
			/* ... */
		}
    };
}
