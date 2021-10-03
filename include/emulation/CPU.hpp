
# pragma once

# define GBMU_NAMESPACE gbmu

# include <cstdint>
# include <exception>

# define PREFIX_XMASK 0b00000011
# define PREFIX_YMASK 0b00011100
# define PREFIX_ZMASK 0b11100000
# define PREFIX_PMASK 0b00001100
# define PREFIX_QMASK 0b00010000

# define OPCODE_PREFIX_SZMAX 8
# define OPCODE_X_SZMAX 4
# define OPCODE_Y_SZMAX 8
# define OPCODE_Z_SZMAX 8
# define OPCODE_P_SZMAX 4
# define OPCODE_Q_SZMAX 2

# define OPCODE_ERROR_INIT "Ilegal opcode prefix"

namespace GBMU_NAMESPACE {

    class Opcode
    {
        Opcode();

        typedef int32_t opcode_t;
		typedef int8_t	opcode_byte_t;

		opcode_t		prefixedValue;
		opcode_byte_t	unprefixedValue;
		int8_t			prefixFamily;

		struct OpcodeInitializerException
		: public std::exception
		{ const char* what() const noexcept { return OPCODE_ERROR_INIT; } };

        public:

		enum
		{
			UNPREFIXED	= (1 << 0),
			PREFIX_CB	= (UNPREFIXED << 1),
			PREFIX_ED	= (PREFIX_CB << 1),
			PREFIX_DD	= (PREFIX_ED << 1),
			PREFIX_FD	= (PREFIX_DD << 1),
		};

		opcode_byte_t 	getX() const { return this->unprefixedValue & PREFIX_XMASK; }
		opcode_byte_t 	getY() const { return this->unprefixedValue & PREFIX_YMASK; }
		opcode_byte_t 	getZ() const { return this->unprefixedValue & PREFIX_ZMASK; }
		opcode_byte_t	getP() const { return this->unprefixedValue & PREFIX_PMASK; }
		opcode_byte_t	getQ() const { return this->unprefixedValue & PREFIX_QMASK; }
		int8_t			getPrefixFamily() const { return this->prefixFamily; }

		bool			isValid()
		{
			return (
				   this->getX() < OPCODE_X_SZMAX \
				&& this->getY() < OPCODE_Y_SZMAX \
				&& this->getZ() < OPCODE_Z_SZMAX \
				&& this->getP() < OPCODE_P_SZMAX \
				&& this->getQ() < OPCODE_Q_SZMAX
			);
		}

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

		typedef struct afwords_ { uword_t a; uword_t f; } __attribute__ ((packed)) afwords_t;
		typedef struct bcwords_ { uword_t b; uword_t c; } __attribute__ ((packed)) bcwords_t;
		typedef struct dewords_ { uword_t d; uword_t e; } __attribute__ ((packed)) dewords_t;
		typedef struct lhwords_ { uword_t h; uword_t l; } __attribute__ ((packed)) hlwords_t;

		union
		{
			udword_t	af;
			afwords_t	afwords;
		};

		union
		{
			udword_t	bc;
			bcwords_t	bcwords;
		};

		union
		{
			udword_t	de;
			dewords_t	dewords;
		};

		union
		{
			udword_t	hl;
			hlwords_t	hlwords;
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

		typedef CPU<mem_t, reg_t, flags_t> cpu_t;
		typedef void (*exeOpcode_t)(const cpu_t& core);

		mem_t	memory;
		reg_t	regs;
		flags_t	flags;

		CPU(const mem_t& mem, const reg_t& registers, flags_t flagsCt)
		: memory(mem), regs(registers), flags(flagsCt)
		{ }

		CPU(const mem_t&& mem, const reg_t&& registers, flags_t flagsCt)
		: memory(mem), regs(registers), flags(flagsCt)
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
		typedef Z80Registers::word_t	word_t;
		typedef Z80Registers::uword_t	uword;
		typedef Z80Registers::dword_t	dword_t;
		typedef Z80Registers::udword_t	udword_t;

		typedef typename CPU<Memory, Z80Registers, uint8_t>::cpu_t	cpu_t;
		typedef typename cpu_t::exeOpcode_t							exeOpcode_t;
		typedef typename cpu_t::mem_t								mem_t;
		typedef typename cpu_t::reg_t								reg_t;
		typedef typename cpu_t::flags_t								flags_t;

		typedef std::size_t 	size_type;
		typedef std::ptrdiff_t	ptrdiff_type;

		public:

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

		private:

		struct ExceptionBadUse : std::exception
		{ const char* what() { return "Bad use of method"; } };

		size_type
		getPrefixIndex(int8_t prefixFamily)
		// throw ExceptionBadUse
		{
			switch (prefixFamily)
			{
				case Opcode::UNPREFIXED:
					return 0;
				case Opcode::PREFIX_CB:
					return 1;
				case Opcode::PREFIX_ED:
					return 2;
				case Opcode::PREFIX_DD:
					return 3;
				case Opcode::PREFIX_FD:
					return 4;
				case Opcode::PREFIX_DD | Opcode::PREFIX_CB:
					return 5;
				case Opcode::PREFIX_FD | Opcode::PREFIX_CB:
					return 6;
				default:
					throw ExceptionBadUse();
			}
		}

		//////////////////////////////////
		// Operations function pointers //
		//////////////////////////////////

		static void OperNop(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ static_cast<void>(core); }

		///////////////////////////////////////////////////////
		// All operations are indexed & statically preloaded //
		///////////////////////////////////////////////////////

		/// NOTE: To skip array go forward 45396 lines (yeah, it's worth)

		constexpr static const typename CPU<Memory, Z80Registers, uint8_t>::exeOpcode_t
		operations[OPCODE_PREFIX_SZMAX][OPCODE_X_SZMAX][OPCODE_Z_SZMAX][OPCODE_Y_SZMAX][OPCODE_Q_SZMAX][OPCODE_P_SZMAX] = {
			/* Unprefixed */
			{
				/* X = 0 */
				{
					/* Z = 0 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{

							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 1 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 2 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 3 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 4 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 5 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 6 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 7 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					}
				},
				/* X = 1 */
				{
					/* Z = 0 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 1 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 2 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 3 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 4 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 5 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 6 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 7 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					}
				},
				/* X = 2 */
				{
					/* Z = 0 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 1 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 2 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 3 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 4 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 5 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 6 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 7 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					}
				},
				/* X = 3 */
				{
					/* Z = 0 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 1 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 2 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 3 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 4 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 5 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 6 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 7 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					}
				}
			},
			/* Prefix CB */
			{
				/* X = 0 */
				{
					/* Z = 0 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 1 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 2 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 3 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 4 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 5 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 6 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 7 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					}
				},
				/* X = 1 */
				{
					/* Z = 0 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 1 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 2 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 3 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 4 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 5 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 6 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 7 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					}
				},
				/* X = 2 */
				{
					/* Z = 0 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 1 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 2 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 3 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 4 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 5 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 6 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 7 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					}
				},
				/* X = 3 */
				{
					/* Z = 0 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 1 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 2 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 3 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 4 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 5 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 6 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 7 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					}
				}
			},
			/* Prefix ED */
			{
				/* X = 0 */
				{
					/* Z = 0 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 1 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 2 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 3 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 4 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 5 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 6 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 7 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					}
				},
				/* X = 1 */
				{
					/* Z = 0 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 1 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 2 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 3 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 4 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 5 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 6 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 7 */
					{

					}
				},
				/* X = 2 */
				{
					/* Z = 0 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 1 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 2 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 3 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 4 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 5 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 6 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 7 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					}
				},
				/* X = 3 */
				{
					/* Z = 0 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 1 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 2 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 3 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 4 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 5 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 6 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 7 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					}
				}
			},
			/* Prefix DD */
			{
				/* X = 0 */
				{
					/* Z = 0 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 1 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 2 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 3 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 4 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 5 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 6 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 7 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{

						}
					}
				},
				/* X = 1 */
				{
					/* Z = 0 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 1 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 2 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 3 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 4 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 5 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 6 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 7 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					}
				},
				/* X = 2 */
				{
					/* Z = 0 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 1 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 2 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 3 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 4 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 5 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 6 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 7 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					}
				},
				/* X = 3 */
				{
					/* Z = 0 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 1 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 2 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 3 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 4 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 5 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 6 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 7 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					}
				}
			},
			/* Prefix FD */
			{
				/* X = 0 */
				{
					/* Z = 0 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 1 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 2 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 3 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 4 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 5 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 6 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 7 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					}
				},
				/* X = 1 */
				{
					/* Z = 0 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 1 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 2 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 3 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 4 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 5 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 6 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 7 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					}
				},
				/* X = 2 */
				{
					/* Z = 0 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 1 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 2 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 3 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 4 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 5 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 6 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 7 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					}
				},
				/* X = 3 */
				{
					/* Z = 0 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 1 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 2 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 3 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 4 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 5 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 6 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 7 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					}
				}
			},
			/* Prefix DDCB */
			{
				/* X = 0 */
				{
					/* Z = 0 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 1 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 2 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 3 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 4 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 5 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 6 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 7 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					}
				},
				/* X = 1 */
				{
					/* Z = 0 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 1 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 2 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 3 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 4 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 5 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 6 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 7 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					}
				},
				/* X = 2 */
				{
					/* Z = 0 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 1 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 2 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 3 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 4 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 5 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 6 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 7 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					}
				},
				/* X = 3 */
				{
					/* Z = 0 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 1 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 2 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 3 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}

					},
					/* Z = 4 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 5 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 6 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 7 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					}
				}
			},
			/* Prefix FDCB */
			{
				/* X = 0 */
				{
					/* Z = 0 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 1 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 2 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 3 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 4 */
					{
												/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 5 */
					{
												/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 6 */
					{
												/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 7 */
					{
												/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					}
				},
				/* X = 1 */
				{
					/* Z = 0 */
					{
												/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 1 */
					{
												/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 2 */
					{
												/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 3 */
					{
												/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 4 */
					{
												/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 5 */
					{
												/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 6 */
					{
												/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 7 */
					{
												/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					}
				},
				/* X = 2 */
				{
					/* Z = 0 */
					{
												/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 1 */
					{
												/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 2 */
					{
												/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 3 */
					{
												/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 4 */
					{
												/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 5 */
					{
												/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 6 */
					{
												/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 7 */
					{
												/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					}
				},
				/* X = 3 */
				{
					/* Z = 0 */
					{
												/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 1 */
					{
												/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 2 */
					{
												/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 3 */
					{
												/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 4 */
					{
												/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 5 */
					{
												/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 6 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					},
					/* Z = 7 */
					{
						/* Y = 0 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&OperNop,
								/* P = 1 */
								&OperNop,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							}
						}
					}
				}
			}
		};

		struct ExceptionInvalidOpcode : std::exception
		{ const char* what() const noexcept { return "invalid opcode: out of bounds"; } };

		void executeOperation(Opcode opcode)
		// may throw ExceptionInvalidOpcode
		{
			if (opcode.isValid())
			{
				this->operations[this->getPrefixIndex(opcode.getPrefixFamily())]
				[opcode.getX()][opcode.getZ()][opcode.getY()]
				[opcode.getQ()][opcode.getP()](*this);

				// TO DO: Add cycles handler
			}
			else
				throw ExceptionInvalidOpcode();
		}

		public:

		CPUZ80(const mem_t& memory_type)
		: CPU<mem_t, reg_t, flags_t>(memory_type, Z80Registers(0/*TODO*/), (flags_t)0)
		{
			/* ... */
		}
    };
}
