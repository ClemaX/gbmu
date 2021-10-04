
# pragma once

# define GBMU_NAMESPACE gbmu

# include <cstdint>
# include <exception>
# include <chrono>
# include <thread>

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

# define CLOCK_CYCLE_MHZ 4.194304
# define CLOCK_CYCLE_HZ (CLOCK_CYCLE_MHZ * 1000000.0)
# define CLOCK_CYCLE_S (1.0 / CLOCK_CYCLE_HZ)
# define CLOCK_CYCLE_MS (CLOCK_CYCLE_S * 1000.0)
# define CLOCK_CYCLE_NS (CLOCK_CYCLE_MS * 1000000.0)

# define FGMASK_ADD(target, mask) (target |= mask)
# define FGMASK_DEL(target, mask) (target &= ~mask)
# define FGMASK_HAS(target, mask) (target & mask)

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

		// TODO: Wait a minute ... This return true in all the cases !!!
		bool			isValid()
		{
			return (
				   this->getX() < OPCODE_X_SZMAX
				&& this->getY() < OPCODE_Y_SZMAX
				&& this->getZ() < OPCODE_Z_SZMAX
				&& this->getP() < OPCODE_P_SZMAX
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
		: memory(other.mem), regs(other.regs), flags(other.flags)
		{ }

		CPU(const CPU&& other)
		: memory(other.mem), regs(other.regs), flags(other.flags)
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
		enum flags
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


		///TODO: should i add the time spent parsing an opcode ?

		template <class Chrono>
		static inline void
		__attribute__ ((always_inline))
		waitForClockCycles(const Chrono& startT, int64_t totalCycles)
		noexcept
		{
			const Chrono& endT = std::chrono::high_resolution_clock::now();
			/// NOTE: sleep_for ignores negatice values & call nanosleep
			std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>
			(endT - startT) - ((int64_t)CLOCK_CYCLE_NS * totalCycles));
		}

		/// NOTE: Should only be used followed by 'WAIT_CYCLES'
		# define START_TIME_RECORD const std::chrono::_V2::system_clock::time_point& beginT = std::chrono::high_resolution_clock::now();

		/// NOTE: Should only be used preceded by 'START_TIME_RECORD'
		# define WAIT_CYCLES(cycles) waitForClockCycles(beginT, cycles);

		template <typename T, int64_t cycles>
		__attribute__ ((always_inline))
		static inline void
		operBaseLD(T& dest, T src)
		noexcept
		{
			START_TIME_RECORD
			dest = src;
			WAIT_CYCLES(cycles)
		}

		template <typename T, typename Flags, int64_t cycles>
		__attribute__ ((always_inline))
		static inline void
		operBaseInc(T& target, Flags& flags)
		noexcept
		{
			/** NOTE: Flags affected:
				Z - Set if result is zero.
  				N - Reset.
  				H - Set if carry from bit 3.
  				C - Not affected.
			*/
			static_cast<void>(flags);

			// typedef typename CPUZ80::flags::FLAG_ZERO		FLAG_ZERO;
			// typedef typename CPUZ80::flags::FLAG_SUBSTRACT	FLAG_SUBSTRACT;
			// typedef typename CPUZ80::flags::FLAG_HALF_CARRY	FLAG_HALF_CARRY;
			// typedef typename CPUZ80::flags::FLAG_CARRY		FLAG_CARRY;

			START_TIME_RECORD

			/* if (*/++target;// == 0)
			// 	FGMASK_ADD(flags, FLAG_ZERO);
			// FGMASK_DEL(flags, FLAG_SUBSTRACT);
			// /// TODO:
			// if (FGMASK_HAS(flags, FLAG_CARRY))
			// {
			// 	// H - Set if carry from bit 3.
			// }

			WAIT_CYCLES(cycles)
		}

		template <typename T, typename Flags, int64_t cycles>
		__attribute__ ((always_inline))
		static inline void
		operBaseDec(T& target, Flags& flags)
		noexcept
		{
			/** NOTE: Flags affected:
				Z - Set if result is zero.
  				N - Reset.
  				H - Set if carry from bit 4.
  				C - Not affected.
			*/
		static_cast<void>(flags);

			// typedef typename CPUZ80::flags::FLAG_ZERO		FLAG_ZERO;
			// typedef typename CPUZ80::flags::FLAG_SUBSTRACT	FLAG_SUBSTRACT;
			// typedef typename CPUZ80::flags::FLAG_HALF_CARRY	FLAG_HALF_CARRY;
			// typedef typename CPUZ80::flags::FLAG_CARRY		FLAG_CARRY;

			START_TIME_RECORD

			/* if (*/--target;// == 0)
			// 	FGMASK_ADD(flags, FLAG_ZERO);
			// FGMASK_DEL(flags, FLAG_SUBSTRACT);
			// /// TODO:
			// if (FGMASK_HAS(flags, FLAG_CARRY))
			// {
			// 	// H - Set if carry from bit 4.
			// }

			WAIT_CYCLES(cycles)
		}

		///TODO: A fucntion to handle falgs for INC & DEC
		///TODO: A Chrono class that mesures time between frames

		template <int64_t cycles = 4>
		static inline void
		__attribute__ ((always_inline))
		operInlNop(const CPU<Memory, Z80Registers, uint8_t>& core)
		{
			START_TIME_RECORD
			static_cast<void>(core);
			WAIT_CYCLES(cycles)
		}

		/// Opcode 0x0 + all ignored instructions
		static void OperNop(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operInlNop(core); }


//
///TODO: No y, find the exact spot to place them with opcode value
//
		/// Opcode 0X02, cycles: 8 ( LD (BC), A )
		static void operLD_AinPAddrBC(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X8>(/* memory at this addr */core.regs.bc, static_cast<udword_t>(core.regs.afwords.a)); }

		/// Opcode 0X12, cycles: 8 ( LD (DE), A )
		static void operLD_AinPAddrDE(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X8>(/* memory at this addr */core.regs.de, static_cast<udword_t>(core.regs.afwords.a)); }

		/// Opcode 0X0A, cycles: 8 ( LD A, (BC) )
		static void operLD_PAdrrBCinA(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X8>(core.regs.afwords.a, /* value at the memory at this addr */core.regs.bc); }

		/// Opcode 0X1A, cycles: 8 ( LD A, (DE) )
		static void operLD_PAdrrDEinA(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X8>(core.regs.afwords.a, /* value at the memory at this addr */core.regs.de); }

		/// Opcode: ? ( LD (nn), HL )

		/// Opcode: ? ( LD (nn), A )

		/// Opcode: ?, ( LD HL, (nn) )

		/// Opcode: ? ( LD A, (nn) )



//
///TODO: No y, find the exact spot to place them with opcode value
///TODO: <WNG> Nome flags are afected for those
//
		/// Opcode: 0X3, cycles: 8 ( INC BC )
		static void operIncBC(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseInc<0X8>(core.regs.bc, core.flags); }

		/// Opcode: 0X13, cycles: 8 ( INC DE )
		static void operIncDE(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseInc<0X8>(core.regs.de, core.flags); }

		/// Opcode: 0X23, cycles: 8 ( INC HL )
		static void operIncHL(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseInc<0X8>(core.regs.hl, core.flags); }

		/// Opcode: 0X33, cycles: 8 ( INC SP )
		static void operIncSP(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseInc<0X8>(core.regs.af, core.flags); }

		/// Opcode: 0X0B, cycles: 8 ( DEC BC )
		static void operDecBC(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseDec<0X8>(core.regs.bc, core.flags); }

		/// Opcode: 0X1B, cycles: 8 ( DEC DE )
		static void operDecDE(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseDec<0X8>(core.regs.de, core.flags); }

		/// Opcode: 0X2B, cycles: 8 ( DEC HL )
		static void operDecHL(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseDec<0X8>(core.regs.hl, core.flags); }

		/// Opcode: 0X3B, cycles: 8 ( DEC SP )
		static void operDecSP(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseDec<0X8>(core.regs.af, core.flags); }




//
///TODO: All p & q have been set, find which one set to operNop
//
		/// Opcode: 0X04, cycles: 4 ( INC B )
		static void operIncB(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseInc<0X4>(core.regs.bcwords.b, core.flags); }

		/// Opcode: 0X0C, cycles: 4 ( INC C )
		static void operIncC(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseInc<0X4>(core.regs.bcwords.c, core.flags); }

		/// Opcode: 0X14, cycles: 4 ( INC D )
		static void operIncD(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseInc<0X4>(core.regs.dewords.d, core.flags); }

		/// Opcode: 0X1C, cycles: 4 ( INC E )
		static void operIncE(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseInc<0X4>(core.regs.dewords.e, core.flags); }

		/// Opcode: 0X24, cycles: 4 ( INC H )
		static void operIncH(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseInc<0X4>(core.regs.hlwords.h, core.flags); }

		/// Opcode: 0X2C, cycles: 4 ( INC L )
		static void operIncL(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseInc<0X4>(core.regs.hlwords.l, core.flags); }

		/// Opcode: 0X34, cycles: 12 ( INC (HL) )
		static void operIncPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseInc<0XC>(core.regs.hl /* What is pointed by hl */, core.flags); }

		/// Opcode: 0X3C, cycles: 4 ( INC A )
		static void operIncA(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseInc<0X4>(core.regs.afwords.a, core.flags); }

		/// Opcode: 0X05, cycles: 4 ( DEC B )
		static void operDecB(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseDec<0X4>(core.regs.bcwords.b, core.flags); }

		/// Opcode: 0X0D, cycles: 4 ( DEC C )
		static void operDecC(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseDec<0X4>(core.regs.bcwords.c, core.flags); }

		/// Opcode: 0X15, cycles: 4 ( DEC D )
		static void operDecD(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseDec<0X4>(core.regs.dewords.d, core.flags); }

		/// Opcode: 0X1D, cycles: 4 ( DEC E )
		static void operDecE(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseDec<0X4>(core.regs.dewords.e, core.flags); }

		/// Opcode: 0X25, cycles: 4 ( DEC H )
		static void operDecH(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseDec<0X4>(core.regs.hlwords.h, core.flags); }

		/// Opcode: 0X2D, cycles: 4 ( DEC L )
		static void operDecL(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseDec<0X4>(core.regs.hlwords.l, core.flags); }

		/// Opcode: 0X35, cycles: 12 ( DEC (HL) )
		static void operDecPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseDec<0XC>(core.regs.hl /* What is pointed by hl */, core.flags); }

		/// Opcode: 0X3D, cycles: 4 ( DEC A )
		static void operDecA(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseDec<0X4>(core.regs.afwords.a, core.flags); }






		/// Opcode: ? ( RLCA )
		/// Opcode: ? ( RRCA )
		/// Opcode: ? ( RLA )
		/// Opcode: ? ( RRA )
		/// Opcode: ? ( DAA )
		/// Opcode: ? ( CPL )
		/// Opcode: ? ( SCF )
		/// Opcode: ? ( CCF )


//
///TODO: All p & q have been set, find which one set to operNop
//
		/// Opcode: 0X40, cycles: 4 ( LD B, B )
		static void operLD_BinB(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operInlNop(core); }

		/// Opcode: 0X48, cycles: 4 ( LD C, B )
		static void operLD_BinC(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X4>(core.regs.bcwords.c, core.regs.bcwords.b); }

		/// Opcode: 0X50, cycles: 4 ( LD D, B )
		static void operLD_BinD(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X4>(core.regs.dewords.d, core.regs.bcwords.b); }

		/// Opcode: 0X58, cycles: 4 ( LD E, B )
		static void operLD_BinE(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X4>(core.regs.dewords.e, core.regs.bcwords.b); }

		/// Opcode: 0X60, cycles: 4 ( LD H, B )
		static void operLD_BinH(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X4>(core.regs.hlwords.h, core.regs.bcwords.b); }

		/// Opcode: 0x68, cycles: 4 ( LD L, B )
		static void operLD_BinL(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X4>(core.regs.hlwords.l, core.regs.bcwords.b); }

		/// Opcode: 0X70, cycles: 8 ( LD (HL), B )
		static void operLD_BinPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X8>(core.regs.hl /* what is pointed by hl */, static_cast<udword_t>(core.regs.bcwords.b)); }

		/// Opcode: 0X78, cycles: 4 ( LD A, B )
		static void operLD_BinA(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0x4>(core.regs.afwords.a, core.regs.bcwords.b); }

		/// Opcode: 0X41, cycles: 4 ( LD B, C )
		static void operLD_CinB(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X4>(core.regs.bcwords.b, core.regs.bcwords.c); }

		/// Opcode: 0X49, cycles: 4 ( LD C, C )
		static void operLD_CinC(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operInlNop(core); }

		/// Opcode: 0x4A, cycles: 4 ( LD D, C )
		static void operLD_CinD(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X4>(core.regs.dewords.d, core.regs.bcwords.c); }

		/// Opcode: 0X59, cycles: 4  ( LD E, C )
		static void operLD_CinE(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X4>(core.regs.dewords.e, core.regs.bcwords.c); }

		/// Opcode: 0X61, cycles: 4 ( LD H, C )
		static void operLD_CinH(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X4>(core.regs.hlwords.h, core.regs.bcwords.c); }

		/// Opcode: 0X69, cycles: 4 ( LD L, C )
		static void operLD_CinL(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X4>(core.regs.hlwords.l, core.regs.bcwords.c); }

		/// Opcode: 0X71, cycles: 8 ( LD (HL), C )
		static void operLD_CinPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X8>(core.regs.hl /* what is pointed by hl */, static_cast<udword_t>(core.regs.bcwords.c)); }

		/// Opcode: 0X79, cycles:4 ( LD A, C )
		static void operLD_CinA(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X4>(core.regs.afwords.a, core.regs.bcwords.c); }

		/// Opcode: 0X42, cycles: 4 ( LD B, D )
		static void operLD_DinB(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X4>(core.regs.bcwords.b, core.regs.dewords.d); }

		/// Opcode: 0X4A, cycles: 4 ( LD C, D )
		static void operLD_DinC(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X4>(core.regs.bcwords.b, core.regs.dewords.d); }

		/// Opcode: 0X52, cycles: 4 ( LD D, D )
		static void operLD_DinD(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operInlNop(core); }

		/// Opcode: 0X5A, cycles: 4 ( LD E, D )
		static void operLD_DinE(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X4>(core.regs.dewords.e, core.regs.dewords.d); }

		/// Opcode: 0X62, cycles: 4 ( LD H, D )
		static void operLD_DinH(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X4>(core.regs.hlwords.h, core.regs.dewords.d); }

		/// Opcode: 0X6A, cycles: 4 ( LD L, D )
		static void operLD_DinL(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X4>(core.regs.hlwords.l, core.regs.dewords.d); }

		/// Opcode: 0X72, cycles: 8 ( LD (HL), D )
		static void operLD_DinPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X8>(core.regs.hl /* what is pointed by hl */, static_cast<udword_t>(core.regs.dewords.d)); }

		/// Opcode: 0X7A, cycles: 4 ( LD A, D )
		static void operLD_DinA(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0x4>(core.regs.afwords.a, core.regs.dewords.d); }

		/// Opcode: 0X43, cycles: 4 ( LD B, E )
		static void operLD_EinB(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X4>(core.regs.bcwords.b, core.regs.dewords.e); }

		/// Opcode: 0X4B, cycles: 4 ( LD C, E )
		static void operLD_EinC(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X4>(core.regs.bcwords.b, core.regs.dewords.e); }

		/// Opcode: 0X53, cycles: 4 ( LD D, E )
		static void operLD_EinD(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X4>(core.regs.dewords.d, core.regs.dewords.e); }

		/// Opcode: 0X5B, cycles: 4 ( LD E, E )
		static void operLD_EinE(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operInlNop(core); }

		/// Opcode: 0X63, cycles: 4 ( LD H, E )
		static void operLD_EinH(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X4>(core.regs.hlwords.h, core.regs.dewords.e); }

		/// Opcode: 0X6B, cycles: 4 ( LD L, E )
		static void operLD_EinL(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X4>(core.regs.hlwords.l, core.regs.dewords.e); }

		/// Opcode: 0X73, cycles: 8 ( LD (HL), E )
		static void operLD_EinPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X8>(core.regs.hl /* what is pointed by hl */, static_cast<udword_t>(core.regs.dewords.e)); }

		/// Opcode: 0X7B, cycles: 4 ( LD A, E )
		static void operLD_EinA(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0x4>(core.regs.afwords.a, core.regs.dewords.e); }

		/// Opcode: 0X44, cycles: 4 ( LD B, H )
		static void operLD_HinB(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X4>(core.regs.bcwords.b, core.regs.hlwords.h); }

		/// Opcode: 0X4C, cycles: 4 ( LD C, H )
		static void operLD_HinC(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X4>(core.regs.bcwords.b, core.regs.hlwords.h); }

		/// Opcode: 0X54, cycles: 4 ( LD D, H )
		static void operLD_HinD(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X4>(core.regs.dewords.d, core.regs.hlwords.h); }

		/// Opcode: 0X5C, cycles: 4 ( LD E, H )
		static void operLD_HinE(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X4>(core.regs.dewords.e, core.regs.hlwords.h); }

		/// Opcode: 0X64, cycles: 4 ( LD H, H )
		static void operLD_HinH(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operInlNop(core); }

		/// Opcode: 0X6C, cycles: 4 ( LD L, H )
		static void operLD_HinL(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X4>(core.regs.hlwords.l, core.regs.hlwords.h); }

		/// Opcode: 0X74, cycles: 8 ( LD (HL), H )
		static void operLD_HinPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X8>(core.regs.hl /* what is pointed by hl */, static_cast<udword_t>(core.regs.hlwords.h)); }

		/// Opcode: 0X7C,  cycles: 4 ( LD A, H )
		static void operLD_HinA(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0x4>(core.regs.afwords.a, core.regs.hlwords.h); }

		/// Opcode: 0X45, cycles: 4 ( LD B, L )
		static void operLD_LinB(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X4>(core.regs.bcwords.b, core.regs.hlwords.l); }

		/// Opcode: 0X4D, cycles: 4 ( LD C, L )
		static void operLD_LinC(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X4>(core.regs.bcwords.b, core.regs.hlwords.l); }

		/// Opcode: 0X55, cycles: 4 ( LD D, L )
		static void operLD_LinD(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X4>(core.regs.dewords.d, core.regs.hlwords.l); }

		/// Opcode: 0X5D, cycles: 4 ( LD E, L )
		static void operLD_LinE(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X4>(core.regs.dewords.e, core.regs.hlwords.l); }

		/// Opcode: 0X65, cycles: 4 ( LD H, L )
		static void operLD_LinH(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X4>(core.regs.hlwords.h, core.regs.hlwords.l); }

		/// Opcode: 0X6D, cycles: 4 ( LD L, L )
		static void operLD_LinL(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operInlNop<0X4>(core); }

		/// Opcode: 0X75, cycles: 8 ( LD (HL), L )
		static void operLD_LinPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X8>(core.regs.hl /* what is pointed by hl */, static_cast<udword_t>(core.regs.hlwords.l)); }

		/// Opcode: 0X7D, cycles: 4 ( LD A, L )
		static void operLD_LinA(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0x4>(core.regs.afwords.a, core.regs.hlwords.l); }

		/// Opcode: 0X46, cycles: 8 ( LD B, (HL) )
		static void operLD_PAddrHLinB(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X8>(core.regs.bcwords.b, static_cast<udword_t>(core.regs.hl /* The value poited by */)); }

		/// Opcode: 0X4E, cycles: 8 ( LD C, (HL) )
		static void operLD_PAddrHLinC(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X8>(core.regs.bcwords.b, static_cast<udword_t>(core.regs.hl /* The value poited by */)); }

		/// Opcode: 0X56, cycles: 8 ( LD D, (HL) )
		static void operLD_PAddrHLinD(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X8>(core.regs.dewords.d, static_cast<udword_t>(core.regs.hl /* The value poited by */)); }

		/// Opcode: 0X5E, cycles: 8 ( LD E, (HL) )
		static void operLD_PAddrHLinE(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X8>(core.regs.dewords.e, static_cast<udword_t>(core.regs.hl /* The value poited by */)); }

		/// Opcode: 0X66, cycles: 8 ( LD H, (HL) )
		static void operLD_PAddrHLinH(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X8>(core.regs.hlwords.h, static_cast<udword_t>(core.regs.hl/* The value poited by */)); }

		/// Opcode: 0X6E, cycles: 8 ( LD L, (HL) )
		static void operLD_PAddrHLinL(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0X8>(core.regs.hlwords.l, static_cast<udword_t>(core.regs.hl/* The value poited by */)); }

		/// Opcode: ?, cycles: 8 ( LD (HL), (HL) ) // may be 0X76
		static void operLD_PAddrHLinPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operInlNop<0X8>(core); }

		/// Opcode: 0X7E, cycles: 8 ( LD A, (HL) )
		static void operLD_PAddrHLinA(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0x4>(core.regs.afwords.a, static_cast<udword_t>(core.regs.hl /* The value poited by */)); }

		/// Opcode: 0X47, cycles: 4 ( LD B, A )
		static void operLD_AinB(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0x4>(core.regs.bcwords.b, core.regs.afwords.a); }

		/// Opcode: 0X4F, cycles: 4 ( LD C, A )
		static void operLD_AinC(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0x4>(core.regs.bcwords.b, core.regs.afwords.a); }

		/// Opcode: 0X57, cycles: 4 ( LD D, A )
		static void operLD_AinD(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0x4>(core.regs.dewords.d, core.regs.afwords.a); }

		/// Opcode: 0X5F, cycles: 4 ( LD E, A )
		static void operLD_AinE(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0x4>(core.regs.dewords.e, core.regs.afwords.a); }

		/// Opcode: 0X67, cycles: 4 ( LD H, A )
		static void operLD_AinH(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0x4>(core.regs.hlwords.h, core.regs.afwords.a); }

		/// Opcode: 0X6F, cycles: 4 ( LD L, A )
		static void operLD_AinL(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0x4>(core.regs.hlwords.l, core.regs.afwords.a); }

		/// Opcode: 0X77, cycles: 8 ( LD (HL), A )
		static void operLD_AinPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ operBaseLD<0x8>(core.regs.hl /* what is pointed by hl */, static_cast<udword_t>(core.regs.afwords.a)); }

		/// Opcode: 0X7F, cycles: 4 ( LD A, A )
		static void operLD_AinA(const CPU<Memory, Z80Registers, uint8_t>& core)
		{ OperNop(core); }




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
								&operLD_AinPAddrBC,
								/* P = 1 */
								&operLD_AinPAddrDE,
								/* P = 2 */
								&OperNop,
								/* P = 3 */
								&OperNop
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_AinPAddrDE,
								/* P = 1 */
								&operLD_PAdrrDEinA,
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
								&operIncBC,
								/* P = 1 */
								&operIncDE,
								/* P = 2 */
								&operIncHL,
								/* P = 3 */
								&operIncSP
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operDecBC,
								/* P = 1 */
								&operDecDE,
								/* P = 2 */
								&operDecHL,
								/* P = 3 */
								&operDecSP
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
								&operIncB,
								/* P = 1 */
								&operIncB,
								/* P = 2 */
								&operIncB,
								/* P = 3 */
								&operIncB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operIncB,
								/* P = 1 */
								&operIncB,
								/* P = 2 */
								&operIncB,
								/* P = 3 */
								&operIncB
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operIncC,
								/* P = 1 */
								&operIncC,
								/* P = 2 */
								&operIncC,
								/* P = 3 */
								&operIncC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operIncC,
								/* P = 1 */
								&operIncC,
								/* P = 2 */
								&operIncC,
								/* P = 3 */
								&operIncC
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operIncD,
								/* P = 1 */
								&operIncD,
								/* P = 2 */
								&operIncD,
								/* P = 3 */
								&operIncD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operIncD,
								/* P = 1 */
								&operIncD,
								/* P = 2 */
								&operIncD,
								/* P = 3 */
								&operIncD
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operIncE,
								/* P = 1 */
								&operIncE,
								/* P = 2 */
								&operIncE,
								/* P = 3 */
								&operIncE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operIncE,
								/* P = 1 */
								&operIncE,
								/* P = 2 */
								&operIncE,
								/* P = 3 */
								&operIncE
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operIncH,
								/* P = 1 */
								&operIncH,
								/* P = 2 */
								&operIncH,
								/* P = 3 */
								&operIncH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operIncH,
								/* P = 1 */
								&operIncH,
								/* P = 2 */
								&operIncH,
								/* P = 3 */
								&operIncH
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operIncL,
								/* P = 1 */
								&operIncL,
								/* P = 2 */
								&operIncL,
								/* P = 3 */
								&operIncL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operIncL,
								/* P = 1 */
								&operIncL,
								/* P = 2 */
								&operIncL,
								/* P = 3 */
								&operIncL
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operIncPAddrHL,
								/* P = 1 */
								&operIncPAddrHL,
								/* P = 2 */
								&operIncPAddrHL,
								/* P = 3 */
								&operIncPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operIncPAddrHL,
								/* P = 1 */
								&operIncPAddrHL,
								/* P = 2 */
								&operIncPAddrHL,
								/* P = 3 */
								&operIncPAddrHL
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operIncA,
								/* P = 1 */
								&operIncA,
								/* P = 2 */
								&operIncA,
								/* P = 3 */
								&operIncA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operIncA,
								/* P = 1 */
								&operIncA,
								/* P = 2 */
								&operIncA,
								/* P = 3 */
								&operIncA
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
								&operDecB,
								/* P = 1 */
								&operDecB,
								/* P = 2 */
								&operDecB,
								/* P = 3 */
								&operDecB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operDecB,
								/* P = 1 */
								&operDecB,
								/* P = 2 */
								&operDecB,
								/* P = 3 */
								&operDecB
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operDecC,
								/* P = 1 */
								&operDecC,
								/* P = 2 */
								&operDecC,
								/* P = 3 */
								&operDecC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operDecC,
								/* P = 1 */
								&operDecC,
								/* P = 2 */
								&operDecC,
								/* P = 3 */
								&operDecC
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operDecD,
								/* P = 1 */
								&operDecD,
								/* P = 2 */
								&operDecD,
								/* P = 3 */
								&operDecD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operDecD,
								/* P = 1 */
								&operDecD,
								/* P = 2 */
								&operDecD,
								/* P = 3 */
								&operDecD
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operDecE,
								/* P = 1 */
								&operDecE,
								/* P = 2 */
								&operDecE,
								/* P = 3 */
								&operDecE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operDecE,
								/* P = 1 */
								&operDecE,
								/* P = 2 */
								&operDecE,
								/* P = 3 */
								&operDecE
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operDecH,
								/* P = 1 */
								&operDecH,
								/* P = 2 */
								&operDecH,
								/* P = 3 */
								&operDecH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operDecH,
								/* P = 1 */
								&operDecH,
								/* P = 2 */
								&operDecH,
								/* P = 3 */
								&operDecH
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operDecL,
								/* P = 1 */
								&operDecL,
								/* P = 2 */
								&operDecL,
								/* P = 3 */
								&operDecL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operDecL,
								/* P = 1 */
								&operDecL,
								/* P = 2 */
								&operDecL,
								/* P = 3 */
								&operDecL
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operDecPAddrHL,
								/* P = 1 */
								&operDecPAddrHL,
								/* P = 2 */
								&operDecPAddrHL,
								/* P = 3 */
								&operDecPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operDecPAddrHL,
								/* P = 1 */
								&operDecPAddrHL,
								/* P = 2 */
								&operDecPAddrHL,
								/* P = 3 */
								&operDecPAddrHL
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operDecA,
								/* P = 1 */
								&operDecA,
								/* P = 2 */
								&operDecA,
								/* P = 3 */
								&operDecA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operDecA,
								/* P = 1 */
								&operDecA,
								/* P = 2 */
								&operDecA,
								/* P = 3 */
								&operDecA
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
								&operLD_BinB,
								/* P = 1 */
								&operLD_BinB,
								/* P = 2 */
								&operLD_BinB,
								/* P = 3 */
								&operLD_BinB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_BinB,
								/* P = 1 */
								&operLD_BinB,
								/* P = 2 */
								&operLD_BinB,
								/* P = 3 */
								&operLD_BinB
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_BinC,
								/* P = 1 */
								&operLD_BinC,
								/* P = 2 */
								&operLD_BinC,
								/* P = 3 */
								&operLD_BinC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_BinC,
								/* P = 1 */
								&operLD_BinC,
								/* P = 2 */
								&operLD_BinC,
								/* P = 3 */
								&operLD_BinC
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_BinD,
								/* P = 1 */
								&operLD_BinD,
								/* P = 2 */
								&operLD_BinD,
								/* P = 3 */
								&operLD_BinD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_BinD,
								/* P = 1 */
								&operLD_BinD,
								/* P = 2 */
								&operLD_BinD,
								/* P = 3 */
								&operLD_BinD
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_BinE,
								/* P = 1 */
								&operLD_BinE,
								/* P = 2 */
								&operLD_BinE,
								/* P = 3 */
								&operLD_BinE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_BinE,
								/* P = 1 */
								&operLD_BinE,
								/* P = 2 */
								&operLD_BinE,
								/* P = 3 */
								&operLD_BinE
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_BinH,
								/* P = 1 */
								&operLD_BinH,
								/* P = 2 */
								&operLD_BinH,
								/* P = 3 */
								&operLD_BinH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_BinH,
								/* P = 1 */
								&operLD_BinH,
								/* P = 2 */
								&operLD_BinH,
								/* P = 3 */
								&operLD_BinH
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_BinL,
								/* P = 1 */
								&operLD_BinL,
								/* P = 2 */
								&operLD_BinL,
								/* P = 3 */
								&operLD_BinL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_BinL,
								/* P = 1 */
								&operLD_BinL,
								/* P = 2 */
								&operLD_BinL,
								/* P = 3 */
								&operLD_BinL
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_BinPAddrHL,
								/* P = 1 */
								&operLD_BinPAddrHL,
								/* P = 2 */
								&operLD_BinPAddrHL,
								/* P = 3 */
								&operLD_BinPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_BinPAddrHL,
								/* P = 1 */
								&operLD_BinPAddrHL,
								/* P = 2 */
								&operLD_BinPAddrHL,
								/* P = 3 */
								&operLD_BinPAddrHL
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_BinA,
								/* P = 1 */
								&operLD_BinA,
								/* P = 2 */
								&operLD_BinA,
								/* P = 3 */
								&operLD_BinA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_BinA,
								/* P = 1 */
								&operLD_BinA,
								/* P = 2 */
								&operLD_BinA,
								/* P = 3 */
								&operLD_BinA
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
								&operLD_CinB,
								/* P = 1 */
								&operLD_CinB,
								/* P = 2 */
								&operLD_CinB,
								/* P = 3 */
								&operLD_CinB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_CinB,
								/* P = 1 */
								&operLD_CinB,
								/* P = 2 */
								&operLD_CinB,
								/* P = 3 */
								&operLD_CinB
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_CinC,
								/* P = 1 */
								&operLD_CinC,
								/* P = 2 */
								&operLD_CinC,
								/* P = 3 */
								&operLD_CinC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_CinC,
								/* P = 1 */
								&operLD_CinC,
								/* P = 2 */
								&operLD_CinC,
								/* P = 3 */
								&operLD_CinC
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_CinD,
								/* P = 1 */
								&operLD_CinD,
								/* P = 2 */
								&operLD_CinD,
								/* P = 3 */
								&operLD_CinD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_CinD,
								/* P = 1 */
								&operLD_CinD,
								/* P = 2 */
								&operLD_CinD,
								/* P = 3 */
								&operLD_CinD
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_CinE,
								/* P = 1 */
								&operLD_CinE,
								/* P = 2 */
								&operLD_CinE,
								/* P = 3 */
								&operLD_CinE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_CinE,
								/* P = 1 */
								&operLD_CinE,
								/* P = 2 */
								&operLD_CinE,
								/* P = 3 */
								&operLD_CinE
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_CinH,
								/* P = 1 */
								&operLD_CinH,
								/* P = 2 */
								&operLD_CinH,
								/* P = 3 */
								&operLD_CinH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_CinH,
								/* P = 1 */
								&operLD_CinH,
								/* P = 2 */
								&operLD_CinH,
								/* P = 3 */
								&operLD_CinH
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_CinL,
								/* P = 1 */
								&operLD_CinL,
								/* P = 2 */
								&operLD_CinL,
								/* P = 3 */
								&operLD_CinL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_CinL,
								/* P = 1 */
								&operLD_CinL,
								/* P = 2 */
								&operLD_CinL,
								/* P = 3 */
								&operLD_CinL
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_CinPAddrHL,
								/* P = 1 */
								&operLD_CinPAddrHL,
								/* P = 2 */
								&operLD_CinPAddrHL,
								/* P = 3 */
								&operLD_CinPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_CinPAddrHL,
								/* P = 1 */
								&operLD_CinPAddrHL,
								/* P = 2 */
								&operLD_CinPAddrHL,
								/* P = 3 */
								&operLD_CinPAddrHL
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_CinA,
								/* P = 1 */
								&operLD_CinA,
								/* P = 2 */
								&operLD_CinA,
								/* P = 3 */
								&operLD_CinA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_CinA,
								/* P = 1 */
								&operLD_CinA,
								/* P = 2 */
								&operLD_CinA,
								/* P = 3 */
								&operLD_CinA
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
								&operLD_DinB,
								/* P = 1 */
								&operLD_DinB,
								/* P = 2 */
								&operLD_DinB,
								/* P = 3 */
								&operLD_DinB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_DinB,
								/* P = 1 */
								&operLD_DinB,
								/* P = 2 */
								&operLD_DinB,
								/* P = 3 */
								&operLD_DinB
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_DinC,
								/* P = 1 */
								&operLD_DinC,
								/* P = 2 */
								&operLD_DinC,
								/* P = 3 */
								&operLD_DinC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_DinC,
								/* P = 1 */
								&operLD_DinC,
								/* P = 2 */
								&operLD_DinC,
								/* P = 3 */
								&operLD_DinC
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_DinD,
								/* P = 1 */
								&operLD_DinD,
								/* P = 2 */
								&operLD_DinD,
								/* P = 3 */
								&operLD_DinD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_DinD,
								/* P = 1 */
								&operLD_DinD,
								/* P = 2 */
								&operLD_DinD,
								/* P = 3 */
								&operLD_DinD
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_DinE,
								/* P = 1 */
								&operLD_DinE,
								/* P = 2 */
								&operLD_DinE,
								/* P = 3 */
								&operLD_DinE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_DinE,
								/* P = 1 */
								&operLD_DinE,
								/* P = 2 */
								&operLD_DinE,
								/* P = 3 */
								&operLD_DinE
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_DinH,
								/* P = 1 */
								&operLD_DinH,
								/* P = 2 */
								&operLD_DinH,
								/* P = 3 */
								&operLD_DinH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_DinH,
								/* P = 1 */
								&operLD_DinH,
								/* P = 2 */
								&operLD_DinH,
								/* P = 3 */
								&operLD_DinH
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_DinL,
								/* P = 1 */
								&operLD_DinL,
								/* P = 2 */
								&operLD_DinL,
								/* P = 3 */
								&operLD_DinL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_DinL,
								/* P = 1 */
								&operLD_DinL,
								/* P = 2 */
								&operLD_DinL,
								/* P = 3 */
								&operLD_DinL
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_DinPAddrHL,
								/* P = 1 */
								&operLD_DinPAddrHL,
								/* P = 2 */
								&operLD_DinPAddrHL,
								/* P = 3 */
								&operLD_DinPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_DinPAddrHL,
								/* P = 1 */
								&operLD_DinPAddrHL,
								/* P = 2 */
								&operLD_DinPAddrHL,
								/* P = 3 */
								&operLD_DinPAddrHL
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_DinA,
								/* P = 1 */
								&operLD_DinA,
								/* P = 2 */
								&operLD_DinA,
								/* P = 3 */
								&operLD_DinA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_DinA,
								/* P = 1 */
								&operLD_DinA,
								/* P = 2 */
								&operLD_DinA,
								/* P = 3 */
								&operLD_DinA
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
								&operLD_EinB,
								/* P = 1 */
								&operLD_EinB,
								/* P = 2 */
								&operLD_EinB,
								/* P = 3 */
								&operLD_EinB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_EinB,
								/* P = 1 */
								&operLD_EinB,
								/* P = 2 */
								&operLD_EinB,
								/* P = 3 */
								&operLD_EinB
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_EinC,
								/* P = 1 */
								&operLD_EinC,
								/* P = 2 */
								&operLD_EinC,
								/* P = 3 */
								&operLD_EinC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_EinC,
								/* P = 1 */
								&operLD_EinC,
								/* P = 2 */
								&operLD_EinC,
								/* P = 3 */
								&operLD_EinC
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_EinD,
								/* P = 1 */
								&operLD_EinD,
								/* P = 2 */
								&operLD_EinD,
								/* P = 3 */
								&operLD_EinD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_EinD,
								/* P = 1 */
								&operLD_EinD,
								/* P = 2 */
								&operLD_EinD,
								/* P = 3 */
								&operLD_EinD
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_EinE,
								/* P = 1 */
								&operLD_EinE,
								/* P = 2 */
								&operLD_EinE,
								/* P = 3 */
								&operLD_EinE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_EinE,
								/* P = 1 */
								&operLD_EinE,
								/* P = 2 */
								&operLD_EinE,
								/* P = 3 */
								&operLD_EinE
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_EinH,
								/* P = 1 */
								&operLD_EinH,
								/* P = 2 */
								&operLD_EinH,
								/* P = 3 */
								&operLD_EinH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_EinH,
								/* P = 1 */
								&operLD_EinH,
								/* P = 2 */
								&operLD_EinH,
								/* P = 3 */
								&operLD_EinH
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_EinL,
								/* P = 1 */
								&operLD_EinL,
								/* P = 2 */
								&operLD_EinL,
								/* P = 3 */
								&operLD_EinL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_EinL,
								/* P = 1 */
								&operLD_EinL,
								/* P = 2 */
								&operLD_EinL,
								/* P = 3 */
								&operLD_EinL
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_EinPAddrHL,
								/* P = 1 */
								&operLD_EinPAddrHL,
								/* P = 2 */
								&operLD_EinPAddrHL,
								/* P = 3 */
								&operLD_EinPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_EinPAddrHL,
								/* P = 1 */
								&operLD_EinPAddrHL,
								/* P = 2 */
								&operLD_EinPAddrHL,
								/* P = 3 */
								&operLD_EinPAddrHL
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_EinA,
								/* P = 1 */
								&operLD_EinA,
								/* P = 2 */
								&operLD_EinA,
								/* P = 3 */
								&operLD_EinA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_EinA,
								/* P = 1 */
								&operLD_EinA,
								/* P = 2 */
								&operLD_EinA,
								/* P = 3 */
								&operLD_EinA
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
								&operLD_HinB,
								/* P = 1 */
								&operLD_HinB,
								/* P = 2 */
								&operLD_HinB,
								/* P = 3 */
								&operLD_HinB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_HinB,
								/* P = 1 */
								&operLD_HinB,
								/* P = 2 */
								&operLD_HinB,
								/* P = 3 */
								&operLD_HinB
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_HinC,
								/* P = 1 */
								&operLD_HinC,
								/* P = 2 */
								&operLD_HinC,
								/* P = 3 */
								&operLD_HinC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_HinC,
								/* P = 1 */
								&operLD_HinC,
								/* P = 2 */
								&operLD_HinC,
								/* P = 3 */
								&operLD_HinC
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_HinD,
								/* P = 1 */
								&operLD_HinD,
								/* P = 2 */
								&operLD_HinD,
								/* P = 3 */
								&operLD_HinD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_HinD,
								/* P = 1 */
								&operLD_HinD,
								/* P = 2 */
								&operLD_HinD,
								/* P = 3 */
								&operLD_HinD
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_HinE,
								/* P = 1 */
								&operLD_HinE,
								/* P = 2 */
								&operLD_HinE,
								/* P = 3 */
								&operLD_HinE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_HinE,
								/* P = 1 */
								&operLD_HinE,
								/* P = 2 */
								&operLD_HinE,
								/* P = 3 */
								&operLD_HinE
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_HinH,
								/* P = 1 */
								&operLD_HinH,
								/* P = 2 */
								&operLD_HinH,
								/* P = 3 */
								&operLD_HinH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_HinH,
								/* P = 1 */
								&operLD_HinH,
								/* P = 2 */
								&operLD_HinH,
								/* P = 3 */
								&operLD_HinH
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_HinL,
								/* P = 1 */
								&operLD_HinL,
								/* P = 2 */
								&operLD_HinL,
								/* P = 3 */
								&operLD_HinL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_HinL,
								/* P = 1 */
								&operLD_HinL,
								/* P = 2 */
								&operLD_HinL,
								/* P = 3 */
								&operLD_HinL
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_HinPAddrHL,
								/* P = 1 */
								&operLD_HinPAddrHL,
								/* P = 2 */
								&operLD_HinPAddrHL,
								/* P = 3 */
								&operLD_HinPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_HinPAddrHL,
								/* P = 1 */
								&operLD_HinPAddrHL,
								/* P = 2 */
								&operLD_HinPAddrHL,
								/* P = 3 */
								&operLD_HinPAddrHL
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_HinA,
								/* P = 1 */
								&operLD_HinA,
								/* P = 2 */
								&operLD_HinA,
								/* P = 3 */
								&operLD_HinA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_HinA,
								/* P = 1 */
								&operLD_HinA,
								/* P = 2 */
								&operLD_HinA,
								/* P = 3 */
								&operLD_HinA
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
								&operLD_LinB,
								/* P = 1 */
								&operLD_LinB,
								/* P = 2 */
								&operLD_LinB,
								/* P = 3 */
								&operLD_LinB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_LinB,
								/* P = 1 */
								&operLD_LinB,
								/* P = 2 */
								&operLD_LinB,
								/* P = 3 */
								&operLD_LinB
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_LinC,
								/* P = 1 */
								&operLD_LinC,
								/* P = 2 */
								&operLD_LinC,
								/* P = 3 */
								&operLD_LinC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_LinC,
								/* P = 1 */
								&operLD_LinC,
								/* P = 2 */
								&operLD_LinC,
								/* P = 3 */
								&operLD_LinC
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_LinD,
								/* P = 1 */
								&operLD_LinD,
								/* P = 2 */
								&operLD_LinD,
								/* P = 3 */
								&operLD_LinD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_LinD,
								/* P = 1 */
								&operLD_LinD,
								/* P = 2 */
								&operLD_LinD,
								/* P = 3 */
								&operLD_LinD
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_LinE,
								/* P = 1 */
								&operLD_LinE,
								/* P = 2 */
								&operLD_LinE,
								/* P = 3 */
								&operLD_LinE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_LinE,
								/* P = 1 */
								&operLD_LinE,
								/* P = 2 */
								&operLD_LinE,
								/* P = 3 */
								&operLD_LinE
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_LinH,
								/* P = 1 */
								&operLD_LinH,
								/* P = 2 */
								&operLD_LinH,
								/* P = 3 */
								&operLD_LinH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_LinH,
								/* P = 1 */
								&operLD_LinH,
								/* P = 2 */
								&operLD_LinH,
								/* P = 3 */
								&operLD_LinH
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_LinL,
								/* P = 1 */
								&operLD_LinL,
								/* P = 2 */
								&operLD_LinL,
								/* P = 3 */
								&operLD_LinL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_LinL,
								/* P = 1 */
								&operLD_LinL,
								/* P = 2 */
								&operLD_LinL,
								/* P = 3 */
								&operLD_LinL
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_LinPAddrHL,
								/* P = 1 */
								&operLD_LinPAddrHL,
								/* P = 2 */
								&operLD_LinPAddrHL,
								/* P = 3 */
								&operLD_LinPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_LinPAddrHL,
								/* P = 1 */
								&operLD_LinPAddrHL,
								/* P = 2 */
								&operLD_LinPAddrHL,
								/* P = 3 */
								&operLD_LinPAddrHL
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_LinA,
								/* P = 1 */
								&operLD_LinA,
								/* P = 2 */
								&operLD_LinA,
								/* P = 3 */
								&operLD_LinA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_LinA,
								/* P = 1 */
								&operLD_LinA,
								/* P = 2 */
								&operLD_LinA,
								/* P = 3 */
								&operLD_LinA
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
								&operLD_PAddrHLinB,
								/* P = 1 */
								&operLD_PAddrHLinB,
								/* P = 2 */
								&operLD_PAddrHLinB,
								/* P = 3 */
								&operLD_PAddrHLinB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_PAddrHLinB,
								/* P = 1 */
								&operLD_PAddrHLinB,
								/* P = 2 */
								&operLD_PAddrHLinB,
								/* P = 3 */
								&operLD_PAddrHLinB
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_PAddrHLinC,
								/* P = 1 */
								&operLD_PAddrHLinC,
								/* P = 2 */
								&operLD_PAddrHLinC,
								/* P = 3 */
								&operLD_PAddrHLinC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_PAddrHLinC,
								/* P = 1 */
								&operLD_PAddrHLinC,
								/* P = 2 */
								&operLD_PAddrHLinC,
								/* P = 3 */
								&operLD_PAddrHLinC
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_PAddrHLinD,
								/* P = 1 */
								&operLD_PAddrHLinD,
								/* P = 2 */
								&operLD_PAddrHLinD,
								/* P = 3 */
								&operLD_PAddrHLinD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_PAddrHLinD,
								/* P = 1 */
								&operLD_PAddrHLinD,
								/* P = 2 */
								&operLD_PAddrHLinD,
								/* P = 3 */
								&operLD_PAddrHLinD
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_PAddrHLinE,
								/* P = 1 */
								&operLD_PAddrHLinE,
								/* P = 2 */
								&operLD_PAddrHLinE,
								/* P = 3 */
								&operLD_PAddrHLinE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_PAddrHLinE,
								/* P = 1 */
								&operLD_PAddrHLinE,
								/* P = 2 */
								&operLD_PAddrHLinE,
								/* P = 3 */
								&operLD_PAddrHLinE
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_PAddrHLinH,
								/* P = 1 */
								&operLD_PAddrHLinH,
								/* P = 2 */
								&operLD_PAddrHLinH,
								/* P = 3 */
								&operLD_PAddrHLinH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_PAddrHLinH,
								/* P = 1 */
								&operLD_PAddrHLinH,
								/* P = 2 */
								&operLD_PAddrHLinH,
								/* P = 3 */
								&operLD_PAddrHLinH
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_PAddrHLinL,
								/* P = 1 */
								&operLD_PAddrHLinL,
								/* P = 2 */
								&operLD_PAddrHLinL,
								/* P = 3 */
								&operLD_PAddrHLinL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_PAddrHLinL,
								/* P = 1 */
								&operLD_PAddrHLinL,
								/* P = 2 */
								&operLD_PAddrHLinL,
								/* P = 3 */
								&operLD_PAddrHLinL
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_PAddrHLinPAddrHL,
								/* P = 1 */
								&operLD_PAddrHLinPAddrHL,
								/* P = 2 */
								&operLD_PAddrHLinPAddrHL,
								/* P = 3 */
								&operLD_PAddrHLinPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_PAddrHLinPAddrHL,
								/* P = 1 */
								&operLD_PAddrHLinPAddrHL,
								/* P = 2 */
								&operLD_PAddrHLinPAddrHL,
								/* P = 3 */
								&operLD_PAddrHLinPAddrHL
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_PAddrHLinA,
								/* P = 1 */
								&operLD_PAddrHLinA,
								/* P = 2 */
								&operLD_PAddrHLinA,
								/* P = 3 */
								&operLD_PAddrHLinA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_PAddrHLinA,
								/* P = 1 */
								&operLD_PAddrHLinA,
								/* P = 2 */
								&operLD_PAddrHLinA,
								/* P = 3 */
								&operLD_PAddrHLinA
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
								&operLD_AinB,
								/* P = 1 */
								&operLD_AinB,
								/* P = 2 */
								&operLD_AinB,
								/* P = 3 */
								&operLD_AinB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_AinB,
								/* P = 1 */
								&operLD_AinB,
								/* P = 2 */
								&operLD_AinB,
								/* P = 3 */
								&operLD_AinB
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_AinC,
								/* P = 1 */
								&operLD_AinC,
								/* P = 2 */
								&operLD_AinC,
								/* P = 3 */
								&operLD_AinC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_AinC,
								/* P = 1 */
								&operLD_AinC,
								/* P = 2 */
								&operLD_AinC,
								/* P = 3 */
								&operLD_AinC
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_AinD,
								/* P = 1 */
								&operLD_AinD,
								/* P = 2 */
								&operLD_AinD,
								/* P = 3 */
								&operLD_AinD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_AinD,
								/* P = 1 */
								&operLD_AinD,
								/* P = 2 */
								&operLD_AinD,
								/* P = 3 */
								&operLD_AinD
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_AinE,
								/* P = 1 */
								&operLD_AinE,
								/* P = 2 */
								&operLD_AinE,
								/* P = 3 */
								&operLD_AinE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_AinE,
								/* P = 1 */
								&operLD_AinE,
								/* P = 2 */
								&operLD_AinE,
								/* P = 3 */
								&operLD_AinE
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_AinH,
								/* P = 1 */
								&operLD_AinH,
								/* P = 2 */
								&operLD_AinH,
								/* P = 3 */
								&operLD_AinH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_AinH,
								/* P = 1 */
								&operLD_AinH,
								/* P = 2 */
								&operLD_AinH,
								/* P = 3 */
								&operLD_AinH
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_AinL,
								/* P = 1 */
								&operLD_AinL,
								/* P = 2 */
								&operLD_AinL,
								/* P = 3 */
								&operLD_AinL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_AinL,
								/* P = 1 */
								&operLD_AinL,
								/* P = 2 */
								&operLD_AinL,
								/* P = 3 */
								&operLD_AinL
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_AinPAddrHL,
								/* P = 1 */
								&operLD_AinPAddrHL,
								/* P = 2 */
								&operLD_AinPAddrHL,
								/* P = 3 */
								&operLD_AinPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_AinPAddrHL,
								/* P = 1 */
								&operLD_AinPAddrHL,
								/* P = 2 */
								&operLD_AinPAddrHL,
								/* P = 3 */
								&operLD_AinPAddrHL
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operLD_AinA,
								/* P = 1 */
								&operLD_AinA,
								/* P = 2 */
								&operLD_AinA,
								/* P = 3 */
								&operLD_AinA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operLD_AinA,
								/* P = 1 */
								&operLD_AinA,
								/* P = 2 */
								&operLD_AinA,
								/* P = 3 */
								&operLD_AinA
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
			// TODO: isValid seems to always return true ...
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
		: CPU<mem_t, reg_t, flags_t>(memory_type, Z80Registers(0XFFFE, 0x100), (flags_t)0)
		{
			/* ... */
		}
    };
}
