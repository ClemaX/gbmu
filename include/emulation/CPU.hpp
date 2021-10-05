
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

# define MASKBIT0 (1 << 0)
# define MASKBIT1 (MASKBIT0 << 1)
# define MASKBIT2 (MASKBIT1 << 1)
# define MASKBIT3 (MASKBIT2 << 1)
# define MASKBIT4 (MASKBIT3 << 1)
# define MASKBIT5 (MASKBIT4 << 1)
# define MASKBIT6 (MASKBIT5 << 1)
# define MASKBIT7 (MASKBIT6 << 1)
# define FGMASK_ADD(target, mask) ((target) |= (mask))
# define FGMASK_DEL(target, mask) ((target) &= ~(mask))
# define FGMASK_HAS(target, mask) ((target) & (mask))

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
		bool			isValid() const
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
			/// TODO: Check if i shift the right amount
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

		~Z80Registers()
		{ }
	};

	class Chrono
	{
		typedef std::chrono::_V2::system_clock::time_point time_point;

		time_point	start;
		int64_t		cycles;

		public:

		Chrono()
		: start(std::chrono::high_resolution_clock::now()), cycles(4)
		{ }

		Chrono(int64_t cyclesNb)
		: start(std::chrono::high_resolution_clock::now()), cycles(cyclesNb)
		{ }

		~Chrono()
		{ }

		void setCycles(int64_t cycles) { this->cycles = cycles; }

		# include <sys/time.h> // to remove when i found how to construct a time_point with an integer

		void waitForCycles()
		{
#ifdef TODO4242
			//std::chrono::duration<int64_t> dur(); // dur in seconds
			const std::chrono::milliseconds dur(CLOCK_CYCLE_NS * this->cycles); // dur in ms
			const std::chrono::time_point<std::chrono::high_resolution_clock> dt(dur); // more precision
			/// TODO: Check conversions: i need nanoseconds (CLOCK_CYCLE_NS value)

			const time_point& endT = std::chrono::high_resolution_clock::now();
			/// NOTE: sleep_for ignores negatice values & call nanosleep
			std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>
			(dt - endT - this->start));
#endif
			// to remove when i found how to construct a time_point with an integer
			const time_point& endT = std::chrono::high_resolution_clock::now();

			struct timespec tp = {
				.tv_sec = 0,
				.tv_nsec = static_cast<int64_t>(CLOCK_CYCLE_NS) * this->cycles - endT.time_since_epoch().count() - this->start.time_since_epoch().count()
			};

			nanosleep(NULL, &tp);
		}
	};

	template <class Memory, class Registers, typename Flags>
	class CPU
	{
		protected:

		typedef Memory		mem_t;
		typedef Registers	reg_t;
		typedef Flags		flags_t;

		typedef CPU<mem_t, reg_t, flags_t> cpu_t;
		typedef void (*exeOpcode_t)(const cpu_t& core, Chrono& c);

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
			FLAG_BIT0		= (1 << 0),
			FLAG_BIT1		= (FLAG_BIT0 << 1),
			FLAG_BIT2		= (FLAG_BIT1 << 1),
			FLAG_BIT3		= (FLAG_BIT2 << 1),
			FLAG_ZERO		= (FLAG_BIT3 << 1),
			FLAG_SUBSTRACT	= (FLAG_ZERO << 1),
			FLAG_HALF_CARRY	= (FLAG_SUBSTRACT << 1),
			FLAG_CARRY		= (FLAG_HALF_CARRY << 1),

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

		template <typename T, int64_t cycles>
		__attribute__ ((always_inline))
		static inline void
		operBaseLD(T& dest, T src, Chrono& c)
		noexcept
		{
			c.setCycles(cycles);
			dest = src;
		}

		template <typename T, typename Flags, int64_t cycles>
		__attribute__ ((always_inline))
		static inline void
		operBaseInc(T& target, Chrono& c)
		noexcept
		{
			++target;
			c.setCycles(cycles);
		}

		template <typename T, typename Flags, int64_t cycles>
		__attribute__ ((always_inline))
		static inline void
		operBaseDec(T& target, Chrono& c)
		noexcept
		{
			--target;
			c.setCycles(cycles);
		}

		template <int64_t cycles = 4>
		static inline void
		__attribute__ ((always_inline))
		operInlNop(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			static_cast<void>(core);
			c.setCycles(cycles);
		}

		template <typename T, int64_t cycles = 4>
		static inline void
		__attribute__ ((always_inline))
		operBaseAdd(T& dest, T src, Chrono& c)
		noexcept
		{
			dest += src;
			c.setCycles(cycles);
		}

		template <typename T, typename Flags, int64_t cycles = 4>
		static inline void
		__attribute__ ((always_inline))
		operBaseAddCarry(T& dest, T src, Chrono& c, Flags flags)
		noexcept
		{
			operBaseAdd(dest, src, c);
			///TODO: Add carry flag
			static_cast<void>(flags);
		}

		template <typename T, int64_t cycles = 4>
		static inline void
		__attribute__ ((always_inline))
		operBaseSub(T& dest, T src, Chrono& c)
		noexcept
		{
			dest -= src;
			c.setCycles(cycles);
		}

		template <typename T, typename Flags, int64_t cycles = 4>
		static inline void
		__attribute__ ((always_inline))
		operBaseSubCarry(T& dest, T src, Chrono& c, Flags flags)
		noexcept
		{
			operBaseSub(dest, src, c);
			///TODO: Add carry flag
			static_cast<void>(flags);
		}

		template <typename T, int64_t cycles = 4>
		static inline void
		__attribute__ ((always_inline))
		operBaseAnd(T& dest, T src, Chrono& c)
		noexcept
		{
			dest &= src;
			c.setCycles(cycles);
		}

		template <typename T, int64_t cycles = 4>
		static inline void
		__attribute__ ((always_inline))
		operBaseXor(T& dest, T src, Chrono& c)
		noexcept
		{
			dest ^= src;
			c.setCycles(cycles);
		}

		template <typename T, int64_t cycles = 4>
		static inline void
		__attribute__ ((always_inline))
		operBaseOr(T& dest, T src, Chrono& c)
		noexcept
		{
			dest |= src;
			c.setCycles(cycles);
		}

		template <typename T, int64_t cycles = 4>
		static inline void
		__attribute__ ((always_inline))
		operBaseCp(T& dest, T src, Chrono& c)
		noexcept
		{
			static_cast<void>(dest);
			static_cast<void>(src);
			///TODO:
			c.setCycles(cycles);
		}

		template <typename T, int64_t cycles = 8>
		static inline void
		__attribute__ ((always_inline))
		operBaseSet(T& dest, T bitMask, Chrono& c)
		noexcept
		{
			FGMASK_ADD(dest, bitMask);
			c.setCycles(cycles);
		}

		template <typename T, int64_t cycles = 8>
		static inline void
		__attribute__ ((always_inline))
		operBaseRes(T& dest, T bitMask, Chrono& c)
		noexcept
		{
			FGMASK_DEL(dest, bitMask);
			c.setCycles(cycles);
		}

		template <typename T, typename Flags, int64_t cycles = 8>
		static inline void
		__attribute__ ((always_inline))
		operBaseBit(T& dest, T bitMask, Flags flags, Chrono& c)
		noexcept
		{
			/** TODO:
			 	Z - Set if bit b of register r is 0.
  				N - Reset.
  				H - Set.
  				C - Not affected.
			*/
			static_cast<void>(flags);


			if (FGMASK_HAS(dest, bitMask) == false)
				; // Set Z flag
			// Reset N flag
			// Set H flag
			c.setCycles(cycles);
		}

		//////////////////////////////////
		// Operations function pointers //
		//////////////////////////////////

		/// Opcode 0x0 + all ignored instructions
		static void OperNop(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operInlNop(core, c); }

		/////////////////////////////
		// 16 BIT LOADS OPERATIONS //
		/////////////////////////////

//
///TODO: No y, find the exact spot to place them with opcode value
//
		/// Opcode 0X02, cycles: 8 ( LD (BC), A )
		static void
		operLD_AinPAddrBC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X8>(/* memory at this addr */core.regs.bc, static_cast<udword_t>(core.regs.afwords.a, c)); }

		/// Opcode 0X12, cycles: 8 ( LD (DE), A )
		static void
		operLD_AinPAddrDE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X8>(/* memory at this addr */core.regs.de, static_cast<udword_t>(core.regs.afwords.a, c)); }

		/// Opcode 0X0A, cycles: 8 ( LD A, (BC) )
		static void
		operLD_PAdrrBCinA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X8>(core.regs.afwords.a, /* value at the memory at this addr */core.regs.bc, c); }

		/// Opcode 0X1A, cycles: 8 ( LD A, (DE) )
		static void
		operLD_PAdrrDEinA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X8>(core.regs.afwords.a, /* value at the memory at this addr */core.regs.de, c); }

		/// Opcode: ? ( LD (nn), HL )

		/// Opcode: ? ( LD (nn), A )

		/// Opcode: ?, ( LD HL, (nn) )

		/// Opcode: ? ( LD A, (nn) )

		/////////////////////////////////////
		// 16 BIT INC/DEC (ALU) OPERATIONS //
		/////////////////////////////////////

//
///TODO: No y, find the exact spot to place them with opcode value
//
		/// Opcode: 0X3, cycles: 8 ( INC BC )
		static void
		operIncBC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseInc<0X8>(core.regs.bc, c); }

		/// Opcode: 0X13, cycles: 8 ( INC DE )
		static void
		operIncDE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseInc<0X8>(core.regs.de, c); }

		/// Opcode: 0X23, cycles: 8 ( INC HL )
		static void
		operIncHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseInc<0X8>(core.regs.hl, c); }

		/// Opcode: 0X33, cycles: 8 ( INC SP )
		static void
		operIncSP(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseInc<0X8>(core.regs.af, c); }

		/// Opcode: 0X0B, cycles: 8 ( DEC BC )
		static void
		operDecBC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseDec<0X8>(core.regs.bc, c); }

		/// Opcode: 0X1B, cycles: 8 ( DEC DE )
		static void
		operDecDE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseDec<0X8>(core.regs.de, c); }

		/// Opcode: 0X2B, cycles: 8 ( DEC HL )
		static void
		operDecHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseDec<0X8>(core.regs.hl, c); }

		/// Opcode: 0X3B, cycles: 8 ( DEC SP )
		static void
		operDecSP(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseDec<0X8>(core.regs.af, c); }


		////////////////////////////////////
		// 8 BIT INC/DEC (ALU) OPERATIONS //
		////////////////////////////////////
//
///TODO: All p & q have been set, find which one set to operNop
///TODO: may need std::forward to pass c as move (or maybe i can't pass it as move value)
//

		/** NOTE: Flags affected:
			Z - Set if result is zero.
			N - Reset.
			H - Set if carry from bit 4.
			C - Not affected.
		*/
		template <typename V>
		static void inline
		handleFlagsIncWord(typename CPU<Memory, Z80Registers, uint8_t>::flags_t& flags, const V value)
		noexcept
		{
			///TODO:
			static_cast<void>(flags);
			static_cast<void>(value);
		}

		/// Opcode: 0X04, cycles: 4 ( INC B )
		static void
		operIncB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseInc<0X4>(core.regs.bcwords.b, c);
			handleFlagsIncWord(core.flags, core.regs.bcwords.b);
		}

		/// Opcode: 0X0C, cycles: 4 ( INC C )
		static void
		operIncC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseInc<0X4>(core.regs.bcwords.c, c);
			handleFlagsIncWord(core.flags, core.regs.bcwords.c);
		}

		/// Opcode: 0X14, cycles: 4 ( INC D )
		static void
		operIncD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseInc<0X4>(core.regs.dewords.d, c);
			handleFlagsIncWord(core.flags, core.regs.dewords.d);
		}

		/// Opcode: 0X1C, cycles: 4 ( INC E )
		static void
		operIncE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseInc<0X4>(core.regs.dewords.e, c);
			handleFlagsIncWord(core.flags, core.regs.dewords.e);
		}

		/// Opcode: 0X24, cycles: 4 ( INC H )
		static void
		operIncH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseInc<0X4>(core.regs.hlwords.h, c);
			handleFlagsIncWord(core.flags, core.regs.hlwords.h);
		}

		/// Opcode: 0X2C, cycles: 4 ( INC L )
		static void
		operIncL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseInc<0X4>(core.regs.hlwords.l, c);
			handleFlagsIncWord(core.flags,core.regs.hlwords.l);
		}

		/// Opcode: 0X34, cycles: 12 ( INC (HL) )
		static void
		operIncPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseInc<0XC>(core.regs.hl /* What is pointed by hl */, c);
			handleFlagsIncWord(core.flags, core.regs.hl /* What is pointed by hl */);
		}

		/// Opcode: 0X3C, cycles: 4 ( INC A )
		static void
		operIncA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseInc<0X4>(core.regs.afwords.a, c);
			handleFlagsIncWord(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0X05, cycles: 4 ( DEC B )
		static void
		operDecB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseDec<0X4>(core.regs.bcwords.b, c);
			handleFlagsIncWord(core.flags, core.regs.bcwords.b);
		}

		/// Opcode: 0X0D, cycles: 4 ( DEC C )
		static void
		operDecC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseDec<0X4>(core.regs.bcwords.c, c);
			handleFlagsIncWord(core.flags, core.regs.bcwords.c);
		}

		/// Opcode: 0X15, cycles: 4 ( DEC D )
		static void
		operDecD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseDec<0X4>(core.regs.dewords.d, c);
			handleFlagsIncWord(core.flags, core.regs.dewords.d);
		}

		/// Opcode: 0X1D, cycles: 4 ( DEC E )
		static void
		operDecE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseDec<0X4>(core.regs.dewords.e, c);
			handleFlagsIncWord(core.flags, core.regs.dewords.e);
		}

		/// Opcode: 0X25, cycles: 4 ( DEC H )
		static void
		operDecH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseDec<0X4>(core.regs.hlwords.h, c);
			handleFlagsIncWord(core.flags, core.regs.hlwords.h);
		}

		/// Opcode: 0X2D, cycles: 4 ( DEC L )
		static void
		operDecL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseDec<0X4>(core.regs.hlwords.l, c);
			handleFlagsIncWord(core.flags, core.regs.hlwords.l);
		}

		/// Opcode: 0X35, cycles: 12 ( DEC (HL) )
		static void
		operDecPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseDec<0XC>(core.regs.hl /* What is pointed by hl */, c);
			handleFlagsIncWord(core.flags, core.regs.hl /* What is pointed by hl */);
		}

		/// Opcode: 0X3D, cycles: 4 ( DEC A )
		static void
		operDecA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseDec<0X4>(core.regs.afwords.a, c);
			handleFlagsIncWord(core.flags, core.regs.afwords.a);
		}






		/// Opcode: 0X07, cycles: 4 ( RLCA )
		/// Opcode: 0X0F, cycles: 4 ( RRCA )
		/// Opcode: 0X17, cycles: 4 ( RLA )
		/// Opcode: 0X1F, cycles: 4 ( RRA )
		/// Opcode: ? ( DAA )
		/// Opcode: ? ( CPL )
		/// Opcode: ? ( SCF )
		/// Opcode: ? ( CCF )

		////////////////////////////
		// 8 BIT LOADS OPERATIONS //
		////////////////////////////

//
///TODO: All p & q have been set, find which one set to operNop
//

		/// Opcode: 0X40, cycles: 4 ( LD B, B )
		static void
		operLD_BinB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operInlNop(core, c); }

		/// Opcode: 0X48, cycles: 4 ( LD C, B )
		static void
		operLD_BinC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X4>(core.regs.bcwords.c, core.regs.bcwords.b, c); }

		/// Opcode: 0X50, cycles: 4 ( LD D, B )
		static void
		operLD_BinD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X4>(core.regs.dewords.d, core.regs.bcwords.b, c); }

		/// Opcode: 0X58, cycles: 4 ( LD E, B )
		static void
		operLD_BinE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X4>(core.regs.dewords.e, core.regs.bcwords.b, c); }

		/// Opcode: 0X60, cycles: 4 ( LD H, B )
		static void
		operLD_BinH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X4>(core.regs.hlwords.h, core.regs.bcwords.b, c); }

		/// Opcode: 0x68, cycles: 4 ( LD L, B )
		static void
		operLD_BinL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X4>(core.regs.hlwords.l, core.regs.bcwords.b, c); }

		/// Opcode: 0X70, cycles: 8 ( LD (HL), B )
		static void
		operLD_BinPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X8>(core.regs.hl /* what is pointed by hl */, static_cast<udword_t>(core.regs.bcwords.b, c)); }

		/// Opcode: 0X78, cycles: 4 ( LD A, B )
		static void
		operLD_BinA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0x4>(core.regs.afwords.a, core.regs.bcwords.b, c); }

		/// Opcode: 0X41, cycles: 4 ( LD B, C )
		static void
		operLD_CinB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X4>(core.regs.bcwords.b, core.regs.bcwords.c, c); }

		/// Opcode: 0X49, cycles: 4 ( LD C, C )
		static void
		operLD_CinC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operInlNop(core, c); }

		/// Opcode: 0x4A, cycles: 4 ( LD D, C )
		static void
		operLD_CinD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X4>(core.regs.dewords.d, core.regs.bcwords.c, c); }

		/// Opcode: 0X59, cycles: 4  ( LD E, C )
		static void
		operLD_CinE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X4>(core.regs.dewords.e, core.regs.bcwords.c, c); }

		/// Opcode: 0X61, cycles: 4 ( LD H, C )
		static void
		operLD_CinH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X4>(core.regs.hlwords.h, core.regs.bcwords.c, c); }

		/// Opcode: 0X69, cycles: 4 ( LD L, C )
		static void
		operLD_CinL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X4>(core.regs.hlwords.l, core.regs.bcwords.c, c); }

		/// Opcode: 0X71, cycles: 8 ( LD (HL), C )
		static void
		operLD_CinPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X8>(core.regs.hl /* what is pointed by hl */, static_cast<udword_t>(core.regs.bcwords.c, c)); }

		/// Opcode: 0X79, cycles:4 ( LD A, C )
		static void
		operLD_CinA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X4>(core.regs.afwords.a, core.regs.bcwords.c, c); }

		/// Opcode: 0X42, cycles: 4 ( LD B, D )
		static void
		operLD_DinB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X4>(core.regs.bcwords.b, core.regs.dewords.d, c); }

		/// Opcode: 0X4A, cycles: 4 ( LD C, D )
		static void
		operLD_DinC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X4>(core.regs.bcwords.b, core.regs.dewords.d, c); }

		/// Opcode: 0X52, cycles: 4 ( LD D, D )
		static void
		operLD_DinD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operInlNop(core, c); }

		/// Opcode: 0X5A, cycles: 4 ( LD E, D )
		static void
		operLD_DinE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X4>(core.regs.dewords.e, core.regs.dewords.d, c); }

		/// Opcode: 0X62, cycles: 4 ( LD H, D )
		static void
		operLD_DinH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X4>(core.regs.hlwords.h, core.regs.dewords.d, c); }

		/// Opcode: 0X6A, cycles: 4 ( LD L, D )
		static void
		operLD_DinL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X4>(core.regs.hlwords.l, core.regs.dewords.d, c); }

		/// Opcode: 0X72, cycles: 8 ( LD (HL), D )
		static void
		operLD_DinPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X8>(core.regs.hl /* what is pointed by hl */, static_cast<udword_t>(core.regs.dewords.d), c); }

		/// Opcode: 0X7A, cycles: 4 ( LD A, D )
		static void
		operLD_DinA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0x4>(core.regs.afwords.a, core.regs.dewords.d, c); }

		/// Opcode: 0X43, cycles: 4 ( LD B, E )
		static void
		operLD_EinB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X4>(core.regs.bcwords.b, core.regs.dewords.e, c); }

		/// Opcode: 0X4B, cycles: 4 ( LD C, E )
		static void
		operLD_EinC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X4>(core.regs.bcwords.b, core.regs.dewords.e, c); }

		/// Opcode: 0X53, cycles: 4 ( LD D, E )
		static void
		operLD_EinD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X4>(core.regs.dewords.d, core.regs.dewords.e, c); }

		/// Opcode: 0X5B, cycles: 4 ( LD E, E )
		static void
		operLD_EinE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operInlNop(core, c); }

		/// Opcode: 0X63, cycles: 4 ( LD H, E )
		static void
		operLD_EinH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X4>(core.regs.hlwords.h, core.regs.dewords.e, c); }

		/// Opcode: 0X6B, cycles: 4 ( LD L, E )
		static void
		operLD_EinL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X4>(core.regs.hlwords.l, core.regs.dewords.e, c); }

		/// Opcode: 0X73, cycles: 8 ( LD (HL), E )
		static void
		operLD_EinPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X8>(core.regs.hl /* what is pointed by hl */, static_cast<udword_t>(core.regs.dewords.e), c); }

		/// Opcode: 0X7B, cycles: 4 ( LD A, E )
		static void
		operLD_EinA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0x4>(core.regs.afwords.a, core.regs.dewords.e, c); }

		/// Opcode: 0X44, cycles: 4 ( LD B, H )
		static void
		operLD_HinB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X4>(core.regs.bcwords.b, core.regs.hlwords.h, c); }

		/// Opcode: 0X4C, cycles: 4 ( LD C, H )
		static void
		operLD_HinC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X4>(core.regs.bcwords.b, core.regs.hlwords.h, c); }

		/// Opcode: 0X54, cycles: 4 ( LD D, H )
		static void
		operLD_HinD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X4>(core.regs.dewords.d, core.regs.hlwords.h, c); }

		/// Opcode: 0X5C, cycles: 4 ( LD E, H )
		static void
		operLD_HinE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X4>(core.regs.dewords.e, core.regs.hlwords.h, c); }

		/// Opcode: 0X64, cycles: 4 ( LD H, H )
		static void
		operLD_HinH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operInlNop(core, c); }

		/// Opcode: 0X6C, cycles: 4 ( LD L, H )
		static void
		operLD_HinL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X4>(core.regs.hlwords.l, core.regs.hlwords.h, c); }

		/// Opcode: 0X74, cycles: 8 ( LD (HL), H )
		static void
		operLD_HinPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X8>(core.regs.hl /* what is pointed by hl */, static_cast<udword_t>(core.regs.hlwords.h, c)); }

		/// Opcode: 0X7C,  cycles: 4 ( LD A, H )
		static void
		operLD_HinA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0x4>(core.regs.afwords.a, core.regs.hlwords.h, c); }

		/// Opcode: 0X45, cycles: 4 ( LD B, L )
		static void
		operLD_LinB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X4>(core.regs.bcwords.b, core.regs.hlwords.l, c); }

		/// Opcode: 0X4D, cycles: 4 ( LD C, L )
		static void
		operLD_LinC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X4>(core.regs.bcwords.b, core.regs.hlwords.l, c); }

		/// Opcode: 0X55, cycles: 4 ( LD D, L )
		static void
		operLD_LinD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X4>(core.regs.dewords.d, core.regs.hlwords.l, c); }

		/// Opcode: 0X5D, cycles: 4 ( LD E, L )
		static void
		operLD_LinE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X4>(core.regs.dewords.e, core.regs.hlwords.l, c); }

		/// Opcode: 0X65, cycles: 4 ( LD H, L )
		static void
		operLD_LinH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X4>(core.regs.hlwords.h, core.regs.hlwords.l, c); }

		/// Opcode: 0X6D, cycles: 4 ( LD L, L )
		static void
		operLD_LinL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operInlNop<0X4>(core, c); }

		/// Opcode: 0X75, cycles: 8 ( LD (HL), L )
		static void
		operLD_LinPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X8>(core.regs.hl /* what is pointed by hl */, static_cast<udword_t>(core.regs.hlwords.l), c); }

		/// Opcode: 0X7D, cycles: 4 ( LD A, L )
		static void
		operLD_LinA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0x4>(core.regs.afwords.a, core.regs.hlwords.l, c); }

		/// Opcode: 0X46, cycles: 8 ( LD B, (HL) )
		static void
		operLD_PAddrHLinB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X8>(core.regs.bcwords.b, static_cast<udword_t>(core.regs.hl /* The value poited by */), c); }

		/// Opcode: 0X4E, cycles: 8 ( LD C, (HL) )
		static void
		operLD_PAddrHLinC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X8>(core.regs.bcwords.b, static_cast<udword_t>(core.regs.hl /* The value poited by */), c); }

		/// Opcode: 0X56, cycles: 8 ( LD D, (HL) )
		static void
		operLD_PAddrHLinD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X8>(core.regs.dewords.d, static_cast<udword_t>(core.regs.hl /* The value poited by */), c); }

		/// Opcode: 0X5E, cycles: 8 ( LD E, (HL) )
		static void
		operLD_PAddrHLinE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X8>(core.regs.dewords.e, static_cast<udword_t>(core.regs.hl /* The value poited by */), c); }

		/// Opcode: 0X66, cycles: 8 ( LD H, (HL) )
		static void
		operLD_PAddrHLinH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X8>(core.regs.hlwords.h, static_cast<udword_t>(core.regs.hl/* The value poited by */), c); }

		/// Opcode: 0X6E, cycles: 8 ( LD L, (HL) )
		static void
		operLD_PAddrHLinL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0X8>(core.regs.hlwords.l, static_cast<udword_t>(core.regs.hl/* The value poited by */), c); }

		/// Opcode: ?, cycles: 8 ( LD (HL), (HL) ) // may be 0X76
		static void
		operLD_PAddrHLinPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operInlNop<0X8>(core, c); }

		/// Opcode: 0X7E, cycles: 8 ( LD A, (HL) )
		static void
		operLD_PAddrHLinA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0x4>(core.regs.afwords.a, static_cast<udword_t>(core.regs.hl /* The value poited by */), c); }

		/// Opcode: 0X47, cycles: 4 ( LD B, A )
		static void
		operLD_AinB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0x4>(core.regs.bcwords.b, core.regs.afwords.a, c); }

		/// Opcode: 0X4F, cycles: 4 ( LD C, A )
		static void
		operLD_AinC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0x4>(core.regs.bcwords.b, core.regs.afwords.a, c); }

		/// Opcode: 0X57, cycles: 4 ( LD D, A )
		static void
		operLD_AinD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0x4>(core.regs.dewords.d, core.regs.afwords.a, c); }

		/// Opcode: 0X5F, cycles: 4 ( LD E, A )
		static void
		operLD_AinE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0x4>(core.regs.dewords.e, core.regs.afwords.a, c); }

		/// Opcode: 0X67, cycles: 4 ( LD H, A )
		static void
		operLD_AinH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0x4>(core.regs.hlwords.h, core.regs.afwords.a, c); }

		/// Opcode: 0X6F, cycles: 4 ( LD L, A )
		static void
		operLD_AinL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0x4>(core.regs.hlwords.l, core.regs.afwords.a, c); }

		/// Opcode: 0X77, cycles: 8 ( LD (HL), A )
		static void
		operLD_AinPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseLD<0x8>(core.regs.hl /* what is pointed by hl */, static_cast<udword_t>(core.regs.afwords.a), c); }

		/// Opcode: 0X7F, cycles: 4 ( LD A, A )
		static void
		operLD_AinA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operInlNop(core, c); }

		//////////////////////////
		// 8 BIT ALU OPERATIONS //
		//////////////////////////

//
///TODO: All p & q have been set, find which one set to operNop
//

		template <typename Flags, typename T>
		static inline void
		handleFlagsOperAdd(Flags& flags, T value)
		{
			/** TODO: Flags affected:
			 	Z - Set if result is zero.
  				N - Reset.
  				H - Set if carry from bit 3.
  				C - Set if carry from bit 7.
			*/

			static_cast<void>(flags);
			static_cast<void>(value);
		}

		template <typename Flags, typename T>
		static inline void
		handleFlagsOperSub(Flags& flags, T value)
		{
			/** TODO: Flags affected:
			 	Z - Set if result is zero.
   				N - Reset.
   				H - Set if carry from bit 4.
   				C - Set if carry from bit 7.
			*/

			static_cast<void>(flags);
			static_cast<void>(value);
		}

		template <typename Flags, typename T>
		static inline void
		handleFlagsOperAnd(Flags& flags, T value)
		{
			/** TODO: Flags affected:
			 	Z - Set if result is zero.
  				N - Reset.
  				H - Set.
  				C - Reset
			*/

			static_cast<void>(flags);
			static_cast<void>(value);
		}

		template <typename Flags, typename T>
		static inline void
		handleFlagsOperXor(Flags& flags, T value)
		{
			/** TODO: Flags affected:
			 	Z - Set if result is zero.
  				N - Reset.
  				H - Reset.
  				C - Reset.
			*/

			static_cast<void>(flags);
			static_cast<void>(value);
		}

		template <typename Flags, typename T>
		static inline void
		handleFlagsOperOr(Flags& flags, T value)
		{
			/** TODO: Flags affected:
			 	Z - Set if result is zero.
  				N - Reset.
  				H - Reset.
  				C - Reset.
			*/

			static_cast<void>(flags);
			static_cast<void>(value);
		}

		template <typename Flags, typename T>
		static inline void
		handleFlagsOperCp(Flags& flags, T value)
		{
			/** TODO: Flags affected:
			 	Z - Set if result is zero. (Set if A = n.)
  				N - Set.
  				H - Set if no borrow from bit 4.
  				C - Set for no borrow. (Set if A < n.)
			*/

			static_cast<void>(flags);
			static_cast<void>(value);
		}

		/// Opcode: 0X80, cycles: 4 ( ADD A, B )
		static void
		operAdd_BtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseAdd<0X4>(core.regs.afwords.a, core.regs.bcwords.b, c);
			handleFlagsOperAdd(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0X88, cycles: 4 ( ADC A, B )
		static void
		operAddCarry_BtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseAddCarry<0X4>(core.regs.afwords.a, core.regs.bcwords.b, c, core.flags);
			handleFlagsOperAdd(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0X90, cycles: 4 ( SUB B )
		static void
		operSub_BtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseSub<0X4>(core.regs.afwords.a, core.regs.bcwords.b, c);
			handleFlagsOperSub(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0X98, cycles: 4 ( SBC A, B )
		static void
		operSubCarry_BtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseSubCarry<0X4>(core.regs.afwords.a, core.regs.bcwords.b, c, core.flags);
			handleFlagsOperSub(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0XA0, cycles: 4 ( AND B )
		static void
		operAnd_BtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseAnd<0X4>(core.regs.afwords.a, core.regs.bcwords.b, c);
			handleFlagsOperAnd(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0XA8, cycles: 4 ( XOR B )
		static void
		operXor_BtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseXor<0X4>(core.regs.afwords.a, core.regs.bcwords.b, c);
			handleFlagsOperXor(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0XB0, cycles: 4 ( OR B )
		static void
		operOr_BtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseOr<0X4>(core.regs.afwords.a, core.regs.bcwords.b, c);
			handleFlagsOperOr(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0XB8, cycles: 4 ( CP B )
		static void
		operCp_BtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseCp<0X4>(core.regs.afwords.a, core.regs.bcwords.b, c);
			handleFlagsOperCp(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0X81, cycles: 4 ( ADD A, C )
		static void
		operAdd_CtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseAdd<0X4>(core.regs.afwords.a, core.regs.bcwords.c, c);
			handleFlagsOperAdd(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0X89, cycles: 4 ( ADC A, C )
		static void
		operAddCarry_CtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseAddCarry<0X4>(core.regs.afwords.a, core.regs.bcwords.c, c, core.flags);
			handleFlagsOperAdd(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0X91, cycles: 4 ( SUB C )
		static void
		operSub_CtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseSub<0X4>(core.regs.afwords.a, core.regs.bcwords.c, c);
			handleFlagsOperSub(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0X99, cycles: 4 ( SBC A, C )
		static void
		operSubCarry_CtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseSubCarry<0X4>(core.regs.afwords.a, core.regs.bcwords.c, c, core.flags);
			handleFlagsOperSub(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0XA1, cycles: 4 ( AND C )
		static void
		operAnd_CtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseAnd<0X4>(core.regs.afwords.a, core.regs.bcwords.c, c);
			handleFlagsOperAnd(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0XA9, cycles: 4 ( XOR C )
		static void
		operXor_CtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseXor<0X4>(core.regs.afwords.a, core.regs.bcwords.c, c);
			handleFlagsOperXor(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0XB1, cycles: 4 ( OR C )
		static void
		operOr_CtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseOr<0X4>(core.regs.afwords.a, core.regs.bcwords.c, c);
			handleFlagsOperOr(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0XB9, cycles: 4 ( CP C )
		static void
		operCp_CtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseCp<0X4>(core.regs.afwords.a, core.regs.bcwords.c, c);
			handleFlagsOperCp(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0X82, cycles: 4 ( ADD A, D )
		static void
		operAdd_DtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseAdd<0X4>(core.regs.afwords.a, core.regs.dewords.d, c);
			handleFlagsOperAdd(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0X8A, cycles: 4 ( ADC A, D )
		static void
		operAddCarry_DtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseAddCarry<0X4>(core.regs.afwords.a, core.regs.dewords.d, c, core.flags);
			handleFlagsOperAdd(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0X92, cycles: 4 ( SUB D )
		static void
		operSub_DtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseSub<0X4>(core.regs.afwords.a, core.regs.dewords.d, c);
			handleFlagsOperSub(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0X9A, cycles: 4 ( SBC A, D )
		static void
		operSubCarry_DtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseSubCarry<0X4>(core.regs.afwords.a, core.regs.dewords.d, c, core.flags);
			handleFlagsOperSub(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0XA2, cycles: 4 ( AND D )
		static void
		operAnd_DtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseAnd<0X4>(core.regs.afwords.a, core.regs.dewords.d, c);
			handleFlagsOperAnd(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0XAA, cycles: 4 ( XOR D )
		static void
		operXor_DtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseXor<0X4>(core.regs.afwords.a, core.regs.dewords.d, c);
			handleFlagsOperXor(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0XB2, cycles: 4 ( OR D )
		static void
		operOr_DtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseOr<0X4>(core.regs.afwords.a, core.regs.dewords.d, c);
			handleFlagsOperOr(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0XBA, cycles: 4 ( CP D )
		static void
		operCp_DtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseCp<0X4>(core.regs.afwords.a, core.regs.dewords.d, c);
			handleFlagsOperCp(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0X83, cycles: 4 ( ADD A, E )
		static void
		operAdd_EtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseAdd<0X4>(core.regs.afwords.a, core.regs.dewords.e, c);
			handleFlagsOperAdd(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0X8B, cycles: 4 ( ADC A, E )
		static void
		operAddCarry_EtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseAddCarry<0X4>(core.regs.afwords.a, core.regs.dewords.e, c, core.flags);
			handleFlagsOperAdd(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0X93, cycles: 4 ( SUB E )
		static void
		operSub_EtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseSub<0X4>(core.regs.afwords.a, core.regs.dewords.e, c);
			handleFlagsOperSub(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0X9B, cycles: 4 ( SBC A, E )
		static void
		operSubCarry_EtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseSubCarry<0X4>(core.regs.afwords.a, core.regs.dewords.e, c, core.flags);
			handleFlagsOperSub(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0XA3, cycles: 4 ( AND E )
		static void
		operAnd_EtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseAnd<0X4>(core.regs.afwords.a, core.regs.dewords.e, c);
			handleFlagsOperAnd(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0XAB, cycles: 4 ( XOR E )
		static void
		operXor_EtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseXor<0X4>(core.regs.afwords.a, core.regs.dewords.e, c);
			handleFlagsOperXor(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0XB3, cycles: 4 ( OR E )
		static void
		operOr_EtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseOr<0X4>(core.regs.afwords.a, core.regs.dewords.e, c);
			handleFlagsOperOr(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0XBB, cycles: 4 ( CP E )
		static void
		operCp_EtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseCp<0X4>(core.regs.afwords.a, core.regs.dewords.e, c);
			handleFlagsOperCp(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0X84, cycles: 4 ( ADD A, H )
		static void
		operAdd_HtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseAdd<0X4>(core.regs.afwords.a, core.regs.hlwords.h, c);
			handleFlagsOperAdd(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0X8C, cycles: 4 ( ADC A, H )
		static void
		operAddCarry_HtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseAddCarry<0X4>(core.regs.afwords.a, core.regs.hlwords.h, c, core.flags);
			handleFlagsOperAdd(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0X94, cycles: 4 ( SUB H )
		static void
		operSub_HtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseSub<0X4>(core.regs.afwords.a, core.regs.hlwords.h, c);
			handleFlagsOperSub(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0X9C, cycles: 4 ( SBC A, H )
		static void
		operSubCarry_HtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseSubCarry<0X4>(core.regs.afwords.a, core.regs.hlwords.h, c, core.flags);
			handleFlagsOperSub(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0XA4, cycles: 4 ( AND H )
		static void
		operAnd_HtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseAnd<0X4>(core.regs.afwords.a, core.regs.hlwords.h, c);
			handleFlagsOperAnd(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0XAC, cycles: 4 ( XOR H )
		static void
		operXor_HtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseXor<0X4>(core.regs.afwords.a, core.regs.hlwords.h, c);
			handleFlagsOperXor(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0XB4, cycles: 4 ( OR H )
		static void
		operOr_HtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseOr<0X4>(core.regs.afwords.a, core.regs.hlwords.h, c);
			handleFlagsOperOr(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0XBC, cycles: 4 ( CP H )
		static void
		operCp_HtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseCp<0X4>(core.regs.afwords.a, core.regs.hlwords.h, c);
			handleFlagsOperCp(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0X85, cycles: 4 ( ADD A, L )
		static void
		operAdd_LtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseAdd<0X4>(core.regs.afwords.a, core.regs.hlwords.l, c);
			handleFlagsOperAdd(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0X8D, cycles: 4 ( ADC A, L )
		static void
		operAddCarry_LtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseAddCarry<0X4>(core.regs.afwords.a, core.regs.hlwords.l, c, core.flags);
			handleFlagsOperAdd(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0X95, cycles: 4 ( SUB L )
		static void
		operSub_LtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseSub<0X4>(core.regs.afwords.a, core.regs.hlwords.l, c);
			handleFlagsOperSub(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0X9D, cycles: 4 ( SBC A, L )
		static void
		operSubCarry_LtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseSubCarry<0X4>(core.regs.afwords.a, core.regs.hlwords.l, c, core.flags);
			handleFlagsOperSub(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0XA5, cycles: 4 ( AND L )
		static void
		operAnd_LtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseAnd<0X4>(core.regs.afwords.a, core.regs.hlwords.l, c);
			handleFlagsOperAnd(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0XAD, cycles: 4 ( XOR L )
		static void
		operXor_LtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseXor<0X4>(core.regs.afwords.a, core.regs.hlwords.l, c);
			handleFlagsOperXor(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0XB5, cycles: 4 ( OR L )
		static void
		operOr_LtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseOr<0X4>(core.regs.afwords.a, core.regs.hlwords.l, c);
			handleFlagsOperOr(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0XBD, cycles: 4 ( CP L )
		static void
		operCp_LtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseCp<0X4>(core.regs.afwords.a, core.regs.hlwords.l, c);
			handleFlagsOperCp(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0X86, cycles: 8 ( ADD A, (HL) )
		static void
		operAdd_PAddrHLtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseAdd<0X8>(core.regs.afwords.a, core.regs.hl /* the value pointed by */, c);
			handleFlagsOperAdd(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0X8E, cycles: 8 ( ADC A, (HL) )
		static void
		operAddCarry_PAddrHLtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseAddCarry<0X8>(core.regs.afwords.a, core.regs.hl /* the value pointed by */, c, core.flags);
			handleFlagsOperAdd(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0X96, cycles: 8 ( SUB (HL) )
		static void
		operSub_PAddrHLtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseSub<0X8>(core.regs.afwords.a, core.regs.hl /* the value pointed by */, c);
			handleFlagsOperSub(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0X9E, cycles: 8 ( SBC A, (HL) )
		static void
		operSubCarry_PAddrHLtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseSubCarry<0X8>(core.regs.afwords.a, core.regs.hl /* the value pointed by */, c, core.flags);
			handleFlagsOperSub(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0XA6, cycles: 8( AND (HL) )
		static void
		operAnd_PAddrHLtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseAnd<0X8>(core.regs.afwords.a, core.regs.hl /* the value pointed by */, c);
			handleFlagsOperAnd(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0XAE, cycles: 8 ( XOR (HL) )
		static void
		operXor_PAddrHLtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseXor<0X8>(core.regs.afwords.a, core.regs.hl /* the value pointed by */, c);
			handleFlagsOperXor(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0XB6, cycles: 8 ( OR (HL) )
		static void
		operOr_PAddrHLtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseOr<0X8>(core.regs.afwords.a, core.regs.hl /* the value pointed by */, c);
			handleFlagsOperOr(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0XBE, cycles: 8 ( CP (HL) )
		static void
		operCp_PAddrHLtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseCp<0X8>(core.regs.afwords.a, core.regs.hl /* the value pointed by */, c);
			handleFlagsOperCp(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0X87, cycles: 4 ( ADD A, A )
		static void
		operAdd_AtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseAdd<0X4>(core.regs.afwords.a, core.regs.afwords.a, c);
			handleFlagsOperAdd(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0X8F, cycles: 4 ( ADC A, A )
		static void
		operAddCarry_AtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseAddCarry<0X4>(core.regs.afwords.a, core.regs.afwords.a, c, core.flags);
			handleFlagsOperAdd(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0X97, cycles: 4 ( SUB A )
		static void
		operSub_AtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseSub<0X4>(core.regs.afwords.a, core.regs.afwords.a, c);
			handleFlagsOperSub(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0X9F, cycles: 4 ( SBC A, A )
		static void
		operSubCarry_AtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseSubCarry<0X4>(core.regs.afwords.a, core.regs.afwords.a, c, core.flags);
			handleFlagsOperSub(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0XA7, cycles: 4 ( AND A )
		static void
		operAnd_AtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseAnd<0X4>(core.regs.afwords.a, core.regs.afwords.a, c);
			handleFlagsOperAnd(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0XAF, cycles: 4 ( XOR A )
		static void
		operXor_AtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseXor<0X4>(core.regs.afwords.a, core.regs.afwords.a, c);
			handleFlagsOperXor(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0XB7, cycles: 4 ( OR A )
		static void
		operOr_AtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseOr<0X4>(core.regs.afwords.a, core.regs.afwords.a, c);
			handleFlagsOperOr(core.flags, core.regs.afwords.a);
		}

		/// Opcode: 0XBF, cycles: 4 ( CP A )
		static void
		operCp_AtoA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{
			operBaseCp<0X4>(core.regs.afwords.a, core.regs.afwords.a, c);
			handleFlagsOperCp(core.flags, core.regs.afwords.a);
		}

		//////////////////////////////////
		// ROTATES & SHIFTS OPEREATIONS //
		//////////////////////////////////
//
///TODO: Document SLL
//
		// Opcode: 0XCB00, cycles: 8 ( RCL B )
		// Opcode: 0XCB08, cycles: 8 ( RRC B )
		// Opcode: 0XCB10, cycles: 8 ( RL B )
		// Opcode: 0XCB18, cycles: 8 ( RR B )
		// Opcode: 0XCB20, cycles: 8 ( SLA B )
		// Opcode: 0XCB28, cycles: 8 ( SRA B )
		// Opcode: ?, cycles: 8 ( SLL B )
		// Opcode: 0XCB38, cycles: 8 ( SRL B )
		// Opcode: 0XCB01, cycles: 8 ( RCL C )
		// Opcode: 0XCB09, cycles: 8 ( RRC C )
		// Opcode: 0XCB11, cycles: 8 ( RL C )
		// Opcode: 0XCB19, cycles: 8 ( RR C )
		// Opcode: 0XCB21, cycles: 8 ( SLA C )
		// Opcode: 0XCB29, cycles: 8 ( SRA C )
		// Opcode: ?, cycles: 8 ( SLL C )
		// Opcode: 0XCB39, cycles: 8 ( SRL C )
		// Opcode: 0XCB02, cycles: 8 ( RCL D )
		// Opcode: 0XCB0A, cycles: 8 ( RRC D )
		// Opcode: 0XCB12, cycles: 8 ( RL D )
		// Opcode: 0XCB1A, cycles: 8 ( RR D )
		// Opcode: 0XCB22, cycles: 8 ( SLA D )
		// Opcode: 0XCB2A, cycles: 8 ( SRA D )
		// Opcode: ?, cycles: 8 ( SLL D )
		// Opcode: 0XCB3A, cycles: 8 ( SRL D )
		// Opcode: 0XCB03, cycles: 8 ( RCL E )
		// Opcode: 0XCB0B, cycles: 8 ( RRC E )
		// Opcode: 0XCB13, cycles: 8 ( RL E )
		// Opcode: 0XCB1B, cycles: 8 ( RR E )
		// Opcode: 0XCB23, cycles: 8 ( SLA E )
		// Opcode: 0XCB2B, cycles: 8 ( SRA E )
		// Opcode: ?, cycles: 8 ( SLL E )
		// Opcode: 0XCB3B, cycles: 8 ( SRL E )
		// Opcode: 0XCB04, cycles: 8 ( RCL H )
		// Opcode: 0XCB0C, cycles: 8 ( RRC H )
		// Opcode: 0XCB14, cycles: 8 ( RL H )
		// Opcode: 0XCB1C, cycles: 8 ( RR H )
		// Opcode: 0XCB24, cycles: 8 ( SLA H )
		// Opcode: 0XCB2C, cycles: 8 ( SRA H )
		// Opcode: ?, cycles: 8 ( SLL H )
		// Opcode: 0XCB3C, cycles: 8 ( SRL H )
		// Opcode: 0XCB05, cycles: 8 ( RCL L )
		// Opcode: 0XCB0D, cycles: 8 ( RRC L )
		// Opcode: 0XCB15, cycles: 8 ( RL L )
		// Opcode: 0XCB1D, cycles: 8 ( RR L )
		// Opcode: 0XCB25, cycles: 8 ( SLA L )
		// Opcode: 0XCB2D, cycles: 8 ( SRA L )
		// Opcode: ?, cycles: 8 ( SLL L )
		// Opcode: 0XCB3D, cycles: 8 ( SRL L )
		// Opcode: 0XCB06, cycles: 16 ( RCL (HL) )
		// Opcode: 0XCB0E, cycles: 16 ( RRC (HL) )
		// Opcode: 0XCB16, cycles: 16 ( RL (HL) )
		// Opcode: 0XCB1E, cycles: 16 ( RR (HL) )
		// Opcode: 0XCB26, cycles: 16 ( SLA (HL) )
		// Opcode: 0XCB2E, cycles: 16 ( SRA (HL) )
		// Opcode: ?, cycles: 16 ( SLL (HL) )
		// Opcode: 0XCB3E, cycles: 16 ( SRL (HL) )
		// Opcode: 0XCB07, cycles: 8 ( RCL A )
		// Opcode: 0XCB0F, cycles: 8 ( RRC A )
		// Opcode: 0XCB17, cycles: 8 ( RL A )
		// Opcode: 0XCB1F, cycles: 8 ( RR A )
		// Opcode: 0XCB27, cycles: 8 ( SLA A )
		// Opcode: 0XCB2F, cycles: 8 ( SRA A )
		// Opcode: ?, cycles: 8 ( SLL A )
		// Opcode: 0XCB3F, cycles: 8 ( SRL A )









//
///TODO: All p & q have been set, find which one set to operNop
//
		// Opcode: 0XCB40, cycles: 8 ( BIT 0, B )
		static void
		operBit_Bit0inB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.bcwords.b, MASKBIT0, core.flags, c); }

		// Opcode: 0XCB48, cycles: 8 ( BIT 1, B )
		static void
		operBit_Bit1inB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.bcwords.b, MASKBIT1, core.flags, c); }

		// Opcode: 0XCB50, cycles: 8 ( BIT 2, B )
		static void
		operBit_Bit2inB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.bcwords.b, MASKBIT2, core.flags, c); }

		// Opcode: 0XCB58, cycles: 8 ( BIT 3, B )
		static void
		operBit_Bit3inB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.bcwords.b, MASKBIT3, core.flags, c); }

		// Opcode: 0XCB60, cycles: 8 ( BIT 4, B )
		static void
		operBit_Bit4inB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.bcwords.b, MASKBIT4, core.flags, c); }

		// Opcode: 0XCB68, cycles: 8 ( BIT 5, B )
		static void
		operBit_Bit5inB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.bcwords.b, MASKBIT5, core.flags, c); }

		// Opcode: 0XCB70, cycles: 8 ( BIT 6, B )
		static void
		operBit_Bit6inB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.bcwords.b, MASKBIT6, core.flags, c); }

		// Opcode: 0XCB78, cycles: 8 ( BIT 7, B )
		static void
		operBit_Bit7inB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.bcwords.b, MASKBIT7, core.flags, c); }

		// Opcode: 0XCB41, cycles: 8 ( BIT 0, C )
		static void
		operBit_Bit0inC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.bcwords.c, MASKBIT0, core.flags, c); }

		// Opcode: 0XCB49, cycles: 8 ( BIT 1, C )
		static void
		operBit_Bit1inC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.bcwords.c, MASKBIT1, core.flags, c); }

		// Opcode: 0XCB51, cycles: 8 ( BIT 2, C )
		static void
		operBit_Bit2inC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.bcwords.c, MASKBIT2, core.flags, c); }

		// Opcode: 0XCB59, cycles: 8 ( BIT 3, C )
		static void
		operBit_Bit3inC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.bcwords.c, MASKBIT3, core.flags, c); }

		// Opcode: 0XCB61, cycles: 8 ( BIT 4, C )
		static void
		operBit_Bit4inC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.bcwords.c, MASKBIT4, core.flags, c); }

		// Opcode: 0XCB69, cycles: 8 ( BIT 5, C )
		static void
		operBit_Bit5inC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.bcwords.c, MASKBIT5, core.flags, c); }

		// Opcode: 0XCB71, cycles: 8 ( BIT 6, C )
		static void
		operBit_Bit6inC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.bcwords.c, MASKBIT6, core.flags, c); }

		// Opcode: 0XCB79, cycles: 8 ( BIT 7, C )
		static void
		operBit_Bit7inC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.bcwords.c, MASKBIT7, core.flags, c); }

		// Opcode: 0XCB42, cycles: 8 ( BIT 0, D )
		static void
		operBit_Bit0inD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.dewords.d, MASKBIT0, core.flags, c); }

		// Opcode: 0XCB4A, cycles: 8 ( BIT 1, D )
		static void
		operBit_Bit1inD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.dewords.d, MASKBIT1, core.flags, c); }

		// Opcode: 0XCB52, cycles: 8 ( BIT 2, D )
		static void
		operBit_Bit2inD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.dewords.d, MASKBIT2, core.flags, c); }

		// Opcode: 0XCB5A, cycles: 8 ( BIT 3, D )
		static void
		operBit_Bit3inD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.dewords.d, MASKBIT3, core.flags, c); }

		// Opcode: 0XCB62, cycles: 8 ( BIT 4, D )
		static void
		operBit_Bit4inD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.dewords.d, MASKBIT4, core.flags, c); }

		// Opcode: 0XCB6A, cycles: 8 ( BIT 5, D )
		static void
		operBit_Bit5inD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.dewords.d, MASKBIT5, core.flags, c); }

		// Opcode: 0XCB72, cycles: 8 ( BIT 6, D )
		static void
		operBit_Bit6inD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.dewords.d, MASKBIT6, core.flags, c); }

		// Opcode: 0XCB7A, cycles: 8 ( BIT 7, D )
		static void
		operBit_Bit7inD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.dewords.d, MASKBIT7, core.flags, c); }

		// Opcode: 0XCB43, cycles: 8 ( BIT 0, E )
		static void
		operBit_Bit0inE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.dewords.e, MASKBIT0, core.flags, c); }

		// Opcode: 0XCB4B, cycles: 8 ( BIT 1, E )
		static void
		operBit_Bit1inE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.dewords.e, MASKBIT1, core.flags, c); }

		// Opcode: 0XCB53, cycles: 8 ( BIT 2, E )
		static void
		operBit_Bit2inE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.dewords.e, MASKBIT2, core.flags, c); }

		// Opcode: 0XCB5B, cycles: 8 ( BIT 3, E )
		static void
		operBit_Bit3inE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.dewords.e, MASKBIT3, core.flags, c); }

		// Opcode: 0XCB63, cycles: 8 ( BIT 4, E )
		static void
		operBit_Bit4inE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.dewords.e, MASKBIT4, core.flags, c); }

		// Opcode: 0XCB6B, cycles: 8 ( BIT 5, E )
		static void
		operBit_Bit5inE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.dewords.e, MASKBIT5, core.flags, c); }

		// Opcode: 0XCB73, cycles: 8 ( BIT 6, E )
		static void
		operBit_Bit6inE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.dewords.e, MASKBIT6, core.flags, c); }

		// Opcode: 0XCB7B, cycles: 8 ( BIT 7, E )
		static void
		operBit_Bit7inE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.dewords.e, MASKBIT7, core.flags, c); }

		// Opcode: 0XCB44, cycles: 8 ( BIT 0, H )
		static void
		operBit_Bit0inH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.hlwords.h, MASKBIT0, core.flags, c); }

		// Opcode: 0XCB4C, cycles: 8 ( BIT 1, H )
		static void
		operBit_Bit1inH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.hlwords.h, MASKBIT1, core.flags, c); }

		// Opcode: 0XCB54, cycles: 8 ( BIT 2, H )
		static void
		operBit_Bit2inH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.hlwords.h, MASKBIT2, core.flags, c); }

		// Opcode: 0XCB5C, cycles: 8 ( BIT 3, H )
		static void
		operBit_Bit3inH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.hlwords.h, MASKBIT3, core.flags, c); }

		// Opcode: 0XCB64, cycles: 8 ( BIT 4, H )
		static void
		operBit_Bit4inH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.hlwords.h, MASKBIT4, core.flags, c); }

		// Opcode: 0XCB6C, cycles: 8 ( BIT 5, H )
		static void
		operBit_Bit5inH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.hlwords.h, MASKBIT5, core.flags, c); }

		// Opcode: 0XCB74, cycles: 8 ( BIT 6, H )
		static void
		operBit_Bit6inH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.hlwords.h, MASKBIT6, core.flags, c); }

		// Opcode: 0XCB7C, cycles: 8 ( BIT 7, H )
		static void
		operBit_Bit7inH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.hlwords.h, MASKBIT7, core.flags, c); }

		// Opcode: 0XCB45, cycles: 8 ( BIT 0, L )
		static void
		operBit_Bit0inL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.hlwords.l, MASKBIT0, core.flags, c); }

		// Opcode: 0XCB4D, cycles: 8 ( BIT 1, L )
		static void
		operBit_Bit1inL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.hlwords.l, MASKBIT1, core.flags, c); }

		// Opcode: 0XCB55, cycles: 8 ( BIT 2, L )
		static void
		operBit_Bit2inL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.hlwords.l, MASKBIT2, core.flags, c); }

		// Opcode: 0XCB5D, cycles: 8 ( BIT 3, L )
		static void
		operBit_Bit3inL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.hlwords.l, MASKBIT3, core.flags, c); }

		// Opcode: 0XCB65, cycles: 8 ( BIT 4, L )
		static void
		operBit_Bit4inL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.hlwords.l, MASKBIT4, core.flags, c); }

		// Opcode: 0XCB6D, cycles: 8 ( BIT 5, L )
		static void
		operBit_Bit5inL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.hlwords.l, MASKBIT5, core.flags, c); }

		// Opcode: 0XCB75, cycles: 8 ( BIT 6, L )
		static void
		operBit_Bit6inL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.hlwords.l, MASKBIT6, core.flags, c); }

		// Opcode: 0XCB7D, cycles: 8 ( BIT 7, L )
		static void
		operBit_Bit7inL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.hlwords.l, MASKBIT7, core.flags, c); }

		// Opcode: 0XCB46, cycles: 16 ( BIT 0, (HL) )
		static void
		operBit_Bit0inPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0X10>(core.regs.hl /* the value pointed by */, MASKBIT0, core.flags, c); }

		// Opcode: 0XCB4E, cycles: 16 ( BIT 1, (HL) )
		static void
		operBit_Bit1inPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0X10>(core.regs.hl /* the value pointed by */, MASKBIT1, core.flags, c); }

		// Opcode: 0XCB56, cycles: 16 ( BIT 2, (HL) )
		static void
		operBit_Bit2inPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0X10>(core.regs.hl /* the value pointed by */, MASKBIT2, core.flags, c); }

		// Opcode: 0XCB5E, cycles: 16 ( BIT 3, (HL) )
		static void
		operBit_Bit3inPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0X10>(core.regs.hl /* the value pointed by */, MASKBIT3, core.flags, c); }

		// Opcode: 0XCB66, cycles: 16 ( BIT 4, (HL) )
		static void
		operBit_Bit4inPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0X10>(core.regs.hl /* the value pointed by */, MASKBIT4, core.flags, c); }

		// Opcode: 0XCB6E, cycles: 16 ( BIT 5, (HL) )
		static void
		operBit_Bit5inPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0X10>(core.regs.hl /* the value pointed by */, MASKBIT5, core.flags, c); }

		// Opcode: 0XCB76, cycles: 16 ( BIT 6, (HL) )
		static void
		operBit_Bit6inPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0X10>(core.regs.hl /* the value pointed by */, MASKBIT6, core.flags, c); }

		// Opcode: 0XCB7E, cycles: 16 ( BIT 7, (HL) )
		static void
		operBit_Bit7inPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0X10>(core.regs.hl /* the value pointed by */, MASKBIT7, core.flags, c); }

		// Opcode: 0XCB47, cycles: 8 ( BIT 0, A )
		static void
		operBit_Bit0inA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.afwords.a, MASKBIT0, core.flags, c); }

		// Opcode: 0XCB4F, cycles: 8 ( BIT 1, A )
		static void
		operBit_Bit1inA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.afwords.a, MASKBIT1, core.flags, c); }

		// Opcode: 0XCB57, cycles: 8 ( BIT 2, A )
		static void
		operBit_Bit2inA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.afwords.a, MASKBIT2, core.flags, c); }

		// Opcode: 0XCB5F, cycles: 8 ( BIT 3, A )
		static void
		operBit_Bit3inA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.afwords.a, MASKBIT3, core.flags, c); }

		// Opcode: 0XCB67, cycles: 8 ( BIT 4, A )
		static void
		operBit_Bit4inA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.afwords.a, MASKBIT4, core.flags, c); }

		// Opcode: 0XCB6F, cycles: 8 ( BIT 5, A )
		static void
		operBit_Bit5inA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.afwords.a, MASKBIT5, core.flags, c); }

		// Opcode: 0XCB77, cycles: 8 ( BIT 6, A )
		static void
		operBit_Bit6inA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.afwords.a, MASKBIT6, core.flags, c); }

		// Opcode: 0XCB7F, cycles: 8 ( BIT 7, A )
		static void
		operBit_Bit7inA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseBit<0x8>(core.regs.afwords.a, MASKBIT7, core.flags, c); }

//
///TODO: All p & q have been set, find which one set to operNop
//

		// Opcode: 0XCB80, cycles: 8 ( RES 0, B )
		static void
		operRes_Bit0inB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.bcwords.b, MASKBIT0, c); }

		// Opcode: 0XCB88, cycles: 8 ( RES 1, B )
		static void
		operRes_Bit1inB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.bcwords.b, MASKBIT1, c); }

		// Opcode: 0XCB90, cycles: 8 ( RES 2, B )
		static void
		operRes_Bit2inB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.bcwords.b, MASKBIT2, c); }

		// Opcode: 0XCB98, cycles: 8 ( RES 3, B )
		static void
		operRes_Bit3inB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.bcwords.b, MASKBIT3, c); }

		// Opcode: 0XCBA0, cycles: 8 ( RES 4, B )
		static void
		operRes_Bit4inB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.bcwords.b, MASKBIT4, c); }

		// Opcode: 0XCBA8, cycles: 8 ( RES 5, B )
		static void
		operRes_Bit5inB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.bcwords.b, MASKBIT5, c); }

		// Opcode: 0XCBB0, cycles: 8 ( RES 6, B )
		static void
		operRes_Bit6inB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.bcwords.b, MASKBIT6, c); }

		// Opcode: 0XCBB8, cycles: 8 ( RES 7, B )
		static void
		operRes_Bit7inB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.bcwords.b, MASKBIT7, c); }

		// Opcode: 0XCB81, cycles: 8 ( RES 0, C )
		static void
		operRes_Bit0inC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.bcwords.c, MASKBIT0, c); }

		// Opcode: 0XCB89, cycles: 8 ( RES 1, C )
		static void
		operRes_Bit1inC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.bcwords.c, MASKBIT1, c); }

		// Opcode: 0XCB91, cycles: 8 ( RES 2, C )
		static void
		operRes_Bit2inC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.bcwords.c, MASKBIT2, c); }

		// Opcode: 0XCB99, cycles: 8 ( RES 3, C )
		static void
		operRes_Bit3inC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.bcwords.c, MASKBIT3, c); }

		// Opcode: 0XCBA1, cycles: 8 ( RES 4, C )
		static void
		operRes_Bit4inC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.bcwords.c, MASKBIT4, c); }

		// Opcode: 0XCBA9, cycles: 8 ( RES 5, C )
		static void
		operRes_Bit5inC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.bcwords.c, MASKBIT5, c); }

		// Opcode: 0XCBB1, cycles: 8 ( RES 6, C )
		static void
		operRes_Bit6inC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.bcwords.c, MASKBIT6, c); }

		// Opcode: 0XCBB9, cycles: 8 ( RES 7, C )
		static void
		operRes_Bit7inC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.bcwords.c, MASKBIT7, c); }

		// Opcode: 0ZCB82, cycles: 8 ( RES 0, D )
		static void
		operRes_Bit0inD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.dewords.d, MASKBIT0, c); }

		// Opcode: 0XCB8A, cycles: 8 ( RES 1, D )
		static void
		operRes_Bit1inD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.dewords.d, MASKBIT1, c); }

		// Opcode: 0XCB92, cycles: 8 ( RES 2, D )
		static void
		operRes_Bit2inD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.dewords.d, MASKBIT2, c); }

		// Opcode: 0XCB9A, cycles: 8 ( RES 3, D )
		static void
		operRes_Bit3inD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.dewords.d, MASKBIT3, c); }

		// Opcode: OXCBA2, cycles: 8 ( RES 4, D )
		static void
		operRes_Bit4inD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.dewords.d, MASKBIT4, c); }

		// Opcode: 0XCBAA, cycles: 8 ( RES 5, D )
		static void
		operRes_Bit5inD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.dewords.d, MASKBIT5, c); }

		// Opcode: 0XCBB2, cycles: 8 ( RES 6, D )
		static void
		operRes_Bit6inD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.dewords.d, MASKBIT6, c); }

		// Opcode: 0XCBBA, cycles: 8 ( RES 7, D )
		static void
		operRes_Bit7inD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.dewords.d, MASKBIT7, c); }

		// Opcode: 0XCB83, cycles: 8 ( RES 0, E )
		static void
		operRes_Bit0inE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.dewords.e, MASKBIT0, c); }

		// Opcode: 0XCB8B, cycles: 8 ( RES 1, E )
		static void
		operRes_Bit1inE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.dewords.e, MASKBIT1, c); }

		// Opcode: 0XCB93, cycles: 8 ( RES 2, E )
		static void
		operRes_Bit2inE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.dewords.e, MASKBIT2, c); }

		// Opcode: 0XCB9B, cycles: 8 ( RES 3, E )
		static void
		operRes_Bit3inE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.dewords.e, MASKBIT3, c); }

		// Opcode: 0XCBA3, cycles: 8 ( RES 4, E )
		static void
		operRes_Bit4inE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.dewords.e, MASKBIT4, c); }

		// Opcode: 0XCBAB, cycles: 8 ( RES 5, E )
		static void
		operRes_Bit5inE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.dewords.e, MASKBIT5, c); }

		// Opcode: 0XCBB3, cycles: 8 ( RES 6, E )
		static void
		operRes_Bit6inE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.dewords.e, MASKBIT6, c); }

		// Opcode: 0XCBBB, cycles: 8 ( RES 7, E )
		static void
		operRes_Bit7inE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.dewords.e, MASKBIT7, c); }

		// Opcode: 0XCB84, cycles: 8 ( RES 0, H )
		static void
		operRes_Bit0inH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.hlwords.h, MASKBIT0, c); }

		// Opcode: 0XCB8C, cycles: 8 ( RES 1, H )
		static void
		operRes_Bit1inH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.hlwords.h, MASKBIT1, c); }

		// Opcode: 0XCB94, cycles: 8 ( RES 2, H )
		static void
		operRes_Bit2inH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.hlwords.h, MASKBIT2, c); }

		// Opcode: 0XCB9C, cycles: 8 ( RES 3, H )
		static void
		operRes_Bit3inH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.hlwords.h, MASKBIT3, c); }

		// Opcode: 0XCBA4, cycles: 8 ( RES 4, H )
		static void
		operRes_Bit4inH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.hlwords.h, MASKBIT4, c); }

		// Opcode: 0XCBAC, cycles: 8 ( RES 5, H )
		static void
		operRes_Bit5inH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.hlwords.h, MASKBIT5, c); }

		// Opcode: 0XCBB4, cycles: 8 ( RES 6, H )
		static void
		operRes_Bit6inH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.hlwords.h, MASKBIT6, c); }

		// Opcode: 0XCBBC, cycles: 8 ( RES 7, H )
		static void
		operRes_Bit7inH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.hlwords.h, MASKBIT7, c); }

		// Opcode: 0XCB85, cycles: 8 ( RES 0, L )
		static void
		operRes_Bit0inL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.hlwords.l, MASKBIT0, c); }

		// Opcode: 0XCB8D, cycles: 8 ( RES 1, L )
		static void
		operRes_Bit1inL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.hlwords.l, MASKBIT1, c); }

		// Opcode: 0XCB95, cycles: 8 ( RES 2, L )
		static void
		operRes_Bit2inL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.hlwords.l, MASKBIT2, c); }

		// Opcode: 0XCB9D, cycles: 8 ( RES 3, L )
		static void
		operRes_Bit3inL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.hlwords.l, MASKBIT3, c); }

		// Opcode: 0XCBA5, cycles: 8 ( RES 4, L )
		static void
		operRes_Bit4inL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.hlwords.l, MASKBIT4, c); }

		// Opcode: 0XCBAD, cycles: 8 ( RES 5, L )
		static void
		operRes_Bit5inL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.hlwords.l, MASKBIT5, c); }

		// Opcode: 0XCBB5, cycles: 8 ( RES 6, L )
		static void
		operRes_Bit6inL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.hlwords.l, MASKBIT6, c); }

		// Opcode: 0XCBBD, cycles: 8 ( RES 7, L )
		static void
		operRes_Bit7inL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.hlwords.l, MASKBIT7, c); }

		// Opcode: 0XCB86, cycles: 8 ( RES 0, (HL) )
		static void
		operRes_Bit0inPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x10>(core.regs.hl /* The value pointed by */, MASKBIT0, c); }

		// Opcode: 0XCB8E, cycles: 8 ( RES 1, (HL) )
		static void
		operRes_Bit1inPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x10>(core.regs.hl /* The value pointed by */, MASKBIT1, c); }

		// Opcode: 0XCB96, cycles: 8 ( RES 2, (HL) )
		static void
		operRes_Bit2inPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x10>(core.regs.hl /* The value pointed by */, MASKBIT2, c); }

		// Opcode: 0XCB9E, cycles: 8 ( RES 3, (HL) )
		static void
		operRes_Bit3inPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x10>(core.regs.hl /* The value pointed by */, MASKBIT3, c); }

		// Opcode: 0XCBA6, cycles: 8 ( RES 4, (HL) )
		static void
		operRes_Bit4inPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x10>(core.regs.hl /* The value pointed by */, MASKBIT4, c); }

		// Opcode: 0XCBAE, cycles: 8 ( RES 5, (HL) )
		static void
		operRes_Bit5inPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x10>(core.regs.hl /* The value pointed by */, MASKBIT5, c); }

		// Opcode: 0XCBB6, cycles: 8 ( RES 6, (HL) )
		static void
		operRes_Bit6inPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x10>(core.regs.hl /* The value pointed by */, MASKBIT6, c); }

		// Opcode: 0XCBBE, cycles: 8 ( RES 7, (HL) )
		static void
		operRes_Bit7inPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x10>(core.regs.hl /* The value pointed by */, MASKBIT7, c); }

		// Opcode: 0XCB87, cycles: 8 ( RES 0, A )
		static void
		operRes_Bit0inA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.afwords.a, MASKBIT0, c); }

		// Opcode: 0XCB8F, cycles: 8 ( RES 1, A )
		static void
		operRes_Bit1inA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.afwords.a, MASKBIT1, c); }

		// Opcode: 0XCB97, cycles: 8 ( RES 2, A )
		static void
		operRes_Bit2inA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.afwords.a, MASKBIT2, c); }

		// Opcode: 0XCB9F, cycles: 8 ( RES 3, A )
		static void
		operRes_Bit3inA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.afwords.a, MASKBIT3, c); }

		// Opcode: 0XCBA7, cycles: 8 ( RES 4, A )
		static void
		operRes_Bit4inA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.afwords.a, MASKBIT4, c); }

		// Opcode: 0XCBAF, cycles: 8 ( RES 5, A )
		static void
		operRes_Bit5inA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.afwords.a, MASKBIT5, c); }

		// Opcode: 0XCBB7, cycles: 8 ( RES 6, A )
		static void
		operRes_Bit6inA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.afwords.a, MASKBIT6, c); }

		// Opcode: 0XCBBF, cycles: 8 ( RES 7, A )
		static void
		operRes_Bit7inA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseRes<0x8>(core.regs.afwords.a, MASKBIT7, c); }

//
///TODO: All p & q have been set, find which one set to operNop
//

		// Opcode: 0XCBC0, cycles: 8 ( SET 0, B )
		static void
		operSet_Bit0inB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.bcwords.b, MASKBIT0, c); }

		// Opcode: OXCBC8, cycles: 8 ( SET 1, B )
		static void
		operSet_Bit1inB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.bcwords.b, MASKBIT1, c); }

		// Opcode: 0XCBD0, cycles: 8 ( SET 2, B )
		static void
		operSet_Bit2inB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.bcwords.b, MASKBIT2, c); }

		// Opcode: 0XCBD8, cycles: 8 ( SET 3, B )
		static void
		operSet_Bit3inB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.bcwords.b, MASKBIT3, c); }

		// Opcode: 0XCBE0, cycles: 8 ( SET 4, B )
		static void
		operSet_Bit4inB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.bcwords.b, MASKBIT4, c); }

		// Opcode: 0XCBE8, cycles: 8 ( SET 5, B )
		static void
		operSet_Bit5inB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.bcwords.b, MASKBIT5, c); }

		// Opcode: 0XCBF0, cycles: 8 ( SET 6, B )
		static void
		operSet_Bit6inB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.bcwords.b, MASKBIT6, c); }

		// Opcode: 0XCBF8, cycles: 8 ( SET 7, B )
		static void
		operSet_Bit7inB(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.bcwords.b, MASKBIT7, c); }

		// Opcode: 0XCBC1, cycles: 8 ( SET 0, C )
		static void
		operSet_Bit0inC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.bcwords.c, MASKBIT0, c); }

		// Opcode: 0XCBC9, cycles: 8 ( SET 1, C )
		static void
		operSet_Bit1inC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.bcwords.c, MASKBIT1, c); }

		// Opcode: 0XCBD1, cycles: 8 ( SET 2, C )
		static void
		operSet_Bit2inC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.bcwords.c, MASKBIT2, c); }

		// Opcode: 0XCBD9, cycles: 8 ( SET 3, C )
		static void
		operSet_Bit3inC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.bcwords.c, MASKBIT3, c); }

		// Opcode: 0XCBE1, cycles: 8 ( SET 4, C )
		static void
		operSet_Bit4inC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.bcwords.c, MASKBIT4, c); }

		// Opcode: 0XCBE9, cycles: 8 ( SET 5, C )
		static void
		operSet_Bit5inC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.bcwords.c, MASKBIT5, c); }

		// Opcode: 0XCBF1, cycles: 8 ( SET 6, C )
		static void
		operSet_Bit6inC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.bcwords.c, MASKBIT6, c); }

		// Opcode: 0XCBF9, cycles: 8 ( SET 7, C )
		static void
		operSet_Bit7inC(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.bcwords.c, MASKBIT7, c); }

		// Opcode: 0XCBC2, cycles: 8 ( SET 0, D )
		static void
		operSet_Bit0inD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.dewords.d, MASKBIT0, c); }

		// Opcode: 0CCBCA, cycles: 8 ( SET 1, D )
		static void
		operSet_Bit1inD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.dewords.d, MASKBIT1, c); }

		// Opcode: 0XCBD2, cycles: 8 ( SET 2, D )
		static void
		operSet_Bit2inD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.dewords.d, MASKBIT2, c); }

		// Opcode: 0XCBDA, cycles: 8 ( SET 3, D )
		static void
		operSet_Bit3inD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.dewords.d, MASKBIT3, c); }

		// Opcode: 0XCBE2, cycles: 8 ( SET 4, D )
		static void
		operSet_Bit4inD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.dewords.d, MASKBIT4, c); }

		// Opcode: 0XCBEA, cycles: 8 ( SET 5, D )
		static void
		operSet_Bit5inD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.dewords.d, MASKBIT5, c); }

		// Opcode: 0XCBF2, cycles: 8 ( SET 6, D )
		static void
		operSet_Bit6inD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.dewords.d, MASKBIT6, c); }

		// Opcode: 0XCBFA, cycles: 8 ( SET 7, D )
		static void
		operSet_Bit7inD(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.dewords.d, MASKBIT7, c); }

		// Opcode: 0XCBC3, cycles: 8 ( SET 0, E )
		static void
		operSet_Bit0inE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.dewords.e, MASKBIT0, c); }

		// Opcode: 0XCBCB, cycles: 8 ( SET 1, E )
		static void
		operSet_Bit1inE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.dewords.e, MASKBIT1, c); }

		// Opcode: 0XCBD3, cycles: 8 ( SET 2, E )
		static void
		operSet_Bit2inE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.dewords.e, MASKBIT2, c); }

		// Opcode: 0XCBDB, cycles: 8 ( SET 3, E )
		static void
		operSet_Bit3inE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.dewords.e, MASKBIT3, c); }

		// Opcode: 0XCBE3, cycles: 8 ( SET 4, E )
		static void
		operSet_Bit4inE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.dewords.e, MASKBIT4, c); }

		// Opcode: 0XCBEB, cycles: 8 ( SET 5, E )
		static void
		operSet_Bit5inE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.dewords.e, MASKBIT5, c); }

		// Opcode: 0XCBF3, cycles: 8 ( SET 6, E )
		static void
		operSet_Bit6inE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.dewords.e, MASKBIT6, c); }

		// Opcode: 0XCBFB, cycles: 8 ( SET 7, E )
		static void
		operSet_Bit7inE(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.dewords.e, MASKBIT7, c); }

		// Opcode: 0XCBC4, cycles: 8 ( SET 0, H )
		static void
		operSet_Bit0inH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.hlwords.h, MASKBIT0, c); }

		// Opcode: 0XCBCC, cycles: 8 ( SET 1, H )
		static void
		operSet_Bit1inH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.hlwords.h, MASKBIT1, c); }

		// Opcode: 0XCBD4, cycles: 8 ( SET 2, H )
		static void
		operSet_Bit2inH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.hlwords.h, MASKBIT2, c); }

		// Opcode: 0XCBDC, cycles: 8 ( SET 3, H )
		static void
		operSet_Bit3inH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.hlwords.h, MASKBIT3, c); }

		// Opcode: 0XCBE4, cycles: 8 ( SET 4, H )
		static void
		operSet_Bit4inH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.hlwords.h, MASKBIT4, c); }

		// Opcode: 0XCBEC, cycles: 8 ( SET 5, H )
		static void
		operSet_Bit5inH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.hlwords.h, MASKBIT5, c); }

		// Opcode: 0XCBF4, cycles: 8 ( SET 6, H )
		static void
		operSet_Bit6inH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.hlwords.h, MASKBIT6, c); }

		// Opcode: 0XCBFC, cycles: 8 ( SET 7, H )
		static void
		operSet_Bit7inH(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.hlwords.h, MASKBIT7, c); }

		// Opcode: 0XCBC5, cycles: 8 ( SET 0, L )
		static void
		operSet_Bit0inL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.hlwords.l, MASKBIT0, c); }

		// Opcode: 0XCBCD, cycles: 8 ( SET 1, L )
		static void
		operSet_Bit1inL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.hlwords.l, MASKBIT1, c); }

		// Opcode: 0XCBD5, cycles: 8 ( SET 2, L )
		static void
		operSet_Bit2inL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.hlwords.l, MASKBIT3, c); }

		// Opcode: 0XCBDD, cycles: 8 ( SET 3, L )
		static void
		operSet_Bit3inL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.hlwords.l, MASKBIT3, c); }

		// Opcode: 0XCBE5, cycles: 8 ( SET 4, L )
		static void
		operSet_Bit4inL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.hlwords.l, MASKBIT4, c); }

		// Opcode: 0XCBED, cycles: 8 ( SET 5, L )
		static void
		operSet_Bit5inL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.hlwords.l, MASKBIT5, c); }

		// Opcode: 0XCBF5, cycles: 8 ( SET 6, L )
		static void
		operSet_Bit6inL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.hlwords.l, MASKBIT6, c); }

		// Opcode: OXCBFD, cycles: 8 ( SET 7, L )
		static void
		operSet_Bit7inL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.hlwords.l, MASKBIT7, c); }

		// Opcode: 0XCBC6, cycles: 16 ( SET 0, (HL) )
		static void
		operSet_Bit0inPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x10>(core.regs.hl /* The value pointed by */, MASKBIT0, c); }

		// Opcode: 0XCBCE, cycles: 16 ( SET 1, (HL) )
		static void
		operSet_Bit1inPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x10>(core.regs.hl /* The value pointed by */, MASKBIT1, c); }

		// Opcode: 0XCBD6, cycles: 16 ( SET 2, (HL) )
		static void
		operSet_Bit2inPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x10>(core.regs.hl /* The value pointed by */, MASKBIT2, c); }

		// Opcode: 0XCBDE, cycles: 16 ( SET 3, (HL) )
		static void
		operSet_Bit3inPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x10>(core.regs.hl /* The value pointed by */, MASKBIT3, c); }

		// Opcode: 0XCBE6, cycles: 16 ( SET 4, (HL) )
		static void
		operSet_Bit4inPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x10>(core.regs.hl /* The value pointed by */, MASKBIT4, c); }

		// Opcode: 0XCBEE, cycles: 16 ( SET 5, (HL) )
		static void
		operSet_Bit5inPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x10>(core.regs.hl /* The value pointed by */, MASKBIT5, c); }

		// Opcode: 0XCBF6, cycles: 16 ( SET 6, (HL) )
		static void
		operSet_Bit6inPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x10>(core.regs.hl /* The value pointed by */, MASKBIT6, c); }

		// Opcode: 0XCBFE, cycles: 16 ( SET 7, (HL) )
		static void
		operSet_Bit7inPAddrHL(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x10>(core.regs.hl /* The value pointed by */, MASKBIT7, c); }

		// Opcode: 0XCBC7, cycles: 8 ( SET 0, A )
		static void
		operSet_Bit0inA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.afwords.a, MASKBIT0, c); }

		// Opcode: 0XCBCF, cycles: 8 ( SET 1, A )
		static void
		operSet_Bit1inA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.afwords.a, MASKBIT1, c); }

		// Opcode: 0XCBD7, cycles: 8 ( SET 2, A )
		static void
		operSet_Bit2inA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.afwords.a, MASKBIT2, c); }

		// Opcode: 0XCBDF, cycles: 8 ( SET 3, A )
		static void
		operSet_Bit3inA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.afwords.a, MASKBIT3, c); }

		// Opcode: 0XCBE7, cycles: 8 ( SET 4, A )
		static void
		operSet_Bit4inA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.afwords.a, MASKBIT4, c); }

		// Opcode: 0XCBEF, cycles: 8 ( SET 5, A )
		static void
		operSet_Bit5inA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.afwords.a, MASKBIT5, c); }

		// Opcode: 0XCBF7, cycles: 8 ( SET 6, A )
		static void
		operSet_Bit6inA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.afwords.a, MASKBIT6, c); }

		// Opcode: 0XCBFF, cycles: 8 ( SET 7, A )
		static void
		operSet_Bit7inA(const CPU<Memory, Z80Registers, uint8_t>& core, Chrono& c)
		{ operBaseSet<0x8>(core.regs.afwords.a, MASKBIT7, c); }







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
								/* P core.regs.bcwords.b= 1 */
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
								&operAdd_BtoA,
								/* P = 1 */
								&operAdd_BtoA,
								/* P = 2 */
								&operAdd_BtoA,
								/* P = 3 */
								&operAdd_BtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operAdd_BtoA,
								/* P = 1 */
								&operAdd_BtoA,
								/* P = 2 */
								&operAdd_BtoA,
								/* P = 3 */
								&operAdd_BtoA
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operAddCarry_BtoA,
								/* P = 1 */
								&operAddCarry_BtoA,
								/* P = 2 */
								&operAddCarry_BtoA,
								/* P = 3 */
								&operAddCarry_BtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operAddCarry_BtoA,
								/* P = 1 */
								&operAddCarry_BtoA,
								/* P = 2 */
								&operAddCarry_BtoA,
								/* P = 3 */
								&operAddCarry_BtoA
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSub_BtoA,
								/* P = 1 */
								&operSub_BtoA,
								/* P = 2 */
								&operSub_BtoA,
								/* P = 3 */
								&operSub_BtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSub_BtoA,
								/* P = 1 */
								&operSub_BtoA,
								/* P = 2 */
								&operSub_BtoA,
								/* P = 3 */
								&operSub_BtoA
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSubCarry_BtoA,
								/* P = 1 */
								&operSubCarry_BtoA,
								/* P = 2 */
								&operSubCarry_BtoA,
								/* P = 3 */
								&operSubCarry_BtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSubCarry_BtoA,
								/* P = 1 */
								&operSubCarry_BtoA,
								/* P = 2 */
								&operSubCarry_BtoA,
								/* P = 3 */
								&operSubCarry_BtoA
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operAnd_BtoA,
								/* P = 1 */
								&operAnd_BtoA,
								/* P = 2 */
								&operAnd_BtoA,
								/* P = 3 */
								&operAnd_BtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operAnd_BtoA,
								/* P = 1 */
								&operAnd_BtoA,
								/* P = 2 */
								&operAnd_BtoA,
								/* P = 3 */
								&operAnd_BtoA
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operXor_BtoA,
								/* P = 1 */
								&operXor_BtoA,
								/* P = 2 */
								&operXor_BtoA,
								/* P = 3 */
								&operXor_BtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operXor_BtoA,
								/* P = 1 */
								&operXor_BtoA,
								/* P = 2 */
								&operXor_BtoA,
								/* P = 3 */
								&operXor_BtoA
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operOr_BtoA,
								/* P = 1 */
								&operOr_BtoA,
								/* P = 2 */
								&operOr_BtoA,
								/* P = 3 */
								&operOr_BtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operOr_BtoA,
								/* P = 1 */
								&operOr_BtoA,
								/* P = 2 */
								&operOr_BtoA,
								/* P = 3 */
								&operOr_BtoA
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operCp_BtoA,
								/* P = 1 */
								&operCp_BtoA,
								/* P = 2 */
								&operCp_BtoA,
								/* P = 3 */
								&operCp_BtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operCp_BtoA,
								/* P = 1 */
								&operCp_BtoA,
								/* P = 2 */
								&operCp_BtoA,
								/* P = 3 */
								&operCp_BtoA
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
								&operAdd_CtoA,
								/* P = 1 */
								&operAdd_CtoA,
								/* P = 2 */
								&operAdd_CtoA,
								/* P = 3 */
								&operAdd_CtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operAdd_CtoA,
								/* P = 1 */
								&operAdd_CtoA,
								/* P = 2 */
								&operAdd_CtoA,
								/* P = 3 */
								&operAdd_CtoA
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operAddCarry_CtoA,
								/* P = 1 */
								&operAddCarry_CtoA,
								/* P = 2 */
								&operAddCarry_CtoA,
								/* P = 3 */
								&operAddCarry_CtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operAddCarry_CtoA,
								/* P = 1 */
								&operAddCarry_CtoA,
								/* P = 2 */
								&operAddCarry_CtoA,
								/* P = 3 */
								&operAddCarry_CtoA
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSub_CtoA,
								/* P = 1 */
								&operSub_CtoA,
								/* P = 2 */
								&operSub_CtoA,
								/* P = 3 */
								&operSub_CtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSub_CtoA,
								/* P = 1 */
								&operSub_CtoA,
								/* P = 2 */
								&operSub_CtoA,
								/* P = 3 */
								&operSub_CtoA
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSubCarry_CtoA,
								/* P = 1 */
								&operSubCarry_CtoA,
								/* P = 2 */
								&operSubCarry_CtoA,
								/* P = 3 */
								&operSubCarry_CtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSubCarry_CtoA,
								/* P = 1 */
								&operSubCarry_CtoA,
								/* P = 2 */
								&operSubCarry_CtoA,
								/* P = 3 */
								&operSubCarry_CtoA
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operAnd_CtoA,
								/* P = 1 */
								&operAnd_CtoA,
								/* P = 2 */
								&operAnd_CtoA,
								/* P = 3 */
								&operAnd_CtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operAnd_CtoA,
								/* P = 1 */
								&operAnd_CtoA,
								/* P = 2 */
								&operAnd_CtoA,
								/* P = 3 */
								&operAnd_CtoA
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operXor_CtoA,
								/* P = 1 */
								&operXor_CtoA,
								/* P = 2 */
								&operXor_CtoA,
								/* P = 3 */
								&operXor_CtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operXor_CtoA,
								/* P = 1 */
								&operXor_CtoA,
								/* P = 2 */
								&operXor_CtoA,
								/* P = 3 */
								&operXor_CtoA
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operOr_CtoA,
								/* P = 1 */
								&operOr_CtoA,
								/* P = 2 */
								&operOr_CtoA,
								/* P = 3 */
								&operOr_CtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operOr_CtoA,
								/* P = 1 */
								&operOr_CtoA,
								/* P = 2 */
								&operOr_CtoA,
								/* P = 3 */
								&operOr_CtoA
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operCp_CtoA,
								/* P = 1 */
								&operCp_CtoA,
								/* P = 2 */
								&operCp_CtoA,
								/* P = 3 */
								&operCp_CtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operCp_CtoA,
								/* P = 1 */
								&operCp_CtoA,
								/* P = 2 */
								&operCp_CtoA,
								/* P = 3 */
								&operCp_CtoA
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
								&operAdd_DtoA,
								/* P = 1 */
								&operAdd_DtoA,
								/* P = 2 */
								&operAdd_DtoA,
								/* P = 3 */
								&operAdd_DtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operAdd_DtoA,
								/* P = 1 */
								&operAdd_DtoA,
								/* P = 2 */
								&operAdd_DtoA,
								/* P = 3 */
								&operAdd_DtoA
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operAddCarry_DtoA,
								/* P = 1 */
								&operAddCarry_DtoA,
								/* P = 2 */
								&operAddCarry_DtoA,
								/* P = 3 */
								&operAddCarry_DtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operAddCarry_DtoA,
								/* P = 1 */
								&operAddCarry_DtoA,
								/* P = 2 */
								&operAddCarry_DtoA,
								/* P = 3 */
								&operAddCarry_DtoA
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSub_DtoA,
								/* P = 1 */
								&operSub_DtoA,
								/* P = 2 */
								&operSub_DtoA,
								/* P = 3 */
								&operSub_DtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSub_DtoA,
								/* P = 1 */
								&operSub_DtoA,
								/* P = 2 */
								&operSub_DtoA,
								/* P = 3 */
								&operSub_DtoA
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSubCarry_DtoA,
								/* P = 1 */
								&operSubCarry_DtoA,
								/* P = 2 */
								&operSubCarry_DtoA,
								/* P = 3 */
								&operSubCarry_DtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSubCarry_DtoA,
								/* P = 1 */
								&operSubCarry_DtoA,
								/* P = 2 */
								&operSubCarry_DtoA,
								/* P = 3 */
								&operSubCarry_DtoA
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operAnd_DtoA,
								/* P = 1 */
								&operAnd_DtoA,
								/* P = 2 */
								&operAnd_DtoA,
								/* P = 3 */
								&operAnd_DtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operAnd_DtoA,
								/* P = 1 */
								&operAnd_DtoA,
								/* P = 2 */
								&operAnd_DtoA,
								/* P = 3 */
								&operAnd_DtoA
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operXor_DtoA,
								/* P = 1 */
								&operXor_DtoA,
								/* P = 2 */
								&operXor_DtoA,
								/* P = 3 */
								&operXor_DtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operXor_DtoA,
								/* P = 1 */
								&operXor_DtoA,
								/* P = 2 */
								&operXor_DtoA,
								/* P = 3 */
								&operXor_DtoA
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operOr_DtoA,
								/* P = 1 */
								&operOr_DtoA,
								/* P = 2 */
								&operOr_DtoA,
								/* P = 3 */
								&operOr_DtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operOr_DtoA,
								/* P = 1 */
								&operOr_DtoA,
								/* P = 2 */
								&operOr_DtoA,
								/* P = 3 */
								&operOr_DtoA
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operCp_DtoA,
								/* P = 1 */
								&operCp_DtoA,
								/* P = 2 */
								&operCp_DtoA,
								/* P = 3 */
								&operCp_DtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operCp_DtoA,
								/* P = 1 */
								&operCp_DtoA,
								/* P = 2 */
								&operCp_DtoA,
								/* P = 3 */
								&operCp_DtoA
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
								&operAdd_EtoA,
								/* P = 1 */
								&operAdd_EtoA,
								/* P = 2 */
								&operAdd_EtoA,
								/* P = 3 */
								&operAdd_EtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operAdd_EtoA,
								/* P = 1 */
								&operAdd_EtoA,
								/* P = 2 */
								&operAdd_EtoA,
								/* P = 3 */
								&operAdd_EtoA
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operAddCarry_EtoA,
								/* P = 1 */
								&operAddCarry_EtoA,
								/* P = 2 */
								&operAddCarry_EtoA,
								/* P = 3 */
								&operAddCarry_EtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operAddCarry_EtoA,
								/* P = 1 */
								&operAddCarry_EtoA,
								/* P = 2 */
								&operAddCarry_EtoA,
								/* P = 3 */
								&operAddCarry_EtoA
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSub_EtoA,
								/* P = 1 */
								&operSub_EtoA,
								/* P = 2 */
								&operSub_EtoA,
								/* P = 3 */
								&operSub_EtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSub_EtoA,
								/* P = 1 */
								&operSub_EtoA,
								/* P = 2 */
								&operSub_EtoA,
								/* P = 3 */
								&operSub_EtoA
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSubCarry_EtoA,
								/* P = 1 */
								&operSubCarry_EtoA,
								/* P = 2 */
								&operSubCarry_EtoA,
								/* P = 3 */
								&operSubCarry_EtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSubCarry_EtoA,
								/* P = 1 */
								&operSubCarry_EtoA,
								/* P = 2 */
								&operSubCarry_EtoA,
								/* P = 3 */
								&operSubCarry_EtoA
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operAnd_EtoA,
								/* P = 1 */
								&operAnd_EtoA,
								/* P = 2 */
								&operAnd_EtoA,
								/* P = 3 */
								&operAnd_EtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operAnd_EtoA,
								/* P = 1 */
								&operAnd_EtoA,
								/* P = 2 */
								&operAnd_EtoA,
								/* P = 3 */
								&operAnd_EtoA
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operXor_EtoA,
								/* P = 1 */
								&operXor_EtoA,
								/* P = 2 */
								&operXor_EtoA,
								/* P = 3 */
								&operXor_EtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operXor_EtoA,
								/* P = 1 */
								&operXor_EtoA,
								/* P = 2 */
								&operXor_EtoA,
								/* P = 3 */
								&operXor_EtoA
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operOr_EtoA,
								/* P = 1 */
								&operOr_EtoA,
								/* P = 2 */
								&operOr_EtoA,
								/* P = 3 */
								&operOr_EtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operOr_EtoA,
								/* P = 1 */
								&operOr_EtoA,
								/* P = 2 */
								&operOr_EtoA,
								/* P = 3 */
								&operOr_EtoA
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operCp_EtoA,
								/* P = 1 */
								&operCp_EtoA,
								/* P = 2 */
								&operCp_EtoA,
								/* P = 3 */
								&operCp_EtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operCp_EtoA,
								/* P = 1 */
								&operCp_EtoA,
								/* P = 2 */
								&operCp_EtoA,
								/* P = 3 */
								&operCp_EtoA
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
								&operAdd_HtoA,
								/* P = 1 */
								&operAdd_HtoA,
								/* P = 2 */
								&operAdd_HtoA,
								/* P = 3 */
								&operAdd_HtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operAdd_HtoA,
								/* P = 1 */
								&operAdd_HtoA,
								/* P = 2 */
								&operAdd_HtoA,
								/* P = 3 */
								&operAdd_HtoA
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operAddCarry_HtoA,
								/* P = 1 */
								&operAddCarry_HtoA,
								/* P = 2 */
								&operAddCarry_HtoA,
								/* P = 3 */
								&operAddCarry_HtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operAddCarry_HtoA,
								/* P = 1 */
								&operAddCarry_HtoA,
								/* P = 2 */
								&operAddCarry_HtoA,
								/* P = 3 */
								&operAddCarry_HtoA
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSub_HtoA,
								/* P = 1 */
								&operSub_HtoA,
								/* P = 2 */
								&operSub_HtoA,
								/* P = 3 */
								&operSub_HtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSub_HtoA,
								/* P = 1 */
								&operSub_HtoA,
								/* P = 2 */
								&operSub_HtoA,
								/* P = 3 */
								&operSub_HtoA
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSubCarry_HtoA,
								/* P = 1 */
								&operSubCarry_HtoA,
								/* P = 2 */
								&operSubCarry_HtoA,
								/* P = 3 */
								&operSubCarry_HtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSubCarry_HtoA,
								/* P = 1 */
								&operSubCarry_HtoA,
								/* P = 2 */
								&operSubCarry_HtoA,
								/* P = 3 */
								&operSubCarry_HtoA
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operAnd_HtoA,
								/* P = 1 */
								&operAnd_HtoA,
								/* P = 2 */
								&operAnd_HtoA,
								/* P = 3 */
								&operAnd_HtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operAnd_HtoA,
								/* P = 1 */
								&operAnd_HtoA,
								/* P = 2 */
								&operAnd_HtoA,
								/* P = 3 */
								&operAnd_HtoA
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operXor_HtoA,
								/* P = 1 */
								&operXor_HtoA,
								/* P = 2 */
								&operXor_HtoA,
								/* P = 3 */
								&operXor_HtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operXor_HtoA,
								/* P = 1 */
								&operXor_HtoA,
								/* P = 2 */
								&operXor_HtoA,
								/* P = 3 */
								&operXor_HtoA
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operOr_HtoA,
								/* P = 1 */
								&operOr_HtoA,
								/* P = 2 */
								&operOr_HtoA,
								/* P = 3 */
								&operOr_HtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operOr_HtoA,
								/* P = 1 */
								&operOr_HtoA,
								/* P = 2 */
								&operOr_HtoA,
								/* P = 3 */
								&operOr_HtoA
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operCp_HtoA,
								/* P = 1 */
								&operCp_HtoA,
								/* P = 2 */
								&operCp_HtoA,
								/* P = 3 */
								&operCp_HtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operCp_HtoA,
								/* P = 1 */
								&operCp_HtoA,
								/* P = 2 */
								&operCp_HtoA,
								/* P = 3 */
								&operCp_HtoA
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
								&operAdd_LtoA,
								/* P = 1 */
								&operAdd_LtoA,
								/* P = 2 */
								&operAdd_LtoA,
								/* P = 3 */
								&operAdd_LtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operAdd_LtoA,
								/* P = 1 */
								&operAdd_LtoA,
								/* P = 2 */
								&operAdd_LtoA,
								/* P = 3 */
								&operAdd_LtoA
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operAddCarry_LtoA,
								/* P = 1 */
								&operAddCarry_LtoA,
								/* P = 2 */
								&operAddCarry_LtoA,
								/* P = 3 */
								&operAddCarry_LtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operAddCarry_LtoA,
								/* P = 1 */
								&operAddCarry_LtoA,
								/* P = 2 */
								&operAddCarry_LtoA,
								/* P = 3 */
								&operAddCarry_LtoA
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSub_LtoA,
								/* P = 1 */
								&operSub_LtoA,
								/* P = 2 */
								&operSub_LtoA,
								/* P = 3 */
								&operSub_LtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSub_LtoA,
								/* P = 1 */
								&operSub_LtoA,
								/* P = 2 */
								&operSub_LtoA,
								/* P = 3 */
								&operSub_LtoA
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSubCarry_LtoA,
								/* P = 1 */
								&operSubCarry_LtoA,
								/* P = 2 */
								&operSubCarry_LtoA,
								/* P = 3 */
								&operSubCarry_LtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSubCarry_LtoA,
								/* P = 1 */
								&operSubCarry_LtoA,
								/* P = 2 */
								&operSubCarry_LtoA,
								/* P = 3 */
								&operSubCarry_LtoA
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operAnd_LtoA,
								/* P = 1 */
								&operAnd_LtoA,
								/* P = 2 */
								&operAnd_LtoA,
								/* P = 3 */
								&operAnd_LtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operAnd_LtoA,
								/* P = 1 */
								&operAnd_LtoA,
								/* P = 2 */
								&operAnd_LtoA,
								/* P = 3 */
								&operAnd_LtoA
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operXor_LtoA,
								/* P = 1 */
								&operXor_LtoA,
								/* P = 2 */
								&operXor_LtoA,
								/* P = 3 */
								&operXor_LtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operXor_LtoA,
								/* P = 1 */
								&operXor_LtoA,
								/* P = 2 */
								&operXor_LtoA,
								/* P = 3 */
								&operXor_LtoA
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operOr_LtoA,
								/* P = 1 */
								&operOr_LtoA,
								/* P = 2 */
								&operOr_LtoA,
								/* P = 3 */
								&operOr_LtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operOr_LtoA,
								/* P = 1 */
								&operOr_LtoA,
								/* P = 2 */
								&operOr_LtoA,
								/* P = 3 */
								&operOr_LtoA
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operCp_LtoA,
								/* P = 1 */
								&operCp_LtoA,
								/* P = 2 */
								&operCp_LtoA,
								/* P = 3 */
								&operCp_LtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operCp_LtoA,
								/* P = 1 */
								&operCp_LtoA,
								/* P = 2 */
								&operCp_LtoA,
								/* P = 3 */
								&operCp_LtoA
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
								&operAdd_PAddrHLtoA,
								/* P = 1 */
								&operAdd_PAddrHLtoA,
								/* P = 2 */
								&operAdd_PAddrHLtoA,
								/* P = 3 */
								&operAdd_PAddrHLtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operAdd_PAddrHLtoA,
								/* P = 1 */
								&operAdd_PAddrHLtoA,
								/* P = 2 */
								&operAdd_PAddrHLtoA,
								/* P = 3 */
								&operAdd_PAddrHLtoA
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operAddCarry_PAddrHLtoA,
								/* P = 1 */
								&operAddCarry_PAddrHLtoA,
								/* P = 2 */
								&operAddCarry_PAddrHLtoA,
								/* P = 3 */
								&operAddCarry_PAddrHLtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operAddCarry_PAddrHLtoA,
								/* P = 1 */
								&operAddCarry_PAddrHLtoA,
								/* P = 2 */
								&operAddCarry_PAddrHLtoA,
								/* P = 3 */
								&operAddCarry_PAddrHLtoA
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSub_PAddrHLtoA,
								/* P = 1 */
								&operSub_PAddrHLtoA,
								/* P = 2 */
								&operSub_PAddrHLtoA,
								/* P = 3 */
								&operSub_PAddrHLtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSub_PAddrHLtoA,
								/* P = 1 */
								&operSub_PAddrHLtoA,
								/* P = 2 */
								&operSub_PAddrHLtoA,
								/* P = 3 */
								&operSub_PAddrHLtoA
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSubCarry_PAddrHLtoA,
								/* P = 1 */
								&operSubCarry_PAddrHLtoA,
								/* P = 2 */
								&operSubCarry_PAddrHLtoA,
								/* P = 3 */
								&operSubCarry_PAddrHLtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSubCarry_PAddrHLtoA,
								/* P = 1 */
								&operSubCarry_PAddrHLtoA,
								/* P = 2 */
								&operSubCarry_PAddrHLtoA,
								/* P = 3 */
								&operSubCarry_PAddrHLtoA
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operAnd_PAddrHLtoA,
								/* P = 1 */
								&operAnd_PAddrHLtoA,
								/* P = 2 */
								&operAnd_PAddrHLtoA,
								/* P = 3 */
								&operAnd_PAddrHLtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operAnd_PAddrHLtoA,
								/* P = 1 */
								&operAnd_PAddrHLtoA,
								/* P = 2 */
								&operAnd_PAddrHLtoA,
								/* P = 3 */
								&operAnd_PAddrHLtoA
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operXor_PAddrHLtoA,
								/* P = 1 */
								&operXor_PAddrHLtoA,
								/* P = 2 */
								&operXor_PAddrHLtoA,
								/* P = 3 */
								&operXor_PAddrHLtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operXor_PAddrHLtoA,
								/* P = 1 */
								&operXor_PAddrHLtoA,
								/* P = 2 */
								&operXor_PAddrHLtoA,
								/* P = 3 */
								&operXor_PAddrHLtoA
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operOr_PAddrHLtoA,
								/* P = 1 */
								&operOr_PAddrHLtoA,
								/* P = 2 */
								&operOr_PAddrHLtoA,
								/* P = 3 */
								&operOr_PAddrHLtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operOr_PAddrHLtoA,
								/* P = 1 */
								&operOr_PAddrHLtoA,
								/* P = 2 */
								&operOr_PAddrHLtoA,
								/* P = 3 */
								&operOr_PAddrHLtoA
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operCp_PAddrHLtoA,
								/* P = 1 */
								&operCp_PAddrHLtoA,
								/* P = 2 */
								&operCp_PAddrHLtoA,
								/* P = 3 */
								&operCp_PAddrHLtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operCp_PAddrHLtoA,
								/* P = 1 */
								&operCp_PAddrHLtoA,
								/* P = 2 */
								&operCp_PAddrHLtoA,
								/* P = 3 */
								&operCp_PAddrHLtoA
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
								&operAdd_AtoA,
								/* P = 1 */
								&operAdd_AtoA,
								/* P = 2 */
								&operAdd_AtoA,
								/* P = 3 */
								&operAdd_AtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operAdd_AtoA,
								/* P = 1 */
								&operAdd_AtoA,
								/* P = 2 */
								&operAdd_AtoA,
								/* P = 3 */
								&operAdd_AtoA
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operAddCarry_AtoA,
								/* P = 1 */
								&operAddCarry_AtoA,
								/* P = 2 */
								&operAddCarry_AtoA,
								/* P = 3 */
								&operAddCarry_AtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operAddCarry_AtoA,
								/* P = 1 */
								&operAddCarry_AtoA,
								/* P = 2 */
								&operAddCarry_AtoA,
								/* P = 3 */
								&operAddCarry_AtoA
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSub_AtoA,
								/* P = 1 */
								&operSub_AtoA,
								/* P = 2 */
								&operSub_AtoA,
								/* P = 3 */
								&operSub_AtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSub_AtoA,
								/* P = 1 */
								&operSub_AtoA,
								/* P = 2 */
								&operSub_AtoA,
								/* P = 3 */
								&operSub_AtoA
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSubCarry_AtoA,
								/* P = 1 */
								&operSubCarry_AtoA,
								/* P = 2 */
								&operSubCarry_AtoA,
								/* P = 3 */
								&operSubCarry_AtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSubCarry_AtoA,
								/* P = 1 */
								&operSubCarry_AtoA,
								/* P = 2 */
								&operSubCarry_AtoA,
								/* P = 3 */
								&operSubCarry_AtoA
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operAnd_AtoA,
								/* P = 1 */
								&operAnd_AtoA,
								/* P = 2 */
								&operAnd_AtoA,
								/* P = 3 */
								&operAnd_AtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operAnd_AtoA,
								/* P = 1 */
								&operAnd_AtoA,
								/* P = 2 */
								&operAnd_AtoA,
								/* P = 3 */
								&operAnd_AtoA
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operXor_AtoA,
								/* P = 1 */
								&operXor_AtoA,
								/* P = 2 */
								&operXor_AtoA,
								/* P = 3 */
								&operXor_AtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operXor_AtoA,
								/* P = 1 */
								&operXor_AtoA,
								/* P = 2 */
								&operXor_AtoA,
								/* P = 3 */
								&operXor_AtoA
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operOr_AtoA,
								/* P = 1 */
								&operOr_AtoA,
								/* P = 2 */
								&operOr_AtoA,
								/* P = 3 */
								&operOr_AtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operOr_AtoA,
								/* P = 1 */
								&operOr_AtoA,
								/* P = 2 */
								&operOr_AtoA,
								/* P = 3 */
								&operOr_AtoA
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operCp_AtoA,
								/* P = 1 */
								&operCp_AtoA,
								/* P = 2 */
								&operCp_AtoA,
								/* P = 3 */
								&operCp_AtoA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operCp_AtoA,
								/* P = 1 */
								&operCp_AtoA,
								/* P = 2 */
								&operCp_AtoA,
								/* P = 3 */
								&operCp_AtoA
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
								&operBit_Bit0inB,
								/* P = 1 */
								&operBit_Bit0inB,
								/* P = 2 */
								&operBit_Bit0inB,
								/* P = 3 */
								&operBit_Bit0inB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit0inB,
								/* P = 1 */
								&operBit_Bit0inB,
								/* P = 2 */
								&operBit_Bit0inB,
								/* P = 3 */
								&operBit_Bit0inB
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit1inB,
								/* P = 1 */
								&operBit_Bit1inB,
								/* P = 2 */
								&operBit_Bit1inB,
								/* P = 3 */
								&operBit_Bit1inB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit1inB,
								/* P = 1 */
								&operBit_Bit1inB,
								/* P = 2 */
								&operBit_Bit1inB,
								/* P = 3 */
								&operBit_Bit1inB
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit2inB,
								/* P = 1 */
								&operBit_Bit2inB,
								/* P = 2 */
								&operBit_Bit2inB,
								/* P = 3 */
								&operBit_Bit2inB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit2inB,
								/* P = 1 */
								&operBit_Bit2inB,
								/* P = 2 */
								&operBit_Bit2inB,
								/* P = 3 */
								&operBit_Bit2inB
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit3inB,
								/* P = 1 */
								&operBit_Bit3inB,
								/* P = 2 */
								&operBit_Bit3inB,
								/* P = 3 */
								&operBit_Bit3inB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit3inB,
								/* P = 1 */
								&operBit_Bit3inB,
								/* P = 2 */
								&operBit_Bit3inB,
								/* P = 3 */
								&operBit_Bit3inB
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit4inB,
								/* P = 1 */
								&operBit_Bit4inB,
								/* P = 2 */
								&operBit_Bit4inB,
								/* P = 3 */
								&operBit_Bit4inB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit4inB,
								/* P = 1 */
								&operBit_Bit4inB,
								/* P = 2 */
								&operBit_Bit4inB,
								/* P = 3 */
								&operBit_Bit4inB
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit5inB,
								/* P = 1 */
								&operBit_Bit5inB,
								/* P = 2 */
								&operBit_Bit5inB,
								/* P = 3 */
								&operBit_Bit5inB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit5inB,
								/* P = 1 */
								&operBit_Bit5inB,
								/* P = 2 */
								&operBit_Bit5inB,
								/* P = 3 */
								&operBit_Bit5inB
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit6inB,
								/* P = 1 */
								&operBit_Bit6inB,
								/* P = 2 */
								&operBit_Bit6inB,
								/* P = 3 */
								&operBit_Bit6inB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit6inB,
								/* P = 1 */
								&operBit_Bit6inB,
								/* P = 2 */
								&operBit_Bit6inB,
								/* P = 3 */
								&operBit_Bit6inB
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit7inB,
								/* P = 1 */
								&operBit_Bit7inB,
								/* P = 2 */
								&operBit_Bit7inB,
								/* P = 3 */
								&operBit_Bit7inB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit7inB,
								/* P = 1 */
								&operBit_Bit7inB,
								/* P = 2 */
								&operBit_Bit7inB,
								/* P = 3 */
								&operBit_Bit7inB
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
								&operBit_Bit0inC,
								/* P = 1 */
								&operBit_Bit0inC,
								/* P = 2 */
								&operBit_Bit0inC,
								/* P = 3 */
								&operBit_Bit0inC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit0inC,
								/* P = 1 */
								&operBit_Bit0inC,
								/* P = 2 */
								&operBit_Bit0inC,
								/* P = 3 */
								&operBit_Bit0inC
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit1inC,
								/* P = 1 */
								&operBit_Bit1inC,
								/* P = 2 */
								&operBit_Bit1inC,
								/* P = 3 */
								&operBit_Bit1inC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit1inC,
								/* P = 1 */
								&operBit_Bit1inC,
								/* P = 2 */
								&operBit_Bit1inC,
								/* P = 3 */
								&operBit_Bit1inC
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit2inC,
								/* P = 1 */
								&operBit_Bit2inC,
								/* P = 2 */
								&operBit_Bit2inC,
								/* P = 3 */
								&operBit_Bit2inC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit2inC,
								/* P = 1 */
								&operBit_Bit2inC,
								/* P = 2 */
								&operBit_Bit2inC,
								/* P = 3 */
								&operBit_Bit2inC
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit3inC,
								/* P = 1 */
								&operBit_Bit3inC,
								/* P = 2 */
								&operBit_Bit3inC,
								/* P = 3 */
								&operBit_Bit3inC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit3inC,
								/* P = 1 */
								&operBit_Bit3inC,
								/* P = 2 */
								&operBit_Bit3inC,
								/* P = 3 */
								&operBit_Bit3inC
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit4inC,
								/* P = 1 */
								&operBit_Bit4inC,
								/* P = 2 */
								&operBit_Bit4inC,
								/* P = 3 */
								&operBit_Bit4inC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit4inC,
								/* P = 1 */
								&operBit_Bit4inC,
								/* P = 2 */
								&operBit_Bit4inC,
								/* P = 3 */
								&operBit_Bit4inC
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit5inC,
								/* P = 1 */
								&operBit_Bit5inC,
								/* P = 2 */
								&operBit_Bit5inC,
								/* P = 3 */
								&operBit_Bit5inC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit5inC,
								/* P = 1 */
								&operBit_Bit5inC,
								/* P = 2 */
								&operBit_Bit5inC,
								/* P = 3 */
								&operBit_Bit5inC
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit6inC,
								/* P = 1 */
								&operBit_Bit6inC,
								/* P = 2 */
								&operBit_Bit6inC,
								/* P = 3 */
								&operBit_Bit6inC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit6inC,
								/* P = 1 */
								&operBit_Bit6inC,
								/* P = 2 */
								&operBit_Bit6inC,
								/* P = 3 */
								&operBit_Bit6inC
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit7inC,
								/* P = 1 */
								&operBit_Bit7inC,
								/* P = 2 */
								&operBit_Bit7inC,
								/* P = 3 */
								&operBit_Bit7inC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit7inC,
								/* P = 1 */
								&operBit_Bit7inC,
								/* P = 2 */
								&operBit_Bit7inC,
								/* P = 3 */
								&operBit_Bit7inC
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
								&operBit_Bit0inD,
								/* P = 1 */
								&operBit_Bit0inD,
								/* P = 2 */
								&operBit_Bit0inD,
								/* P = 3 */
								&operBit_Bit0inD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit0inD,
								/* P = 1 */
								&operBit_Bit0inD,
								/* P = 2 */
								&operBit_Bit0inD,
								/* P = 3 */
								&operBit_Bit0inD
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit1inD,
								/* P = 1 */
								&operBit_Bit1inD,
								/* P = 2 */
								&operBit_Bit1inD,
								/* P = 3 */
								&operBit_Bit1inD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit1inD,
								/* P = 1 */
								&operBit_Bit1inD,
								/* P = 2 */
								&operBit_Bit1inD,
								/* P = 3 */
								&operBit_Bit1inD
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit2inD,
								/* P = 1 */
								&operBit_Bit2inD,
								/* P = 2 */
								&operBit_Bit2inD,
								/* P = 3 */
								&operBit_Bit2inD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit2inD,
								/* P = 1 */
								&operBit_Bit2inD,
								/* P = 2 */
								&operBit_Bit2inD,
								/* P = 3 */
								&operBit_Bit2inD
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit3inD,
								/* P = 1 */
								&operBit_Bit3inD,
								/* P = 2 */
								&operBit_Bit3inD,
								/* P = 3 */
								&operBit_Bit3inD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit3inD,
								/* P = 1 */
								&operBit_Bit3inD,
								/* P = 2 */
								&operBit_Bit3inD,
								/* P = 3 */
								&operBit_Bit3inD
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit4inD,
								/* P = 1 */
								&operBit_Bit4inD,
								/* P = 2 */
								&operBit_Bit4inD,
								/* P = 3 */
								&operBit_Bit4inD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit4inD,
								/* P = 1 */
								&operBit_Bit4inD,
								/* P = 2 */
								&operBit_Bit4inD,
								/* P = 3 */
								&operBit_Bit4inD
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit5inD,
								/* P = 1 */
								&operBit_Bit5inD,
								/* P = 2 */
								&operBit_Bit5inD,
								/* P = 3 */
								&operBit_Bit5inD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit5inD,
								/* P = 1 */
								&operBit_Bit5inD,
								/* P = 2 */
								&operBit_Bit5inD,
								/* P = 3 */
								&operBit_Bit5inD
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit6inD,
								/* P = 1 */
								&operBit_Bit6inD,
								/* P = 2 */
								&operBit_Bit6inD,
								/* P = 3 */
								&operBit_Bit6inD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit6inD,
								/* P = 1 */
								&operBit_Bit6inD,
								/* P = 2 */
								&operBit_Bit6inD,
								/* P = 3 */
								&operBit_Bit6inD
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit7inD,
								/* P = 1 */
								&operBit_Bit7inD,
								/* P = 2 */
								&operBit_Bit7inD,
								/* P = 3 */
								&operBit_Bit7inD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit7inD,
								/* P = 1 */
								&operBit_Bit7inD,
								/* P = 2 */
								&operBit_Bit7inD,
								/* P = 3 */
								&operBit_Bit7inD
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
								&operBit_Bit0inE,
								/* P = 1 */
								&operBit_Bit0inE,
								/* P = 2 */
								&operBit_Bit0inE,
								/* P = 3 */
								&operBit_Bit0inE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit0inE,
								/* P = 1 */
								&operBit_Bit0inE,
								/* P = 2 */
								&operBit_Bit0inE,
								/* P = 3 */
								&operBit_Bit0inE
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit1inE,
								/* P = 1 */
								&operBit_Bit1inE,
								/* P = 2 */
								&operBit_Bit1inE,
								/* P = 3 */
								&operBit_Bit1inE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit1inE,
								/* P = 1 */
								&operBit_Bit1inE,
								/* P = 2 */
								&operBit_Bit1inE,
								/* P = 3 */
								&operBit_Bit1inE
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit2inE,
								/* P = 1 */
								&operBit_Bit2inE,
								/* P = 2 */
								&operBit_Bit2inE,
								/* P = 3 */
								&operBit_Bit2inE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit2inE,
								/* P = 1 */
								&operBit_Bit2inE,
								/* P = 2 */
								&operBit_Bit2inE,
								/* P = 3 */
								&operBit_Bit2inE
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit3inE,
								/* P = 1 */
								&operBit_Bit3inE,
								/* P = 2 */
								&operBit_Bit3inE,
								/* P = 3 */
								&operBit_Bit3inE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit3inE,
								/* P = 1 */
								&operBit_Bit3inE,
								/* P = 2 */
								&operBit_Bit3inE,
								/* P = 3 */
								&operBit_Bit3inE
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit4inE,
								/* P = 1 */
								&operBit_Bit4inE,
								/* P = 2 */
								&operBit_Bit4inE,
								/* P = 3 */
								&operBit_Bit4inE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit4inE,
								/* P = 1 */
								&operBit_Bit4inE,
								/* P = 2 */
								&operBit_Bit4inE,
								/* P = 3 */
								&operBit_Bit4inE
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit5inE,
								/* P = 1 */
								&operBit_Bit5inE,
								/* P = 2 */
								&operBit_Bit5inE,
								/* P = 3 */
								&operBit_Bit5inE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit5inE,
								/* P = 1 */
								&operBit_Bit5inE,
								/* P = 2 */
								&operBit_Bit5inE,
								/* P = 3 */
								&operBit_Bit5inE
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit6inE,
								/* P = 1 */
								&operBit_Bit6inE,
								/* P = 2 */
								&operBit_Bit6inE,
								/* P = 3 */
								&operBit_Bit6inE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit6inE,
								/* P = 1 */
								&operBit_Bit6inE,
								/* P = 2 */
								&operBit_Bit6inE,
								/* P = 3 */
								&operBit_Bit6inE
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit7inE,
								/* P = 1 */
								&operBit_Bit7inE,
								/* P = 2 */
								&operBit_Bit7inE,
								/* P = 3 */
								&operBit_Bit7inE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit7inE,
								/* P = 1 */
								&operBit_Bit7inE,
								/* P = 2 */
								&operBit_Bit7inE,
								/* P = 3 */
								&operBit_Bit7inE
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
								&operBit_Bit0inH,
								/* P = 1 */
								&operBit_Bit0inH,
								/* P = 2 */
								&operBit_Bit0inH,
								/* P = 3 */
								&operBit_Bit0inH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit0inH,
								/* P = 1 */
								&operBit_Bit0inH,
								/* P = 2 */
								&operBit_Bit0inH,
								/* P = 3 */
								&operBit_Bit0inH
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit1inH,
								/* P = 1 */
								&operBit_Bit1inH,
								/* P = 2 */
								&operBit_Bit1inH,
								/* P = 3 */
								&operBit_Bit1inH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit1inH,
								/* P = 1 */
								&operBit_Bit1inH,
								/* P = 2 */
								&operBit_Bit1inH,
								/* P = 3 */
								&operBit_Bit1inH
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit2inH,
								/* P = 1 */
								&operBit_Bit2inH,
								/* P = 2 */
								&operBit_Bit2inH,
								/* P = 3 */
								&operBit_Bit2inH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit2inH,
								/* P = 1 */
								&operBit_Bit2inH,
								/* P = 2 */
								&operBit_Bit2inH,
								/* P = 3 */
								&operBit_Bit2inH
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit3inH,
								/* P = 1 */
								&operBit_Bit3inH,
								/* P = 2 */
								&operBit_Bit3inH,
								/* P = 3 */
								&operBit_Bit3inH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit3inH,
								/* P = 1 */
								&operBit_Bit3inH,
								/* P = 2 */
								&operBit_Bit3inH,
								/* P = 3 */
								&operBit_Bit3inH
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit4inH,
								/* P = 1 */
								&operBit_Bit4inH,
								/* P = 2 */
								&operBit_Bit4inH,
								/* P = 3 */
								&operBit_Bit4inH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit4inH,
								/* P = 1 */
								&operBit_Bit4inH,
								/* P = 2 */
								&operBit_Bit4inH,
								/* P = 3 */
								&operBit_Bit4inH
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit5inH,
								/* P = 1 */
								&operBit_Bit5inH,
								/* P = 2 */
								&operBit_Bit5inH,
								/* P = 3 */
								&operBit_Bit5inH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit5inH,
								/* P = 1 */
								&operBit_Bit5inH,
								/* P = 2 */
								&operBit_Bit5inH,
								/* P = 3 */
								&operBit_Bit5inH
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit6inH,
								/* P = 1 */
								&operBit_Bit6inH,
								/* P = 2 */
								&operBit_Bit6inH,
								/* P = 3 */
								&operBit_Bit6inH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit6inH,
								/* P = 1 */
								&operBit_Bit6inH,
								/* P = 2 */
								&operBit_Bit6inH,
								/* P = 3 */
								&operBit_Bit6inH
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit7inH,
								/* P = 1 */
								&operBit_Bit7inH,
								/* P = 2 */
								&operBit_Bit7inH,
								/* P = 3 */
								&operBit_Bit7inH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit7inH,
								/* P = 1 */
								&operBit_Bit7inH,
								/* P = 2 */
								&operBit_Bit7inH,
								/* P = 3 */
								&operBit_Bit7inH
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
								&operBit_Bit0inL,
								/* P = 1 */
								&operBit_Bit0inL,
								/* P = 2 */
								&operBit_Bit0inL,
								/* P = 3 */
								&operBit_Bit0inL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit0inL,
								/* P = 1 */
								&operBit_Bit0inL,
								/* P = 2 */
								&operBit_Bit0inL,
								/* P = 3 */
								&operBit_Bit0inL
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit1inL,
								/* P = 1 */
								&operBit_Bit1inL,
								/* P = 2 */
								&operBit_Bit1inL,
								/* P = 3 */
								&operBit_Bit1inL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit1inL,
								/* P = 1 */
								&operBit_Bit1inL,
								/* P = 2 */
								&operBit_Bit1inL,
								/* P = 3 */
								&operBit_Bit1inL
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit2inL,
								/* P = 1 */
								&operBit_Bit2inL,
								/* P = 2 */
								&operBit_Bit2inL,
								/* P = 3 */
								&operBit_Bit2inL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit2inL,
								/* P = 1 */
								&operBit_Bit2inL,
								/* P = 2 */
								&operBit_Bit2inL,
								/* P = 3 */
								&operBit_Bit2inL
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit3inL,
								/* P = 1 */
								&operBit_Bit3inL,
								/* P = 2 */
								&operBit_Bit3inL,
								/* P = 3 */
								&operBit_Bit3inL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit3inL,
								/* P = 1 */
								&operBit_Bit3inL,
								/* P = 2 */
								&operBit_Bit3inL,
								/* P = 3 */
								&operBit_Bit3inL
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit4inL,
								/* P = 1 */
								&operBit_Bit4inL,
								/* P = 2 */
								&operBit_Bit4inL,
								/* P = 3 */
								&operBit_Bit4inL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit4inL,
								/* P = 1 */
								&operBit_Bit4inL,
								/* P = 2 */
								&operBit_Bit4inL,
								/* P = 3 */
								&operBit_Bit4inL
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit5inL,
								/* P = 1 */
								&operBit_Bit5inL,
								/* P = 2 */
								&operBit_Bit5inL,
								/* P = 3 */
								&operBit_Bit5inL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit5inL,
								/* P = 1 */
								&operBit_Bit5inL,
								/* P = 2 */
								&operBit_Bit5inL,
								/* P = 3 */
								&operBit_Bit5inL
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit6inL,
								/* P = 1 */
								&operBit_Bit6inL,
								/* P = 2 */
								&operBit_Bit6inL,
								/* P = 3 */
								&operBit_Bit6inL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit6inL,
								/* P = 1 */
								&operBit_Bit6inL,
								/* P = 2 */
								&operBit_Bit6inL,
								/* P = 3 */
								&operBit_Bit6inL
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit7inL,
								/* P = 1 */
								&operBit_Bit7inL,
								/* P = 2 */
								&operBit_Bit7inL,
								/* P = 3 */
								&operBit_Bit7inL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit7inL,
								/* P = 1 */
								&operBit_Bit7inL,
								/* P = 2 */
								&operBit_Bit7inL,
								/* P = 3 */
								&operBit_Bit7inL
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
								&operBit_Bit0inPAddrHL,
								/* P = 1 */
								&operBit_Bit0inPAddrHL,
								/* P = 2 */
								&operBit_Bit0inPAddrHL,
								/* P = 3 */
								&operBit_Bit0inPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit0inPAddrHL,
								/* P = 1 */
								&operBit_Bit0inPAddrHL,
								/* P = 2 */
								&operBit_Bit0inPAddrHL,
								/* P = 3 */
								&operBit_Bit0inPAddrHL
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit1inPAddrHL,
								/* P = 1 */
								&operBit_Bit1inPAddrHL,
								/* P = 2 */
								&operBit_Bit1inPAddrHL,
								/* P = 3 */
								&operBit_Bit1inPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit1inPAddrHL,
								/* P = 1 */
								&operBit_Bit1inPAddrHL,
								/* P = 2 */
								&operBit_Bit1inPAddrHL,
								/* P = 3 */
								&operBit_Bit1inPAddrHL
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit2inPAddrHL,
								/* P = 1 */
								&operBit_Bit2inPAddrHL,
								/* P = 2 */
								&operBit_Bit2inPAddrHL,
								/* P = 3 */
								&operBit_Bit2inPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit2inPAddrHL,
								/* P = 1 */
								&operBit_Bit2inPAddrHL,
								/* P = 2 */
								&operBit_Bit2inPAddrHL,
								/* P = 3 */
								&operBit_Bit2inPAddrHL
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit3inPAddrHL,
								/* P = 1 */
								&operBit_Bit3inPAddrHL,
								/* P = 2 */
								&operBit_Bit3inPAddrHL,
								/* P = 3 */
								&operBit_Bit3inPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit3inPAddrHL,
								/* P = 1 */
								&operBit_Bit3inPAddrHL,
								/* P = 2 */
								&operBit_Bit3inPAddrHL,
								/* P = 3 */
								&operBit_Bit3inPAddrHL
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit4inPAddrHL,
								/* P = 1 */
								&operBit_Bit4inPAddrHL,
								/* P = 2 */
								&operBit_Bit4inPAddrHL,
								/* P = 3 */
								&operBit_Bit4inPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit4inPAddrHL,
								/* P = 1 */
								&operBit_Bit4inPAddrHL,
								/* P = 2 */
								&operBit_Bit4inPAddrHL,
								/* P = 3 */
								&operBit_Bit4inPAddrHL
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit5inPAddrHL,
								/* P = 1 */
								&operBit_Bit5inPAddrHL,
								/* P = 2 */
								&operBit_Bit5inPAddrHL,
								/* P = 3 */
								&operBit_Bit5inPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit5inPAddrHL,
								/* P = 1 */
								&operBit_Bit5inPAddrHL,
								/* P = 2 */
								&operBit_Bit5inPAddrHL,
								/* P = 3 */
								&operBit_Bit5inPAddrHL
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit6inPAddrHL,
								/* P = 1 */
								&operBit_Bit6inPAddrHL,
								/* P = 2 */
								&operBit_Bit6inPAddrHL,
								/* P = 3 */
								&operBit_Bit6inPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit6inPAddrHL,
								/* P = 1 */
								&operBit_Bit6inPAddrHL,
								/* P = 2 */
								&operBit_Bit6inPAddrHL,
								/* P = 3 */
								&operBit_Bit6inPAddrHL
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit7inPAddrHL,
								/* P = 1 */
								&operBit_Bit7inPAddrHL,
								/* P = 2 */
								&operBit_Bit7inPAddrHL,
								/* P = 3 */
								&operBit_Bit7inPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit7inPAddrHL,
								/* P = 1 */
								&operBit_Bit7inPAddrHL,
								/* P = 2 */
								&operBit_Bit7inPAddrHL,
								/* P = 3 */
								&operBit_Bit7inPAddrHL
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
								&operBit_Bit0inA,
								/* P = 1 */
								&operBit_Bit0inA,
								/* P = 2 */
								&operBit_Bit0inA,
								/* P = 3 */
								&operBit_Bit0inA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit0inA,
								/* P = 1 */
								&operBit_Bit0inA,
								/* P = 2 */
								&operBit_Bit0inA,
								/* P = 3 */
								&operBit_Bit0inA
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit1inA,
								/* P = 1 */
								&operBit_Bit1inA,
								/* P = 2 */
								&operBit_Bit1inA,
								/* P = 3 */
								&operBit_Bit1inA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit1inA,
								/* P = 1 */
								&operBit_Bit1inA,
								/* P = 2 */
								&operBit_Bit1inA,
								/* P = 3 */
								&operBit_Bit1inA
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit2inA,
								/* P = 1 */
								&operBit_Bit2inA,
								/* P = 2 */
								&operBit_Bit2inA,
								/* P = 3 */
								&operBit_Bit2inA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit2inA,
								/* P = 1 */
								&operBit_Bit2inA,
								/* P = 2 */
								&operBit_Bit2inA,
								/* P = 3 */
								&operBit_Bit2inA
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit3inA,
								/* P = 1 */
								&operBit_Bit3inA,
								/* P = 2 */
								&operBit_Bit3inA,
								/* P = 3 */
								&operBit_Bit3inA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit3inA,
								/* P = 1 */
								&operBit_Bit3inA,
								/* P = 2 */
								&operBit_Bit3inA,
								/* P = 3 */
								&operBit_Bit3inA
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit4inA,
								/* P = 1 */
								&operBit_Bit4inA,
								/* P = 2 */
								&operBit_Bit4inA,
								/* P = 3 */
								&operBit_Bit4inA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit4inA,
								/* P = 1 */
								&operBit_Bit4inA,
								/* P = 2 */
								&operBit_Bit4inA,
								/* P = 3 */
								&operBit_Bit4inA
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit5inA,
								/* P = 1 */
								&operBit_Bit5inA,
								/* P = 2 */
								&operBit_Bit5inA,
								/* P = 3 */
								&operBit_Bit5inA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit5inA,
								/* P = 1 */
								&operBit_Bit5inA,
								/* P = 2 */
								&operBit_Bit5inA,
								/* P = 3 */
								&operBit_Bit5inA
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit6inA,
								/* P = 1 */
								&operBit_Bit6inA,
								/* P = 2 */
								&operBit_Bit6inA,
								/* P = 3 */
								&operBit_Bit6inA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit6inA,
								/* P = 1 */
								&operBit_Bit6inA,
								/* P = 2 */
								&operBit_Bit6inA,
								/* P = 3 */
								&operBit_Bit6inA
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operBit_Bit7inA,
								/* P = 1 */
								&operBit_Bit7inA,
								/* P = 2 */
								&operBit_Bit7inA,
								/* P = 3 */
								&operBit_Bit7inA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operBit_Bit7inA,
								/* P = 1 */
								&operBit_Bit7inA,
								/* P = 2 */
								&operBit_Bit7inA,
								/* P = 3 */
								&operBit_Bit7inA
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
								&operRes_Bit0inB,
								/* P = 1 */
								&operRes_Bit0inB,
								/* P = 2 */
								&operRes_Bit0inB,
								/* P = 3 */
								&operRes_Bit0inB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit0inB,
								/* P = 1 */
								&operRes_Bit0inB,
								/* P = 2 */
								&operRes_Bit0inB,
								/* P = 3 */
								&operRes_Bit0inB
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit1inB,
								/* P = 1 */
								&operRes_Bit1inB,
								/* P = 2 */
								&operRes_Bit1inB,
								/* P = 3 */
								&operRes_Bit1inB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit1inB,
								/* P = 1 */
								&operRes_Bit1inB,
								/* P = 2 */
								&operRes_Bit1inB,
								/* P = 3 */
								&operRes_Bit1inB
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit2inB,
								/* P = 1 */
								&operRes_Bit2inB,
								/* P = 2 */
								&operRes_Bit2inB,
								/* P = 3 */
								&operRes_Bit2inB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit2inB,
								/* P = 1 */
								&operRes_Bit2inB,
								/* P = 2 */
								&operRes_Bit2inB,
								/* P = 3 */
								&operRes_Bit2inB
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit3inB,
								/* P = 1 */
								&operRes_Bit3inB,
								/* P = 2 */
								&operRes_Bit3inB,
								/* P = 3 */
								&operRes_Bit3inB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit3inB,
								/* P = 1 */
								&operRes_Bit3inB,
								/* P = 2 */
								&operRes_Bit3inB,
								/* P = 3 */
								&operRes_Bit3inB
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit4inB,
								/* P = 1 */
								&operRes_Bit4inB,
								/* P = 2 */
								&operRes_Bit4inB,
								/* P = 3 */
								&operRes_Bit4inB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit4inB,
								/* P = 1 */
								&operRes_Bit4inB,
								/* P = 2 */
								&operRes_Bit4inB,
								/* P = 3 */
								&operRes_Bit4inB
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit5inB,
								/* P = 1 */
								&operRes_Bit5inB,
								/* P = 2 */
								&operRes_Bit5inB,
								/* P = 3 */
								&operRes_Bit5inB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit5inB,
								/* P = 1 */
								&operRes_Bit5inB,
								/* P = 2 */
								&operRes_Bit5inB,
								/* P = 3 */
								&operRes_Bit5inB
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit6inB,
								/* P = 1 */
								&operRes_Bit6inB,
								/* P = 2 */
								&operRes_Bit6inB,
								/* P = 3 */
								&operRes_Bit6inB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit6inB,
								/* P = 1 */
								&operRes_Bit6inB,
								/* P = 2 */
								&operRes_Bit6inB,
								/* P = 3 */
								&operRes_Bit6inB
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit7inB,
								/* P = 1 */
								&operRes_Bit7inB,
								/* P = 2 */
								&operRes_Bit7inB,
								/* P = 3 */
								&operRes_Bit7inB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit7inB,
								/* P = 1 */
								&operRes_Bit7inB,
								/* P = 2 */
								&operRes_Bit7inB,
								/* P = 3 */
								&operRes_Bit7inB
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
								&operRes_Bit0inC,
								/* P = 1 */
								&operRes_Bit0inC,
								/* P = 2 */
								&operRes_Bit0inC,
								/* P = 3 */
								&operRes_Bit0inC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit0inC,
								/* P = 1 */
								&operRes_Bit0inC,
								/* P = 2 */
								&operRes_Bit0inC,
								/* P = 3 */
								&operRes_Bit0inC
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit1inC,
								/* P = 1 */
								&operRes_Bit1inC,
								/* P = 2 */
								&operRes_Bit1inC,
								/* P = 3 */
								&operRes_Bit1inC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit1inC,
								/* P = 1 */
								&operRes_Bit1inC,
								/* P = 2 */
								&operRes_Bit1inC,
								/* P = 3 */
								&operRes_Bit1inC
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit2inC,
								/* P = 1 */
								&operRes_Bit2inC,
								/* P = 2 */
								&operRes_Bit2inC,
								/* P = 3 */
								&operRes_Bit2inC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit2inC,
								/* P = 1 */
								&operRes_Bit2inC,
								/* P = 2 */
								&operRes_Bit2inC,
								/* P = 3 */
								&operRes_Bit2inC
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit3inC,
								/* P = 1 */
								&operRes_Bit3inC,
								/* P = 2 */
								&operRes_Bit3inC,
								/* P = 3 */
								&operRes_Bit3inC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit3inC,
								/* P = 1 */
								&operRes_Bit3inC,
								/* P = 2 */
								&operRes_Bit3inC,
								/* P = 3 */
								&operRes_Bit3inC
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit4inC,
								/* P = 1 */
								&operRes_Bit4inC,
								/* P = 2 */
								&operRes_Bit4inC,
								/* P = 3 */
								&operRes_Bit4inC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit4inC,
								/* P = 1 */
								&operRes_Bit4inC,
								/* P = 2 */
								&operRes_Bit4inC,
								/* P = 3 */
								&operRes_Bit4inC
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit5inC,
								/* P = 1 */
								&operRes_Bit5inC,
								/* P = 2 */
								&operRes_Bit5inC,
								/* P = 3 */
								&operRes_Bit5inC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit5inC,
								/* P = 1 */
								&operRes_Bit5inC,
								/* P = 2 */
								&operRes_Bit5inC,
								/* P = 3 */
								&operRes_Bit5inC
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit6inC,
								/* P = 1 */
								&operRes_Bit6inC,
								/* P = 2 */
								&operRes_Bit6inC,
								/* P = 3 */
								&operRes_Bit6inC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit6inC,
								/* P = 1 */
								&operRes_Bit6inC,
								/* P = 2 */
								&operRes_Bit6inC,
								/* P = 3 */
								&operRes_Bit6inC
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit7inC,
								/* P = 1 */
								&operRes_Bit7inC,
								/* P = 2 */
								&operRes_Bit7inC,
								/* P = 3 */
								&operRes_Bit7inC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit7inC,
								/* P = 1 */
								&operRes_Bit7inC,
								/* P = 2 */
								&operRes_Bit7inC,
								/* P = 3 */
								&operRes_Bit7inC
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
								&operRes_Bit0inD,
								/* P = 1 */
								&operRes_Bit0inD,
								/* P = 2 */
								&operRes_Bit0inD,
								/* P = 3 */
								&operRes_Bit0inD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit0inD,
								/* P = 1 */
								&operRes_Bit0inD,
								/* P = 2 */
								&operRes_Bit0inD,
								/* P = 3 */
								&operRes_Bit0inD
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit1inD,
								/* P = 1 */
								&operRes_Bit1inD,
								/* P = 2 */
								&operRes_Bit1inD,
								/* P = 3 */
								&operRes_Bit1inD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit1inD,
								/* P = 1 */
								&operRes_Bit1inD,
								/* P = 2 */
								&operRes_Bit1inD,
								/* P = 3 */
								&operRes_Bit1inD
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit2inD,
								/* P = 1 */
								&operRes_Bit2inD,
								/* P = 2 */
								&operRes_Bit2inD,
								/* P = 3 */
								&operRes_Bit2inD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit2inD,
								/* P = 1 */
								&operRes_Bit2inD,
								/* P = 2 */
								&operRes_Bit2inD,
								/* P = 3 */
								&operRes_Bit2inD
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit3inD,
								/* P = 1 */
								&operRes_Bit3inD,
								/* P = 2 */
								&operRes_Bit3inD,
								/* P = 3 */
								&operRes_Bit3inD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit3inD,
								/* P = 1 */
								&operRes_Bit3inD,
								/* P = 2 */
								&operRes_Bit3inD,
								/* P = 3 */
								&operRes_Bit3inD
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit4inD,
								/* P = 1 */
								&operRes_Bit4inD,
								/* P = 2 */
								&operRes_Bit4inD,
								/* P = 3 */
								&operRes_Bit4inD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit4inD,
								/* P = 1 */
								&operRes_Bit4inD,
								/* P = 2 */
								&operRes_Bit4inD,
								/* P = 3 */
								&operRes_Bit4inD
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit5inD,
								/* P = 1 */
								&operRes_Bit5inD,
								/* P = 2 */
								&operRes_Bit5inD,
								/* P = 3 */
								&operRes_Bit5inD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit5inD,
								/* P = 1 */
								&operRes_Bit5inD,
								/* P = 2 */
								&operRes_Bit5inD,
								/* P = 3 */
								&operRes_Bit5inD
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit6inD,
								/* P = 1 */
								&operRes_Bit6inD,
								/* P = 2 */
								&operRes_Bit6inD,
								/* P = 3 */
								&operRes_Bit6inD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit6inD,
								/* P = 1 */
								&operRes_Bit6inD,
								/* P = 2 */
								&operRes_Bit6inD,
								/* P = 3 */
								&operRes_Bit6inD
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit7inD,
								/* P = 1 */
								&operRes_Bit7inD,
								/* P = 2 */
								&operRes_Bit7inD,
								/* P = 3 */
								&operRes_Bit7inD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit7inD,
								/* P = 1 */
								&operRes_Bit7inD,
								/* P = 2 */
								&operRes_Bit7inD,
								/* P = 3 */
								&operRes_Bit7inD
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
								&operRes_Bit0inE,
								/* P = 1 */
								&operRes_Bit0inE,
								/* P = 2 */
								&operRes_Bit0inE,
								/* P = 3 */
								&operRes_Bit0inE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit0inE,
								/* P = 1 */
								&operRes_Bit0inE,
								/* P = 2 */
								&operRes_Bit0inE,
								/* P = 3 */
								&operRes_Bit0inE
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit1inE,
								/* P = 1 */
								&operRes_Bit1inE,
								/* P = 2 */
								&operRes_Bit1inE,
								/* P = 3 */
								&operRes_Bit1inE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit1inE,
								/* P = 1 */
								&operRes_Bit1inE,
								/* P = 2 */
								&operRes_Bit1inE,
								/* P = 3 */
								&operRes_Bit1inE
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit2inE,
								/* P = 1 */
								&operRes_Bit2inE,
								/* P = 2 */
								&operRes_Bit2inE,
								/* P = 3 */
								&operRes_Bit2inE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit2inE,
								/* P = 1 */
								&operRes_Bit2inE,
								/* P = 2 */
								&operRes_Bit2inE,
								/* P = 3 */
								&operRes_Bit2inE
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit3inE,
								/* P = 1 */
								&operRes_Bit3inE,
								/* P = 2 */
								&operRes_Bit3inE,
								/* P = 3 */
								&operRes_Bit3inE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit3inE,
								/* P = 1 */
								&operRes_Bit3inE,
								/* P = 2 */
								&operRes_Bit3inE,
								/* P = 3 */
								&operRes_Bit3inE
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit4inE,
								/* P = 1 */
								&operRes_Bit4inE,
								/* P = 2 */
								&operRes_Bit4inE,
								/* P = 3 */
								&operRes_Bit4inE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit4inE,
								/* P = 1 */
								&operRes_Bit4inE,
								/* P = 2 */
								&operRes_Bit4inE,
								/* P = 3 */
								&operRes_Bit4inE
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit5inE,
								/* P = 1 */
								&operRes_Bit5inE,
								/* P = 2 */
								&operRes_Bit5inE,
								/* P = 3 */
								&operRes_Bit5inE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit5inE,
								/* P = 1 */
								&operRes_Bit5inE,
								/* P = 2 */
								&operRes_Bit5inE,
								/* P = 3 */
								&operRes_Bit5inE
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit6inE,
								/* P = 1 */
								&operRes_Bit6inE,
								/* P = 2 */
								&operRes_Bit6inE,
								/* P = 3 */
								&operRes_Bit6inE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit6inE,
								/* P = 1 */
								&operRes_Bit6inE,
								/* P = 2 */
								&operRes_Bit6inE,
								/* P = 3 */
								&operRes_Bit6inE
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit7inE,
								/* P = 1 */
								&operRes_Bit7inE,
								/* P = 2 */
								&operRes_Bit7inE,
								/* P = 3 */
								&operRes_Bit7inE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit7inE,
								/* P = 1 */
								&operRes_Bit7inE,
								/* P = 2 */
								&operRes_Bit7inE,
								/* P = 3 */
								&operRes_Bit7inE
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
								&operRes_Bit0inH,
								/* P = 1 */
								&operRes_Bit0inH,
								/* P = 2 */
								&operRes_Bit0inH,
								/* P = 3 */
								&operRes_Bit0inH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit0inH,
								/* P = 1 */
								&operRes_Bit0inH,
								/* P = 2 */
								&operRes_Bit0inH,
								/* P = 3 */
								&operRes_Bit0inH
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit1inH,
								/* P = 1 */
								&operRes_Bit1inH,
								/* P = 2 */
								&operRes_Bit1inH,
								/* P = 3 */
								&operRes_Bit1inH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit1inH,
								/* P = 1 */
								&operRes_Bit1inH,
								/* P = 2 */
								&operRes_Bit1inH,
								/* P = 3 */
								&operRes_Bit1inH
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit2inH,
								/* P = 1 */
								&operRes_Bit2inH,
								/* P = 2 */
								&operRes_Bit2inH,
								/* P = 3 */
								&operRes_Bit2inH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit2inH,
								/* P = 1 */
								&operRes_Bit2inH,
								/* P = 2 */
								&operRes_Bit2inH,
								/* P = 3 */
								&operRes_Bit2inH
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit3inH,
								/* P = 1 */
								&operRes_Bit3inH,
								/* P = 2 */
								&operRes_Bit3inH,
								/* P = 3 */
								&operRes_Bit3inH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit3inH,
								/* P = 1 */
								&operRes_Bit3inH,
								/* P = 2 */
								&operRes_Bit3inH,
								/* P = 3 */
								&operRes_Bit3inH
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit4inH,
								/* P = 1 */
								&operRes_Bit4inH,
								/* P = 2 */
								&operRes_Bit4inH,
								/* P = 3 */
								&operRes_Bit4inH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit4inH,
								/* P = 1 */
								&operRes_Bit4inH,
								/* P = 2 */
								&operRes_Bit4inH,
								/* P = 3 */
								&operRes_Bit4inH
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit5inH,
								/* P = 1 */
								&operRes_Bit5inH,
								/* P = 2 */
								&operRes_Bit5inH,
								/* P = 3 */
								&operRes_Bit5inH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit5inH,
								/* P = 1 */
								&operRes_Bit5inH,
								/* P = 2 */
								&operRes_Bit5inH,
								/* P = 3 */
								&operRes_Bit5inH
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit6inH,
								/* P = 1 */
								&operRes_Bit6inH,
								/* P = 2 */
								&operRes_Bit6inH,
								/* P = 3 */
								&operRes_Bit6inH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit6inH,
								/* P = 1 */
								&operRes_Bit6inH,
								/* P = 2 */
								&operRes_Bit6inH,
								/* P = 3 */
								&operRes_Bit6inH
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit7inH,
								/* P = 1 */
								&operRes_Bit7inH,
								/* P = 2 */
								&operRes_Bit7inH,
								/* P = 3 */
								&operRes_Bit7inH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit7inH,
								/* P = 1 */
								&operRes_Bit7inH,
								/* P = 2 */
								&operRes_Bit7inH,
								/* P = 3 */
								&operRes_Bit7inH
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
								&operRes_Bit0inL,
								/* P = 1 */
								&operRes_Bit0inL,
								/* P = 2 */
								&operRes_Bit0inL,
								/* P = 3 */
								&operRes_Bit0inL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit0inL,
								/* P = 1 */
								&operRes_Bit0inL,
								/* P = 2 */
								&operRes_Bit0inL,
								/* P = 3 */
								&operRes_Bit0inL
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit1inL,
								/* P = 1 */
								&operRes_Bit1inL,
								/* P = 2 */
								&operRes_Bit1inL,
								/* P = 3 */
								&operRes_Bit1inL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit1inL,
								/* P = 1 */
								&operRes_Bit1inL,
								/* P = 2 */
								&operRes_Bit1inL,
								/* P = 3 */
								&operRes_Bit1inL
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit2inL,
								/* P = 1 */
								&operRes_Bit2inL,
								/* P = 2 */
								&operRes_Bit2inL,
								/* P = 3 */
								&operRes_Bit2inL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit2inL,
								/* P = 1 */
								&operRes_Bit2inL,
								/* P = 2 */
								&operRes_Bit2inL,
								/* P = 3 */
								&operRes_Bit2inL
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit3inL,
								/* P = 1 */
								&operRes_Bit3inL,
								/* P = 2 */
								&operRes_Bit3inL,
								/* P = 3 */
								&operRes_Bit3inL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit3inL,
								/* P = 1 */
								&operRes_Bit3inL,
								/* P = 2 */
								&operRes_Bit3inL,
								/* P = 3 */
								&operRes_Bit3inL
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit4inL,
								/* P = 1 */
								&operRes_Bit4inL,
								/* P = 2 */
								&operRes_Bit4inL,
								/* P = 3 */
								&operRes_Bit4inL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit4inL,
								/* P = 1 */
								&operRes_Bit4inL,
								/* P = 2 */
								&operRes_Bit4inL,
								/* P = 3 */
								&operRes_Bit4inL
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit5inL,
								/* P = 1 */
								&operRes_Bit5inL,
								/* P = 2 */
								&operRes_Bit5inL,
								/* P = 3 */
								&operRes_Bit5inL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit5inL,
								/* P = 1 */
								&operRes_Bit5inL,
								/* P = 2 */
								&operRes_Bit5inL,
								/* P = 3 */
								&operRes_Bit5inL
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit6inL,
								/* P = 1 */
								&operRes_Bit6inL,
								/* P = 2 */
								&operRes_Bit6inL,
								/* P = 3 */
								&operRes_Bit6inL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit6inL,
								/* P = 1 */
								&operRes_Bit6inL,
								/* P = 2 */
								&operRes_Bit6inL,
								/* P = 3 */
								&operRes_Bit6inL
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit7inL,
								/* P = 1 */
								&operRes_Bit7inL,
								/* P = 2 */
								&operRes_Bit7inL,
								/* P = 3 */
								&operRes_Bit7inL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit7inL,
								/* P = 1 */
								&operRes_Bit7inL,
								/* P = 2 */
								&operRes_Bit7inL,
								/* P = 3 */
								&operRes_Bit7inL
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
								&operRes_Bit0inPAddrHL,
								/* P = 1 */
								&operRes_Bit0inPAddrHL,
								/* P = 2 */
								&operRes_Bit0inPAddrHL,
								/* P = 3 */
								&operRes_Bit0inPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit0inPAddrHL,
								/* P = 1 */
								&operRes_Bit0inPAddrHL,
								/* P = 2 */
								&operRes_Bit0inPAddrHL,
								/* P = 3 */
								&operRes_Bit0inPAddrHL
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit1inPAddrHL,
								/* P = 1 */
								&operRes_Bit1inPAddrHL,
								/* P = 2 */
								&operRes_Bit1inPAddrHL,
								/* P = 3 */
								&operRes_Bit1inPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit1inPAddrHL,
								/* P = 1 */
								&operRes_Bit1inPAddrHL,
								/* P = 2 */
								&operRes_Bit1inPAddrHL,
								/* P = 3 */
								&operRes_Bit1inPAddrHL
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit2inPAddrHL,
								/* P = 1 */
								&operRes_Bit2inPAddrHL,
								/* P = 2 */
								&operRes_Bit2inPAddrHL,
								/* P = 3 */
								&operRes_Bit2inPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit2inPAddrHL,
								/* P = 1 */
								&operRes_Bit2inPAddrHL,
								/* P = 2 */
								&operRes_Bit2inPAddrHL,
								/* P = 3 */
								&operRes_Bit2inPAddrHL
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit3inPAddrHL,
								/* P = 1 */
								&operRes_Bit3inPAddrHL,
								/* P = 2 */
								&operRes_Bit3inPAddrHL,
								/* P = 3 */
								&operRes_Bit3inPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit3inPAddrHL,
								/* P = 1 */
								&operRes_Bit3inPAddrHL,
								/* P = 2 */
								&operRes_Bit3inPAddrHL,
								/* P = 3 */
								&operRes_Bit3inPAddrHL
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit4inPAddrHL,
								/* P = 1 */
								&operRes_Bit4inPAddrHL,
								/* P = 2 */
								&operRes_Bit4inPAddrHL,
								/* P = 3 */
								&operRes_Bit4inPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit4inPAddrHL,
								/* P = 1 */
								&operRes_Bit4inPAddrHL,
								/* P = 2 */
								&operRes_Bit4inPAddrHL,
								/* P = 3 */
								&operRes_Bit4inPAddrHL
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit5inPAddrHL,
								/* P = 1 */
								&operRes_Bit5inPAddrHL,
								/* P = 2 */
								&operRes_Bit5inPAddrHL,
								/* P = 3 */
								&operRes_Bit5inPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit5inPAddrHL,
								/* P = 1 */
								&operRes_Bit5inPAddrHL,
								/* P = 2 */
								&operRes_Bit5inPAddrHL,
								/* P = 3 */
								&operRes_Bit5inPAddrHL
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit6inPAddrHL,
								/* P = 1 */
								&operRes_Bit6inPAddrHL,
								/* P = 2 */
								&operRes_Bit6inPAddrHL,
								/* P = 3 */
								&operRes_Bit6inPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit6inPAddrHL,
								/* P = 1 */
								&operRes_Bit6inPAddrHL,
								/* P = 2 */
								&operRes_Bit6inPAddrHL,
								/* P = 3 */
								&operRes_Bit6inPAddrHL
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit7inPAddrHL,
								/* P = 1 */
								&operRes_Bit7inPAddrHL,
								/* P = 2 */
								&operRes_Bit7inPAddrHL,
								/* P = 3 */
								&operRes_Bit7inPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit7inPAddrHL,
								/* P = 1 */
								&operRes_Bit7inPAddrHL,
								/* P = 2 */
								&operRes_Bit7inPAddrHL,
								/* P = 3 */
								&operRes_Bit7inPAddrHL
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
								&operRes_Bit0inA,
								/* P = 1 */
								&operRes_Bit0inA,
								/* P = 2 */
								&operRes_Bit0inA,
								/* P = 3 */
								&operRes_Bit0inA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit0inA,
								/* P = 1 */
								&operRes_Bit0inA,
								/* P = 2 */
								&operRes_Bit0inA,
								/* P = 3 */
								&operRes_Bit0inA
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit1inA,
								/* P = 1 */
								&operRes_Bit1inA,
								/* P = 2 */
								&operRes_Bit1inA,
								/* P = 3 */
								&operRes_Bit1inA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit1inA,
								/* P = 1 */
								&operRes_Bit1inA,
								/* P = 2 */
								&operRes_Bit1inA,
								/* P = 3 */
								&operRes_Bit1inA
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit2inA,
								/* P = 1 */
								&operRes_Bit2inA,
								/* P = 2 */
								&operRes_Bit2inA,
								/* P = 3 */
								&operRes_Bit2inA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit2inA,
								/* P = 1 */
								&operRes_Bit2inA,
								/* P = 2 */
								&operRes_Bit2inA,
								/* P = 3 */
								&operRes_Bit2inA
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit3inA,
								/* P = 1 */
								&operRes_Bit3inA,
								/* P = 2 */
								&operRes_Bit3inA,
								/* P = 3 */
								&operRes_Bit3inA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit3inA,
								/* P = 1 */
								&operRes_Bit3inA,
								/* P = 2 */
								&operRes_Bit3inA,
								/* P = 3 */
								&operRes_Bit3inA
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit4inA,
								/* P = 1 */
								&operRes_Bit4inA,
								/* P = 2 */
								&operRes_Bit4inA,
								/* P = 3 */
								&operRes_Bit4inA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit4inA,
								/* P = 1 */
								&operRes_Bit4inA,
								/* P = 2 */
								&operRes_Bit4inA,
								/* P = 3 */
								&operRes_Bit4inA
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit5inA,
								/* P = 1 */
								&operRes_Bit5inA,
								/* P = 2 */
								&operRes_Bit5inA,
								/* P = 3 */
								&operRes_Bit5inA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit5inA,
								/* P = 1 */
								&operRes_Bit5inA,
								/* P = 2 */
								&operRes_Bit5inA,
								/* P = 3 */
								&operRes_Bit5inA
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit6inA,
								/* P = 1 */
								&operRes_Bit6inA,
								/* P = 2 */
								&operRes_Bit6inA,
								/* P = 3 */
								&operRes_Bit6inA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit6inA,
								/* P = 1 */
								&operRes_Bit6inA,
								/* P = 2 */
								&operRes_Bit6inA,
								/* P = 3 */
								&operRes_Bit6inA
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operRes_Bit7inA,
								/* P = 1 */
								&operRes_Bit7inA,
								/* P = 2 */
								&operRes_Bit7inA,
								/* P = 3 */
								&operRes_Bit7inA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operRes_Bit7inA,
								/* P = 1 */
								&operRes_Bit7inA,
								/* P = 2 */
								&operRes_Bit7inA,
								/* P = 3 */
								&operRes_Bit7inA
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
								&operSet_Bit0inB,
								/* P = 1 */
								&operSet_Bit0inB,
								/* P = 2 */
								&operSet_Bit0inB,
								/* P = 3 */
								&operSet_Bit0inB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit0inB,
								/* P = 1 */
								&operSet_Bit0inB,
								/* P = 2 */
								&operSet_Bit0inB,
								/* P = 3 */
								&operSet_Bit0inB
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit1inB,
								/* P = 1 */
								&operSet_Bit1inB,
								/* P = 2 */
								&operSet_Bit1inB,
								/* P = 3 */
								&operSet_Bit1inB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit1inB,
								/* P = 1 */
								&operSet_Bit1inB,
								/* P = 2 */
								&operSet_Bit1inB,
								/* P = 3 */
								&operSet_Bit1inB
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit2inB,
								/* P = 1 */
								&operSet_Bit2inB,
								/* P = 2 */
								&operSet_Bit2inB,
								/* P = 3 */
								&operSet_Bit2inB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit2inB,
								/* P = 1 */
								&operSet_Bit2inB,
								/* P = 2 */
								&operSet_Bit2inB,
								/* P = 3 */
								&operSet_Bit2inB
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit3inB,
								/* P = 1 */
								&operSet_Bit3inB,
								/* P = 2 */
								&operSet_Bit3inB,
								/* P = 3 */
								&operSet_Bit3inB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit3inB,
								/* P = 1 */
								&operSet_Bit3inB,
								/* P = 2 */
								&operSet_Bit3inB,
								/* P = 3 */
								&operSet_Bit3inB
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit4inB,
								/* P = 1 */
								&operSet_Bit4inB,
								/* P = 2 */
								&operSet_Bit4inB,
								/* P = 3 */
								&operSet_Bit4inB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit4inB,
								/* P = 1 */
								&operSet_Bit4inB,
								/* P = 2 */
								&operSet_Bit4inB,
								/* P = 3 */
								&operSet_Bit4inB
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit5inB,
								/* P = 1 */
								&operSet_Bit5inB,
								/* P = 2 */
								&operSet_Bit5inB,
								/* P = 3 */
								&operSet_Bit5inB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit5inB,
								/* P = 1 */
								&operSet_Bit5inB,
								/* P = 2 */
								&operSet_Bit5inB,
								/* P = 3 */
								&operSet_Bit5inB
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit6inB,
								/* P = 1 */
								&operSet_Bit6inB,
								/* P = 2 */
								&operSet_Bit6inB,
								/* P = 3 */
								&operSet_Bit6inB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit6inB,
								/* P = 1 */
								&operSet_Bit6inB,
								/* P = 2 */
								&operSet_Bit6inB,
								/* P = 3 */
								&operSet_Bit6inB
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit7inB,
								/* P = 1 */
								&operSet_Bit7inB,
								/* P = 2 */
								&operSet_Bit7inB,
								/* P = 3 */
								&operSet_Bit7inB
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit7inB,
								/* P = 1 */
								&operSet_Bit7inB,
								/* P = 2 */
								&operSet_Bit7inB,
								/* P = 3 */
								&operSet_Bit7inB
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
								&operSet_Bit0inC,
								/* P = 1 */
								&operSet_Bit0inC,
								/* P = 2 */
								&operSet_Bit0inC,
								/* P = 3 */
								&operSet_Bit0inC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit0inC,
								/* P = 1 */
								&operSet_Bit0inC,
								/* P = 2 */
								&operSet_Bit0inC,
								/* P = 3 */
								&operSet_Bit0inC
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit1inC,
								/* P = 1 */
								&operSet_Bit1inC,
								/* P = 2 */
								&operSet_Bit1inC,
								/* P = 3 */
								&operSet_Bit1inC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit1inC,
								/* P = 1 */
								&operSet_Bit1inC,
								/* P = 2 */
								&operSet_Bit1inC,
								/* P = 3 */
								&operSet_Bit1inC
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit2inC,
								/* P = 1 */
								&operSet_Bit2inC,
								/* P = 2 */
								&operSet_Bit2inC,
								/* P = 3 */
								&operSet_Bit2inC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit2inC,
								/* P = 1 */
								&operSet_Bit2inC,
								/* P = 2 */
								&operSet_Bit2inC,
								/* P = 3 */
								&operSet_Bit2inC
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit3inC,
								/* P = 1 */
								&operSet_Bit3inC,
								/* P = 2 */
								&operSet_Bit3inC,
								/* P = 3 */
								&operSet_Bit3inC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit3inC,
								/* P = 1 */
								&operSet_Bit3inC,
								/* P = 2 */
								&operSet_Bit3inC,
								/* P = 3 */
								&operSet_Bit3inC
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit4inC,
								/* P = 1 */
								&operSet_Bit4inC,
								/* P = 2 */
								&operSet_Bit4inC,
								/* P = 3 */
								&operSet_Bit4inC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit4inC,
								/* P = 1 */
								&operSet_Bit4inC,
								/* P = 2 */
								&operSet_Bit4inC,
								/* P = 3 */
								&operSet_Bit4inC
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit5inC,
								/* P = 1 */
								&operSet_Bit5inC,
								/* P = 2 */
								&operSet_Bit5inC,
								/* P = 3 */
								&operSet_Bit5inC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit5inC,
								/* P = 1 */
								&operSet_Bit5inC,
								/* P = 2 */
								&operSet_Bit5inC,
								/* P = 3 */
								&operSet_Bit5inC
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit6inC,
								/* P = 1 */
								&operSet_Bit6inC,
								/* P = 2 */
								&operSet_Bit6inC,
								/* P = 3 */
								&operSet_Bit6inC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit6inC,
								/* P = 1 */
								&operSet_Bit6inC,
								/* P = 2 */
								&operSet_Bit6inC,
								/* P = 3 */
								&operSet_Bit6inC
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit7inC,
								/* P = 1 */
								&operSet_Bit7inC,
								/* P = 2 */
								&operSet_Bit7inC,
								/* P = 3 */
								&operSet_Bit7inC
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit7inC,
								/* P = 1 */
								&operSet_Bit7inC,
								/* P = 2 */
								&operSet_Bit7inC,
								/* P = 3 */
								&operSet_Bit7inC
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
								&operSet_Bit0inD,
								/* P = 1 */
								&operSet_Bit0inD,
								/* P = 2 */
								&operSet_Bit0inD,
								/* P = 3 */
								&operSet_Bit0inD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit0inD,
								/* P = 1 */
								&operSet_Bit0inD,
								/* P = 2 */
								&operSet_Bit0inD,
								/* P = 3 */
								&operSet_Bit0inD
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit1inD,
								/* P = 1 */
								&operSet_Bit1inD,
								/* P = 2 */
								&operSet_Bit1inD,
								/* P = 3 */
								&operSet_Bit1inD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit1inD,
								/* P = 1 */
								&operSet_Bit1inD,
								/* P = 2 */
								&operSet_Bit1inD,
								/* P = 3 */
								&operSet_Bit1inD
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit2inD,
								/* P = 1 */
								&operSet_Bit2inD,
								/* P = 2 */
								&operSet_Bit2inD,
								/* P = 3 */
								&operSet_Bit2inD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit2inD,
								/* P = 1 */
								&operSet_Bit2inD,
								/* P = 2 */
								&operSet_Bit2inD,
								/* P = 3 */
								&operSet_Bit2inD
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit3inD,
								/* P = 1 */
								&operSet_Bit3inD,
								/* P = 2 */
								&operSet_Bit3inD,
								/* P = 3 */
								&operSet_Bit3inD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit3inD,
								/* P = 1 */
								&operSet_Bit3inD,
								/* P = 2 */
								&operSet_Bit3inD,
								/* P = 3 */
								&operSet_Bit3inD
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit4inD,
								/* P = 1 */
								&operSet_Bit4inD,
								/* P = 2 */
								&operSet_Bit4inD,
								/* P = 3 */
								&operSet_Bit4inD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit4inD,
								/* P = 1 */
								&operSet_Bit4inD,
								/* P = 2 */
								&operSet_Bit4inD,
								/* P = 3 */
								&operSet_Bit4inD
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit5inD,
								/* P = 1 */
								&operSet_Bit5inD,
								/* P = 2 */
								&operSet_Bit5inD,
								/* P = 3 */
								&operSet_Bit5inD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit5inD,
								/* P = 1 */
								&operSet_Bit5inD,
								/* P = 2 */
								&operSet_Bit5inD,
								/* P = 3 */
								&operSet_Bit5inD
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit6inD,
								/* P = 1 */
								&operSet_Bit6inD,
								/* P = 2 */
								&operSet_Bit6inD,
								/* P = 3 */
								&operSet_Bit6inD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit6inD,
								/* P = 1 */
								&operSet_Bit6inD,
								/* P = 2 */
								&operSet_Bit6inD,
								/* P = 3 */
								&operSet_Bit6inD
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit7inD,
								/* P = 1 */
								&operSet_Bit7inD,
								/* P = 2 */
								&operSet_Bit7inD,
								/* P = 3 */
								&operSet_Bit7inD
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit7inD,
								/* P = 1 */
								&operSet_Bit7inD,
								/* P = 2 */
								&operSet_Bit7inD,
								/* P = 3 */
								&operSet_Bit7inD
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
								&operSet_Bit0inE,
								/* P = 1 */
								&operSet_Bit0inE,
								/* P = 2 */
								&operSet_Bit0inE,
								/* P = 3 */
								&operSet_Bit0inE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit0inE,
								/* P = 1 */
								&operSet_Bit0inE,
								/* P = 2 */
								&operSet_Bit0inE,
								/* P = 3 */
								&operSet_Bit0inE
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit1inE,
								/* P = 1 */
								&operSet_Bit1inE,
								/* P = 2 */
								&operSet_Bit1inE,
								/* P = 3 */
								&operSet_Bit1inE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit1inE,
								/* P = 1 */
								&operSet_Bit1inE,
								/* P = 2 */
								&operSet_Bit1inE,
								/* P = 3 */
								&operSet_Bit1inE
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit2inE,
								/* P = 1 */
								&operSet_Bit2inE,
								/* P = 2 */
								&operSet_Bit2inE,
								/* P = 3 */
								&operSet_Bit2inE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit2inE,
								/* P = 1 */
								&operSet_Bit2inE,
								/* P = 2 */
								&operSet_Bit2inE,
								/* P = 3 */
								&operSet_Bit2inE
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit3inE,
								/* P = 1 */
								&operSet_Bit3inE,
								/* P = 2 */
								&operSet_Bit3inE,
								/* P = 3 */
								&operSet_Bit3inE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit3inE,
								/* P = 1 */
								&operSet_Bit3inE,
								/* P = 2 */
								&operSet_Bit3inE,
								/* P = 3 */
								&operSet_Bit3inE
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit4inE,
								/* P = 1 */
								&operSet_Bit4inE,
								/* P = 2 */
								&operSet_Bit4inE,
								/* P = 3 */
								&operSet_Bit4inE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit4inE,
								/* P = 1 */
								&operSet_Bit4inE,
								/* P = 2 */
								&operSet_Bit4inE,
								/* P = 3 */
								&operSet_Bit4inE
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit5inE,
								/* P = 1 */
								&operSet_Bit5inE,
								/* P = 2 */
								&operSet_Bit5inE,
								/* P = 3 */
								&operSet_Bit5inE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit5inE,
								/* P = 1 */
								&operSet_Bit5inE,
								/* P = 2 */
								&operSet_Bit5inE,
								/* P = 3 */
								&operSet_Bit5inE
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit6inE,
								/* P = 1 */
								&operSet_Bit6inE,
								/* P = 2 */
								&operSet_Bit6inE,
								/* P = 3 */
								&operSet_Bit6inE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit6inE,
								/* P = 1 */
								&operSet_Bit6inE,
								/* P = 2 */
								&operSet_Bit6inE,
								/* P = 3 */
								&operSet_Bit6inE
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit7inE,
								/* P = 1 */
								&operSet_Bit7inE,
								/* P = 2 */
								&operSet_Bit7inE,
								/* P = 3 */
								&operSet_Bit7inE
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit7inE,
								/* P = 1 */
								&operSet_Bit7inE,
								/* P = 2 */
								&operSet_Bit7inE,
								/* P = 3 */
								&operSet_Bit7inE
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
								&operSet_Bit0inH,
								/* P = 1 */
								&operSet_Bit0inH,
								/* P = 2 */
								&operSet_Bit0inH,
								/* P = 3 */
								&operSet_Bit0inH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit0inH,
								/* P = 1 */
								&operSet_Bit0inH,
								/* P = 2 */
								&operSet_Bit0inH,
								/* P = 3 */
								&operSet_Bit0inH
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit1inH,
								/* P = 1 */
								&operSet_Bit1inH,
								/* P = 2 */
								&operSet_Bit1inH,
								/* P = 3 */
								&operSet_Bit1inH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit1inH,
								/* P = 1 */
								&operSet_Bit1inH,
								/* P = 2 */
								&operSet_Bit1inH,
								/* P = 3 */
								&operSet_Bit1inH
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit2inH,
								/* P = 1 */
								&operSet_Bit2inH,
								/* P = 2 */
								&operSet_Bit2inH,
								/* P = 3 */
								&operSet_Bit2inH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit2inH,
								/* P = 1 */
								&operSet_Bit2inH,
								/* P = 2 */
								&operSet_Bit2inH,
								/* P = 3 */
								&operSet_Bit2inH
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit3inH,
								/* P = 1 */
								&operSet_Bit3inH,
								/* P = 2 */
								&operSet_Bit3inH,
								/* P = 3 */
								&operSet_Bit3inH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit3inH,
								/* P = 1 */
								&operSet_Bit3inH,
								/* P = 2 */
								&operSet_Bit3inH,
								/* P = 3 */
								&operSet_Bit3inH
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit4inH,
								/* P = 1 */
								&operSet_Bit4inH,
								/* P = 2 */
								&operSet_Bit4inH,
								/* P = 3 */
								&operSet_Bit4inH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit4inH,
								/* P = 1 */
								&operSet_Bit4inH,
								/* P = 2 */
								&operSet_Bit4inH,
								/* P = 3 */
								&operSet_Bit4inH
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit5inH,
								/* P = 1 */
								&operSet_Bit5inH,
								/* P = 2 */
								&operSet_Bit5inH,
								/* P = 3 */
								&operSet_Bit5inH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit5inH,
								/* P = 1 */
								&operSet_Bit5inH,
								/* P = 2 */
								&operSet_Bit5inH,
								/* P = 3 */
								&operSet_Bit5inH
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit6inH,
								/* P = 1 */
								&operSet_Bit6inH,
								/* P = 2 */
								&operSet_Bit6inH,
								/* P = 3 */
								&operSet_Bit6inH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit6inH,
								/* P = 1 */
								&operSet_Bit6inH,
								/* P = 2 */
								&operSet_Bit6inH,
								/* P = 3 */
								&operSet_Bit6inH
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit7inH,
								/* P = 1 */
								&operSet_Bit7inH,
								/* P = 2 */
								&operSet_Bit7inH,
								/* P = 3 */
								&operSet_Bit7inH
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit7inH,
								/* P = 1 */
								&operSet_Bit7inH,
								/* P = 2 */
								&operSet_Bit7inH,
								/* P = 3 */
								&operSet_Bit7inH
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
								&operSet_Bit0inL,
								/* P = 1 */
								&operSet_Bit0inL,
								/* P = 2 */
								&operSet_Bit0inL,
								/* P = 3 */
								&operSet_Bit0inL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit0inL,
								/* P = 1 */
								&operSet_Bit0inL,
								/* P = 2 */
								&operSet_Bit0inL,
								/* P = 3 */
								&operSet_Bit0inL
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit1inL,
								/* P = 1 */
								&operSet_Bit1inL,
								/* P = 2 */
								&operSet_Bit1inL,
								/* P = 3 */
								&operSet_Bit1inL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit1inL,
								/* P = 1 */
								&operSet_Bit1inL,
								/* P = 2 */
								&operSet_Bit1inL,
								/* P = 3 */
								&operSet_Bit1inL
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit2inL,
								/* P = 1 */
								&operSet_Bit2inL,
								/* P = 2 */
								&operSet_Bit2inL,
								/* P = 3 */
								&operSet_Bit2inL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit2inL,
								/* P = 1 */
								&operSet_Bit2inL,
								/* P = 2 */
								&operSet_Bit2inL,
								/* P = 3 */
								&operSet_Bit2inL
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit3inL,
								/* P = 1 */
								&operSet_Bit3inL,
								/* P = 2 */
								&operSet_Bit3inL,
								/* P = 3 */
								&operSet_Bit3inL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit3inL,
								/* P = 1 */
								&operSet_Bit3inL,
								/* P = 2 */
								&operSet_Bit3inL,
								/* P = 3 */
								&operSet_Bit3inL
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit4inL,
								/* P = 1 */
								&operSet_Bit4inL,
								/* P = 2 */
								&operSet_Bit4inL,
								/* P = 3 */
								&operSet_Bit4inL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit4inL,
								/* P = 1 */
								&operSet_Bit4inL,
								/* P = 2 */
								&operSet_Bit4inL,
								/* P = 3 */
								&operSet_Bit4inL
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit5inL,
								/* P = 1 */
								&operSet_Bit5inL,
								/* P = 2 */
								&operSet_Bit5inL,
								/* P = 3 */
								&operSet_Bit5inL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit5inL,
								/* P = 1 */
								&operSet_Bit5inL,
								/* P = 2 */
								&operSet_Bit5inL,
								/* P = 3 */
								&operSet_Bit5inL
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit6inL,
								/* P = 1 */
								&operSet_Bit6inL,
								/* P = 2 */
								&operSet_Bit6inL,
								/* P = 3 */
								&operSet_Bit6inL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit6inL,
								/* P = 1 */
								&operSet_Bit6inL,
								/* P = 2 */
								&operSet_Bit6inL,
								/* P = 3 */
								&operSet_Bit6inL
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit7inL,
								/* P = 1 */
								&operSet_Bit7inL,
								/* P = 2 */
								&operSet_Bit7inL,
								/* P = 3 */
								&operSet_Bit7inL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit7inL,
								/* P = 1 */
								&operSet_Bit7inL,
								/* P = 2 */
								&operSet_Bit7inL,
								/* P = 3 */
								&operSet_Bit7inL
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
								&operSet_Bit0inPAddrHL,
								/* P = 1 */
								&operSet_Bit0inPAddrHL,
								/* P = 2 */
								&operSet_Bit0inPAddrHL,
								/* P = 3 */
								&operSet_Bit0inPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit0inPAddrHL,
								/* P = 1 */
								&operSet_Bit0inPAddrHL,
								/* P = 2 */
								&operSet_Bit0inPAddrHL,
								/* P = 3 */
								&operSet_Bit0inPAddrHL
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit1inPAddrHL,
								/* P = 1 */
								&operSet_Bit1inPAddrHL,
								/* P = 2 */
								&operSet_Bit1inPAddrHL,
								/* P = 3 */
								&operSet_Bit1inPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit1inPAddrHL,
								/* P = 1 */
								&operSet_Bit1inPAddrHL,
								/* P = 2 */
								&operSet_Bit1inPAddrHL,
								/* P = 3 */
								&operSet_Bit1inPAddrHL
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit2inPAddrHL,
								/* P = 1 */
								&operSet_Bit2inPAddrHL,
								/* P = 2 */
								&operSet_Bit2inPAddrHL,
								/* P = 3 */
								&operSet_Bit2inPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit2inPAddrHL,
								/* P = 1 */
								&operSet_Bit2inPAddrHL,
								/* P = 2 */
								&operSet_Bit2inPAddrHL,
								/* P = 3 */
								&operSet_Bit2inPAddrHL
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit3inPAddrHL,
								/* P = 1 */
								&operSet_Bit3inPAddrHL,
								/* P = 2 */
								&operSet_Bit3inPAddrHL,
								/* P = 3 */
								&operSet_Bit3inPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit3inPAddrHL,
								/* P = 1 */
								&operSet_Bit3inPAddrHL,
								/* P = 2 */
								&operSet_Bit3inPAddrHL,
								/* P = 3 */
								&operSet_Bit3inPAddrHL
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit4inPAddrHL,
								/* P = 1 */
								&operSet_Bit4inPAddrHL,
								/* P = 2 */
								&operSet_Bit4inPAddrHL,
								/* P = 3 */
								&operSet_Bit4inPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit4inPAddrHL,
								/* P = 1 */
								&operSet_Bit4inPAddrHL,
								/* P = 2 */
								&operSet_Bit4inPAddrHL,
								/* P = 3 */
								&operSet_Bit4inPAddrHL
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit5inPAddrHL,
								/* P = 1 */
								&operSet_Bit5inPAddrHL,
								/* P = 2 */
								&operSet_Bit5inPAddrHL,
								/* P = 3 */
								&operSet_Bit5inPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit5inPAddrHL,
								/* P = 1 */
								&operSet_Bit5inPAddrHL,
								/* P = 2 */
								&operSet_Bit5inPAddrHL,
								/* P = 3 */
								&operSet_Bit5inPAddrHL
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit6inPAddrHL,
								/* P = 1 */
								&operSet_Bit6inPAddrHL,
								/* P = 2 */
								&operSet_Bit6inPAddrHL,
								/* P = 3 */
								&operSet_Bit6inPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit6inPAddrHL,
								/* P = 1 */
								&operSet_Bit6inPAddrHL,
								/* P = 2 */
								&operSet_Bit6inPAddrHL,
								/* P = 3 */
								&operSet_Bit6inPAddrHL
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit7inPAddrHL,
								/* P = 1 */
								&operSet_Bit7inPAddrHL,
								/* P = 2 */
								&operSet_Bit7inPAddrHL,
								/* P = 3 */
								&operSet_Bit7inPAddrHL
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit7inPAddrHL,
								/* P = 1 */
								&operSet_Bit7inPAddrHL,
								/* P = 2 */
								&operSet_Bit7inPAddrHL,
								/* P = 3 */
								&operSet_Bit7inPAddrHL
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
								&operSet_Bit0inA,
								/* P = 1 */
								&operSet_Bit0inA,
								/* P = 2 */
								&operSet_Bit0inA,
								/* P = 3 */
								&operSet_Bit0inA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit0inA,
								/* P = 1 */
								&operSet_Bit0inA,
								/* P = 2 */
								&operSet_Bit0inA,
								/* P = 3 */
								&operSet_Bit0inA
							}
						},
						/* Y = 1 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit1inA,
								/* P = 1 */
								&operSet_Bit1inA,
								/* P = 2 */
								&operSet_Bit1inA,
								/* P = 3 */
								&operSet_Bit1inA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit1inA,
								/* P = 1 */
								&operSet_Bit1inA,
								/* P = 2 */
								&operSet_Bit1inA,
								/* P = 3 */
								&operSet_Bit1inA
							}
						},
						/* Y = 2 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit1inA,
								/* P = 1 */
								&operSet_Bit1inA,
								/* P = 2 */
								&operSet_Bit1inA,
								/* P = 3 */
								&operSet_Bit1inA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit1inA,
								/* P = 1 */
								&operSet_Bit1inA,
								/* P = 2 */
								&operSet_Bit1inA,
								/* P = 3 */
								&operSet_Bit1inA
							}
						},
						/* Y = 3 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit3inA,
								/* P = 1 */
								&operSet_Bit3inA,
								/* P = 2 */
								&operSet_Bit3inA,
								/* P = 3 */
								&operSet_Bit3inA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit3inA,
								/* P = 1 */
								&operSet_Bit3inA,
								/* P = 2 */
								&operSet_Bit3inA,
								/* P = 3 */
								&operSet_Bit3inA
							}
						},
						/* Y = 4 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit4inA,
								/* P = 1 */
								&operSet_Bit4inA,
								/* P = 2 */
								&operSet_Bit4inA,
								/* P = 3 */
								&operSet_Bit4inA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit4inA,
								/* P = 1 */
								&operSet_Bit4inA,
								/* P = 2 */
								&operSet_Bit4inA,
								/* P = 3 */
								&operSet_Bit4inA
							}
						},
						/* Y = 5 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit5inA,
								/* P = 1 */
								&operSet_Bit5inA,
								/* P = 2 */
								&operSet_Bit5inA,
								/* P = 3 */
								&operSet_Bit5inA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit5inA,
								/* P = 1 */
								&operSet_Bit5inA,
								/* P = 2 */
								&operSet_Bit5inA,
								/* P = 3 */
								&operSet_Bit5inA
							}
						},
						/* Y = 6 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit6inA,
								/* P = 1 */
								&operSet_Bit6inA,
								/* P = 2 */
								&operSet_Bit6inA,
								/* P = 3 */
								&operSet_Bit6inA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit6inA,
								/* P = 1 */
								&operSet_Bit6inA,
								/* P = 2 */
								&operSet_Bit6inA,
								/* P = 3 */
								&operSet_Bit6inA
							}
						},
						/* Y = 7 */
						{
							/* Q = 0 */
							{
								/* P = 0 */
								&operSet_Bit7inA,
								/* P = 1 */
								&operSet_Bit7inA,
								/* P = 2 */
								&operSet_Bit7inA,
								/* P = 3 */
								&operSet_Bit7inA
							},
							/* Q = 1 */
							{
								/* P = 0 */
								&operSet_Bit7inA,
								/* P = 1 */
								&operSet_Bit7inA,
								/* P = 2 */
								&operSet_Bit7inA,
								/* P = 3 */
								&operSet_Bit7inA
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

		void executeOperation(const Opcode& opcode)
		// may throw ExceptionInvalidOpcode
		{
			Chrono c;
			// TODO: isValid seems to always return true ...
			if (opcode.isValid())
			{
				this->operations[this->getPrefixIndex(opcode.getPrefixFamily())]
				[opcode.getX()][opcode.getZ()][opcode.getY()]
				[opcode.getQ()][opcode.getP()](*this, c);

				// TO DO: Add cycles handler
			}
			else
				throw ExceptionInvalidOpcode();
			c.waitForCycles();
		}

		public:

		CPUZ80(const mem_t& memory_type)
		: CPU<mem_t, reg_t, flags_t>(memory_type, Z80Registers(0XFFFE, 0x100), (flags_t)0)
		{
			/* ... */
		}

		~CPUZ80()
		{ }
    };
}
