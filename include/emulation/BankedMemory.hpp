#pragma once

#include <stdexcept>

#include <array>

#include <emulation/IMMU.hpp>

#include <emulation/RAM.hpp>

namespace	emulation
{
	/**
	 * @brief Switchable bank memory.
	 *
	 * @tparam Data:		Data type.
	 * @tparam Size:		Size type.
	 * @tparam bankCount:	Bank count available for switching.
	 * @tparam size:		Size of each memory bank.
	 */
	template<typename Data, typename Size, unsigned bankCount, Size size>
	class	BankedMemory:	public IMMU<Data, Size>
	{
	private:
		/// Memory unit type
		typedef RAM<Data, Size, size>				unit_type;
		/// Memory bank container type
		typedef std::array<unit_type, bankCount>	bank_container;

		/// Array of memory banks
		bank_container	banks;
		/// The currently selected memory bank
		unsigned		index;

	public:
		/**
		 * @brief Construct a new Banked Memory object.
		 */
		BankedMemory()
			:	banks(),
				index()
		{}

		/**
		 * @brief Read from the currently selected memory bank.
		 *
		 * @param address:	The address in virtual memory.
		 * @return Data
		 */
		Data	read(Size address) const noexcept(false)
		{ return banks[index].read(address); }

		/**
		 * @brief Write to the currently selected memory bank.
		 *
		 * @param address:	The address in virtual memory.
		 * @param value:	The value to write.
		 */
		void	write(Size address, Data value) noexcept(false)
		{ banks[index].write(address, value); }

		/**
		 * @brief Switch to another memory bank.
		 *
		 * @param newIndex:	The new memory bank's index.
		 */
		void	switchTo(unsigned newIndex)
			noexcept(false)
		{
			if (newIndex > banks.size())
				throw std::out_of_range("Memory bank index out of range");
			index = newIndex;
		}

		Size	getSize() const noexcept(true)
		{ return size; }
	};
}
