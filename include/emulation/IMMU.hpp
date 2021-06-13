#pragma once

#include <cstdint>

namespace	emulation
{
	/// To be thrown to emulate segmentation fault
	class segmentation_fault
	{ };

	/**
	 * @brief Memory Management Unit
	 *
	 * @tparam Data:	Data type.
	 * @tparam Size:	Size type.
	 */
	template<typename Data = uint8_t, typename Size = std::uintptr_t>
	class IMMU
	{
	public:
		virtual	~IMMU() { };

		/**
		 * @brief Read Data at the given address.
		 *
		 * @param address:	The address in virtual memory space.
		 * @return Data
		 */
		virtual Data	read(Size address) const = 0;

		/**
		 * @brief Write Data at the given address.
		 *
		 * @param address:	The address in virtual memory space.
		 * @param value:	The value to write.
		 */
		virtual void	write(Size address, Data value) = 0;

		/**
		 * @brief Get the virtual memory space size.
		 *
		 * @return Size
		 */
		virtual Size	getSize() const noexcept = 0;
	};
}
