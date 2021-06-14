#pragma once

#include <stdexcept>

#include <deque>

#include <emulation/IMMU.hpp>

namespace emulation
{
	/**
	 * @brief Paged memory manager.
	 *
	 * @tparam Data:	Data type.
	 * @tparam Size:	Size type.
	 * @tparam size:	Total size of the virtual address space.
	 */
	template<typename Data, typename Size, Size size>
	class	PagedMemory:	public IMMU<Data, Size>
	{
	public:
		/// Memory unit type
		typedef IMMU<Data, Size>	unit_type;
		/// Memory size type
		typedef Size				size_type;

	protected:
		class	Page:	unit_type
		{
		private:
			// Offset in virtual memory space
			Size		offset;
			// Underlying Memory Management Unit
			unit_type*	data;

		public:
			/**
			 * @brief Construct a new Page with the given offset.
			 *
			 * @param offset:	Offset in the virtual memory address space.
			 * @param data:		The Memory Management Unit to map.
			 */
			Page(Size offset, unit_type* data) noexcept
				:	offset(offset),
					data(data)
			{ }

			/**
			 * @brief Check if the page contains a given virtual address.
			 *
			 * @param address:	The address in the virtual memory space.
			 * @return true if the virtual address is contained.
			 */
			bool	contains(Size address) const noexcept
			{ return address - offset <= getSize(); }

			Data	read(Size address) const
			{ return data->read(address - offset); }

			void	write(Size address, Data value)
			{ data->write(address - offset, value); }

			/**
			 * @brief Get the page's offset in virtual memory.
			 */
			Size	getOffset() const noexcept
			{ return offset; }

			Size	getSize() const noexcept
			{ return data->getSize(); }
		};

	private:
		/// Memory page container type
		typedef	std::deque<Page>	page_container;

		page_container	pages;

		/**
		 * @brief Get the page's iterator for the given offset.
		 *
		 * @param address:	Address in the virtual memory space.
		 */
		typename page_container::iterator		getPage(Size address) noexcept
		{
			typename page_container::iterator	it;

			for (it = pages.begin(); it != pages.end() && !it->contains(address); it++);

			return it;
		}

		/**
		 * @brief Get the page's const iterator for the given offset.
		 *
		 * @param address:	Address in the virtual memory space.
		 */
		typename page_container::const_iterator	getPage(Size address) const noexcept
		{
			typename page_container::const_iterator	it;

			for (it = pages.begin(); it != pages.end() && !it->contains(address); it++);

			return it;
		}


	public:
		/**
		 * @brief Construct a new Paged Memory object.
		 */
		PagedMemory()
		{ }

		/**
		 * @brief Read Data at the given address.
		 *
		 * @param address:	The address in virtual memory space.
		 * @return Data
		 */
		Data	read(Size address) const
		{
			typename page_container::const_iterator	it = getPage(address);

			if (it == pages.end())
				throw segmentation_fault();

			return it->read(address);
		}

		/**
		 * @brief Write Data at the given address.
		 *
		 * @param address:	The address in virtual memory space.
		 * @param value:	The value to write.
		 */
		void	write(Size address, Data value)
		{
			typename page_container::iterator	it = getPage(address);

			if (it == pages.end())
				throw segmentation_fault();

			it->write(address, value);
		}

		/**
		 * @brief Map a memory unit to a given offset.
		 *
		 * @param offset:	Offset in the virtual memory space.
		 * @param unit:		Memory unit to be mapped.
		 */
		void	map(Size offset, unit_type* unit)

		{
			if (offset > size)
				throw std::out_of_range("Cannot map: offset exceeds maximal size");

			typename page_container::iterator	it = getPage(offset);

			if (it != pages.end())
				throw std::logic_error("Cannot map: offset overlaps with an already mapped range");

			for (it = pages.begin(); it != pages.end() && it->getOffset() > offset; it++);

			pages.insert(it, Page(offset, unit));
		}

		/**
		 * @brief Unmap a memory unit at a given offset.
		 *
		 * Note: The memory unit is not deleted!
		 *
		 * @param offset:	Offset in the virtual memory space.
		 */
		void	unmap(Size offset)
		{
			typename page_container::iterator	it = getPage(offset);

			if (it == pages.end())
				throw std::logic_error("Cannot unmap: offset has not been mapped yet");

			it = pages.erase(it);
		}

		/**
		 * @brief Check if a specific address has been mapped
		 *
		 * @param address:	Address in the virtual memory space.
		 * @return true if mapped.
		 */
		bool	isMapped(Size address)
		{ return getPage(address) != pages.end(); };

		Size	getSize() const noexcept
		{ return size; }
	};
}
