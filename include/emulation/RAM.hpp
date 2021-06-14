#pragma once

#include <cstddef>
#include <cstring>

#include <emulation/IMMU.hpp>

#include <iostream>

namespace emulation
{
	template<typename Data = uint8_t, typename Size = std::uintptr_t>
	class RAM:	public IMMU<Data>
	{
	private:
		Data*	data;
		Size	size;

	public:
		RAM()
			:	data(nullptr),
				size(0)
		{
		}

		RAM(Size size)
			:	data(new Data[size]),
				size(size)
		{
			std::cerr << "Creating RAM of size " << size << "!" << std::endl;
		}

		Data	read(Size address) const
		{
			if (address > size)
				throw segmentation_fault();

			return data[address];
		}

		void	write(Size address, Data value)
		{
			if (address > size)
				throw segmentation_fault();

			this->data[address] = value;
		}

		Size	getSize() const noexcept
		{ return size; }

		RAM&	operator=(RAM const& rhs)
		{
			if (this != &rhs)
			{
				if (size != rhs.size)
				{
					delete[] data;
					data = new Data[rhs.size];
					size = rhs.size;
				}
				std::memcpy(data, rhs.data, size);
			}
		}

		virtual ~RAM()
		{
			delete[] data;
		}
	};
}
