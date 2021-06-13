#pragma once

#include <cstddef>
#include <cstring>

#include <emulation/IMMU.hpp>

namespace emulation
{
	template<typename Data = uint8_t, typename Size = std::uintptr_t>
	class ARAM:	public IMMU<Data>
	{
	private:
		Data*	data;
		Size	size;

	public:
		ARAM()
			:	data(nullptr),
				size(0)
		{
		}

		ARAM(Size size)
			:	data(new Data[size]),
				size(size)
		{
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

		ARAM&	operator=(ARAM const& rhs)
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

		virtual ~ARAM()
		{
			delete[] data;
		}
	};
}
