#pragma once

#include <cstring>

#include <emulation/IMMU.hpp>

namespace emulation
{
	template<typename Data, typename Size, Size size>
	class RAM:	public IMMU<Data, Size>
	{
	private:
		Data*	data;

	public:
		RAM()
			:	data(new Data[size])
		{ }

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
