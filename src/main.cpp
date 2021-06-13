#include <emulation/PagedMemory.hpp>

#include <emulation/ARAM.hpp>

#include <iostream>

int	main()
{
	emulation::PagedMemory<>	pagedMemory(16);
	emulation::ARAM<>			ram(8);


	pagedMemory.map(8, &ram);

	ram.write(0, 1);
	ram.write(1, 2);

	std::cout << (int)pagedMemory.read(8 + 0) << std::endl;
	std::cout << (int)pagedMemory.read(8 + 1) << std::endl;

	return 0;
}
