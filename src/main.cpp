#include <emulation/PagedMemory.hpp>

#include <emulation/BankedMemory.hpp>

#include <iostream>

int	main()
{
	emulation::PagedMemory<std::uint8_t, std::uintptr_t, 8>		paged;
	emulation::BankedMemory<std::uint8_t, std::uintptr_t, 2, 8>	banked;

	paged.map(8, &banked);

	std::cout << "Writing 1 to bank 0..." << std::endl;
	banked.write(0, 1);
	banked.switchTo(1);
	std::cout << "Writing 2 to bank 1..." << std::endl;
	paged.write(8, 2);

	std::cout << "Reading from bank 1: " << (int)paged.read(8) << std::endl;
	banked.switchTo(0);
	std::cout << "Reading from bank 0: " << (int)paged.read(8) << std::endl;

	return 0;
}
