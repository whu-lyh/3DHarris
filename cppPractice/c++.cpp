
//parallel library test
#include <tbb/tbb.h>

#include <vector>

#ifdef _DEBUG
#pragma comment(lib, "libboost_thread-vc140-mt-gd-x64-1_67.lib")
#pragma comment(lib, "tbb_debug.lib")
#else
#pragma comment(lib, "libboost_thread-vc140-mt-x64-1_67.lib")
#pragma comment(lib, "tbb.lib")
#endif

//#define CPLUSPLUS
//#define TBB_TEST
//#define BOOST_TEST

int main()
{

#ifdef BOOST_TEST


#endif

#ifdef TBB_TEST
	//tbb parallel
	tbb::parallel_for(0, 10, [](int num) {std::cout << num << " : hello tbb " << std::endl; });

#endif

#ifdef CPLUSPLUS

	std::cout << sizeof(short int) << std::endl;
	std::cout << sizeof(char) << std::endl;
	std::cout << sizeof(bool) << std::endl;

	//float 6 valid value will be keeped(the 1 will be saved)
	float slamm = 0.000111111;
	std::cout << slamm << std::endl;

	vector<short int> occupation_grid;
	occupation_grid.reserve(40000000);
	occupation_grid.resize(40000000, false);
	//for (auto singlegrid:occupation_grid)
	//{
	//	std::cout << singlegrid << std::endl;
	//}

#endif // CPLUSPLUS

	return true;
}