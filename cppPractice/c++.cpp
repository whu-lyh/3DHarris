
//parallel library test
#include <tbb/tbb.h>

#include <Windows.h>
#include <direct.h>

#include <string>
#include <iostream>
#include <istream>

#include <algorithm>

#include <vector>
#include <list>
#include <array>
#include <queue>
#include <deque>
#include <set>
#include <unordered_set>
#include <map>
#include <unordered_map>
/*
<hash_set> is deprecated and will be REMOVED. "
"Please use <unordered_set>. You can define "
"_SILENCE_STDEXT_HASH_DEPRECATION_WARNINGS "
"to acknowledge that you have received this warning.
*/
//#include <hash_set>
//#include <hash_map>

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
//#define VECTOR_TEST
//#define MAP_TEST
//#define SET_TEST
#define SET_TEST_2
//#define VECTOR_TEST_2
//#define DEQUE_TEST
//#define LIST_TEST

using namespace std;

#ifdef VECTOR_TEST
inline void printInfo(vector<int> iv) {
	cout << "size=" << iv.size() << " capacity=" << iv.capacity() << endl;
}

void printVector(vector<int> iv) {
	for (int i = 0; i < iv.size(); i++) {
		cout << iv[i] << " ";
	}
	cout << endl;
}
#endif

#if defined(SET_TEST_2) || defined(VECTOR_TEST_2)
struct frame{
	std::string imagename;
	double gpstime;
	int imageid;
	double x;
	double y;
	double z;
	double phi;
	double omega;
	double kappa;

public:
	//需要把运算符重载函数声明为类的友元函数，这样我们就能不用创建对象而直接调用函数。
	friend std::ostream& operator << (std::ostream &os,const frame &frame_)
	{
		os << frame_.imagename << "," << frame_.gpstime << "," << frame_.imageid <<
			frame_.x << "," << frame_.y << "," << frame_.z << std::endl;
		return os;
	}

	// std::set没有任何东西使用operator==。它只使用operator<
	// set(associative container) is known more as a adapter than a container, in the aspect of lower technical perspect
	// set is based on a dynamic balance binary tree (or also called red-black tree)
	// vector(sequence container) is dynamic array which is different to the set and vector will not call the operator <
	bool operator <(const frame & frame_) const
	{
		return this->imageid < frame_.imageid;
	}

	// Question, any func calls operator ()?
	// if extra function for compare is used, then the operator() should be override, more detail in example

	// but the std::find in algorithm function call the == operator
	bool operator ==(const frame & frame_) const
	{
		return this->imageid == frame_.imageid;
	}
};

class Compare
{
public:
	bool operator()(const frame& f1, const frame& f2) const
	{
		if (f1.imageid == f2.imageid)
			return f1.gpstime > f2.gpstime;
		else
			return f1.gpstime > f2.gpstime;
	}
};

#endif //SET_TEST_2

int main()
{
	char buffer[256];
	char *val = _getcwd(buffer, sizeof(buffer));
	if (val)
	{
		std::cout << "current position: " << buffer << std::endl;
	}

#ifdef VECTOR_TEST
	int i;
	vector<int> iv(2, 9);
	printInfo(iv);

	for (int i = 1; i < 5; i++) {
		iv.push_back(i);
		printInfo(iv);
	}

	printVector(iv);

	iv.push_back(i);
	printInfo(iv);
	printVector(iv);

	iv.pop_back();
	iv.pop_back();
	printInfo(iv);

	iv.pop_back();
	printInfo(iv);

	vector<int>::iterator ivite = find(iv.begin(), iv.end(), 1);
	if (ivite != iv.end())
		iv.erase(ivite);
	printInfo(iv);
	printVector(iv);

	ivite = find(iv.begin(), iv.end(), 2);
	if (ivite != iv.end())
		iv.insert(ivite, 3, 7);
	printInfo(iv);
	printVector(iv);

	iv.clear();
	printInfo(iv);
#endif

#ifdef MAP_TEST
	map<string, int> simap;
	simap[string("jjhou")] = 1;
	simap[string("jerry")] = 2;
	simap[string("jason")] = 3;
	simap[string("jimmy")] = 4;

	pair<string, int> value(string("david"), 5);
	simap.insert(value);

	map<string, int>::iterator simap_iter = simap.begin();
	for (; simap_iter != simap.end(); ++simap_iter)
		cout << simap_iter->first << ' '
		<< simap_iter->second << endl;

	int number = simap[string("jjhou")];
	cout << number << endl;

	map<string, int>::iterator ite1;
	// 对于关联式容器，专用find函数比STL算法find效率更高
	ite1 = simap.find(string("mchen"));
	if (ite1 == simap.end())
		cout << "mchen not found" << endl;

	ite1 = simap.find(string("jerry"));
	if (ite1 != simap.end())
		cout << "jerry found" << endl;

	ite1->second = 9; // 可以修改value
	int number2 = simap[string("jerry")];
	cout << number2 << endl;
#endif

#ifdef DEQUE_TEST
	// allocator, default allocator or custom allocator
	deque<int, allocator<int>> ideq(20, 9);
	cout << "size=" << ideq.size() << endl;

	for (int i = 0; i < ideq.size(); ++i)
		cout << ideq[i] << ' ';
	cout << endl;

	for (int i = 0; i < 3; ++i)
		ideq.push_back(i);

	for (int i = 0; i < ideq.size(); ++i)
		cout << ideq[i] << ' ';
	cout << endl;
	cout << "size=" << ideq.size() << endl;

	ideq.push_back(3);
	for (int i = 0; i < ideq.size(); ++i)
		cout << ideq[i] << ' ';
	cout << endl;
	cout << "size=" << ideq.size() << endl;

	ideq.push_front(99);
	for (int i = 0; i < ideq.size(); ++i)
		cout << ideq[i] << ' ';
	cout << endl;
	cout << "size=" << ideq.size() << endl;

	ideq.push_front(98);
	ideq.push_front(97);
	for (int i = 0; i < ideq.size(); ++i)
		cout << ideq[i] << ' ';
	cout << endl;
	cout << "size=" << ideq.size() << endl;

	deque<int, allocator<int>>::iterator itr;
	itr = find(ideq.begin(), ideq.end(), 99);
	cout << *itr << endl;
	// ?
	//cout << *(itr._M_cur) << endl;
#endif

#ifdef SET_TEST
	//int i;
	int ia[5] = { 0, 1, 2, 3, 4 };
	set<int> iset{ ia, ia + 5 };
	// for extra operator ()
	set<int,Compare> iset2{ ia, ia + 5 };

	cout << "size=" << iset.size() << endl;
	cout << "3 count =" << iset.count(3) << endl;
	iset.insert(3);
	cout << "size=" << iset.size() << endl;
	cout << "3 count =" << iset.count(3) << endl;

	iset.insert(5);
	cout << "size=" << iset.size() << endl;
	cout << "3 count =" << iset.count(3) << endl;

	iset.erase(1);
	cout << "size=" << iset.size() << endl;
	cout << "3 count =" << iset.count(3) << endl;
	cout << "1 count =" << iset.count(1) << endl;

	set<int>::iterator ite1 = iset.begin();
	set<int>::iterator ite2 = iset.end();
	for (; ite1 != ite2; ++ite1) {
		cout << *ite1;
	}
	cout << endl;

	// 使用STL算法find可以搜索元素，但不推荐
	ite1 = find(iset.begin(), iset.end(), 3);
	if (ite1 != iset.end())
		cout << "3 found" << endl;

	ite1 = find(iset.begin(), iset.end(), 1);
	if (ite1 == iset.end())
		cout << "1 not found" << endl;

	// 关联式容器应使用专用的find函数搜索更有效率
	ite1 = iset.find(3);
	if (ite1 != iset.end())
		cout << "3 found" << endl;

	ite1 = iset.find(1);
	if (ite1 == iset.end())
		cout << "1 not found" << endl;

	// *ite1 = 9; // 修改失败
#endif

#ifdef SET_TEST_2

	std::set<frame> frame_set;
	for (int i = 0; i < 10; i++)
	{
		frame tmp_frame;
		tmp_frame.imagename = "imagename" + std::to_string(i) + ".png";
		tmp_frame.gpstime = 13875 + 2 * i;
		tmp_frame.imageid = i;
		tmp_frame.x = 3458131.5068 + 2 * i;
		tmp_frame.y = 639438.0068 + 2 * i;
		tmp_frame.z = 15.281 + 2 * i;
		tmp_frame.phi = 0.31508 + 3 * i;
		tmp_frame.omega = 1.80411 + 3 * i;
		tmp_frame.kappa = 118.2008 + 3 * i;
		frame_set.insert(tmp_frame);
	}

	set<frame>::iterator ite1 = frame_set.begin();
	set<frame>::iterator ite2 = frame_set.end();
	for (; ite1 != ite2; ++ite1) {
		// correct
		//cout << (ite1->imagename) << endl;
		// failure
		cout << *ite1 << endl;
	}
	cout << endl;

	// 使用STL算法find可以搜索元素，但不推荐
	frame tgt_frame;
	tgt_frame.imageid = 6;
	tgt_frame.gpstime = 13875 + 12;
	//call both < and ==
	ite1 = find(frame_set.begin(), frame_set.end(), tgt_frame);
	if (ite1 != frame_set.end())
		cout << "found" << endl;

	// only call <
	// 关联式容器应使用专用的find函数搜索更有效率
	ite1 = frame_set.find(tgt_frame);
	if (ite1 != frame_set.end())
		cout << "2 found" << endl;

#endif

#ifdef VECTOR_TEST_2

	std::vector<frame> frame_vec;
	for (int i = 0; i < 10; i++)
	{
		frame tmp_frame;
		tmp_frame.imagename = "imagename" + std::to_string(i) + ".png";
		tmp_frame.gpstime = 13875 + 2 * i;
		tmp_frame.imageid = i;
		tmp_frame.x = 3458131.5068 + 2 * i;
		tmp_frame.y = 639438.0068 + 2 * i;
		tmp_frame.z = 15.281 + 2 * i;
		tmp_frame.phi = 0.31508 + 3 * i;
		tmp_frame.omega = 1.80411 + 3 * i;
		tmp_frame.kappa = 118.2008 + 3 * i;
		frame_vec.push_back(tmp_frame);
	}

	vector<frame>::iterator ite1 = frame_vec.begin();
	vector<frame>::iterator ite2 = frame_vec.end();
	for (; ite1 != ite2; ++ite1) {
		// correct
		//cout << (ite1->imagename) << endl;
		// failure
		cout << *ite1 << endl;
	}
	cout << endl;

	// 使用STL算法find可以搜索元素，但不推荐
	frame tgt_frame;
	tgt_frame.imageid = 6;
	tgt_frame.gpstime = 13875 + 12;
	//call both < and ==
	ite1 = find(frame_vec.begin(), frame_vec.end(), tgt_frame);
	if (ite1 != frame_vec.end())
		cout << "found" << endl;

#endif

#ifdef LIST_TEST
	int i;
	list<int> ilist;
	cout << "size=" << ilist.size() << endl;

	for (int j = 0; j < 5; j++) {
		ilist.push_back(j);
}
	cout << "size=" << ilist.size() << endl;

	list<int>::iterator ite;
	for (ite = ilist.begin(); ite != ilist.end(); ++ite) {
		cout << *ite << ' ';
	}
	cout << endl;

	ite = find(ilist.begin(), ilist.end(), 3);
	if (ite != ilist.end()) {
		ilist.insert(ite, 99);
	}
	cout << "size=" << ilist.size() << endl;
	cout << *ite << endl;

	for (ite = ilist.begin(); ite != ilist.end(); ++ite) {
		cout << *ite << ' ';
	}
	cout << endl;

	ite = find(ilist.begin(), ilist.end(), 1);
	if (ite != ilist.end()) {
		cout << *(ilist.erase(ite)) << endl;
	}

	for (ite = ilist.begin(); ite != ilist.end(); ++ite) {
		cout << *ite << ' ';
	}
	cout << endl;

	int iv[5] = { 5,6,7,8,9 };
	list<int> ilist2(iv, iv + 5);
	//目前，ilist的内容为 0 2 99 3 4
	ite = find(ilist.begin(), ilist.end(), 99);
	ilist.splice(ite, ilist2);
	ilist.reverse();
	ilist.sort();


#endif

#ifdef BOOST_TEST
	// serialization = convert the object to a byte stream 
#include <boost/archive/text_oarchive.hpp> //文本格式输入存档
#include <boost/archive/text_iarchive.hpp> //文本格式输出存档
#include <boost/serialization/vector.hpp>  //vector的序列化实现头文件
	using namespace boost:archive;//打开名称空间

	// more detail in the future

#endif

#ifdef TBB_TEST
	//tbb parallel
	tbb::parallel_for(0, 10, [](int num) {std::cout << num << " : hello tbb " << std::endl; });

#endif

#ifdef CPLUSPLUS
	std::cout << std::fixed;
	//std::cout << std::setprecision(10);

	std::cout << sizeof(short int) << std::endl;
	std::cout << sizeof(char) << std::endl;
	std::cout << sizeof(bool) << std::endl;

	double zero_d = 0.0;
	float zero_f = 0.f;
	if (zero_d == zero_f)
	{
		std::cout << "equal!" << std::endl;
	}

	//float 6 valid value will be keeped(the 1 will be saved)
	float slamm = 0.000111111;
	std::cout << slamm << std::endl;

	double offset_x = 3453464.397;
	std::cout << offset_x << std::endl;
	float offset_xf = offset_x;
	std::cout << offset_xf << std::endl;

	vector<short int> occupation_grid;
	occupation_grid.reserve(40000000);
	occupation_grid.resize(40000000, false);
	//for (auto singlegrid:occupation_grid)
	//{
	//	std::cout << singlegrid << std::endl;
	//}

#endif // CPLUSPLUS

	system("pause");
	return true;
}