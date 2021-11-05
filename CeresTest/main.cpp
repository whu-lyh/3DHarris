
#include <ceres/ceres.h>

#ifdef _DEBUG
#pragma comment(lib, "ceres-debug.lib")
#else
#pragma comment(lib, "ceres.lib")
#endif

class optimizer
{
public:
	typedef std::shared_ptr<optimizer> Ptr;
	optimizer()
	{
		m_problem = std::make_shared<ceres::Problem>();
		m_summary = std::make_shared<ceres::Solver::Summary>();
	}
	~optimizer(){}

	bool exec()
	{
		ceres::Solve(*m_options, m_problem.get(), m_summary.get());
		return true;
	}

public:
	ceres::Problem::Options m_problem_option;
	std::shared_ptr<ceres::Problem> m_problem;
	std::shared_ptr<ceres::Solver::Options> m_options; // do not use std::shared_ptr
	std::shared_ptr<ceres::Solver::Summary> m_summary;
};


int main()
{
	optimizer::Ptr op = std::make_shared<optimizer>();
	op->exec();
	std::cout << op->m_summary->FullReport();
	system("pause");
	return 1;
}