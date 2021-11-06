
#include <ceres/ceres.h>

#ifdef _DEBUG
#pragma comment(lib, "glogd.lib")
#pragma comment(lib, "ceres-debug.lib")
#else
#pragma comment(lib, "glog.lib")
#pragma comment(lib, "ceres.lib")
#endif

struct QuadraticAutoDiffCostFunction
{
	QuadraticAutoDiffCostFunction(double x_, double y_)
		:x(x_), y(y_)
	{}
	template<typename T>
	bool operator()(const T* const p, T* residual) const
	{
		residual[0] = (T)y - p[0] * (T)x * (T)x - p[1] * (T)x - p[2];
		return true;
	}
private:
	const double x;
	const double y;
};

class MyClass
{
public:
	typedef std::shared_ptr<MyClass> Ptr;
	MyClass(){};
	~MyClass(){};

public:
	double parameters[3] = { 0.0,0.0,0.0 };
};

class optimizer
{
public:
	typedef std::shared_ptr<optimizer> Ptr;
	optimizer()
	{
		m_problem_options = std::make_shared<ceres::Problem::Options>();
		m_problem_options->loss_function_ownership = ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
		m_options = std::make_shared<ceres::Solver::Options>();
		m_problem = std::make_shared<ceres::Problem>(*m_problem_options);
		m_summary = std::make_shared<ceres::Solver::Summary>();
		m_robust_kernel_function = std::make_shared<ceres::HuberLoss>(0.5);
	}
	~optimizer(){}

	bool exec()
	{
		MyClass::Ptr tmp = std::make_shared<MyClass>();
		double ar = 1.0, br = 2.0, cr = 1.0;
		double ae = 2.0, be = -1.0, ce = 5.0;
		int N = 100;
		double w_sigma = 1.0;
		double inv_sigma = 1.0 / w_sigma;
		m_problem->AddParameterBlock(tmp->parameters, 3);
		std::vector<double> x_data, y_data;
		for (int i = 0; i < N; i++) 
		{
			double x = i / 100.0;
			x_data.push_back(x);
			y_data.push_back(exp(ar * x * x + br * x + cr) + rand() / (1.0 + RAND_MAX));

			ceres::CostFunction* cost_function =
				new ceres::AutoDiffCostFunction<QuadraticAutoDiffCostFunction, 1, 3>(
					new QuadraticAutoDiffCostFunction(x_data[i], y_data[i]));
			m_problem->AddResidualBlock(cost_function, m_robust_kernel_function.get(), tmp->parameters);
		}

		m_options->linear_solver_type = ceres::DENSE_QR;
		m_options->max_num_iterations = 100;
		m_options->minimizer_progress_to_stdout = false;

		ceres::Solve(*m_options, m_problem.get(), m_summary.get());
		return true;
	}

public:
	std::shared_ptr<ceres::Problem::Options > m_problem_options;
	std::shared_ptr<ceres::Problem> m_problem;
	std::shared_ptr<ceres::Solver::Options> m_options; // do not use std::shared_ptr
	std::shared_ptr<ceres::Solver::Summary> m_summary;
	std::shared_ptr<ceres::LossFunction> m_robust_kernel_function;
};


int main()
{
	optimizer::Ptr op = std::make_shared<optimizer>();
	op->exec();
	std::cout << op->m_summary->FullReport();
	system("pause");
	return 1;
}