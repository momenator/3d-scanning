#include "utils/io.h"
#include "utils/points.h"
#include "ceres/ceres.h"
#include <math.h>
// #include <Eigen>


// TODO: Implement the cost function
struct RegistrationCostFunction
{
	RegistrationCostFunction(const Point2D& p1_, const Point2D& p2_, const Weight& w_)
		: p1(p1_), p2(p2_), w(w_)
	{
	}

	template<typename T>
	bool operator()(const T* const theta, const T* const tx, const T* const ty, T* residual) const
	{
		// Implement the cost function
		// Apply the transformation formula
		// Soluton 1 - 1 component
		auto comp = T(pow(T(cos(theta[0]) * T(p1.x) - sin(theta[0]) * T(p1.y)) + T(tx[0]) - T(p2.x), 2));
		comp += T(pow(T(sin(theta[0]) * T(p1.x) + cos(theta[0]) * T(p1.y)) + T(ty[0]) - T(p2.y), 2));
		residual[0] = T(w.w) * T(pow(T(sqrt(comp)), 2));
		
		// Solution 2 - 2 components
		// residual[0] = abs(cos(theta[0]) * T(p1.x) - sin(theta[0]) * T(p1.y) + T(tx[0]) - T(p2.x));
		// residual[1] = abs(sin(theta[0]) * T(p1.x) + cos(theta[0]) * T(p1.y) + T(ty[0]) - T(p2.y));
		return true;
	}

private:
	const Point2D p1;
	const Point2D p2;
	const Weight w;
};

int main(int argc, char** argv)
{
	google::InitGoogleLogging(argv[0]);

	// Read data points and the weights. Define the parameters of the problem
	const std::string file_path_1 = "./data/points_dragon_1.txt";
	const std::string file_path_2 = "./data/points_dragon_2.txt";
	const std::string file_path_weights = "./data/weights_dragon.txt";

	const auto points1 = read_points_from_file<Point2D>(file_path_1);
	const auto points2 = read_points_from_file<Point2D>(file_path_2);
	const auto weights = read_points_from_file<Weight>(file_path_weights);

	ceres::Problem problem;

	double theta = 1.0, tx = 1.0, ty = 1.0;

	// For each weighted correspondence create one residual block
	for (int i = 0; i < weights.size(); i++) {

		auto& p1 = points1[i];
		auto& p2 = points2[i];
		auto& w = weights[i];

		problem.AddResidualBlock(
			new ceres::AutoDiffCostFunction<RegistrationCostFunction, 1, 1, 1, 1>(
				new RegistrationCostFunction(p1, p2, w)),
			nullptr, &theta, &tx, &ty
		);
	}


	ceres::Solver::Options options;
	options.max_num_iterations = 25;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	std::cout << summary.BriefReport() << std::endl;

	// convert to degree
	theta = theta * 180 / M_PI;

	// Output the final values of the translation and rotation (in degree)
	std::cout << "Final theta: " << theta << "\ttx: " << tx << "\tty: " << ty << std::endl;

	system("pause");
	return 0;
}