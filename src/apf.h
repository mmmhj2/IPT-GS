#include "vector.h"
#include <vector>
//#include <memory>

class APFHelper
{
	const double coef_attr, coef_repl, dist_threshold;
public:
	APFHelper(double coef_attr = 1.0, double coef_repl = 10.0, double dist_threshold = 3);
	
	vector2d GetReplusion(const vector2d & quad, const std::vector <vector2d> & obstacles) const noexcept;
	vector2d GetAttraction(const vector2d & quad, const vector2d & dest) const noexcept;
};

/// TODO : Refactor the code to use std::unique_ptr
class ArtificialPotentialField
{
	const int max_iter;
	const double march, tolerance;
	const APFHelper * const helper;
public:
	ArtificialPotentialField(APFHelper * helper,
			int max_iter = 5000,
			double march = 0.1, 
			double tolerance = 0.1
			);
	
	vector2d
	GetStep(const vector2d & start, const vector2d & finish, const std::vector <vector2d> & obstacles) const noexcept;
	
	void
	Planning(const vector2d & start, const vector2d & finish, const std::vector <vector2d> & obstacles, std::vector <vector2d> & result) const noexcept;
	
};

