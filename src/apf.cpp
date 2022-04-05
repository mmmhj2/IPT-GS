#include "apf.h"

APFHelper::APFHelper(double d1, double d2, double d3) :
	coef_attr(d1), coef_repl(d2), dist_threshold(d3)
{
}

vector2d APFHelper::
GetReplusion(const vector2d & quad, const std::vector <vector2d> & obstacles)
const noexcept
{
	vector2d repl;
	
	for(const auto & obst : obstacles)
	{
		vector2d dist = quad - obst;
		double distnorm = dist.norm();
		
		if(distnorm > this->dist_threshold)
			continue;
		
		vector2d distUniform = dist * (1/distnorm);
		repl = repl + distUniform * this->coef_repl * (1/distnorm - 1/this->dist_threshold) * (1/(distnorm * distnorm));
	}
	
	return repl;
}

vector2d 
APFHelper::GetAttraction(const vector2d & quad, const vector2d & dest)
const noexcept
{
	return (dest - quad) * coef_attr;
}

ArtificialPotentialField::ArtificialPotentialField(APFHelper * _helper,
			int _max_iter,
			double _march, 
			double _tolerance
			) :
			helper(_helper), max_iter(_max_iter), march(_march), tolerance(_tolerance)
{
}

vector2d
ArtificialPotentialField::GetStep(const vector2d & start, const vector2d & finish, const std::vector <vector2d> & obstacles)
const noexcept
{
	if((start - finish).norm() < tolerance)
		return finish;
	vector2d force = helper->GetAttraction(start, finish) + helper->GetReplusion(start, obstacles);
	vector2d delta = force  * (1/force.norm()) * march;
	return (start + delta);
}

void
ArtificialPotentialField::Planning
(const vector2d & start, const vector2d & finish, const std::vector <vector2d> & obstacles, std::vector <vector2d> & result)
const noexcept
{
	vector2d currentPose = start;
	int iter = 0;
	
	while(iter < max_iter)
	{
		iter++;
		
		if((currentPose - finish).norm() < tolerance)
			break;
		
		//vector2d force = helper->GetAttraction(currentPose, finish) + helper->GetReplusion(currentPose, obstacles);
		//vector2d delta = force * (1/force.norm()) * march;
		
		currentPose = this->GetStep(currentPose, finish, obstacles);
		result.push_back(currentPose);
	}
}

/*
#include <iostream>

int main()
{
	APFHelper helper;
	ArtificialPotentialField apf(&helper);
	
	vector2d start{0,0}, end{15, 15};
	std::vector <vector2d> obs, result;
	obs.emplace_back(4,3);
	obs.emplace_back(7,8);
	obs.emplace_back(9,10);
	obs.emplace_back(5,7);
	obs.emplace_back(13,8);
	
	apf.Planning(start, end, obs, result);
	
	for(const auto & itr : result)
		std::cout << itr.x << "," << itr.y << std::endl ;
}
*/


