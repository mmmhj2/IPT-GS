#include <valarray>
#include <cmath>


/// TODO : Replace vector2d to geometry_msgs::PoseStamped
struct vector2d
{
	double x, y;
	
	vector2d() : x(0), y(0){};
	vector2d(double _x, double _y) : x(_x), y(_y){};

	double norm() const
	{
		return sqrt(x * x + y * y);
	}
	
	vector2d operator + (const vector2d & rhs) const
	{
		return {x + rhs.x, y + rhs.y};
	}
	
	vector2d operator -() const
	{
		return {-x, -y};
	}
	
	vector2d operator - (const vector2d & rhs) const
	{
		return {x - rhs.x, y - rhs.y};
	}
	
	vector2d operator * (double rhs) const
	{
		return {x * rhs, y * rhs};
	}
	
	double operator * (const vector2d & rhs) const
	{
		return x * rhs.x + y * rhs.y;
	}
	
	double operator ^ (const vector2d & rhs) const
	{
		return x * rhs.y - y * rhs.x;
	}
};

