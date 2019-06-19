#pragma once
#include "Vec3.hpp"

#define base(i, n, t) (comp(n, i) * pow(t, i) * pow(1 - t, n - i))

class BezierCurve //assert: y为单调增序列 x = P(y)
{
	Vec3* cont; //assert: Vec3(x, y, 0)
	int order; //assert: len(cont) = order + 1
	Vec3 ymax_p, xmax_p, ymin_p, xmin_p;

public:
	BezierCurve(int _order = 0, Vec3* _cont = nullptr)
	{
		double max_y = -INF, min_y = INF;
		order = _order;
		cont = new Vec3[order + 1];
		for (int i = 0; i <= order; i++)
		{
			cont[i] = _cont[i];
			if (cont[i].y > max_y) ymax_p = cont[i], max_y = cont[i].y;
			else if (cont[i].y < min_y) ymin_p = cont[i], min_y = cont[i].y;
		}
		xmax_p = cont[order], xmin_p = cont[0];
	}
	std::tuple<Vec3, Vec3, Vec3, Vec3> maxmin_xy() { return std::make_tuple(xmax_p, xmin_p, ymax_p, ymin_p); }
	Vec3 value(double t) //assert: 0 <= t <= 1
	{
		Vec3 val(0, 0, 0);
		for (int i = 0; i <= order; i++)
			val = val + cont[i] * base(i, order, t);
		return val;
	}
	Vec3 derive(double t)
	{
		Vec3 drv(0, 0, 0);
		for (int i = 0; i < order; i++)
			drv = drv + (cont[i + 1] - cont[i]) * base(i, order - 1, t);
		drv = drv * order;
		return drv;
	}
	~BezierCurve()
	{ 
		if (order > 0) delete[] cont;
	}
	friend class BezierVase;
};