#pragma once
#include "Vec3.hpp"

class ViewPoint //�ӵ�����(�����ȶԹ�������һ��)
{
public:
	Vec3 lum; //������
	double totprob; //�ܸ���
	ViewPoint() : lum(Vec3()), totprob(0) {}
	ViewPoint(Vec3 _lum, double _prob) : ViewPoint() { add(_lum, _prob); }
	void add(Vec3 dlum, double dprob) { lum = lum + dlum; totprob += dprob; }
	void add(const ViewPoint& vp) { add(vp.lum, vp.totprob); }
	ViewPoint normalize() { return totprob > eps ? ViewPoint(lum / totprob, 1.0) : ViewPoint(); }
	Vec3 get() { return totprob < eps ? lum : lum / totprob; }
};