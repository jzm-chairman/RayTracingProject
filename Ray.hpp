#pragma once
#include "Vec3.hpp"

class Ray
{
public:
	Vec3 orig, dir;
	Ray(Vec3 o = Vec3(), Vec3 d = Vec3()) : orig(o), dir(d) {} //assert: dir.sqrlen() == 1 if valid ray
	Vec3 getpoint(double dist) const { return orig + dir * dist; }
};

class RayPoint //前向追踪的碰撞点
{
public:
	Vec3 pos; //位置
	Vec3 its; //光亮度
	Vec3 dir; //光线方向
	double prob; //归一化概率

	RayPoint(Vec3 _pos = Vec3(), Vec3 _its = Vec3(), Vec3 _dir = Vec3(), double _prob = 1.0) :
		pos(_pos), its(_its), dir(_dir), prob(_prob) {}
};