#pragma once
#include "Vec3.hpp"

class Ray
{
public:
	Vec3 orig, dir;
	Ray(Vec3 o = Vec3(), Vec3 d = Vec3()) : orig(o), dir(d) {} //assert: dir.sqrlen() == 1 if valid ray
	Vec3 getpoint(double dist) const { return orig + dir * dist; }
};