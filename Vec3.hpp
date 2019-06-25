#pragma once
#include <cmath>
#include "utils.hpp"

class Vec3
{
public:
	double x, y, z;
	Vec3(double _x = 0.0, double _y = 0.0, double _z = 0.0) : x(_x), y(_y), z(_z) {}
	Vec3(const Vec3& o) : Vec3(o.x, o.y, o.z) {}
	Vec3 operator+(const Vec3& o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
	Vec3 operator-(const Vec3& o) const { return Vec3(x - o.x, y - o.y, z - o.z); } //加减向量
	Vec3 operator+(double t) const { return Vec3(x + t, y + t, z + t); }
	Vec3 operator-(double t) const { return *this + (-t); } //加减常数
	Vec3 operator*(double k) const { return Vec3(k * x, k * y, k * z); } //数乘
	Vec3 operator/(double k) const { return *this * (1.0 / k); }
	double sqrlen() const { return this->dot(*this); }
	double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; } //点积
	Vec3 cross(const Vec3 &o) const { return Vec3(y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x); } //叉积
	Vec3& norm() { *this = *this / sqrt(sqrlen()); return *this; } //归一化
	Vec3 mult(const Vec3& o) const { return Vec3(x * o.x, y * o.y, z * o.z); } //逐点积
	double max() const { return std::max(std::max(x, y), z); }
	double min() const { return std::min(std::min(x, y), z); }
	double norminf() const { return std::max(std::max(abs(x), abs(y)), abs(z)); }
	bool isNormal() const { return std::isnormal(x) && std::isnormal(y) && std::isnormal(z); }
	bool outrange(const Vec3& bmin, const Vec3& bmax) const
	{ return x < bmin.x || x > bmax.x || y < bmin.y || y > bmax.y || z < bmin.z || z > bmax.z; }
	std::tuple<double, double, double> unpack() const { return std::make_tuple(x, y, z); }
	std::string toRGB()
	{
		char out[20];
		memset(out, 0, sizeof(char) * 20);
		sprintf(out, "%d %d %d ", torgb(x), torgb(y), torgb(z));
		return std::string(out);
	}
	void print() const { printf("Vector: (%.2f, %.2f, %.2f)\n", x, y, z); }
	Vec3 fix() { return Vec3(fix01(x), fix01(y), fix01(z)); }
};

Vec3 minvec(const Vec3& v1, const Vec3& v2)
{
	return Vec3(std::min(v1.x, v2.x), std::min(v1.y, v2.y), std::min(v1.z, v2.z));
}
Vec3 maxvec(const Vec3& v1, const Vec3& v2)
{
	return Vec3(std::max(v1.x, v2.x), std::max(v1.y, v2.y), std::max(v1.z, v2.z));
}