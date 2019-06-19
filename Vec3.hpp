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
	Vec3 operator-(const Vec3& o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
	Vec3 operator*(double k) const { return Vec3(k * x, k * y, k * z); } //����
	Vec3 operator/(double k) const { return *this * (1.0 / k); }
	double sqrlen() const { return this->dot(*this); }
	double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; } //���
	Vec3 cross(const Vec3 &o) const { return Vec3(y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x); } //���
	Vec3& norm() { *this = *this / sqrt(sqrlen()); return *this; } //��һ��
	Vec3 mult(const Vec3& o) const { return Vec3(x * o.x, y * o.y, z * o.z); } //����
	double max() const { return std::max(std::max(x, y), z); }
	double min() const { return std::min(std::min(x, y), z); }
	double norminf() const { return std::max(std::max(abs(x), abs(y)), abs(z)); }
	bool isNull() const { return abs(x) < eps && abs(y) < eps && abs(z) < eps; }
	bool isNormal() const { return std::isnormal(x) && std::isnormal(y) && std::isnormal(z); }
	std::tuple<double, double, double> unpack() { return std::make_tuple(x, y, z); }
	std::string toRGB()
	{
		char out[20];
		memset(out, 0, sizeof(char) * 20);
		sprintf(out, "%d %d %d ", torgb(x), torgb(y), torgb(z));
		return std::string(out);
	}
	void print() { printf("(x, y, z) = (%f, %f, %f)", x, y, z); }
	Vec3 fix() { return Vec3(fix01(x), fix01(y), fix01(z)); }
};