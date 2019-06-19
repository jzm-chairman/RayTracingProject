#pragma once
#include "Scene.hpp"

class Renderer
{
	Scene* scene;

public:
	Renderer() : scene(nullptr) {}
	Renderer(Scene* s) : scene(s) {}
	Vec3 pathtrace(const Ray& r, int depth);
	//~Renderer() { if(scene) delete scene; }
};

Vec3 Renderer::pathtrace(const Ray& r, int depth) //Path Tracing,返回值为(R,G,B)光强度
{
	Geometry* obj = nullptr;
	Vec3 point;
	std::tie(obj, point) = scene->intersect_any(r);
	//printf("Depth = %d, Collision Object = %p\n", depth, obj);
	if (obj == nullptr) return Vec3();
	Vec3 L = r.dir, N = obj->get_normvec(point); //L是入射光向量，N是归一化交点法向量
	int light_out = -sign(L.dot(N)); //light_out == 1表示光源在外 -1表示光源在内
	N = N * light_out; //修正法向量方向以便后续操作
	Vec3 I = obj->isLight() ? Vec3() : obj->get_color(point); //I是物体表面对(R,G,B)光的反照率
	Vec3 E = obj->isLight() ? obj->get_color(point) : Vec3(); //E是物体表面(R,G,B)光的发光强度
	//I.print();
	//if (depth > 10000) { printf("Now Depth = %d, Object Property = %d, I:", depth, obj->get_op()); I.print(); }
	if (depth > 4)
	{
		if (depth > 1000) return E;
		if (random01() < I.max()) I = I / I.max();
		else return E;
	}

	Opt_Prop op = obj->get_op();
	if (op == DIFF) //漫反射:随机均匀反射
	{
		Vec3 x, y, z = N; //以交点为原点建立坐标系
		x = abs(N.x) < 0.5 ? z.cross(Vec3(1, 0, 0)).norm() : z.cross(Vec3(0, 1, 0)).norm(); //随便叉积一个向量来确定其他坐标(但要保证数值精度);
		y = z.cross(x);
		double theta = 2 * PI * random01(); //theta是反射光在坐标系上xy面的角度
		double sqrdz = random01();
		double dz = sqrt(sqrdz); //dz是反射光在坐标系z分量值
		double dxy = sqrt(1 - sqrdz);
		Vec3 R = (x * cos(theta) * dxy + y * sin(theta) * dxy + z * dz).norm();//随机选择的漫反射方向
		return E + I.mult(pathtrace(Ray(point, R), depth + 1));
	}
	else if (op == SPEC) //镜面反射:反射定律
	{
		Vec3 R = L - N * L.dot(N) * 2; //镜面反射方向
		return E + I.mult(pathtrace(Ray(point, R), depth + 1));
	}
	else if (op == REFR) //折射+反射
	{
		Vec3 R = L - N * L.dot(N) * 2; //反射方向
		double ni = 1.50; //玻璃的折射率为1.5，空气的折射率为1
		double ratio = light_out == 1 ? 1 / ni : ni; //ratio是入射介质和折射介质折射率之比
		double costi = -(r.dir).dot(N); //入射角的余弦值
		double sqrcostt = 1 - sqr(ratio) * (1 - sqr(costi)); //折射角余弦值平方
		//printf("%f\n", sqrcostt);
		if (sqrcostt < 0) 
			return E + I.mult(pathtrace(Ray(point, R), depth + 1)); //全反射
		double costt = sqrt(sqrcostt); //折射角余弦值
		Vec3 RT = ((L * ratio) - (N * (costt - costi * ratio))).norm(); //折射方向

		double a = ni - 1, b = ni + 1, r0 = sqr(a) / sqr(b), c = 1 - (light_out == 1 ? costi : costt); //菲涅尔方程???
		//printf("入射方向:"); L.print(); printf("折射方向:"); RT.print()
		//printf("cosi=%f,cost=%f,light_out=%d,r0=%f,c=%f\n", costi, costt, light_out, r0, c);
		double Re = r0 + (1 - r0) * pow(c, 5), Tr = 1 - Re, P = 0.25 + 0.5 * Re, RP = Re / P, TP = Tr / (1 - P);
		//printf("Re=%f,Tr=%f, P=%f, RP=%f, TP=%f\n", Re, Tr, P, RP, TP);
		return E + I.mult(depth > 1 ? (random01() < P ? 
			pathtrace(Ray(point, R), depth + 1) * RP : pathtrace(Ray(point, RT), depth + 1) * TP) :
			pathtrace(Ray(point, R), depth + 1) * Re + pathtrace(Ray(point, RT), depth + 1) * Tr); //深度加深则只选取折射或反射
	}
	return Vec3();
}