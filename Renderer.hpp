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

Vec3 Renderer::pathtrace(const Ray& r, int depth) //Path Tracing,����ֵΪ(R,G,B)��ǿ��
{
	Geometry* obj = nullptr;
	Vec3 point;
	std::tie(obj, point) = scene->intersect_any(r);
	//printf("Depth = %d, Collision Object = %p\n", depth, obj);
	if (obj == nullptr) return Vec3();
	Vec3 L = r.dir, N = obj->get_normvec(point); //L�������������N�ǹ�һ�����㷨����
	int light_out = -sign(L.dot(N)); //light_out == 1��ʾ��Դ���� -1��ʾ��Դ����
	N = N * light_out; //���������������Ա��������
	Vec3 I = obj->isLight() ? Vec3() : obj->get_color(point); //I����������(R,G,B)��ķ�����
	Vec3 E = obj->isLight() ? obj->get_color(point) : Vec3(); //E���������(R,G,B)��ķ���ǿ��
	//I.print();
	//if (depth > 10000) { printf("Now Depth = %d, Object Property = %d, I:", depth, obj->get_op()); I.print(); }
	if (depth > 4)
	{
		if (depth > 1000) return E;
		if (random01() < I.max()) I = I / I.max();
		else return E;
	}

	Opt_Prop op = obj->get_op();
	if (op == DIFF) //������:������ȷ���
	{
		Vec3 x, y, z = N; //�Խ���Ϊԭ�㽨������ϵ
		x = abs(N.x) < 0.5 ? z.cross(Vec3(1, 0, 0)).norm() : z.cross(Vec3(0, 1, 0)).norm(); //�����һ��������ȷ����������(��Ҫ��֤��ֵ����);
		y = z.cross(x);
		double theta = 2 * PI * random01(); //theta�Ƿ����������ϵ��xy��ĽǶ�
		double sqrdz = random01();
		double dz = sqrt(sqrdz); //dz�Ƿ����������ϵz����ֵ
		double dxy = sqrt(1 - sqrdz);
		Vec3 R = (x * cos(theta) * dxy + y * sin(theta) * dxy + z * dz).norm();//���ѡ��������䷽��
		return E + I.mult(pathtrace(Ray(point, R), depth + 1));
	}
	else if (op == SPEC) //���淴��:���䶨��
	{
		Vec3 R = L - N * L.dot(N) * 2; //���淴�䷽��
		return E + I.mult(pathtrace(Ray(point, R), depth + 1));
	}
	else if (op == REFR) //����+����
	{
		Vec3 R = L - N * L.dot(N) * 2; //���䷽��
		double ni = 1.50; //������������Ϊ1.5��������������Ϊ1
		double ratio = light_out == 1 ? 1 / ni : ni; //ratio��������ʺ��������������֮��
		double costi = -(r.dir).dot(N); //����ǵ�����ֵ
		double sqrcostt = 1 - sqr(ratio) * (1 - sqr(costi)); //���������ֵƽ��
		//printf("%f\n", sqrcostt);
		if (sqrcostt < 0) 
			return E + I.mult(pathtrace(Ray(point, R), depth + 1)); //ȫ����
		double costt = sqrt(sqrcostt); //���������ֵ
		Vec3 RT = ((L * ratio) - (N * (costt - costi * ratio))).norm(); //���䷽��

		double a = ni - 1, b = ni + 1, r0 = sqr(a) / sqr(b), c = 1 - (light_out == 1 ? costi : costt); //����������???
		//printf("���䷽��:"); L.print(); printf("���䷽��:"); RT.print()
		//printf("cosi=%f,cost=%f,light_out=%d,r0=%f,c=%f\n", costi, costt, light_out, r0, c);
		double Re = r0 + (1 - r0) * pow(c, 5), Tr = 1 - Re, P = 0.25 + 0.5 * Re, RP = Re / P, TP = Tr / (1 - P);
		//printf("Re=%f,Tr=%f, P=%f, RP=%f, TP=%f\n", Re, Tr, P, RP, TP);
		return E + I.mult(depth > 1 ? (random01() < P ? 
			pathtrace(Ray(point, R), depth + 1) * RP : pathtrace(Ray(point, RT), depth + 1) * TP) :
			pathtrace(Ray(point, R), depth + 1) * Re + pathtrace(Ray(point, RT), depth + 1) * Tr); //��ȼ�����ֻѡȡ�������
	}
	return Vec3();
}