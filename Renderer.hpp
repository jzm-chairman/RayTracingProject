#pragma once
#include "Scene.hpp"
#include <omp.h>
#include "KDTree.hpp"

#define THREAD 6
#define ITER 50 //��������
#define PTC 600000 //ÿ�ֹ�����
#define MAXPTC 12000000 //��������
#define APERT 0 //��Ȧ��С
#define SAMP 2
#define LIGHTRATE 1.5

class Renderer
{
	Scene* scene;
	std::vector<std::vector<ViewPoint>> vision;
	int width, height, samples;
	std::string outfile;

public:
	Renderer(Scene* s) : scene(s) {}
	void setsize(int _width, int _height, int _samp, const char* filename) 
	{
		width = _width, height = _height, samples = _samp;
		vision.resize(height);
		for (auto it = vision.begin(); it != vision.end(); ++it)
			it->resize(width);
		outfile = filename;
	}
	void raytrace(const Ray& r, int depth, Vec3 intense, std::vector<HitPoint>& hparr, std::pair<int, int> vp);
	void photonmap(const Ray& r, int depth, Vec3 intense, double prob, std::vector<std::vector<ViewPoint>>& viewbuf, KDTree& hptree);
	Vec3 pathtrace(const Ray& r, int depth);
	void ptrender();
	void ppmrender();
	void writeout(const char* filename);
};

void Renderer::ppmrender()
{
	Ray camera(Vec3(50, 52, 296), Vec3(0, -0.04, -1).norm()); //���Դ�㣬���߷���
	Vec3 cx = Vec3(width * 0.5 / height, 0, 0), cy = cx.cross(camera.dir).norm() * 0.5; //???

	int ptc = PTC;
	for (auto iter = 0; iter < ITER; iter++) //��������
	{
		fprintf(stderr, "Iteration %d:\n", iter);
		std::vector<HitPoint> loc_hp[THREAD], hp;
		KDTree hptree;
#	pragma omp parallel for num_threads(THREAD) schedule(dynamic, 32)
		for (int y = 0; y < height; y++) //��һ�׶�:����׷�ٲ���
		{
			int rank = omp_get_thread_num();
			if(y % 32 == 31)
				fprintf(stderr, "\rRay Tracing Sampling %d / %d Rows (%d Samples)", y + 1, height, SAMP * 4);
			for (int x = 0; x < width; x++)
				for (int suby = 0; suby < 2; suby++) //ÿ���طֳ��ĸ�����ֱ����
					for (int subx = 0; subx < 2; subx++)
					{
						for (int s = 0; s < SAMP; s++)
						{
							double rand1 = 2 * random01(), rand2 = 2 * random01();
							double dx = rand1 < 1 ? sqrt(rand1) - 1 : 1 - sqrt(2 - rand1);
							double dy = rand2 < 1 ? sqrt(rand2) - 1 : 1 - sqrt(2 - rand2);
							Vec3 raydir = cx * ((x + (subx + dx + 0.5) / 2) / width - 0.5) + cy * ((y + (suby + dy + 0.5) / 2) / height - 0.5) + camera.dir;
							Vec3 raysrc = camera.orig + raydir * 140;
							Vec3 bias = camera.orig + Vec3(2 * (random01() - 0.5), 2 * (random01() - 0.5), 0) * APERT;//������Ȧ���þ���
							raytrace(Ray(raysrc, (raysrc - bias).norm()), 0, Vec3(1, 1, 1), loc_hp[rank], std::make_pair(y, x));
						}
					}
					
			/*{
				Vec3 raydir = cx * (x / width) + cy * (y / height) + camera.dir;
				raytrace(Ray(camera.orig + raydir * 140, raydir.norm()), 0, Vec3(1, 1, 1), loc_hp[rank], std::make_pair(y, x));
			}*/
				
		}

		fprintf(stderr, "\n");
		for (int i = 0; i < THREAD; i++)
			hp.insert(hp.end(), loc_hp[i].begin(), loc_hp[i].end());
		//printf("Tree Size = %d\n", hp.size());
		/*for (auto it = hp.begin(); it != hp.end(); ++it)
		{
			int w, h; std::tie(h, w) = it->viewpoint;
			vision[h][w] = vision[h][w] + it->col;
		}
		break;*/
		hptree.build(hptree.root, hp, 0, 0, hp.size());

		std::vector<std::vector<ViewPoint>> viewbuf[THREAD];
		for (int i = 0; i < THREAD; i++)
		{
			viewbuf[i].resize(height);
			for (auto it = viewbuf[i].begin(); it != viewbuf[i].end(); ++it)
				it->resize(width);
		}
#	pragma omp parallel for num_threads(THREAD) schedule(dynamic, 1000)
		for (int p = 0; p < ptc; p++) //�ڶ��׶�:ǰ��׷���͹�
		{
			int rank = omp_get_thread_num();
			if (p % 1000 == 999)
				fprintf(stderr, "\rPhoton Mapping %d / %d Rays  ", p + 1, ptc);
			double dr = 30 * random01() - 15;
			double dt = 2 * PI * random01();
			Vec3 light = Vec3(50 + dr * cos(dt) , 81.6 - eps, 81.6 + dr * sin(dt));
			Vec3 x = (1, 0, 0), y = (0, -1, 0), z = (0, 0, 1);
			double t = 2 * PI * random01();
			double s = random01();
			//double d = random01() < 0.5 ? 1 : -1;
			Vec3 raydir = ((x * cos(t) + z * sin(t)) * sqrt(s) + y * sqrt(1 - s)).norm();
			Vec3 rayintense = Vec3(1, 1, 1) * LIGHTRATE;
			hptree.query(hptree.root, viewbuf[rank], RayPoint(light, rayintense, raydir, 1.0));
			photonmap(Ray(light, raydir), 0, rayintense, 1.0, viewbuf[rank], hptree);
		}

		for (int i = 0; i < height; i++)
			for (int j = 0; j < width; j++)
				for(int t = 0; t < THREAD; t++)
					vision[i][j].add(viewbuf[t][i][j].normalize());
		hptree.reducerad(hptree.root);
		if(ptc < MAXPTC)
			ptc = int(ptc / sqrt(ALPHA));
		fprintf(stderr, "\n");
		writeout((outfile + std::string("_") + std::to_string(iter) + ".ppm").c_str());
	}
}

void Renderer::raytrace(const Ray& r, int depth, Vec3 intense, std::vector<HitPoint>& hparr, std::pair<int, int> vp)
{ //PPM��һ��:���ӵ㷢����������������ֹͣ����¼��ײ��
	Geometry* obj = nullptr;
	Vec3 point;
	std::tie(obj, point) = scene->intersect_any(r);
	if (obj == nullptr) return;
	Vec3 L = r.dir, N = obj->get_normvec(point); //L�������������N�ǹ�һ�����㷨����
	int light_out = -sign(L.dot(N)); //light_out == 1��ʾ��Դ���� -1��ʾ��Դ����
	N = N * light_out; //���������������Ա��������
	Vec3 I = obj->isLight() ? Vec3() : obj->get_color(point); //I����������(R,G,B)��ķ�����
	if (depth > 200) return;
	if (depth > 4)
	{
		if (random01() < I.max()) I = I / I.max();
		else return;
	}

	Opt_Prop op = obj->get_op();
	if (op == DIFF) //������:������ײ��
	{
		hparr.push_back(HitPoint(point, intense.mult(I), N, vp));
	}
	else if (op == SPEC) //���淴��:���䶨��
	{
		Vec3 R = L - N * L.dot(N) * 2; //���淴�䷽��
		raytrace(Ray(point, R), depth + 1, intense.mult(I), hparr, vp);
	}
	else if (op == REFR) //����+����
	{
		Vec3 R = L - N * L.dot(N) * 2; //���䷽��
		double ni = 1.50; //������������Ϊ1.5��������������Ϊ1
		double ratio = light_out == 1 ? 1 / ni : ni; //ratio��������ʺ��������������֮��
		double costi = -(r.dir).dot(N); //����ǵ�����ֵ
		double sqrcostt = 1 - sqr(ratio) * (1 - sqr(costi)); //���������ֵƽ��
		if (sqrcostt < 0)
			raytrace(Ray(point, R), depth + 1, intense.mult(I), hparr, vp);
		double costt = sqrt(sqrcostt); //���������ֵ
		Vec3 RT = ((L * ratio) - (N * (costt - costi * ratio))).norm(); //���䷽��

		double a = ni - 1, b = ni + 1, r0 = sqr(a) / sqr(b), c = 1 - (light_out == 1 ? costi : costt); //����������???
		double Re = r0 + (1 - r0) * pow(c, 5), Tr = 1 - Re, P = 0.25 + 0.5 * Re, RP = Re / P, TP = Tr / (1 - P);
		if (depth <= 1)
		{
			raytrace(Ray(point, R), depth + 1, intense.mult(I), hparr, vp);
			raytrace(Ray(point, RT), depth + 1, intense.mult(I), hparr, vp);
		}
		random01() < P ?
			raytrace(Ray(point, R), depth + 1, intense.mult(I), hparr, vp) :
			raytrace(Ray(point, RT), depth + 1, intense.mult(I), hparr, vp);
	}
}

void Renderer::photonmap(const Ray& r, int depth, Vec3 intense,
	double prob, std::vector<std::vector<ViewPoint>>& viewbuf, KDTree& hptree)
{ //PPM�ڶ���:�ӹ�Դ�����������ݹ���ֹ
	Geometry* obj = nullptr;
	Vec3 point;
	std::tie(obj, point) = scene->intersect_any(r);
	if (obj == nullptr) return;
	Vec3 L = r.dir, N = obj->get_normvec(point); //L�������������N�ǹ�һ�����㷨����
	int light_out = -sign(L.dot(N)); //light_out == 1��ʾ��Դ���� -1��ʾ��Դ����
	N = N * light_out; //���������������Ա��������
	Vec3 I = obj->isLight() ? Vec3() : obj->get_color(point); //I����������(R,G,B)��ķ�����
	if (depth > 100 || prob < 1e-4) return;
	if (depth > 4)
	{
		if (random01() < I.max()) I = I / I.max();
		else
		{
			hptree.query(hptree.root, viewbuf, RayPoint(point, intense, r.dir, prob));
			return;
		}
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
		hptree.query(hptree.root, viewbuf, RayPoint(point, intense, r.dir, prob));
		photonmap(Ray(point, R), depth + 1, intense.mult(I), prob, viewbuf, hptree);
	}
	else if (op == SPEC) //���淴��:���䶨��
	{
		Vec3 R = L - N * L.dot(N) * 2; //���淴�䷽��
		photonmap(Ray(point, R), depth + 1, intense.mult(I), prob, viewbuf, hptree);
	}
	else if (op == REFR) //����+����
	{
		Vec3 R = L - N * L.dot(N) * 2; //���䷽��
		double ni = 1.50; //������������Ϊ1.5��������������Ϊ1
		double ratio = light_out == 1 ? 1 / ni : ni; //ratio��������ʺ��������������֮��
		double costi = -(r.dir).dot(N); //����ǵ�����ֵ
		double sqrcostt = 1 - sqr(ratio) * (1 - sqr(costi)); //���������ֵƽ��
		if (sqrcostt < 0)
		{
			photonmap(Ray(point, R), depth + 1, intense.mult(I), prob, viewbuf, hptree);
			return;
		}
		double costt = sqrt(sqrcostt); //���������ֵ
		Vec3 RT = ((L * ratio) - (N * (costt - costi * ratio))).norm(); //���䷽��
		double a = ni - 1, b = ni + 1, r0 = sqr(a) / sqr(b), c = 1 - (light_out == 1 ? costi : costt);
		double Re = r0 + (1 - r0) * pow(c, 5), Tr = 1 - Re, P = 0.25 + 0.5 * Re, RP = Re / P, TP = Tr / (1 - P);
		if (depth <= 1)
		{
			photonmap(Ray(point, R), depth + 1, intense.mult(I), prob * Re, viewbuf, hptree);
			photonmap(Ray(point, RT), depth + 1, intense.mult(I), prob * Tr, viewbuf, hptree);
		}
		random01() < P ?
			photonmap(Ray(point, R), depth + 1, intense.mult(I), prob * RP, viewbuf, hptree):
			photonmap(Ray(point, R), depth + 1, intense.mult(I), prob * TP, viewbuf, hptree);
	}
}

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

void Renderer::ptrender()
{
	Ray camera(Vec3(50, 52, 296), Vec3(0, -0.04, -1).norm()); //���Դ�㣬���߷���
	Vec3 cx = Vec3(width * 0.5 / height, 0, 0), cy = cx.cross(camera.dir).norm() * 0.5; //???

#	pragma omp parallel for num_threads(THREAD)\
		schedule(dynamic, 1)
	for (int y = 0; y < height; y++) //������Ⱦ
	{
		fprintf(stderr, "\rPath Tracing Rendering %d / %d Rows (%d Samples)", y + 1, height, samples * 4);
		for (int x = 0; x < width; x++)
			for (int suby = 0; suby < 2; suby++) //ÿ���طֳ��ĸ�����ֱ����
				for (int subx = 0; subx < 2; subx++)
				{
					Vec3 ray_intense; //�����ص���ۻ���ǿ
					for (int s = 0; s < samples; s++)
					{
						double rand1 = 2 * random01(), rand2 = 2 * random01();
						double dx = rand1 < 1 ? sqrt(rand1) - 1 : 1 - sqrt(2 - rand1);
						double dy = rand2 < 1 ? sqrt(rand2) - 1 : 1 - sqrt(2 - rand2);
						Vec3 ray_dir = cx * ((x + (subx + dx + 0.5) / 2) / width - 0.5) + cy * ((y + (suby + dy + 0.5) / 2) / height - 0.5) + camera.dir; //???
						//printf("ray direction:"); ray_dir.print();
						Vec3 delta_intense = pathtrace(Ray(camera.orig + ray_dir * 140, ray_dir.norm()), 0) / samples;
						ray_intense = ray_intense + delta_intense;
					}
					//printf("ray intensity:"); ray_intense.print();
					vision[y][x].add(ray_intense.fix() / 4, 0);
					//printf("(%d, %d):", y, x);
					//vision[y][x].print();
				}
	}
	writeout((outfile + ".ppm").c_str());
}

void Renderer::writeout(const char* filename)
{
	std::ofstream wf(filename);
	wf << "P3\n" << width << " " << height << "\n" << MAX_USHORT << "\n";
	for (int y = height - 1; y >= 0; y--)
	{
		for (int x = width - 1; x >= 0; x--)
			wf << vision[y][x].get().toRGB().c_str();
		wf << "\n";
	}
	wf.close();
}