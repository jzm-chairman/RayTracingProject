#pragma once

#include "utils.hpp"
#include "Vec3.hpp"
#include "Ray.hpp"
#include "Scene.hpp"
#include "Texture.hpp"
#include "Bezier.hpp"

//#include "eigen3/Eigen/Dense"

class Geometry
{
protected:
	bool light;
	Opt_Prop op;
	Object_Type type;
	Texture* texture;

public:
	Geometry(bool l = false, Opt_Prop _op = BLACK, Object_Type t = NONE, Texture* _texture = nullptr) : 
		light(l), op(_op), type(t), texture(_texture) {}
	bool isLight() const { return light; }
	virtual Vec3 get_color(Vec3 point) const { return Vec3(); }
	Opt_Prop get_op() const { return op; }
	virtual std::tuple<double, Vec3> intersect(const Ray& r) = 0;
	virtual Vec3 get_normvec(Vec3 point) { return Vec3(); }
	virtual ~Geometry() { if (texture) delete texture; }
};

class Plane : public Geometry
{
	Vec3 normvec;
	Vec3 vertex;
	Vec3* edge; //assert: edge[0] 正交于 edge[1] edge = nullptr <-> 无穷大平面
	double posparam;

public: 
	//构造参数(法向量，参考点，反射率，是否为光源，光学属性)
	Plane(Vec3 n = Vec3(), Vec3 _vertex = Vec3(), Vec3 *_edge = nullptr, Texture* _texture = nullptr, bool l = false, Opt_Prop _op = BLACK) :
		Geometry(l, _op, PLANE, _texture), normvec(n.norm()), vertex(_vertex), posparam(-n.dot(_vertex))
	{
		if (_edge == nullptr) edge = nullptr;
		else
		{
			edge = new Vec3[2];
			edge[0] = _edge[0], edge[1] = _edge[1];
		}
	}
	virtual Vec3 get_color(Vec3 point) const//assert: 所求点在平面内，下同
	{
		if (texture->get_type() == PURE) return texture->query();
		Vec3 dir = point - vertex;
		return texture->query(dir.dot(edge[0]) / edge[0].sqrlen(), dir.dot(edge[1]) / edge[1].sqrlen());
	}
	virtual Vec3 get_normvec(Vec3 point) { return normvec; }
	virtual std::tuple<double, Vec3> intersect(const Ray& r)
	{
		double t = -(posparam + normvec.dot(r.orig)) / normvec.dot(r.dir); //无穷大平面求交只需求解交点
		if (t > 0)
		{
			Vec3 point = r.getpoint(t);
			if (edge == nullptr) return std::make_tuple(t, point);
			//有限大平面还需判断交点是否在面内 (A, B, C, D)分别是(x1, y1), (x2, y1), (x2, y2), (x1, y2), x1 < x2, y1 < y2
			//A是顶点 E是交点 (AB X AE).(CD X CE) > 0 & (DA X DE).(BC X BE) > 0 AB = -CD = edge[0], BC = -DA = edge[1]
			Vec3 a2p = point - vertex; //AE
			Vec3 c2p = a2p - edge[0] - edge[1]; //CE
			Vec3 b2p = a2p - edge[0]; //BE
			Vec3 d2p = a2p - edge[1]; //DE
			if ((edge[0].cross(a2p)).dot((edge[0] * -1).cross(c2p)) > 0 && ((edge[1] * -1).cross(d2p)).dot(edge[1].cross(b2p)) > 0)
			{
				//printf("Point"); point.print(); printf("on the plane\n");
				return std::make_tuple(t, point);
			}
		}
		return std::make_tuple(INF, Vec3());
	}
	virtual ~Plane() { if (edge) delete[] edge; }
};

class Sphere : public Geometry
{
	double radius;
	Vec3 center;

public: 
	//构造参数(球心，反射率，半径，是否为光源，光学属性)
	Sphere(Vec3 cen = Vec3(), Texture* _texture = nullptr, double r = 0.0, bool l = false, Opt_Prop _op = BLACK) :
		Geometry(l, _op, SPHERE, _texture), center(cen), radius(r) {}
	virtual Vec3 get_normvec(Vec3 point) { return (point - center).norm(); }
	virtual Vec3 get_color(Vec3 point) const { return texture->query(); }
	virtual std::tuple<double, Vec3> intersect(const Ray& r) //几何方法求交
	{
		Vec3 lc = center - r.orig; //由光源指向球心的向量
		double dop = lc.dot(r.dir); //光源沿光方向到(球心到光线所在直线投影点)距离
		if (lc.sqrlen() > sqr(radius) && dop < 0) return std::make_tuple(INF, Vec3()); //光源在球体外且光线反向则无交点
		double sqrdcp = lc.sqrlen() - sqr(dop); //球心到光线所在直线距离平方
		if (sqrdcp > sqr(radius)) return std::make_tuple(INF, Vec3()); //球心到光线距离大于半径显然无交点
		double sqrdip = sqr(radius) - sqrdcp; //投影点到光线与球面交点距离平方
		double doi = lc.sqrlen() > sqr(radius) ? dop - sqrt(sqrdip) : dop + sqrt(sqrdip); //光源到交点距离(分类光源在球体内部/外部)
		Vec3 hitpoint = r.getpoint(doi);
		return std::make_tuple(doi, hitpoint);
	}
	/*std::tuple<double, Vec3> intersect(const Ray &r) {
		Vec3 op = center - r.orig; // Solve t^2*d.d + 2*t*(o-p).d + (o-p).(o-p)-R^2 = 0 
		double t, b = op.dot(r.dir), det = b * b - op.sqrlen() + sqr(radius);
		if (det < 0) return std::make_tuple(INF, r.orig); else det = sqrt(det);
		t =  b - det > eps ? b - det : (b + det > eps ? b + det : 0.0);
		return std::make_tuple(t, r.getpoint(t));
	}*/
	virtual ~Sphere() {}
	friend class BezierVase;
};

class BezierVase : public Geometry//assert: r = f(t), t = y 曲线方向指向上方向
{
	Vec3 center;
	BezierCurve* curve; //assert: curve.x <-> vase.y, curve.y <-> vase.r
	Sphere* bound;
	Vec3 ymax_point, ymin_point, rmax_point, rmin_point;

	Vec3 update_value(const Ray& r, Vec3 solution)
	{
		double s, t, u, xt, yt, _;
		std::tie(s, t, u) = solution.unpack();
		std::tie(xt, yt, _) = curve->value(t).unpack();
		return Vec3(center.x + yt * cos(u) - s * r.dir.x - r.orig.x, center.y + xt - s * r.dir.y - r.orig.y, center.z + yt * sin(u) - s * r.dir.z - r.orig.z);
	}
	Vec3 solve_lineq(double A[3][3], double b[3])
	{
		//选主元LU分解
		for (int k = 0; k < 2; k++)
		{
			//选主元过程
			int s = k;
			for (int i = k + 1; i < 3; i++)
				if (A[s][k] < A[i][k]) s = i;
			if (s != k)
			{
				std::swap(b[s], b[k]);
				for (int j = 0; j < 3; j++)
					std::swap(A[s][j], A[k][j]);
			}

			//LU分解
			for (int i = k + 1; i < 3; i++)
			{
				A[i][k] = A[i][k] / A[k][k];
				for (int j = k + 1; j < 3; j++)
					A[i][j] = A[i][j] - A[i][k] * A[k][j];
			}
		}

		//回代解方程
		double y[3], x[3];
		for (int i = 0; i < 3; i++)
		{
			y[i] = b[i];
			for (int j = 0; j < i; j++)
				y[i] = y[i] - A[i][j] * y[j];
		}
		for (int i = 2; i >= 0; i--)
		{
			x[i] = y[i];
			for (int j = 2; j > i; j--)
				x[i] = x[i] - A[i][j] * x[j];
			x[i] = x[i] / A[i][i];
		}
		return Vec3(x[0], x[1], x[2]);
	}

public:
	BezierVase(Vec3 cen = Vec3(), int _order = 0, Vec3 *_cont = nullptr, Texture* _texture = nullptr, bool l = false, Opt_Prop _op = BLACK) :
		Geometry(l, _op, BEZIER, _texture), center(cen), curve(new BezierCurve(_order, _cont))
	{
		bound = nullptr;
		set_bound();
	}
	Vec3 dCu(double t, double u) //计算dC/du
	{
		double xt, yt, _;  std::tie(xt, yt, _) = curve->value(t).unpack();
		return Vec3(yt * (-sin(u)), 0.0, yt * cos(u));
	}
	Vec3 dCt(double t, double u) //计算dC/dt
	{
		double dxt, dyt, _; std::tie(dxt, dyt, _) = curve->derive(t).unpack();
		return Vec3(dyt * cos(u), dxt, dyt * sin(u));
	}
	virtual Vec3 get_normvec(Vec3 point) //Warning: 参数为Vec3(t, u, 0)
	//\vec{n} = (\frac{dC}{du} \times \frac{dC}{dt})
	{
		double t, u, _; std::tie(t, u, _) = point.unpack();
		return dCu(t, u).cross(dCt(t, u));
		//return dCt(t, u).cross(dCu(t, u));
	}
	virtual Vec3 get_color(Vec3 point) const // Warning: 参数为Vec3(t, u, 0)
	{
		double t, u, _; std::tie(t, u, _) = point.unpack();
		//printf("(t, u) = (%f, %f), color: ", t, u);
		//texture->query((u / PI + 1) / 2, t).print(); printf("\n");
		return texture->query((u / PI + 1) / 2, t);
	}
	void set_bound()
	{
		std::tie(ymax_point, ymin_point, rmax_point, rmin_point) = curve->maxmin_xy();
		//ymax_point.print(); ymin_point.print(); rmax_point.print(); rmin_point.print();
		//assert: min_y = 0
		double max_y = ymax_point.x, min_y = ymin_point.x, max_r = rmax_point.y, min_r = rmin_point.y;
		if (max_y - min_y > 2 * (max_r - min_r)) //初始包围球:横向较宽或纵向较长的情况
		{
			double y0 = center.y + (max_y - min_y) / 2;
			bound = new Sphere(Vec3(center.x, y0, center.z), new Texture(), (max_y - min_y) / 2);
		}
		else
		{
			double y0 = center.y + rmax_point.x;
			bound = new Sphere(Vec3(center.x, y0, center.z), new Texture(), max_r);
		}

		int bound_count = 0;
		while (bound_count <= curve->order) //迭代求解包围球直到所有控制点被包含
		{
			//printf("bound_count=%d\n", bound_count);
			bound_count = 0;
			for (int i = 0; i <= curve->order; i++)
			{
				Vec3 trans_cont = center + Vec3(curve->cont[i].y, curve->cont[i].x, 0.0);
				if ((trans_cont - bound->center).sqrlen() > sqr(bound->radius))
				{
					//trans_cont.print(); printf("\n"); printf("bound_center:"); bound->center.print(); printf("radius: %f", bound->radius); printf("\n");
					double dy = (trans_cont - bound->center).y / 2;
					bound->center = bound->center + Vec3(0.0, dy, 0.0);
					bound->radius += abs(dy);
					break;
				}
				bound_count++;
			}
		}
	}
	Vec3 fix(Vec3 origsolu) { return Vec3(origsolu.x, fix01(origsolu.y), origsolu.z > PI ? PI : (origsolu.z < -PI ? -PI : origsolu.z)); }

	//解方程L(s) - C(t, u) = 0 s:光线距离 t:Bezier参数 u:交点角度 u\belong (-pi,pi]
	//L(s) = (x_s, y_s, z_s) + s * (x_d, y_d, z_d)
	//C(t,u) = (x_c, y_c, z_c) + (curve_y(t)\cos{u}, curve_x(t), curve_y(t)\sin{u})
	//dC/du = Vec3(curve_y(t)(-\sin{u}), 0, curve_y(t)(\cos{u}))
	//dC/dt = Vec3(curve_y'(t)\cos{u}, curve_x'(t), curve_y'(t)\sin{u})
	//Warning: return Vec3(t, u, 0)
	virtual std::tuple<double, Vec3> intersect(const Ray& r)
	{
		Vec3 p0;
		double s0;
		std::tie(s0, p0) = bound->intersect(r); //先对包围盒求交
		if (s0 > INF - 1) return std::make_tuple(INF, Vec3());
		double x0, y0, z0; std::tie(x0, y0, z0) = (p0 - center).unpack();
		//printf("initial (x, y, z) = (%f, %f, %f)\n", x0, y0, z0);
		double r0 = sqrt(sqr(x0) + sqr(z0));
		//double u0 = z0 < 0 ? -acos(x0 / r0) : acos(x0 / r0); //u0的初值
		//double t0 = fix01(y0 / (ymax_point.x - ymin_point.x)); //用直线来估计t0的初值
		//printf("initial solution (s, t, u)=(%f, %f, %f)\n", s0, t0, u0);

		//取9个点分别求交 找合适的交点
		Vec3 retsolu(INF, 0, 0);
		for(double t0 = 0.2; t0 < 0.81; t0 += 0.3)
			for (double u0 = -2 * PI / 3; u0 < 2.01 * PI / 3; u0 += 2 * PI / 3)
			{
				int step = 0;
				Vec3 solu(s0, t0, u0);
				Vec3 last_solu;
				Vec3 value = update_value(r, solu);

				while (value.norminf() > eps)
				{
					step++;
					last_solu = solu;
					Vec3 dcu = dCu(solu.y, solu.z);
					Vec3 dct = dCt(solu.y, solu.z);
					double jacobi[3][3] = { {-r.dir.x, dct.x, dcu.x}, {-r.dir.y, dct.y, dcu.y}, {-r.dir.z, dct.z, dcu.z} };
					double cvalue[3] = { value.x, value.y, value.z };
					Vec3 delta_solu = solve_lineq(jacobi, cvalue);
					solu = fix(last_solu - delta_solu);
					//printf("present solution:"); solu.print(); printf("\n");
					value = update_value(r, solu);
					//printf("present value:"); value.print(); printf("\n");
					if (step > 15 || !solu.isNormal()) { solu = Vec3(INF, 0, 0); break; }
				}
				if (solu.x < retsolu.x) retsolu = solu;
			}

		//value.print(); printf("\nEND\n");
		return std::make_tuple(retsolu.x, Vec3(retsolu.y, retsolu.z, 0.0));

	}
	~BezierVase() { if (curve) delete curve; if (bound) delete bound; }
};
