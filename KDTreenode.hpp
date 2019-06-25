#pragma once
#include "Vec3.hpp"
#include "KDTree.hpp"

#define R0 0.2
#define ALPHA 0.81

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

class HitPoint //逆向追踪的碰撞点
{
public:
	Vec3 pos; //球心位置
	Vec3 col; //颜色
	Vec3 normvec; //法向量
	std::tuple<int, int> viewpoint; //对应的视点坐标
	double ptcall, ptclast; //光子数量(合计数量和上次接收的数量)
	double rad; //对应半径
	static int depth;

	HitPoint(Vec3 _pos = Vec3(), Vec3 _col = Vec3(), Vec3 _normvec = Vec3(), std::tuple<int, int> _vp = std::tuple<int, int>()) :
		pos(_pos), col(_col), normvec(_normvec), viewpoint(_vp), ptcall(0), ptclast(0), rad(R0) {}
	HitPoint(const HitPoint& h) : HitPoint(h.pos, h.col, h.normvec, h.viewpoint) {}
	void update()
	{
		if (ptcall == 0 && ptclast == 0) return;
		//rad = rad * ALPHA;
		rad = rad * sqrt((ptcall + ALPHA * ptclast) / (ptcall + ptclast));
		ptcall = ptcall + ALPHA * ptclast;
		ptclast = 0;
	}
	bool operator<(const HitPoint& n) const //层内排序使用
	{
		switch (depth)
		{
		case 0: return pos.x < n.pos.x;
		case 1: return pos.y < n.pos.y;
		case 2: return pos.z < n.pos.z;
		}
	}
	void print() const
	{
		printf("( %.2lf %.2lf %.2lf )\n ", pos.x, pos.y, pos.z);
	}
};

class Node
{
public:
	HitPoint hp;
	Vec3 boundmin, boundmax; //对应球的包围盒的算术最大值和最小值边界
	Node* left;
	Node* right;
	Node(const HitPoint& _hp, Node* _left = nullptr, Node* _right = nullptr) :
		hp(_hp), left(_left), right(_right) { initbound(); }
	void initbound() { boundmin = hp.pos - hp.rad; boundmax = hp.pos + hp.rad; }
	void updatebound() //层次包围盒?
	{
		Vec3 lbmin = left ? left->boundmin : Vec3(INF, INF, INF);
		Vec3 lbmax = left ? left->boundmax : Vec3(-INF, -INF, -INF);
		Vec3 rbmin = right ? right->boundmin : Vec3(INF, INF, INF);
		Vec3 rbmax = right ? right->boundmax : Vec3(-INF, -INF, -INF);
		boundmin = minvec(boundmin, minvec(lbmin, rbmin));
		boundmax = maxvec(boundmax, maxvec(lbmax, rbmax));
	}
};
int HitPoint::depth = 0;