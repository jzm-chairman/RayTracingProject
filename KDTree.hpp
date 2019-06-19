#pragma once
#include "Vec3.hpp"
#include "utils.hpp"

#define DIM 3
class Node
{
public:
	Vec3 vec;
	Node* left;
	Node* right;
	Node(Vec3 _vec = Vec3(), Node* _left = nullptr, Node* _right = nullptr) : vec(_vec), left(_left), right(_right) {}

	double getByOrder(int order) const
	{  //这个是根据当前树的层，是按x 或者 y 或者 z划分，来取得对应的下标，0 为 x， 1 为 y ，2 为z
		return order == 0 ? vec.x : (order == 1 ? vec.y : (order == 2 ? vec.z : 0));
	}
	void print() const
	{
		printf("( %.2lf, %.2lf, %.2lf )\n ", this->vec.x, this->vec.y, this->vec.z);
	}
};

bool xcomp(const Vec3 &v1, const Vec3 &v2) { return v1.x < v2.x; }
bool ycomp(const Vec3 &v1, const Vec3 &v2) { return v1.y < v2.y; }
bool zcomp(const Vec3 &v1, const Vec3 &v2) { return v1.z < v2.z; }

class KDTree //TODO: 加东西
{
public:
	Node* root;

	KDTree() : root(nullptr) {}
	void build(Node*& present, const std::vector<Vec3>& points, int depth, int begin, int end) {
		//present是根结点，LR是判断此时构建的是左子树还是右子树的标记，0为左，1为右
		//points是输入的容器
		//depth是判断当前层次分类标准的标记，0为x，1为y，2为z
		if (end - begin <= 0)
			return;
		int mid = (begin + end) >> 1;
		auto st = points.begin();
		if (depth == 0)
			std::nth_element(st + begin, st + mid, st + end, xcomp);
		else if (depth == 1)
			std::nth_element(st + begin, st + mid, st + end, ycomp);
		else if (depth == 2)
			std::nth_element(st + begin, st + mid, st + end, zcomp);
		present = new Node(points[mid], nullptr, nullptr);
		depth = (depth + 1) % DIM;
		build(present->left, points, depth, begin, mid);
		build(present->right, points, depth, mid + 1, end);
	}

	void query(Node* present, std::vector<Vec3>& points, const Node& target, double instance, int depth) {
		//present是树的根结点，points是存结果的容器，target是要查询的点的node
		//instance就是要查询的距离
		//depth是当前树的层次的分类标准，0为x，1为y，2为z
		if (present) 
		{
			//present->print();
			//printf("%lf\n", present->getByOrder(depth) - target.getByOrder(depth));
			double pd1 = present->getByOrder(depth);
			double td1 = target.getByOrder(depth);
			depth = (depth + 1) % DIM;
			if (abs(td1 - pd1) < instance) //如果当前层维度差小于Instance
			{
				if ((present->vec - target.vec).sqrlen() < sqr(instance)) { //如果当前结点符合距离，则加入
					points.push_back(present->vec);
					//printf("push this: ( %lf, %lf, %lf ) \n",avec->x, avec->y, avec->z);
				}
				query(present->left, points, target, instance, depth);
				query(present->right, points, target, instance, depth);
			}
			else if (td1 < pd1 - instance)
				query(present->left, points, target, instance, depth);
			else if (td1 > pd1 + instance)
				query(present->right, points, target, instance, depth);
		}
	}
	void traverse(Node* present) {
		if (present != nullptr) {
			traverse(present->left);
			present->print();
			traverse(present->right);
		}
	}
};