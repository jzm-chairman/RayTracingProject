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
	{  //����Ǹ��ݵ�ǰ���Ĳ㣬�ǰ�x ���� y ���� z���֣���ȡ�ö�Ӧ���±꣬0 Ϊ x�� 1 Ϊ y ��2 Ϊz
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

class KDTree //TODO: �Ӷ���
{
public:
	Node* root;

	KDTree() : root(nullptr) {}
	void build(Node*& present, const std::vector<Vec3>& points, int depth, int begin, int end) {
		//present�Ǹ���㣬LR���жϴ�ʱ�������������������������ı�ǣ�0Ϊ��1Ϊ��
		//points�����������
		//depth���жϵ�ǰ��η����׼�ı�ǣ�0Ϊx��1Ϊy��2Ϊz
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
		//present�����ĸ���㣬points�Ǵ�����������target��Ҫ��ѯ�ĵ��node
		//instance����Ҫ��ѯ�ľ���
		//depth�ǵ�ǰ���Ĳ�εķ����׼��0Ϊx��1Ϊy��2Ϊz
		if (present) 
		{
			//present->print();
			//printf("%lf\n", present->getByOrder(depth) - target.getByOrder(depth));
			double pd1 = present->getByOrder(depth);
			double td1 = target.getByOrder(depth);
			depth = (depth + 1) % DIM;
			if (abs(td1 - pd1) < instance) //�����ǰ��ά�Ȳ�С��Instance
			{
				if ((present->vec - target.vec).sqrlen() < sqr(instance)) { //�����ǰ�����Ͼ��룬�����
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