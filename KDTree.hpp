#pragma once
#include "Vec3.hpp"
#include "utils.hpp"
#include "KDTreenode.hpp"
#include "Viewpoint.hpp"

#define DIM 3

class KDTree
{
public:
	Node* root;

	KDTree() : root(nullptr) {}
	~KDTree() { clean(root); }
	void build(Node*& present, std::vector<HitPoint>& points, int depth, int begin, int end)
	{
		if (end - begin <= 0)
			return;
		int mid = (begin + end) >> 1;
		auto st = points.begin();
		HitPoint::depth = depth;
		std::nth_element(st + begin, st + mid, st + end);
		present = new Node(points[mid]);
		depth = (depth + 1) % DIM;
		build(present->left, points, depth, begin, mid);
		build(present->right, points, depth, mid + 1, end);
		present->updatebound();
	}

	void query(Node* present, std::vector<std::vector<ViewPoint>>& view, const RayPoint& rp)
	{
		if (present == nullptr) return;
		if (rp.pos.outrange(present->boundmin, present->boundmax))
			return;
		if ((rp.pos - present->hp.pos).sqrlen() < sqr(present->hp.rad))
		{
			present->hp.ptclast += 1;
			int h, w; std::tie(h, w) = present->hp.viewpoint;
			view[h][w].add(rp.its.mult(present->hp.col) * rp.prob, rp.prob);
		}
		query(present->left, view, rp);
		query(present->right, view, rp);
	}
	/*int query(Node* present, const RayPoint& rp)
	{
		if (present == nullptr) return 0;
		if (rp.pos.outrange(present->boundmin, present->boundmax))
			return 0;
		if ((rp.pos - present->hp.pos).sqrlen() < sqr(present->hp.rad))
			return 1 + query(present->left, rp) + query(present->right, rp);
		return query(present->left, rp) + query(present->right, rp);
	}*/
	void traverse(Node* present)
	{
		if (present != nullptr) {
			printf("GO LEFT\n");
			traverse(present->left);
			present->hp.print();
			printf("GO RIGHT\n");
			traverse(present->right);
		}
		printf("RETURN\n");
	}
	void reducerad(Node* present)
	{
		if (present == nullptr) return;
		present->hp.update();
		present->initbound();
		reducerad(present->left);
		reducerad(present->right);
		present->updatebound();
	}

	void clean(Node* present)
	{
		if (present == nullptr) return;
		clean(present->left);
		clean(present->right);
		delete present;
	}
};