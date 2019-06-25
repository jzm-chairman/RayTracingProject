#include "Vec3.hpp"
#include "Renderer.hpp"
#include "utils.hpp"

/*void test()
{
	KDTree test;
	std::vector<HitPoint> h;
	srand(1);
	for (int i = 0; i < 100000; i++)
		h.push_back(HitPoint(Vec3(rand() % 1000, rand() % 1000, rand() % 1000)));
	//printf("elem:\n"); for (int i = 0; i < 9; i++) h[i].print();
	test.build(test.root, h, 0, 0, h.size());
	//printf("traverse:\n"); test.traverse(test.root);
	for (int i = 0; i < 1000; i++)
	{
		auto qp = RayPoint(Vec3(rand() % 1000, rand() % 1000, rand() % 1000));
		int count = test.query(test.root, qp);
		int realcount = 0;
		for (auto it = h.begin(); it != h.end(); ++it)
		{
			if ((qp.pos - it->pos).sqrlen() < sqr(it->rad)) realcount++;
		}
		printf("Iteration %d, count = %d, realcount = %d\n", i, count, realcount);
		assert(count == realcount);
	}
}*/

int main(int argc, char* argv[])
{
	if (argc != 5) { std::cerr << "Uasge: ./main <width> <height> <samples> <output file>\n"; exit(0); }
	Scene* scene = new Scene();
	scene->build_scene();
	Renderer renderer(scene);

	renderer.setsize(atoi(argv[1]), atoi(argv[2]), atoi(argv[3]) / 4, argv[4]);
	//renderer.ptrender();
	renderer.ppmrender();

	delete scene;
	fprintf(stderr, "\n");
	return 0;
}