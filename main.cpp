#include "Vec3.hpp"
#include "Renderer.hpp"
#include "utils.hpp"
#include <omp.h>

int main(int argc, char* argv[])
{
	if (argc != 5) { std::cerr << "Uasge: ./main <width> <height> <samples> <output file>\n"; exit(0); }
	int width = atoi(argv[1]), height = atoi(argv[2]), samples = atoi(argv[3]) / 4;
	Ray camera(Vec3(50, 52, 296), Vec3(0, -0.04, -1).norm()); //相机源点，视线方向
	Vec3 cx = Vec3(width * 0.5 / height, 0, 0), cy = cx.cross(camera.dir).norm() * 0.5; //???
	Vec3** vision = new Vec3*[width]; //屏幕视场
	for (int i = 0; i < height; i++)
		vision[i] = new Vec3[width];

	/*
	Texture test("1.jpg");
	test.save("11.ppm");
	exit(0);
	*/
	
	Scene* scene = new Scene();
	scene->build_scene();
	Renderer renderer(scene);
	//printf("cx:"); cx.print(); printf("cy:"); cy.print();  system("pause");

#	pragma omp parallel for num_threads(6)\
		schedule(dynamic, 1)
	for (int y = 0; y < height; y++) //逐行渲染
	{
		fprintf(stderr, "\rPath Tracing Rendering %d / %d Rows (%d Samples)", y + 1, height, samples * 4);
		for (int x = 0; x < width; x++)
			for (int suby = 0; suby < 2; suby++) //每像素分成四个子域分别采样
				for (int subx = 0; subx < 2; subx++)
				{
					Vec3 ray_intense; //该像素点的累积光强
					for (int s = 0; s < samples; s++)
					{
						double rand1 = 2 * random01(), rand2 = 2 * random01();
						double dx = rand1 < 1 ? sqrt(rand1) - 1 : 1 - sqrt(2 - rand1);
						double dy = rand2 < 1 ? sqrt(rand2) - 1 : 1 - sqrt(2 - rand2);
						Vec3 ray_dir = cx * ((x + (subx + dx + 0.5) / 2) / width - 0.5) + cy * ((y + (suby + dy + 0.5) / 2) / height - 0.5) + camera.dir; //???
						//printf("ray direction:"); ray_dir.print();
						Vec3 delta_intense = renderer.pathtrace(Ray(camera.orig + ray_dir * 140, ray_dir.norm()), 0) / samples;
						ray_intense = ray_intense + delta_intense;
					}
					//printf("ray intensity:"); ray_intense.print();
					vision[y][x] = vision[y][x] + (ray_intense.fix() / 4);
					//printf("(%d, %d):", y, x);
					//vision[y][x].print();
				}
	}

	std::string filename = argv[4];
	std::ofstream wf(filename);
	wf << "P3\n" << width << " " << height << "\n" << MAX_USHORT << "\n";
	for (int y = height - 1; y >= 0; y--)
	{
		for (int x = width - 1; x >= 0; x--)
			wf << vision[y][x].toRGB().c_str();
		wf << "\n";
	}
	wf.close();

	for (int i = 0; i < height; i++)
		delete[] vision[i];
	delete[] vision;
	delete scene;
	fprintf(stderr, "\n");
	return 0;
}