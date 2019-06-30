#include "Vec3.hpp"
#include "Renderer.hpp"
#include "utils.hpp"

#define WIDTH 2048
#define HEIGHT 1536
#define SAMP 1
#define FILENAME "imageppm1"

int main(int argc, char* argv[])
{
	Scene* scene = new Scene();
	scene->build_scene();
	Renderer renderer(scene);

	renderer.setsize(WIDTH, HEIGHT, SAMP, FILENAME);
	//renderer.ptrender();
	renderer.ppmrender();

	delete scene;
	fprintf(stderr, "\n");
	return 0;
}