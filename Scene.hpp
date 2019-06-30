#pragma once
#include "Geometry.hpp"
#include "Ray.hpp"

#define VASESIZE 1.5

class Scene
{
public:
	std::vector<Geometry*> objects;
	Scene() : objects(std::vector<Geometry*>()) {}
	void add_object(Geometry* object) { objects.push_back(object); }
	void build_scene();
	std::tuple<Geometry*, Vec3> intersect_any(const Ray& r);
	~Scene()
	{
		for (unsigned int i = 0; i < objects.size(); i++)
			delete objects[i];
	}
};

void Scene::build_scene()
{
	Vec3 front_edge[2] = { Vec3(100, 0, 0), Vec3(0, 81.6, 0) };
	Vec3 left_dec_edge1[2] = { Vec3(0, 0, 25), Vec3(-25, 0, 0) };
	//Vec3 left_dec_edge2[2] = { Vec3(0, 15, 15), Vec3(-20, 0, 0) };
	Vec3 vase_controller[] = { Vec3(0, 0) * VASESIZE, Vec3(1, 11.8) * VASESIZE, Vec3(10, 11.4) * VASESIZE,
		Vec3(11.6, 1.4) * VASESIZE, Vec3(18.8, 1.6) * VASESIZE, Vec3(23.4, 5) * VASESIZE };
	add_object(new Sphere(Vec3(1e5 + 1, 40.8, 81.6), new Texture(Vec3(0.75, 0.25, 0.25)), 1e5, false, DIFF)); //Left
	add_object(new Sphere(Vec3(-1e5 + 99, 40.8, 81.6), new Texture(Vec3(0.25, 0.25, 0.75)), 1e5, false, DIFF)); //Rght
	//add_object(new Sphere(Vec3(50, 40.8, 1e5), new Texture(Vec3(0.75, 0.75, 0.75)), 1e5, false, DIFF)); //Back
	add_object(new Sphere(Vec3(50, 40.8, -1e5 + 170), new Texture(Vec3(0.25, 0.25, 0.25)), 1e5, false, DIFF)); //Frnt
	add_object(new Plane(Vec3(0, 0, 1), Vec3(0, 0, 0), front_edge, new Texture("1.jpg"), false, DIFF));
	add_object(new Sphere(Vec3(50, 1e5, 81.6), new Texture(Vec3(0.5, 0.5, 0.5)), 1e5, false, DIFF)); //Botm
	add_object(new Sphere(Vec3(50, -1e5 + 81.6, 81.6), new Texture(Vec3(0.75, 0.75, 0.75)), 1e5, false, DIFF)); //Top
	add_object(new Sphere(Vec3(27, 16.5, 47), new Texture(Vec3(1, 1, 1) * 0.999), 16.5, false, SPEC)); //Mirr
	//add_object(new Sphere(Vec3(73, 16.5, 78), new Texture(Vec3(1, 1, 1) * 0.999), 16.5, false, REFR)); //Glas
	add_object(new Sphere(Vec3(87.5, 52.5, 102.5), new Texture(Vec3(1, 1, 1) * 0.999), 12.5, false, REFR)); //Glas
	//add_object(new Sphere(Vec3(87.5, 52.5, 102.5), new Texture(Vec3(0.25, 0.75, 0.25)), 12.5, false, DIFF)); //Glas
	add_object(new BezierVase(Vec3(60, -2, 95), 5, vase_controller, new Texture("4.jpg"), false, DIFS));
	add_object(new Sphere(Vec3(12.5, 62.5, 82.5), new Texture(Vec3(0.25, 0.75, 0.25)), 12.5, false, DIFF));//Glas
	add_object(new Sphere(Vec3(50, 84, 81.6), new Texture(Vec3(1, 1, 1)), 10, false, DIFR));
	//add_object(new Sphere(Vec3(50, 681.6 - .27, 81.6), new Texture(Vec3(1, 1, 1) * 12), 600, true, DIFF)); //Lite
	add_object(new Plane(Vec3(0, 1, 0), Vec3(100, 40, 90), left_dec_edge1, new Texture("2.jpg"), false, DIFF));
	add_object(new Plane(Vec3(0, 1, 0), Vec3(25, 50, 70), left_dec_edge1, new Texture("2.jpg"), false, DIFF));
	//add_object(new Plane(Vec3(0, 1, -1).norm(), Vec3(100, 20, 50), left_dec_edge2, new Texture("2.jpg"), false, DIFF));
};

std::tuple<Geometry*, Vec3> Scene::intersect_any(const Ray& r) //返回与光线相交对象指针和交点位置
{
	//printf("object count = %d\n", objects.size());
	double min_dist = INF - 1, dist;
	Geometry* iobj = nullptr;
	Vec3 ip = Vec3(), p = Vec3();
	for (Geometry* obj : objects)
	{
		std::tie(dist, p) = obj->intersect(r);
		if (dist < min_dist) iobj = obj, ip = p, min_dist = dist;
	}
	return std::make_tuple(iobj, ip);
}