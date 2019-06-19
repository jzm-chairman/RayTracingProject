#pragma once
#include "Vec3.hpp"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

enum Texture_Type {PURE, IMAGE};

class Texture
{
	Vec3** image;
	Vec3 color;
	Texture_Type type;
	int height, width, channel;

public:
	Texture(Vec3 col = Vec3()) : image(nullptr), width(0), height(0), color(col), type(PURE) {}
	Texture(const char* filename) : type(IMAGE) { load(filename); }
	Texture_Type get_type() { return type; }
	void load(const char* filename)
	{
		unsigned char* buf = stbi_load(filename, &this->width, &this->height, &channel, 0);
		//if (channel == 3) { printf("3 Channels\n"); }
		//else if (channel == 4) { printf("4 Channels\n"); }
		image = new Vec3*[height];

		for (int i = 0; i < height; i++)
		{
			image[i] = new Vec3[width];
			for (int j = 0; j < width; j++)
			{
				int base = channel * (i * width + j);
				//printf("(%d, %d, %d)\n", buf[base], buf[base + 1], buf[base + 2]);
				image[i][j] = Vec3(pow(buf[base] / 255.0, 2.2), pow(buf[base + 1] / 255.0, 2.2), pow(buf[base + 2] / 255.0, 2.2));
			}
		}
		stbi_image_free(buf);
	}
	Vec3 query(double pw = 0.0, double ph = 0.0)  //参数:(pw,ph)=(x方向上比例,y方向上比例)均在01之间
	{ 
		/*
		if (type == IMAGE)
		{
			printf("query (%f, %f)\n", pw, ph);
			assert(pw > 0 && ph > 0);
		}
		*/
		return type == IMAGE ? image[int((1 - ph) * height)][int((1 - pw) * width)] : color; 
	}
	void save(const char* filename)
	{
		std::ofstream wf(filename);
		wf << "P3\n" << width << " " << height << "\n" << MAX_USHORT << "\n";
		for (int y = 0; y < height; y++)
		{
			for (int x = 0; x < width; x++)
				wf << image[y][x].toRGB().c_str();
			wf << "\n";
		}
		wf.close();
	}
	~Texture()
	{
		for (int i = 0; i < height; i++)
			delete[] image[i];
		delete[] image;
	}
};