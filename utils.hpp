#pragma once
#include <cmath>
#include <cstdlib>
#include <tuple>
#include <iostream>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <vector>
#include <cassert>
#include <string>

#define sqr(x) ((x) * (x))
#define sign(x) ((x) > 0 ? 1 : -1)
#define abs(x) ((x) > 0 ? (x) : -(x))
#define random01() ((double)rand() / RAND_MAX)
#define round(x) ((int)((x) + 0.5))
#define fix01(x) ((x) < 0 ? 0 : ((x) > 1 ? 1 : (x)))

#define PI acos(-1)
#define eps 1e-6
#define INF 1e10
#define MAX_USHORT 255

int _fac[100];
bool flag = false;

unsigned short torgb(double orig) { return round(MAX_USHORT * pow(fix01(orig), 1 / 2.2)); }
//unsigned short torgb(double orig) { return round(MAX_USHORT * fix01(orig)); }
int fac(int n)
{
	if (!flag) { flag = true; memset(_fac, 0, sizeof(int) * 100); _fac[0] = _fac[1] = 1; }
	if (n <= 1) return 1;
	if (_fac[n] == 0) _fac[n] = n * fac(n - 1);
	return _fac[n];
}
int comp(int n, int i) { return fac(n) / (fac(i) * fac(n - i)); } //assert: n >= i

enum Opt_Prop { BLACK, DIFF, DIFR, DIFS, SPEC, REFR };
enum Object_Type {NONE, SPHERE, PLANE, BEZIER};