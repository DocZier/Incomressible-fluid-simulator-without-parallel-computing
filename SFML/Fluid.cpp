#include "Fluid.h"
#include <iostream>
#include <algorithm>
Fluid::Fluid()
{
}


Fluid::Fluid(int new_N, int new_M, double new_diffusion, double new_visconcy, double new_dt, int new_Solv)
{
	N = new_N;
	M = new_M;
	diffusion = new_diffusion;
	visconcy = new_visconcy;
	dt = new_dt;
	Solv = new_Solv;
	u.set(N, M);
	v.set(N, M);
	density.set(N, M);
	u0.set(N, M);
	v0.set(N, M);
	density0.set(N, M);
	uForce.set(N, M);
	vForce.set(N, M);
	densityForce.set(N, M);
	div.set(N, M);
	p.set(N, M);
}

void Fluid::Update()
{
	for (int x = 0; x < N; x++)
	{
		for (int y = 0; y < M; y++)
		{
			u0.values[x + y * N] = uForce.values[x + y * N];
			v0.values[x + y * N] = vForce.values[x + y * N];

			uForce.values[x + y * N] = 0;
			vForce.values[x + y * N] = 0;
		}
	}

	DiffusionVelocity();
	Projection();

	Swapper(u0.values, u.values);
	Swapper(v0.values, v.values);

	AdvectionVelocity();
	Projection();

	for (int x = 1; x <= N; x++)
	{
		for (int y = 1; y <= M; y++)
		{
			density0.values[x + y * (N + 2)] += densityForce.values[x + y * (N + 2)];
			densityForce.values[x + y * (N + 2)] = 0;
		}
	}
	DiffusionDensity();
	Swapper(density0.values, density.values);
	AdvectionDensity();
}

void Fluid::Reset()
{
	for (int x = 1; x <= N; x++)
	{
		for (int y = 1; y <= M; y++)
		{
			u0.values[x + y * (N + 2)] = 0;
			v0.values[x + y * (N + 2)] = 0;
			u.values[x + y * (N + 2)] = 0;
			v.values[x + y * (N + 2)] = 0;
			density0.values[x + y * (N + 2)] = 0;
			densityForce.values[x + y * (N + 2)] = 0;
		}
	}
}

void Fluid::Render(int size, sf::Image& image)
{
	for (int x = 0; x <= N + 1; x++)
	{
		for (int y = 0; y <= M + 1; y++)
		{
			if (density.values[x + y * (N + 2)] > 1)
			{
				density.values[x + y * (N + 2)] = 1;
			}
			for (int i = 0; i < size; i++)
			{
				for (int j = 0; j < size; j++)
				{
					image.setPixel(x * size + i, y * size + j, sf::Color(255 * density.values[x + y * (N + 2)], 255 * density.values[x + y * (N + 2)], 255 * density.values[x + y * (N + 2)]));
				}
			}
			density0.values[x + y * (N + 2)] = density.values[x + y * (N + 2)];
			u0.values[x + y * (N + 2)] = u.values[x + y * (N + 2)];
			v0.values[x + y * (N + 2)] = v.values[x + y * (N + 2)];
		}
	}
}

void Fluid::ApplyForce(int x0, int y0, int x1, int y1)
{
	int dx = x1 - x0;
	int dy = y1 - y0;
	if (abs(dx) >= abs(dy))
	{
		int yi = 1;
		if (dy < 0)
		{
			yi = -1;
			dy = -dy;
		}
		int D = 2 * dy - dx;
		int y = y0;
		int elementID = 0;
		if (dx > 0)
		{
			for (int x = x0; x <= x1; x++)
			{
				elementID = x + (N + 2) * y;
					densityForce.values[elementID] = 1;
					uForce.values[elementID] = (x1 - x0) / dt;
					vForce.values[elementID] = (y1 - y0) / dt;
				if (D > 0)
				{
					y += yi;
					D -= 2 * dx;
				}
				D += 2 * dy;
			}
		}
		else if (dx < 0)
		{
			D = 2 * dy + dx;
			for (int x = x0; x >= x1; x--)
			{
				elementID = x + (N + 2) * y;
					densityForce.values[elementID] = 1;
					uForce.values[elementID] = (x1 - x0) / dt;
					vForce.values[elementID] = (y1 - y0) / dt;
				if (D > 0)
				{
					y += yi;
					D += 2 * dx;
				}
				D += 2 * dy;
			}
		}
	}
	else if (abs(dy) > abs(dx))
	{
		int xi = 1;
		if (dx < 0) {
			xi = -1;
			dx = -dx;
		}
		int D = 2 * dx - dy;
		int x = x0;
		int elementID = 0;
		if (dy > 0)
		{
			for (int y = y0; y <= y1; y++)
			{
				elementID = x + (N + 2) * y;
					densityForce.values[elementID] = 1;
					uForce.values[elementID] = (x1 - x0) / dt;
					vForce.values[elementID] = (y1 - y0) / dt;
				if (D > 0)
				{
					x += xi;
					D -= 2 * dy;
				}
				D += 2 * dx;
			}
		}
		else if (dy < 0)
		{
			D = 2 * dx + dy;
			for (int y = y0; y >= y1; y--)
			{
				elementID = x + (N + 2) * y;
					densityForce.values[elementID] = 1;
					uForce.values[elementID] = (x1 - x0) / dt;
					vForce.values[elementID] = (y1 - y0) / dt;
				if (D > 0)
				{
					x += xi;
					D += 2 * dy;
				}
				D += 2 * dx;
			}
		}
	}
}

void Fluid::DiffusionVelocity()
{
	for (int x = 1; x <= N; x++)
	{
		for (int y = 1; y <= M; y++)
		{
				u.values[x + y * (N + 2)] = u0.values[x + y * (N + 2)] + (0.999 + visconcy / 1000) * (0.25 * u.values[x - 1 + y * (N + 2)] + 0.25 * u.values[x + 1 + y * (N + 2)] + 0.25 * u.values[x + (y - 1) * (N + 2)] + 0.25 * u.values[x + (y + 1) * (N + 2)] - u0.values[x + y * (N + 2)]);
				v.values[x + y * (N + 2)] = v0.values[x + y * (N + 2)] + (0.999 + visconcy / 1000) * (0.25 * v.values[x - 1 + y * (N + 2)] + 0.25 * v.values[x + 1 + y * (N + 2)] + 0.25 * v.values[x + (y - 1) * (N + 2)] + 0.25 * v.values[x + (y + 1) * (N + 2)] - v0.values[x + y * (N + 2)]);
		}
	}
	Boundary(1, u);
	Boundary(2, v);
}


void Fluid::AdvectionVelocity()
{
	for (int x = 1; x <= N; x++)
	{
		for (int y = 1; y <= M; y++)
		{
			double ax = 0;
			double ay = 0;
			int ax0 = 0;
			int ax1 = 0;
			int ay0 = 0;
			int ay1 = 0;
			double s0 = 0;
			double s1 = 0;
			double t0 = 0;
			double t1 = 0;
			double dt0 = dt * (N);

			ax = x - dt0 * u.values[x + N * y];
			ay = y - dt0 * v.values[x + N * y];

			if (ax < 0.5)
			{
				ax = 0.5;
			}
			if (ax > N + 0.5)
			{
				ax = N + 0.5;
			}
			ax0 = (int)ax;
			ax1 = ax0 + 1;
			if (ay < 0.5)
			{
				ay = 0.5;
			}
			if (ay > N + 0.5)
			{
				ay = N + 0.5;
			}
			ay0 = (int)ay;
			ay1 = ay0 + 1;
			s1 = ax - ax0;
			s0 = 1 - s1;
			t1 = ay - ay0;
			t0 = 1 - t1;
			{
				u.values[x + (N + 2) * y] = s0 * (t0 * u0.values[ax0 + ay0 * (N + 2)] + t1 * u0.values[ax0 + ay1 * (N + 2)]) +
					s1 * (t0 * u0.values[ax1 + ay0 * (N + 2)] + t1 * u0.values[ax1 + ay1 * (N + 2)]);
				v.values[x + y * (N + 2)] = s0 * (t0 * v0.values[ax0 + ay0 * (N + 2)] + t1 * v0.values[ax0 + ay1 * (N + 2)]) +
					s1 * (t0 * v0.values[ax1 + ay0 * (N + 2)] + t1 * v0.values[ax1 + ay1 * (N + 2)]);
			}
		}
	}
	Boundary(1, u);
	Boundary(2, v);
}


void Fluid::Projection()
{
	float h = 1.0 / N;
	for (int i = 1; i <= N; i++)
	{
		for (int j = 1; j <= M; j++)
		{
			div.values[i + j * (N + 2)] = -0.5 * h * (u.values[i + 1 + j * (N + 2)] - u.values[i - 1 + j * (N + 2)] + v.values[i + (j + 1) * (N + 2)] - v.values[i + (j - 1) * (N + 2)]);
			p.values[i + j * (N + 2)] = 0;
		}
	}

	Boundary(1, u);
	Boundary(2, v);

	for (int k = 0; k < Solv; k++)
	{
		for (int i = 1; i <= N; i++)
		{
			for (int j = 1; j <= M; j++)
			{
				p.values[i + j * (N + 2)] = (div.values[i + j * (N + 2)] + p.values[i - 1 + j * (N + 2)] + p.values[i + 1 + j * (N + 2)] + p.values[i + (j - 1) * (N + 2)] + p.values[i + (j + 1) * (N + 2)]) / 4;
			}
		}
	}

	Boundary(0, density);

	for (int i = 1; i <= N; i++)
	{
		for (int j = 1; j <= M; j++)
		{
			u.values[i + j * (N + 2)] -= 0.5 * (p.values[i + 1 + j * (N + 2)] - p.values[i - 1 + j * (N + 2)]) / h;
			v.values[i + j * (N + 2)] -= 0.5 * (p.values[i + (j + 1) * (N + 2)] - p.values[i + (j - 1) * (N + 2)]) / h;
		}
	}

	Boundary(1, u);
	Boundary(2, v);
}


void Fluid::DiffusionDensity()
{
	for (int x = 1; x <= N; x++)
	{
		for (int y = 1; y <= M; y++)
		{
			density.values[x + y * (N + 2)] = density0.values[x + y * (N + 2)] + diffusion * (0.25 * density.values[x - 1 + y * (N + 2)] + 0.25 * density.values[x + 1 + y * (N + 2)] +
				0.25 * density.values[x + (y - 1) * (N + 2)] + 0.25 * density.values[x + (y + 1) * (N + 2)] - density0.values[x + y * (N + 2)]);
		}
	}
	Boundary(0, density);
}


void Fluid::AdvectionDensity()
{
	double ax = 0;
	double ay = 0;
	int ax0 = 0;
	int ax1 = 0;
	int ay0 = 0;
	int ay1 = 0;
	double s0 = 0;
	double s1 = 0;
	double t0 = 0;
	double t1 = 0;
	double dt0 = dt * (N);
	for (int x = 1; x <= N; x++)
	{
		for (int y = 1; y <= N; y++)
		{
			ax = x - dt0 * u.values[x + (N + 2) * y];
			ay = y - dt0 * v.values[x + (N + 2) * y];
			if (ax < 0.5)
			{
				ax = 0.5;
			}
			if (ax > (N + 2) - 1.5)
			{
				ax = (N + 2) - 1.5;
			}
			ax0 = (int)ax;
			ax1 = ax0 + 1;

			if (ay < 0.5) {
				ay = 0.5;
			}
			if (ay > (N + 2) - 1.5)
			{
				ay = (N + 2) - 1.5;
			}
			ay0 = (int)ay;
			ay1 = ay0 + 1;
			s1 = ax - ax0;
			s0 = 1 - s1;
			t1 = ay - ay0;
			t0 = 1 - t1;
			density.values[x + (N + 2) * y] = s0 * (t0 * density0.values[ax0 + (N + 2) * ay0] + t1 * density0.values[ax0 + (N + 2) * ay1]) +
				s1 * (t0 * density0.values[ax1 + (N + 2) * ay0] + t1 * density0.values[ax1 + (N + 2) * ay1]);
		}
	}
	Boundary(0, density);
}

void Fluid::Boundary(int d, Field f)
{
	for (int i = 1; i <= N; i++)
	{

			f.values[0 + (N + 2) * i] = (d == 1 ? -f.values[1 + (N + 2) * i] : f.values[1 + (N + 2) * i]);
			f.values[(N + 1) + (N + 2) * i] = (d == 1 ? -f.values[N + (N + 2) * i] : f.values[N + (N + 2) * i]);
			f.values[i + (N + 2) * 0] = (d == 2 ? -f.values[i + (N + 2) * 1] : f.values[i + (N + 2) * 1]);
			f.values[i + (N + 2) * (N + 1)] = (d == 2 ? -f.values[i + (N + 2) * (N)] : f.values[i + (N + 2) * (N)]);
			v.values[0 + (N + 2) * i] = 0;
			v.values[(N + 1) + (N + 2) * i] = 0;
			v.values[i + (N + 2) * 0] = 0;
			v.values[i + (N + 2) * (N + 1)] = 0;
			u.values[0 + (N + 2) * i] = 0;
			u.values[(N + 1) + (N + 2) * i] = 0;
			u.values[i + (N + 2) * 0] = 0;
			u.values[i + (N + 2) * (N + 1)] = 0;
	}
		f.values[0 + (N + 2) * 0] = 0.5 * (f.values[1 + (N + 2) * 0] + f.values[0 + (N + 2) * 1]);
		f.values[0 + (N + 2) * (N + 1)] = 0.5 * (f.values[1 + (N + 2) * (N + 1)] + f.values[0 + (N + 2) * (N)]);
		f.values[N + 1 + 0] = 0.5 * (f.values[N + 0] + f.values[N + 1 + 1]);
		f.values[N + 1 + (N + 2) * (N + 1)] = 0.5 * (f.values[N + (N + 2) * (N + 1)] + f.values[N + 1 + (N + 2) * (N)]);
}

void Fluid::Swapper(double*& x0, double*& x1)
{
	double* memory = x0;
	x0 = x1;
	x1 = memory;
}
