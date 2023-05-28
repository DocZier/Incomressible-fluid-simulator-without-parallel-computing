
#include "Field.h"
#include <SFML/Graphics.hpp>

class Fluid
{
private:
	//Характеристики поля
	Field u;
	Field v;
	Field density;
	Field u0;
	Field v0;
	Field density0;
	Field uForce;
	Field vForce;
	Field densityForce;
	Field div;
	Field p;

	//Параметры жидкости
	double diffusion;
	double visconcy;

	//Параметры симуляции
	double dt;
	int Solv;
	int N;
	int M;

	//Функции вычисления диффузии в жидкости
	void DiffusionVelocity();
	void DiffusionDensity();
	//Функции вычисления адвекции
	void AdvectionVelocity();
	void AdvectionDensity();
	//Проекция скоростей
	void Projection();
	void Boundary(int d, Field f);
	void Swapper(double*& x0, double*& x1);
public:
	Fluid();
	Fluid(int new_N, int new_M, double new_diffusion, double new_visconcy, double new_dt, int new_Solv);

	//Функция взаимодействия
	void ApplyForce(int xprev, int yprev, int x, int y);
	//Обновление 
	void Update();
	//Сброс
	void Reset();
	//Отрисовка
	void Render(int size, sf::Image& image);
};
