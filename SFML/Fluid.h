
#include "Field.h"
#include <SFML/Graphics.hpp>

class Fluid
{
private:
	//�������������� ����
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

	//��������� ��������
	double diffusion;
	double visconcy;

	//��������� ���������
	double dt;
	int Solv;
	int N;
	int M;

	//������� ���������� �������� � ��������
	void DiffusionVelocity();
	void DiffusionDensity();
	//������� ���������� ��������
	void AdvectionVelocity();
	void AdvectionDensity();
	//�������� ���������
	void Projection();
	void Boundary(int d, Field f);
	void Swapper(double*& x0, double*& x1);
public:
	Fluid();
	Fluid(int new_N, int new_M, double new_diffusion, double new_visconcy, double new_dt, int new_Solv);

	//������� ��������������
	void ApplyForce(int xprev, int yprev, int x, int y);
	//���������� 
	void Update();
	//�����
	void Reset();
	//���������
	void Render(int size, sf::Image& image);
};
