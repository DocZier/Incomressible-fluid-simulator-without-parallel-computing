
#include<SFML/Graphics.hpp>
class Field
{
private:
	//Массив значений поля
	double* values;
	//Размеры поля
	int x;
	int y;
public:
	Field();
	void set(int N, int M);
	friend class Fluid;
};
