
#include<SFML/Graphics.hpp>
class Field
{
private:
	//������ �������� ����
	double* values;
	//������� ����
	int x;
	int y;
public:
	Field();
	void set(int N, int M);
	friend class Fluid;
};
