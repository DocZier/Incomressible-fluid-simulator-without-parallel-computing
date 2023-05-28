
#include<SFML/Graphics.hpp>
class Field
{
private:
	//Ìàññèâ çíà÷åíèé ïîëÿ
	double* values;
	//Ðàçìåðû ïîëÿ
public:
	Field();
	void set(int N, int M);
	friend class Fluid;
};
