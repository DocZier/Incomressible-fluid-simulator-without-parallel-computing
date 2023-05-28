#include "Field.h"

Field::Field()
{
}

void Field::set(int N, int M)
{
	values = new double[(N + 2) * (M + 2)];
	for (int i = 0; i < (N + 2) * (M + 2); i++)
	{
		values[i] = 0;
	}
}
