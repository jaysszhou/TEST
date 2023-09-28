#include <iostream>

int main()
{
	int a = 1;
	int b = 100;
	int res ;
	for(int i = a; i <= b ; i++)
	{
		res += i ;
	}
	std::cout << "res == " << res << std::endl;
	return 0;
}
