#include "COC.h"
#include <iostream>

ClashOfClans::ClashOfClans()
{
    std::cout << "Welcome to COC" << std::endl;
}

LightDragen::LightDragen()
{
    std::cout << "Create a LightGragen" << std::endl;
}

Savage::Savage()
{
    std::cout << "Create a Savage" << std::endl;
}

void LightDragen::Attack()
{
    std::cout << "LightDragen round " << std::endl;
}

void Savage::Attack()
{
    std::cout << "Savage round " << std::endl;
}
