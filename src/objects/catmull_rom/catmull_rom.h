#pragma once
#include "../coordinates/coordinates.h"

class CatmullRom {

private:
	Coordinates coordinates[];

public:
	CatmullRom(Coordinates coordinates[]);

};