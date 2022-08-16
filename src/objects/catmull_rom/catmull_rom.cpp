#include "catmull_rom.h"
#include "../coordinates/coordinates.h"

CatmullRom::CatmullRom(Coordinates coordinates[]) {
	this->coordinates = coordinates;
}