#pragma once

namespace Constants {

	namespace CatmullRom {
		inline float INFLUENCE_RATIO[4][4] = {
			// ^0   ^1    ^2    ^3
			{0.0, -0.5,  1.0, -0.5},
			{1.0,  0.0, -2.5,  1.5},
			{0.0,  0.5,  2.0, -1.5},
			{0.0,  0.0, -0.5,  0.5}
		};
	}

}