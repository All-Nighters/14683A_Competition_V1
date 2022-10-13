#pragma once

namespace Constants {

	namespace CatmullRom {
		inline float INFLUENCE_RATIO[4][4] = {
			{0.0, -0.5,  1.0, -0.5},
			{1.0,  0.0, -2.5,  1.5},
			{0.0,  0.5,  2.0, -1.5},
			{0.0,  0.0, -0.5,  0.5}
		};
	}

	namespace GraphicalInterface {
		inline int SCREEN_WIDTH = 480;
		inline int SCREEN_HEIGHT = 272 - 30;
	}

}