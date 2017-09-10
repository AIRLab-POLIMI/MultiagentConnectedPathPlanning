/*
 * rrt_planning,
 *
 *
 * Copyright (C) 2016 Davide Tateo
 * Versione 1.0
 *
 * This file is part of rrt_planning.
 *
 * rrt_planning is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * rrt_planning is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with rrt_planning.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "connected_mrpp/utils/RandomGenerator.h"

namespace connected_mrpp
{

std::random_device RandomGenerator::rd;
std::mt19937 RandomGenerator::gen(rd());


int RandomGenerator::sampleUniform(int a, int b)
{
    std::uniform_int_distribution<int> d(a, b);

    return d(gen);
}

int RandomGenerator::sampleDiscrete(std::vector<double>& p)
{
	std::discrete_distribution<int> d(p.begin(), p.end());

	return d(gen);
}

}
