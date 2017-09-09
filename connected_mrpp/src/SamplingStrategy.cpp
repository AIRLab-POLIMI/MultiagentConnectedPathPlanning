/*
 * connected_mrpp,
 *
 *
 * Copyright (C) 2017 Davide Tateo
 * Versione 1.0
 *
 * This file is part of connected_mrpp.
 *
 * connected_mrpp is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * connected_mrpp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with connected_mrpp.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "connected_mrpp/planner/SamplingStrategy.h"

namespace connected_mrpp
{

Configuration SamplingStrategy::sample(const std::vector<Configuration>& candidates, const std::vector<double>& costs)
{

}

SamplingStrategy::~SamplingStrategy()
{

}

PolynomialSamplingStrategy::PolynomialSamplingStrategy(double exponent) : exponent(exponent)
{

}

std::vector<double> PolynomialSamplingStrategy::sampleWeights(const std::vector<Configuration>& candidates)
{

}

std::vector<double> LogarithmicSamplingStrategy::sampleWeights(const std::vector<Configuration>& candidates)
{

}


}
