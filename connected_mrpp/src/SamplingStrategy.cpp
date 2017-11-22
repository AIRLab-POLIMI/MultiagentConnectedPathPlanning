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

#include "connected_mrpp/utils/RandomGenerator.h"

#include <algorithm>


namespace connected_mrpp
{

Configuration SamplingStrategy::sample(const std::vector<Configuration>& candidates, const std::vector<double>& costs)
{
    if(candidates.size() == 0)
    {
        return PI_NULL;
    }

    std::vector<ConfigurationValue> pairs;

    for(int i = 0; i < candidates.size(); i++)
    {
        pairs.push_back(std::make_pair(costs[i], candidates[i]));
    }

    std::sort(pairs.begin(), pairs.end());

    auto&& w = sampleWeights(pairs);

    auto index = RandomGenerator::sampleDiscrete(w);

    return pairs[index].second;
}

SamplingStrategy::~SamplingStrategy()
{

}

PolynomialSamplingStrategy::PolynomialSamplingStrategy(double exponent) : exponent(exponent)
{

}

std::vector<double> PolynomialSamplingStrategy::sampleWeights(const std::vector<ConfigurationValue>& candidates)
{
    std::vector<double> w;

    for(int i = 0; i < candidates.size(); i++)
    {
        double v = 1.0/std::pow(1.0+i, exponent);
        w.push_back(v);
    }

    return w;
}

std::vector<double> LogarithmicSamplingStrategy::sampleWeights(const std::vector<ConfigurationValue>& candidates)
{
    std::vector<double> w;

    for(int i = 0; i < candidates.size(); i++)
    {
        double v = 1.0/(1.0+std::log(1.0f+i));
        w.push_back(v);
    }

    return w;
}

BoltzmannSamplingStrategy::BoltzmannSamplingStrategy(double exponent) : exponent(exponent)
{

}

std::vector<double> BoltzmannSamplingStrategy::sampleWeights(const std::vector<ConfigurationValue>& candidates)
{
    std::vector<double> w;

    double minH = candidates[0].first;

    for(auto& c : candidates)
    {
        double deltaH = c.first - minH;
        double v = std::exp(-exponent*deltaH);
        w.push_back(v);
    }

    return w;
}



}
