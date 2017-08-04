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

#include "connected_mrpp/Configuration.h"

using namespace std;

namespace connected_mrpp
{

Configuration::Configuration()
{

}

Configuration::Configuration(vector<int>& agent) :
		agent(agent)
{

}

bool Configuration::operator==(const Configuration& rhs) const
{
	if(agent.size() == rhs.agent.size())
	{
		for(int i = 0; i < agent.size(); i++)
		{
			if(agent[i] != rhs.agent[i])
			{
				return false;
			}
		}

		return true;
	}

	return false;
}

bool Configuration::operator< (const Configuration& rhs) const
{
	if(agent.size() == rhs.agent.size())
	{
		for(int i = 0; i < agent.size(); i++)
		{
			if(agent[i] < rhs.agent[i])
			{
				return true;
			}
			else if(agent[i] > rhs.agent[i])
			{
				return false;
			}
		}
	}

	return false;
}

PartialConfiguration::PartialConfiguration(Configuration& configuration) :
			Configuration(configuration.agent), count(0)
{

}

bool PartialConfiguration::operator==(const PartialConfiguration& rhs) const
{
	const Configuration& self = *this;
	const Configuration& rhs_c = rhs;

	return self == rhs_c && count == rhs.count;
}

bool PartialConfiguration::operator< (const PartialConfiguration& rhs) const
{
	const Configuration& self = *this;
	const Configuration& rhs_c = rhs;

	return count < rhs.count ||  (count == rhs.count && self < rhs_c);
}


}
