/*
 * connected_mrpp,
 *
 *
 * Copyright (C) 2016 Davide Tateo
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

#ifndef INCLUDE_CONNECTED_MRPP_MAP_DEBUGMAP_H_
#define INCLUDE_CONNECTED_MRPP_MAP_DEBUGMAP_H_

#include "connected_mrpp/map/Map.h"

namespace connected_mrpp
{

class DebugMap : public Map
{
public:
    DebugMap();

    virtual bool isFree(const Eigen::VectorXd& p) override;
    virtual unsigned char getCost(const Eigen::VectorXd& p) override;

    virtual ~DebugMap();

private:

};

}

#endif /* INCLUDE_CONNECTED_MRPP_MAP_DEBUGMAP_H_ */
