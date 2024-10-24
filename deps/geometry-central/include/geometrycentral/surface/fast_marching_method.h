#pragma once

#include <cmath>
#include <utility>
#include <vector>

#include "geometrycentral/surface/intrinsic_geometry_interface.h"
#include "geometrycentral/utilities/utilities.h"


namespace geometrycentral {
	namespace surface {

		VertexData<double> FMMDistance(IntrinsicGeometryInterface& geometry, const std::vector<std::pair<Vertex, double>>& initialDistances);


	}  // namespace surface
}  // namespace geometrycentral
