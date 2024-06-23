#ifndef CPP_MODULES_TOOLS_H
#define CPP_MODULES_TOOLS_H

#include "../../../Pybind/Pybind.h"


py::array_t<int> get_indices_from_coordinates(const py::array_t<double>& cloud_in, const py::array_t<double>& cloud_ref);


#endif //CPP_MODULES_TOOLS_H
