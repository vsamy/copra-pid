/*      File: typedefs.h
*       This file is part of the program copra
*       Program description : This a C++ implementation of a Time Invariant Linear Model Predictive Controller (LMPC) done in C++14 with python bindings
*       Copyright (C) 2017 -  Vincent Samy (LIRMM). All Right reserved.
*
*       This software is free software: you can redistribute it and/or modify
*       it under the terms of the CeCILL-C license as published by
*       the CEA CNRS INRIA, either version 1
*       of the License, or (at your option) any later version.
*       This software is distributed in the hope that it will be useful,
*       but WITHOUT ANY WARRANTY without even the implied warranty of
*       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*       CeCILL-C License for more details.
*
*       You should have received a copy of the CeCILL-C License
*       along with this software. If not, it can be found on the official website
*       of the CeCILL licenses family (http://www.cecill.info/index.en.html).
*/
#pragma once

#include <Eigen/Core>

namespace copra {

template <typename T1, typename T2 = std::true_type, typename T3 = std::true_type>
struct is_all_arithmetic : is_all_arithmetic<typename std::is_arithmetic<std::decay_t<T1> >::type, T2, T3> {
};

template <typename T2, typename T3>
struct is_all_arithmetic<std::false_type, T2, T3> {
    static const bool value = false;
};

template <typename T2, typename T3>
struct is_all_arithmetic<std::true_type, T2, T3> : is_all_arithmetic<typename std::is_arithmetic<std::decay_t<T2> >::type, T3, std::true_type> {
};

template <>
struct is_all_arithmetic<std::true_type, std::true_type, std::true_type> {
    static const bool value = true;
};

} // namespace copra