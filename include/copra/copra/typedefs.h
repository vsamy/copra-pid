
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