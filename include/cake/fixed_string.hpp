/***

A helper to allow passing string literals into templates.

***/

#pragma once

#include <algorithm>
#include <string_view>

namespace cake {

template <size_t N> struct fixed_string {
    constexpr fixed_string(const char (&str)[N]) { std::copy_n(str, N, data); }

    char data[N];

    constexpr operator std::string_view() const { return {data, N - 1}; }

    constexpr const char *c_str() const { return data; }

    constexpr size_t size() const { return N - 1; }
};

} // namespace cake
