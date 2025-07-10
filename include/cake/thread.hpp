#pragma once

#include <type_traits>

#include "context.hpp"

namespace cake {

template <typename ContextType, typename Function>
void create_thread(std::shared_ptr<ContextType> context, Function function) {
    static_assert(std::is_base_of_v<Context, ContextType>, "ContextType must be a child of Context");

    context->threads.emplace_back([context, function]() { function(context); });
}

} // namespace cake
