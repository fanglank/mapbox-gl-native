#include <mbgl/gfx/renderer_backend.hpp>
#include <mbgl/gfx/backend_scope.hpp>
#include <mbgl/gfx/context.hpp>
#include <iostream>

namespace mbgl {
namespace gfx {

RendererBackend::RendererBackend(const ContextMode contextMode_) : contextMode(contextMode_) {
}
RendererBackend::~RendererBackend() = default;

gfx::Context& RendererBackend::getContext() {
    assert(BackendScope::exists());
    std::call_once(initialized, [this] {
        context = createContext();
        std::clog << "context created " << context << "\n";
    });
    assert(context);
    return *context;
}

} // namespace gfx
} // namespace mbgl
