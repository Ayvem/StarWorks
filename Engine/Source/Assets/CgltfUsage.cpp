// ============================================================================
// CgltfUsage.cpp
// Single translation unit hosting the cgltf implementation, with
// third-party warnings silenced locally.
// ============================================================================

#if defined(_MSC_VER)
    #pragma warning(push, 0)
#elif defined(__GNUC__)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-parameter"
    #pragma GCC diagnostic ignored "-Wsign-compare"
#endif

#define CGLTF_IMPLEMENTATION
#include <cgltf.h>

#if defined(_MSC_VER)
    #pragma warning(pop)
#elif defined(__GNUC__)
    #pragma GCC diagnostic pop
#endif
