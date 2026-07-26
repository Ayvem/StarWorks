// ============================================================================
// VmaUsage.cpp
// Single translation unit hosting the VulkanMemoryAllocator implementation.
// Third-party warnings are silenced here and only here — engine code is
// still compiled with the full warning set.
// ============================================================================

#if defined(_MSC_VER)
    #pragma warning(push, 0)
#elif defined(__GNUC__)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-parameter"
    #pragma GCC diagnostic ignored "-Wunused-variable"
    #pragma GCC diagnostic ignored "-Wunused-function"
    #pragma GCC diagnostic ignored "-Wmissing-field-initializers"
    #pragma GCC diagnostic ignored "-Wparentheses"
#endif

#define VMA_IMPLEMENTATION
#include <vk_mem_alloc.h>

#if defined(_MSC_VER)
    #pragma warning(pop)
#elif defined(__GNUC__)
    #pragma GCC diagnostic pop
#endif
