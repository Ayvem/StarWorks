# RenderGraph module

> Status: **planned** — not yet implemented. This directory reserves the module location in the architecture.

Frame-graph based scheduling of GPU passes: automatic barrier placement, transient resource aliasing, async compute. Will sit on top of Renderer/Vulkan once the RHI layer is stable.

Rules: no God objects, minimal dependencies on other modules, every public type documented.
