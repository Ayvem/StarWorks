# Simulation module

> Status: **planned** — not yet implemented. This directory reserves the module location in the architecture.

Fixed-frequency multi-rate simulation core (Physics 50 Hz, Logistics 10 Hz, Automation 5 Hz, Economy 2 Hz, World 1 Hz). Fully decoupled from rendering; keeps running for off-screen objects.

Rules: no God objects, minimal dependencies on other modules, every public type documented.
