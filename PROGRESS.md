# Progress

2026-07-22: Wired `BroadphaseBench` into CMakeLists.txt (built but not a
target since the spatial-hash commit), and replaced the README's stale
"O(n^2) broad phase" limitation with the actual spatial-hash numbers measured
on this machine: 0.34x at 100 bodies, 1.50x at 1,000, 22.33x at 10,000.
