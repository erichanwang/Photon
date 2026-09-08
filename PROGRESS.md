# Photon progress

- 2026-09-07: Re-measured the two README headline numbers on this machine.
  BVH vs linear scan holds at 7.3-7.9x (original 8.0x); 16-thread scaling
  holds at 4.3-5.5x (original 5.6x). Updated the Re-verified table with the
  real range instead of a single smoothed figure. Expanded the build section so
  the non-CMake commands for Tests and Benchmark do not silently pull in
  main.cpp and fail with a duplicate-main error. Suite green, benchmarks run,
  README carries numbers actually measured this session.
