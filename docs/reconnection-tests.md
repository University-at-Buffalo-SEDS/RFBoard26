# Reconnection qualification

Run ./build.py test --all --release --ultra-soak for the unit, short linked,
and 600-second seven-board network tests. The long test restarts only
GroundStation after sample four, then GroundStation and the avionics boards
after sample eight. The other boards and Pico-Fi pair remain running.

Each new GroundStation process must rediscover every board and rebuild traffic
attribution. Fresh Valve and Actuator commands require matching state responses
throughout the run; memory and latency bounds remain enforced. Topology-generation
unit tests catch malformed scenarios without Docker. Passing simulation does not
replace hardware validation.

The v0.4.12 simulator / SEDSNet v4.0.33 qualification passed both complete
600-second seven-board scenarios: two GroundStation-only restarts, and a
GroundStation-only restart followed by GroundStation plus avionics restarts.
All scheduled Valve/Actuator state responses, named discovery, graph/traffic
attribution and memory thresholds passed. Gateway-only and fill-group restart
regressions also passed. Firmware continues to fetch SEDSNet main through
CMake; the simulator image and source fallback are pinned to v0.4.12.
