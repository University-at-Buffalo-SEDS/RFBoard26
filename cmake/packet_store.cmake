# Opt-in queue payload arena. Reservation comes from the existing Rust pool.
set(BOARD_PACKET_ARENA_BYTES "8192" CACHE STRING "Packet arena byte capacity")
set(BOARD_PACKET_ARENA_HANDLES "64" CACHE STRING "Packet arena handle capacity")
if(NOT BOARD_PACKET_ARENA_BYTES MATCHES "^[1-9][0-9]*$" OR
   NOT BOARD_PACKET_ARENA_HANDLES MATCHES "^[1-9][0-9]*$")
    message(FATAL_ERROR "Packet arena capacities must be positive integers")
endif()
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    BOARD_PACKET_ARENA_BYTES=${BOARD_PACKET_ARENA_BYTES}U
    BOARD_PACKET_ARENA_HANDLES=${BOARD_PACKET_ARENA_HANDLES}U)
