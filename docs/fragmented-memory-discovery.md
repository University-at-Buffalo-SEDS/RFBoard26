# Fragmented-memory discovery protection

The 2026-10-06 GroundStation capture showed intermittent RF liveness while
FC/PowerBoard routes were absent; the fill-system link continued receiving.
Receiving radio frames alone does not prove delivery of discovery or reports.
There is no live RF heap capture identifying its exact failure in this test.

The previous RF TLSF admission probe rejected operations whenever their
largest scratch allocation did not fit a single free block, even with many
free bytes. This can starve discovery and let routes expire. RF now uses the
same tested fallback as gateway: 8 KiB carved from existing allocator storage
as 32 adjacent 256-byte blocks. Ordinary TLSF is preferred; the fallback
serves allocations it cannot satisfy without moving any live pointer.
Admission checks hold bounded small probes simultaneously to ensure the
remaining working budget is available, then free them before returning.

Tests reproduce fragmented ordinary memory, nested 2/4 KiB scratch buffers,
medium allocations, rejection at exhaustion, preserved payload contents,
reserve recovery, and bounded admission. Sanitizers cover the native adapter.
ThreadX scheduling, SEDSnet sources and wire protocol are unchanged.

Build with --allocator tlsf to enable this adapter. The protection applies to
both heap and compact packet stores, but does not repair physical radio loss
or prove the observed hardware dropout is fixed. Flash RF and observe sustained
RF/FC/PowerBoard liveness and telemetry to complete live validation.
