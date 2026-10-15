# RF discovery liveness investigation

During an FC telemetry pause, live debugging showed:

- FC's sensor submission reached SEDSNet and returned success.
- The router chose no outbound side; no CAN frame was handed off for that sample.
- The remaining CAN announcer was PB, with a fresh lease and no telemetry
  destination endpoints. RF was absent from that snapshot.

A fresh CAN side does not imply a route to GroundStation. RF must continue
advertising its reachability independently of PB's advertisements.

The SEDSNet development candidate fixes a reproduced failure where outstanding
reliable discovery ACKs suppress all periodic advertisements on that side. It
permits bounded empty keepalives while retaining the pending topology baseline.
It does not extend discovery expiry or add a broadcast override.

The regression withholds a topology ACK, keeps PB present on CAN, and verifies
that FC's sensor data is actually transmitted through its learned RF route.
Router and relay variants are tested. This establishes the library failure
mode; confirmation that it is the cause of the observed hardware pause still
requires deploying the candidate to RF and observing recovery and sustained
telemetry. Reflashing FC alone does not update RF's discovery implementation.

The local candidate may be built using patched fetched dependencies for
qualification. Such a build is not a published SEDSNet release and does not
change the board's upstream dependency reference. Deployable upstream builds
must wait for library validation and publication.

## Recurrence after the keepalive candidate

FC again retained PB but not RF and selected no outbound telemetry side while
commands still arrived. No core fault or allocation failure was recorded. Its
shared SEDSNet usage was 4,353 / 8,192 bytes in one live snapshot; a brief watch
did not catch budget-driven route eviction (not proof that none ever occurred).

A live receive capture then returned SEDS_DESERIALIZE (-13) for a 34-byte SDT
chunk frame, transfer ID bytes `47 e1 0f cf`, index 1 of 2. The router calculated
transfer IDs using CRC32 over a frame *including its appended CRC*. The resulting
constant residue gives different RF packets the same transfer ID. A regression
reproduced the exact ID and failed before the fix; after a lost chunk, a later
packet could be combined with stale fragments and fail decoding.

The development fix uses a sender-seeded content hash and bounds incomplete
assemblies by count, age, and bytes, in both router and relay implementations.
Deploy sender fixes to RF and receiver bounds to FC/other consumers. The wire
format is unchanged. This is a reproduced transport defect consistent with the
hardware failure; sustained hardware recovery still requires candidate testing.

Candidate Release factory images built on 2026-09-17:

- RF: SHA-256 `f7ef4a87d215d72098027cdc2285193d6229b2b2faf73ea7dd83c76bd33177f2`;
  firmware flash 422,924 / 474,624 bytes; RAM 113,448 / 114,688 bytes.
- FC: SHA-256 `e95ca60d15ce3410f5c2ee7705e97696dacbfee019472e7234e45790b0ae9aee`;
  firmware flash 470,940 / 474,624 bytes; aggregate RAM 242,472 / 278,528 bytes,
  with main RAM and DMA regions checked separately by the linker.

These binaries use locally patched FetchContent sources. Cleaning the build
directory removes that candidate patch; do not mistake an unpatched rebuild
against the currently published library for this candidate. No hardware flash
or sustained hardware validation is implied by these build results.
