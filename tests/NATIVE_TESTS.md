Native ownership tests run through the normal command:

```sh
python3 build.py test --release
```

The test prerequisite selects the project's configured SEDSnet `main` or `dev` source and offline fallbacks, then prepares a matching host static library automatically. SEDSnet 4.3.6 or newer, Python 3.11+, Rust/Cargo, CMake and a compatible C sanitizer toolchain are required. Firmware builds keep their existing target, features, allocator, pools and stack settings.

`build/host-sdk/` contains ignored source snapshots, build logs and verified cache metadata. Explicit SDK source overrides are read-only inputs: the helper copies sources and path dependencies before building. Cache identity includes source/header/lock inputs, toolchain, build flags and preparation code; concurrent invocations use an OS lock and publish only complete artifacts. Failed build evidence is retained with its diagnostic path.

No archive environment variable is needed. Qualification tooling may optionally supply `SEDSNET_TEST_SOURCE`, `SEDSNET_TEST_NATIVE_ARCHIVE` and `SEDSNET_TEST_NATIVE_PROOF` together; the helper checks source, header, archive and ignored lock identity before reuse. An arbitrary or stale library is rejected.

The lifecycle regressions insert live production constructor, cache, receive and mutex functions into a host fixture linked to the actual SDK codec/cache. Hardware callbacks are inert. These tests do not prove physical outputs, MCU timing, stack peaks or hardware recovery. The macOS link path does not require GNU `--wrap` or Linux `libdl`; an MSVC host SDK uses `sedsnet.lib`, while the C sanitizer harness requires an appropriate GNU-compatible compiler/pthreads environment (for example Linux/WSL). Platform filename/link handling is distinct from actual Windows/macOS compiler qualification.
