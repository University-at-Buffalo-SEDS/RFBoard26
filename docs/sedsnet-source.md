# SEDSnet source selection

All normal CMake/build.py configurations track SEDSnet `main`. Each configure
attempts to fetch main (15-second network timeout), prints the exact commit,
and keeps a build-local source copy. Board schema and embedded preparation are
applied before Cargo runs. Older prepared copies remain intact.

If fetching fails, configuration uses the last successfully selected source,
then an existing FetchContent checkout or repo-local `third_party/SEDSnet` /
`SEDSnet` copy. It reports the fallback path and revision. A first build with
neither internet nor local sources fails with an actionable message.

Use `-DSEDSNET_OFFLINE=ON` to skip the fetch deliberately. Standard FetchContent
disconnected options are also honored. `-DFETCHCONTENT_SOURCE_DIR_SEDSNET=/path`
selects a local source explicitly; the build does not fetch or reset it, but
still applies board preparation. Keep separate prepared copies for each board.

This fallback covers SEDSnet source retrieval. Offline builds also require the
other firmware dependencies, Rust crates and toolchains already on disk.
Reconfiguring (including through build.py) checks main; a bare incremental
`cmake --build` does not poll the network unless it triggers reconfiguration.
