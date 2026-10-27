use sedsnet::{
    config::{DataType, RuntimeMemoryConfig},
    router::{Router, RouterConfig},
};
use std::sync::{
    Arc, Mutex,
    atomic::{AtomicU64, Ordering},
};
fn bootstrap(budget: usize, start: usize) {
    let config = RouterConfig::new([])
        .with_sender("RF")
        .with_memory_config(RuntimeMemoryConfig::new(budget, 16, start, 1.0).unwrap())
        .unwrap();
    let now = Arc::new(AtomicU64::new(0));
    let clock = now.clone();
    let router = Router::new_with_clock(config, Box::new(move || clock.load(Ordering::Relaxed)));
    let delivered = Arc::new(Mutex::new([[0usize; 3]; 2]));
    for (index, name) in ["radio", "can"].into_iter().enumerate() {
        let counts = delivered.clone();
        router.add_side_packed(name, move |wire| {
            let packet = sedsnet::wire_format::unpack_packet(wire)?;
            let kind = match packet.data_type() {
                DataType::DiscoveryAddress => Some(0),
                DataType::DiscoveryTopology => Some(1),
                DataType::DiscoverySchema => {
                    assert!(!packet.payload().is_empty(), "catalog must be complete");
                    sedsnet::discovery::decode_discovery_schema(&packet)?;
                    Some(2)
                }
                _ => None,
            };
            if let Some(kind) = kind {
                counts.lock().unwrap()[index][kind] += 1;
            }
            Ok(())
        });
    }
    // API acceptance retains an asynchronous catalog intent even on the old
    // two-slot profile. Verify actual CRC-decoded complete callbacks, not only Ok.
    router
        .announce_discovery()
        .expect("bounded discovery intent must be retained");
    for tick in 0..=3000u64 {
        now.store(tick, Ordering::Relaxed);
        router
            .process_all_queues()
            .expect("accepted discovery must drain");
        let counts = *delivered.lock().unwrap();
        let catalogs_complete = counts.iter().all(|c| c[2] > 0);
        // Equal-priority protected prefixes may replace each other in the old
        // two-slot profile. Both catalog owners must nevertheless drain. The
        // corrected starting storage must deliver all three prefixes per side.
        let prefixes_complete = if start == 512 {
            counts.iter().any(|c| c[0] > 0) && counts.iter().any(|c| c[1] > 0)
        } else {
            counts.iter().all(|c| c[0] > 0 && c[1] > 0)
        };
        if catalogs_complete && prefixes_complete {
            eprintln!("bootstrap budget={budget} start={start} decoded={counts:?}");
            return;
        }
    }
    panic!(
        "both sides must complete bootstrap: {:?}",
        *delivered.lock().unwrap()
    );
}
fn main() {
    bootstrap(6144, 512);
    bootstrap(16384, 512);
    bootstrap(16384, 2048);
}
#[unsafe(no_mangle)]
pub extern "C" fn telemetry_lock() {}
#[unsafe(no_mangle)]
pub extern "C" fn telemetry_unlock() {}
