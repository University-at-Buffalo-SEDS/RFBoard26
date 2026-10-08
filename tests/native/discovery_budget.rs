use sedsnet::{config::RuntimeMemoryConfig, router::{Router, RouterConfig}};
fn bootstrap(budget: usize, start: usize) -> bool {
 let config = RouterConfig::new([]).with_sender("RF").with_memory_config(RuntimeMemoryConfig::new(budget,16,start,1.0).unwrap()).unwrap();
 let router = Router::new_with_clock(config, Box::new(||0));
 router.add_side_packed("radio", |_|Ok(()));
 router.add_side_packed("can", |_|Ok(()));
 router.announce_discovery().is_ok()
}
fn main() {
 assert!(!bootstrap(6144,512), "old shared budget unexpectedly accepted bootstrap");
 assert!(!bootstrap(16384,512), "old fixed queue unexpectedly accepted bootstrap");
 assert!(bootstrap(16384,2048), "corrected limits must accept both-side bootstrap");
}
#[unsafe(no_mangle)] pub extern "C" fn telemetry_lock() {}
#[unsafe(no_mangle)] pub extern "C" fn telemetry_unlock() {}
