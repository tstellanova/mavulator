#[macro_use]
extern crate bencher;

use bencher::Bencher;
use std::sync::{Arc, RwLock};
use mavulator::mav_writer;
use flighty::simulato::Simulato;

fn run_sensor_collection(sim: &Arc<RwLock<Simulato>>) {
    let mut last_slow_cadence_send = 0; //1Hz sensors
    let mut last_med_cadence_send = 0; //100Hz sensors
    let mut last_fast_cadence_send = 0; //400Hz sensors

    for _i in 0..100 {
        let _msg_list = mav_writer::collect_messages(&sim,
                                        &mut last_slow_cadence_send,
                                        &mut last_med_cadence_send,
                                        &mut last_fast_cadence_send
        );
    }
}

fn bench_sensor_collection(b:&mut Bencher) {
    let sim: Arc<RwLock<Simulato>> = Arc::new(RwLock::new(Simulato::new()));
    b.iter(||  run_sensor_collection(&sim));
}

benchmark_group!(benches, bench_sensor_collection);
benchmark_main!(benches);