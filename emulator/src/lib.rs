pub static EMULATOR_NODE_ID: &'static str = "micro_sp_emulator";
pub static NODE_ID: &'static str = "micro_sp_emulator";
pub static TEST_TICKER_RATE: u64 = 1000; // milliseconds
pub static CLIENT_TICKER_RATE: u64 = 100; // milliseconds
pub static PUBLISHER_TICKER_RATE: u64 = 100; // milliseconds
pub static NUMBER_OF_TEST_CASES: u64 = 20;

pub mod emulators;
pub use crate::emulators::gantry_emulator::*;
pub use crate::emulators::robot_emulator::*;

pub mod utils;
pub use crate::utils::env_logger::*;
