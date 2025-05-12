use std::error::Error;
use std::sync::{Arc, Mutex};

use emulation::*;

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    // Logs from extern crates to stdout
    initialize_env_logger();

    // Setup the node
    let ctx = r2r::Context::create()?;
    let node = r2r::Node::create(ctx, NODE_ID, "")?;
    let arc_node = Arc::new(Mutex::new(node));

    r2r::log_info!(NODE_ID, "Spawning emulators...");

    let arc_node_clone: Arc<Mutex<r2r::Node>> = arc_node.clone();
    tokio::task::spawn(async move { spawn_gantry_emulator_server(arc_node_clone).await.unwrap() });

    tokio::time::sleep(std::time::Duration::from_millis(200)).await;

    let arc_node_clone: Arc<Mutex<r2r::Node>> = arc_node.clone();
    tokio::task::spawn(async move { spawn_robot_emulator_server(arc_node_clone).await.unwrap() });

    tokio::time::sleep(std::time::Duration::from_millis(200)).await;

    // keep the node alive
    let arc_node_clone: Arc<Mutex<r2r::Node>> = arc_node.clone();
    let handle = std::thread::spawn(move || loop {
        arc_node_clone
            .lock()
            .unwrap()
            .spin_once(std::time::Duration::from_millis(1000));
    });

    r2r::log_info!(NODE_ID, "Node started.");

    handle.join().unwrap();

    Ok(())
}
