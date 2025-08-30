#![no_main]
#![no_std]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
extern crate alloc;
use core::time::Duration;

use vexide::{devices::smart::imu::InertialSensor, prelude::*};

use crate::{
    chassis::{Chassis, ChassisArgs, ChassisConfig},
    odometry::{Pose, TrackingWheelConfig},
    pid::Pid,
};
use core::sync::atomic::{AtomicUsize, Ordering};
static UI_SELECTED_ROUTE: AtomicUsize = AtomicUsize::new(0);
use vexide::fs;
const AUTON_SAVE_PATH: &str = "auton.txt";
use alloc::string::ToString;

mod chassis;
mod autonomous;
mod driver_control;
mod odometry;
mod pid;
mod plan;
mod triggers;
mod utils;
mod routines;

slint::include_modules!();
use slint::ComponentHandle;

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let (mut robot, display) = Robot::new(peripherals).await;
    vexide_slint::initialize_slint_platform(display);
    let ui = VexSelector::new().expect("failed to create application");
    ui.on_route_selected(|idx| {
        let i = idx as usize;
        UI_SELECTED_ROUTE.store(i, Ordering::Relaxed);
        if let Some(name) = routines::list_names().into_iter().nth(i) {
            let _ = fs::write(AUTON_SAVE_PATH, name.as_bytes());
        } else {
            let _ = fs::write(AUTON_SAVE_PATH, i.to_string().as_bytes());
        }
    });
    // set routes from embedded routines
    {
        let names = routines::list_names();
        // convert names into slint model
        let v: alloc::vec::Vec<slint::SharedString> = names.into_iter().map(slint::SharedString::from).collect();
        let model = alloc::rc::Rc::new(slint::VecModel::from(v));
        ui.set_routes(model.into());
        // load saved selection from sd
        if let Ok(saved) = fs::read_to_string(AUTON_SAVE_PATH) {
            let s = saved.trim();
            let mut sel = 0usize;
            if !s.is_empty() {
                // prefer name match, else treat as index
                let list = routines::list_names();
                if let Some(pos) = list.iter().position(|n| n.eq_ignore_ascii_case(s)) {
                    sel = pos;
                } else if let Ok(n) = s.parse::<usize>() {
                    sel = n;
                }
                if sel >= list.len() && !list.is_empty() { sel = 0; }
            }
            UI_SELECTED_ROUTE.store(sel, Ordering::Relaxed);
            ui.set_selected_route(sel as i32);
        }
    }
    robot.ui = Some(ui.as_weak());
    vexide::task::spawn(robot.compete()).detach();
    ui.run().expect("failed to run application");

    vexide::program::exit();
}

pub struct Robot {
    chassis: Chassis<3, 3, 3>,
    ui: Option<slint::Weak<VexSelector>>,
}

impl Robot {
    async fn new(peripherals: Peripherals) -> (Self, Display) {
        let left_motors = [
            Motor::new(peripherals.port_1, Gearset::Blue, Direction::Reverse),
            Motor::new(peripherals.port_3, Gearset::Blue, Direction::Forward),
            Motor::new(peripherals.port_5, Gearset::Blue, Direction::Reverse),
        ];
        let right_motors = [
            Motor::new(peripherals.port_2, Gearset::Blue, Direction::Forward),
            Motor::new(peripherals.port_4, Gearset::Blue, Direction::Reverse),
            Motor::new(peripherals.port_6, Gearset::Blue, Direction::Forward),
        ];
        let imu = InertialSensor::new(peripherals.port_7);
        let parallel_wheel = RotationSensor::new(peripherals.port_8, Direction::Forward);
        let perpendicular_wheel = RotationSensor::new(peripherals.port_9, Direction::Forward);
        let intake_motors = [
            Motor::new(peripherals.port_10, Gearset::Blue, Direction::Reverse),
            Motor::new(peripherals.port_11, Gearset::Blue, Direction::Forward),
            Motor::new(peripherals.port_12, Gearset::Blue, Direction::Forward),
        ];
        let tw_config = TrackingWheelConfig {
            parallel_offset: 0.302,
            perpendicular_offset: -0.021,
            wheel_diameter: 2.0,
        };

        let drive_pid = Pid::new(6.0, 0.0, 0.5, 0.0, 0.0);
        let turn_pid = Pid::new(6.0, 0.0, 0.01, 0.0, 0.0);
        let heading_pid = Pid::new(3.0, 0.0, 0.0, 0.0, 0.0);

        let config = ChassisConfig {
            initial_pose: Pose {
                x: 0.0,
                y: 0.0,
                heading: 0.0,
            },
            wheel_diameter: 3.25,
            ext_gear_ratio: 1.5,
            track_width: 18.0,
            max_volts: 12.0,
            dt: Duration::from_millis(10),
            drive_pid,
            turn_pid,
            heading_pid,
            small_drive_exit_error: 1.0,
            small_drive_settle_time: Duration::from_millis(50),
            big_drive_exit_error: 3.0,
            big_drive_settle_time: Duration::from_millis(250),

            small_turn_exit_error: 2.0,
            small_turn_settle_time: Duration::from_millis(50),
            big_turn_exit_error: 5.0,
            big_turn_settle_time: Duration::from_millis(250),

            stall_current_threshold: 2.4,
            stall_velocity_threshold: 1.0,
            stall_time: Duration::from_millis(400),

            accel_t: 0.5,
            tw_config: Some(tw_config),
        };
        let triggers: &'static [(&'static str, fn())] = &[
            ("do something", || {
                println!("Triggered: do something");
            }),
        ];
        let chassis = Chassis::new(
            ChassisArgs {
                controller: peripherals.primary_controller,
                left_motors,
                right_motors,
                parallel_wheel,
                perpendicular_wheel,
                intake_motors,
                imu,
                config,
                triggers
            }
        ).await;

        (Robot { chassis, ui: None }, peripherals.display)
    }
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        self.chassis.odometry.reset(Default::default());
        let _ = self.chassis.imu.reset_heading();
        let routines = routines::load_all();
        let idx = UI_SELECTED_ROUTE.load(Ordering::Relaxed);
        if routines.is_empty() { return; }
        let target = if idx < routines.len() { idx } else { 0 };
        let plan = routines.into_iter().nth(target).map(|(_, p)| p).unwrap();
        self.run_plan(plan).await;
    }

    async fn driver(&mut self) {
        loop {
            self.chassis.cheesy_control();
            self.chassis.handle_intake_outtake_controls();
            let c_state = self.chassis.controller.state().unwrap_or_default();
            if c_state.button_a.is_pressed() {
                self.chassis.calibrate_tracking_wheels().await;
            }
            sleep(self.chassis.config.dt).await;
        }
    }
}
