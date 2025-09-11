#![no_main]
#![no_std]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
extern crate alloc;
use core::{
    sync::atomic::{AtomicUsize, Ordering},
    time::Duration,
};

use vexide::{devices::smart::imu::InertialSensor, prelude::*};

use crate::{
    chassis::{Chassis, ChassisArgs, ChassisConfig},
    odometry::{Pose, TrackingWheelConfig},
    pid::Pid,
};
static UI_SELECTED_ROUTE: AtomicUsize = AtomicUsize::new(0);
use vexide::fs;
const AUTON_SAVE_PATH: &str = "auton.txt";
use alloc::string::ToString;

mod autonomous;
mod chassis;
mod driver_control;
mod motion_controller;
mod odometry;
mod pid;
mod plan;
mod routines;
mod triggers;
mod utils;

slint::include_modules!();
use slint::ComponentHandle;

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let mut dynamic_peripherals = DynamicPeripherals::new(peripherals);
    vexide_slint::initialize_slint_platform(dynamic_peripherals.take_display().expect("display"));
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
        let v: alloc::vec::Vec<slint::SharedString> =
            names.into_iter().map(slint::SharedString::from).collect();
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
                if sel >= list.len() && !list.is_empty() {
                    sel = 0;
                }
            }
            UI_SELECTED_ROUTE.store(sel, Ordering::Relaxed);
            ui.set_selected_route(sel as i32);
        }
    }
    ui.show().expect("failed to show application");

    let mut dp = dynamic_peripherals;
    let ui_weak = ui.as_weak();
    vexide::task::spawn(async move {
        // give the event loop a moment to start painting
        sleep(Duration::from_millis(1)).await;
        let robot = Robot::new(&mut dp, ui_weak).await;
        vexide::task::spawn(robot.compete()).detach();
    })
    .detach();

    ui.run().expect("failed to run application");

    vexide::program::exit();
}

pub struct Robot {
    chassis: Chassis<3, 3, 3>,
}

impl Robot {
    async fn new(peripherals: &mut DynamicPeripherals, ui: slint::Weak<VexSelector>) -> Self {
        let left_motors = [
            Motor::new(
                peripherals.take_smart_port(1).expect("smart port 1"),
                Gearset::Blue,
                Direction::Reverse,
            ),
            Motor::new(
                peripherals.take_smart_port(3).expect("smart port 3"),
                Gearset::Blue,
                Direction::Forward,
            ),
            Motor::new(
                peripherals.take_smart_port(5).expect("smart port 5"),
                Gearset::Blue,
                Direction::Reverse,
            ),
        ];
        let right_motors = [
            Motor::new(
                peripherals.take_smart_port(2).expect("smart port 2"),
                Gearset::Blue,
                Direction::Forward,
            ),
            Motor::new(
                peripherals.take_smart_port(4).expect("smart port 4"),
                Gearset::Blue,
                Direction::Reverse,
            ),
            Motor::new(
                peripherals.take_smart_port(6).expect("smart port 6"),
                Gearset::Blue,
                Direction::Forward,
            ),
        ];
        let imu = InertialSensor::new(peripherals.take_smart_port(7).expect("smart port 7"));
        let parallel_wheel = RotationSensor::new(
            peripherals.take_smart_port(8).expect("smart port 8"),
            Direction::Forward,
        );
        let perpendicular_wheel = RotationSensor::new(
            peripherals.take_smart_port(9).expect("smart port 9"),
            Direction::Forward,
        );
        let intake_motors = [
            Motor::new(
                peripherals.take_smart_port(10).expect("smart port 10"),
                Gearset::Blue,
                Direction::Reverse,
            ),
            Motor::new(
                peripherals.take_smart_port(11).expect("smart port 11"),
                Gearset::Blue,
                Direction::Forward,
            ),
            Motor::new(
                peripherals.take_smart_port(12).expect("smart port 12"),
                Gearset::Blue,
                Direction::Forward,
            ),
        ];
        let scraper = AdiDigitalOut::new(peripherals.take_adi_port(1).expect("adi port A"));
        let hood = AdiDigitalOut::new(peripherals.take_adi_port(2).expect("adi port B"));
        let colour_sort = AdiDigitalOut::new(peripherals.take_adi_port(3).expect("adi port C"));
        let optical_sensor =
            OpticalSensor::new(peripherals.take_smart_port(13).expect("smart port 13"));
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
            tw_config,
        };
        let triggers: &'static [(&'static str, fn())] = &[("do something", || {
            println!("Triggered: do something");
        })];
        let chassis = Chassis::new(ChassisArgs {
            controller: peripherals
                .take_primary_controller()
                .expect("primary controller"),
            left_motors,
            right_motors,
            parallel_wheel,
            perpendicular_wheel,
            intake_motors,
            scraper,
            hood,
            colour_sort,
            imu,
            optical_sensor,
            config,
            triggers,
            ui,
        })
        .await;

        Robot { chassis }
    }
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        self.chassis.odometry.reset(Default::default());
        let _ = self.chassis.imu.reset_heading();
        let routines = routines::load_all();
        let idx = UI_SELECTED_ROUTE.load(Ordering::Relaxed);
        if routines.is_empty() {
            return;
        }
        let target = if idx < routines.len() { idx } else { 0 };
        let plan = routines.into_iter().nth(target).map(|(_, p)| p).unwrap();
        self.run_plan(plan).await;
    }

    async fn driver(&mut self) {
        loop {
            let c_state = self.chassis.controller.state().unwrap_or_default();
            self.chassis.cheesy_control(&c_state);
            self.chassis.handle_intake_outtake_controls(&c_state);
            self.chassis.toggle_scraper(&c_state);
            if c_state.button_b.is_pressed() {
                self.chassis.calibrate_tracking_wheels().await;
            }
            sleep(self.chassis.config.dt).await;
        }
    }
}
