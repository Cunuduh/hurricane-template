#![no_main]
#![no_std]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
#![feature(portable_simd)]
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
    triggers::{IntakeCommand, PneumaticTarget, TriggerAction, TriggerDefinition},
};
static UI_SELECTED_ROUTE: AtomicUsize = AtomicUsize::new(0);
static IS_RED_ALLIANCE: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(true);
use vexide::fs;
const AUTON_SAVE_PATH: &str = "auton.txt";
const ALLIANCE_SAVE_PATH: &str = "alliance.txt";
use alloc::string::ToString;

mod autonomous;
mod chassis;
mod driver_control;
mod mcl;
mod motion_controller;
mod odometry;
mod pid;
mod plan;
mod routines;
mod triggers;
mod utils;

slint::include_modules!();
use slint::ComponentHandle;

const AUTON_TRIGGER_DEFINITIONS: &[TriggerDefinition] = &[
    TriggerDefinition {
        name: "intake",
        actions: &[TriggerAction::Intake(IntakeCommand::Intake)],
    },
    TriggerDefinition {
        name: "outtake",
        actions: &[TriggerAction::Intake(IntakeCommand::Outtake)],
    },
    TriggerDefinition {
        name: "outtake_middle",
        actions: &[TriggerAction::Intake(IntakeCommand::OuttakeMiddle)],
    },
    TriggerDefinition {
        name: "intake_reverse",
        actions: &[TriggerAction::Intake(IntakeCommand::Reverse)],
    },
    TriggerDefinition {
        name: "intake_stop",
        actions: &[TriggerAction::Intake(IntakeCommand::Stop)],
    },
    TriggerDefinition {
        name: "scraper_up",
        actions: &[TriggerAction::SetPneumatic {
            target: PneumaticTarget::Scraper,
            state: false,
        }],
    },
    TriggerDefinition {
        name: "scraper_down",
        actions: &[TriggerAction::SetPneumatic {
            target: PneumaticTarget::Scraper,
            state: true,
        }],
    },
    TriggerDefinition {
        name: "scraper_toggle",
        actions: &[TriggerAction::TogglePneumatic(PneumaticTarget::Scraper)],
    },
    TriggerDefinition {
        name: "wings_open",
        actions: &[TriggerAction::SetPneumatic {
            target: PneumaticTarget::Wings,
            state: true,
        }],
    },
    TriggerDefinition {
        name: "wings_close",
        actions: &[TriggerAction::SetPneumatic {
            target: PneumaticTarget::Wings,
            state: false,
        }],
    },
    TriggerDefinition {
        name: "wings_toggle",
        actions: &[TriggerAction::TogglePneumatic(PneumaticTarget::Wings)],
    },
    TriggerDefinition {
        name: "block_park_up",
        actions: &[TriggerAction::SetPneumatic {
            target: PneumaticTarget::BlockPark,
            state: true,
        }],
    },
    TriggerDefinition {
        name: "block_park_down",
        actions: &[TriggerAction::SetPneumatic {
            target: PneumaticTarget::BlockPark,
            state: false,
        }],
    },
    TriggerDefinition {
        name: "block_park_toggle",
        actions: &[TriggerAction::TogglePneumatic(PneumaticTarget::BlockPark)],
    },
    TriggerDefinition {
        name: "hood_high",
        actions: &[TriggerAction::SetPneumatic {
            target: PneumaticTarget::Hood,
            state: true,
        }],
    },
    TriggerDefinition {
        name: "hood_low",
        actions: &[TriggerAction::SetPneumatic {
            target: PneumaticTarget::Hood,
            state: false,
        }],
    },
    TriggerDefinition {
        name: "hood_toggle",
        actions: &[TriggerAction::TogglePneumatic(PneumaticTarget::Hood)],
    },
    TriggerDefinition {
        name: "colour_sort_open",
        actions: &[TriggerAction::SetPneumatic {
            target: PneumaticTarget::ColourSort,
            state: true,
        }],
    },
    TriggerDefinition {
        name: "colour_sort_close",
        actions: &[TriggerAction::SetPneumatic {
            target: PneumaticTarget::ColourSort,
            state: false,
        }],
    },
    TriggerDefinition {
        name: "colour_sort_toggle",
        actions: &[TriggerAction::TogglePneumatic(PneumaticTarget::ColourSort)],
    },
    TriggerDefinition {
        name: "colour_sort_enable",
        actions: &[TriggerAction::SetColourSortEnabled(true)],
    },
    TriggerDefinition {
        name: "colour_sort_disable",
        actions: &[TriggerAction::SetColourSortEnabled(false)],
    },
    TriggerDefinition {
        name: "alt_colour_sort_enable",
        actions: &[TriggerAction::SetAltColourSortEnabled(true)],
    },
    TriggerDefinition {
        name: "alt_colour_sort_disable",
        actions: &[TriggerAction::SetAltColourSortEnabled(false)],
    },
];

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
    // load saved alliance from sd
    if let Ok(saved) = fs::read_to_string(ALLIANCE_SAVE_PATH) {
        let s = saved.trim().to_ascii_lowercase();
        let is_red = matches!(
            &s[..],
            "red" | "r" | "1" | "true" | "t" | "yes" | "y"
        );
        ui.set_is_red_alliance(is_red);
        IS_RED_ALLIANCE.store(is_red, Ordering::Relaxed);
    }
    
    let ui_clone = ui.as_weak();
    vexide::task::spawn(async move {
        let mut last_is_red: Option<bool> = None;
        loop {
            if let Some(ui_handle) = ui_clone.upgrade() {
                let is_red = ui_handle.get_is_red_alliance();
                IS_RED_ALLIANCE.store(is_red, Ordering::Relaxed);
                if last_is_red != Some(is_red) {
                    let _ = fs::write(
                        ALLIANCE_SAVE_PATH,
                        if is_red { b"red" as &[u8] } else { b"blue" as &[u8] },
                    );
                    last_is_red = Some(is_red);
                }
            }
            sleep(Duration::from_millis(100)).await;
        }
    }).detach();
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
        let wings = AdiDigitalOut::new(peripherals.take_adi_port(3).expect("adi port C"));
        let block_park = AdiDigitalOut::new(peripherals.take_adi_port(4).expect("adi port D"));
        let colour_sort = AdiDigitalOut::new(peripherals.take_adi_port(5).expect("adi port E"));
        let optical_sensor =
            OpticalSensor::new(peripherals.take_smart_port(13).expect("smart port 13"));
        let dist_back =
            DistanceSensor::new(peripherals.take_smart_port(14).expect("smart port 14"));
        let dist_left =
            DistanceSensor::new(peripherals.take_smart_port(15).expect("smart port 15"));
        let dist_front =
            DistanceSensor::new(peripherals.take_smart_port(16).expect("smart port 16"));
        let dist_right =
            DistanceSensor::new(peripherals.take_smart_port(17).expect("smart port 17"));
        let tw_config = TrackingWheelConfig {
            parallel_offset: -0.020,
            perpendicular_offset: -0.035,
            wheel_diameter: 2.0,
        };

        let drive_pid = Pid::new(3.5, 0.0, 0.25, 0.0);
        let turn_pid = Pid::new(50.0, 0.0, 3.5, 0.0);
        let heading_pid = Pid::new(15.0, 0.0, 0.0, 0.0);

        let config = ChassisConfig {
            initial_pose: Pose {
                x: 0.0,
                y: 0.0,
                heading: 0.0,
            },
            wheel_diameter: 3.25,
            ext_gear_ratio: 48.0 / 36.0,
            track_width: 15.0,
            max_volts: 12.0,
            dt: Duration::from_millis(10),
            turn_pid,
            heading_pid,
            drive_pid,
            small_drive_exit_error: 0.1,
            small_drive_settle_time: Duration::from_millis(50),
            big_drive_exit_error: 0.5,
            big_drive_settle_time: Duration::from_millis(100),

            small_turn_exit_error: 0.25,
            small_turn_settle_time: Duration::from_millis(50),
            big_turn_exit_error: 1.0,
            big_turn_settle_time: Duration::from_millis(100),

            stall_current_threshold: 2.4,
            stall_velocity_threshold: 1.0,
            stall_time: Duration::from_millis(400),

            accel_t: 0.25, // time to reach max velocity
            tw_config,
        };
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
            wings,
            block_park,
            colour_sort,
            imu,
            optical_sensor,
            dist_front,
            dist_right,
            dist_back,
            dist_left,
            config,
            triggers: AUTON_TRIGGER_DEFINITIONS,
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
        let is_red = IS_RED_ALLIANCE.load(Ordering::Relaxed);
        let routines = routines::load_all(is_red);
        let idx = UI_SELECTED_ROUTE.load(Ordering::Relaxed);
        if routines.is_empty() {
            return;
        }
        let target = if idx < routines.len() { idx } else { 0 };
        let plan = routines.into_iter().nth(target).map(|(_, p)| p).unwrap();
        let _ = self.chassis.optical_sensor.set_led_brightness(1.0);
        let _ = self.chassis.optical_sensor.set_integration_time(Duration::from_millis(40));
        self.run_plan(plan).await;
    }

    async fn driver(&mut self) {
        for m in self.chassis.left_motors.iter_mut() {
            let _ = m.set_voltage(0.0);
        }
        for m in self.chassis.right_motors.iter_mut() {
            let _ = m.set_voltage(0.0);
        }
        for m in self.chassis.intake_motors.iter_mut() {
            let _ = m.set_voltage(0.0);
        }
        let _ = self.chassis.optical_sensor.set_integration_time(Duration::from_millis(40));
        let _ = self.chassis.optical_sensor.set_led_brightness(1.0);
        loop {
            let _ = self.chassis.optical_sensor.set_led_brightness(1.0);
            let c_state = self.chassis.controller.state().unwrap_or_default();
            self.chassis.cheesy_control(&c_state);
            self.chassis.handle_intake_subsystem(
                c_state.button_l1.is_now_pressed(),
                c_state.button_l2.is_now_pressed(),
                c_state.button_l2.is_pressed(),
                c_state.button_r1.is_now_pressed(),
                c_state.button_r2.is_now_pressed(),
            );
            if c_state.button_a.is_now_pressed() {
                self.chassis.toggle_scraper();
            }
            if c_state.button_b.is_now_pressed() {
                self.chassis.toggle_wings();
            }
            if c_state.button_y.is_now_pressed() {
                if self.chassis.block_park_macro_is_active() {
                    self.chassis.cancel_block_park_macro();
                } else {
                    self.chassis.start_block_park_macro();
                }
            }
            if c_state.button_up.is_now_pressed() {
                self.chassis.toggle_block_park();
            }
            if c_state.button_l1.is_now_pressed() {
                self.chassis.toggle_hood();
            }
            if c_state.button_x.is_now_pressed() {
                self.chassis.toggle_colour_sort();
            }
            // run block park macro after intake subsystem so it can override motor outputs
            self.chassis.service_block_park_macro();
            sleep(self.chassis.config.dt).await;
        }
    }
}
