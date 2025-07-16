#![no_main]
#![no_std]
#![feature(generic_const_exprs)]
extern crate alloc;
use alloc::{boxed::Box, vec, sync::Arc};
use core::time::Duration;

use control::PoseSettings;
use vexide::{devices::smart::imu::InertialSensor, prelude::*};

use crate::{
    control::{Chassis, ChassisConfig},
    odometry::{Pose, TrackingWheelConfig},
    pid::Pid,
    plan::Action,
};

mod control;
mod odometry;
mod pid;
mod plan;
mod triggers;
mod utils;

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let robot = Robot::new(peripherals).await;
    robot.compete().await;
}

pub struct Robot {
    chassis: Chassis<3, 3>,
}

impl Robot {
    async fn new(peripherals: Peripherals) -> Self {
        let left_motors = [
            Motor::new(peripherals.port_1, Gearset::Blue, Direction::Forward),
            Motor::new(peripherals.port_3, Gearset::Blue, Direction::Forward),
            Motor::new(peripherals.port_5, Gearset::Blue, Direction::Forward),
        ];
        let right_motors = [
            Motor::new(peripherals.port_2, Gearset::Blue, Direction::Reverse),
            Motor::new(peripherals.port_4, Gearset::Blue, Direction::Reverse),
            Motor::new(peripherals.port_6, Gearset::Blue, Direction::Reverse),
        ];

        let imu = InertialSensor::new(peripherals.port_7);
        let parallel_wheel = RotationSensor::new(peripherals.port_8, Direction::Forward);
        let perpendicular_wheel = RotationSensor::new(peripherals.port_9, Direction::Forward);
        let tw_config = TrackingWheelConfig {
            parallel_offset: -6.5,
            perpendicular_offset: -6.5,
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

        let chassis = Chassis::new(
            peripherals.primary_controller,
            left_motors,
            right_motors,
            parallel_wheel,
            perpendicular_wheel,
            imu,
            config,
        )
        .await;
        Robot { chassis }
    }
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        self.chassis.odometry.reset(Default::default());
        let _ = self.chassis.imu.reset_heading();

        let drive_speed = 3.0;
        let points: Arc<[(Pose, PoseSettings)]> = Arc::from(vec![
            (
                Pose::new(0.0, 0.0),
                PoseSettings {
                    max_voltage: drive_speed,
                    is_reversed: false,
                },
            ),
            (
                Pose::new(24.0, 24.0),
                PoseSettings {
                    max_voltage: drive_speed,
                    is_reversed: false,
                },
            ),
            (
                Pose::new(48.0, 0.0),
                PoseSettings {
                    max_voltage: drive_speed,
                    is_reversed: false,
                },
            ),
            (
                Pose::with_heading(72.0, 24.0, 0.0),
                PoseSettings {
                    max_voltage: drive_speed,
                    is_reversed: false,
                },
            ),
        ].into_boxed_slice());
        let plan = vec![
            Action::DrivePtp(points.clone()),
            Action::DriveToPoint(
                Pose::new(0.0, 0.0),
                PoseSettings {
                    max_voltage: 6.0,
                    is_reversed: false,
                },
            ),
            Action::TurnToAngle(
                0.0,
                PoseSettings {
                    max_voltage: 11.0,
                    is_reversed: false,
                },
            ),
            Action::DriveCurve {
                points: points.clone(),
                b: 0.005,
                zeta: 0.75,
            },
        ];
        self.run_plan(plan).await;
    }

    async fn driver(&mut self) {
        loop {
            self.chassis.cheesy_control();
            let c_state = self.chassis.controller.state().unwrap_or_default();
            if c_state.button_a.is_pressed() {
                self.chassis
                    .drive_to_point(
                        0.0,
                        0.0,
                        PoseSettings {
                            max_voltage: 6.0,
                            is_reversed: false,
                        },
                    )
                    .await;
            }
            sleep(self.chassis.config.dt).await;
        }
    }
}
