#![no_main]
#![no_std]
#![feature(generic_const_exprs)]

use control::PoseSettings;
use vexide::prelude::*;
use vexide::devices::smart::imu::InertialSensor;
use crate::control::{Chassis, ChassisConfig};
use crate::pid::Pid;
use crate::odometry::Pose;
use core::time::Duration;

mod pid;
mod control;
mod odometry;
mod utils;
mod triggers;

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let robot = Robot::new(peripherals).await;
    robot.compete().await;
}

struct Robot {
    chassis: Chassis<3, 3>,
}

impl Robot {
    async fn new(peripherals: Peripherals) -> Self {
        let left_motors = [
            Motor::new(peripherals.port_6, Gearset::Blue, Direction::Forward),
            Motor::new(peripherals.port_5, Gearset::Blue, Direction::Reverse),
            Motor::new(peripherals.port_4, Gearset::Blue, Direction::Reverse),
        ];
        let right_motors = [
            Motor::new(peripherals.port_3, Gearset::Blue, Direction::Reverse),
            Motor::new(peripherals.port_2, Gearset::Blue, Direction::Forward),
            Motor::new(peripherals.port_1, Gearset::Blue, Direction::Forward),
        ];
        
        let imu = InertialSensor::new(peripherals.port_7); 

        let drive_pid = Pid::new(6.0, 0.0, 0.5, 0.0, 0.1); 
        let turn_pid = Pid::new(6.0, 0.0, 0.01, 0.0, 0.02);
        let heading_pid = Pid::new(1.0, 0.0, 0.0, 0.0, 0.02);

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
        };
        
        let chassis = Chassis::new(
            peripherals.primary_controller, 
            left_motors, 
            right_motors, 
            imu,
            config,
        ).await;
        Robot { chassis }
    }
}


impl Compete for Robot {
    async fn autonomous(&mut self) {
        self.chassis.odometry.reset(Default::default());
        let _ = self.chassis.imu.reset_heading();

        let drive_speed = 6.0;
        let points: [(Pose, PoseSettings); 4] = [
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
                    is_reversed: true,
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
        ];
        self.chassis.triggers.add_index_trigger(2, || {
            println!("Trigger 2 activated");
        });
        self.chassis.drive_ramsete::<4, 32>(&points, 0.005, 0.75).await;
        self.chassis.triggers.add_distance_trigger(10.0, || {
            println!("Distance trigger activated");
        });
        self.chassis.drive_to_point(0.0, 0.0, 6.0, false).await;
        self.chassis.triggers.add_angle_trigger(90.0, || {
            println!("Angle trigger activated");
        });
        self.chassis.turn_to_angle(0.0, 11.0).await;
        self.chassis.triggers.add_index_trigger(3, || {
            println!("Distance trigger activated");
        }); 
        self.chassis.drive_ptp(&points).await;
    }

    async fn driver(&mut self) {
        loop {
            self.chassis.cheesy_control();
            let c_state = self.chassis.controller.state().unwrap_or_default();
            if c_state.button_a.is_pressed() {
                self.chassis.drive_to_point(0.0, 0.0, 11.0, false).await;
            }
            sleep(self.chassis.config.dt).await;
        }
    }
}
