extern crate alloc;
use alloc::{boxed::Box, string::String, vec::Vec};
use core::time::Duration;

use vexide::prelude::*;

use crate::{Robot, chassis::PoseSettings, odometry::Pose, triggers::TriggerCondition};
pub enum Action {
    DriveCurve(Box<[(Pose, PoseSettings)]>),
    DrivePtp(Box<[(Pose, PoseSettings)]>),
    DriveToPoint(Pose, PoseSettings),
    DriveStraight(f64, PoseSettings),
    TurnToPoint(Pose, PoseSettings),
    TurnToAngle(f64, PoseSettings),
    TriggerOnIndex(usize, String),
    TriggerOnDistance(f64, String),
    TriggerOnAngle(f64, String),
    TriggerNow(String),
    SetPose(Pose),
    Wait(u64),
    DriveFor(u64, f64, bool),
}

impl Robot {
    pub async fn run_plan(&mut self, plan: Vec<Action>) {
        for action in plan {
            match action {
                Action::DriveCurve(ref points) => {
                    println!("Goal: DriveCurve to {:?}", points.last().map(|(p, _)| p));
                    self.chassis
                        .drive_spline(points, 1.0, 5.0, 0.70)
                        .await;
                }
                Action::DrivePtp(ref points) => {
                    println!("Goal: DrivePtp to {:?}", points.last().map(|(p, _)| p));
                    self.chassis.drive_ptp(points).await;
                }
                Action::DriveToPoint(pose, settings) => {
                    println!("Goal: DriveToPoint to {:?}", pose);
                    self.chassis.drive_to_point(pose.x, pose.y, settings).await;
                }
                Action::DriveStraight(distance, settings) => {
                    println!("Goal: DriveStraight {:.2} inches", distance);
                    self.chassis
                        .drive_straight(distance, settings.max_voltage, settings.is_reversed)
                        .await;
                }
                Action::TurnToPoint(pose, settings) => {
                    println!("Goal: TurnToPoint ({:.2}, {:.2})", pose.x, pose.y);
                    self.chassis
                        .turn_to_point(pose.x, pose.y, settings.max_voltage, settings.is_reversed)
                        .await;
                }
                Action::TurnToAngle(angle, settings) => {
                    println!("Goal: TurnToAngle {:.2} degrees", angle);
                    self.chassis
                        .turn_to_angle(angle, settings.max_voltage, settings.is_reversed)
                        .await;
                }
                Action::TriggerOnIndex(index, ref name) => {
                    println!("Goal: TriggerOnIndex {} -> {}", index, name);
                    self.chassis
                        .triggers
                        .arm(TriggerCondition::Index(index), name);
                }
                Action::TriggerOnDistance(distance, ref name) => {
                    println!("Goal: TriggerOnDistance {:.2} -> {}", distance, name);
                    self.chassis
                        .triggers
                        .arm(TriggerCondition::Distance(distance), name);
                }
                Action::TriggerOnAngle(angle, ref name) => {
                    println!("Goal: TriggerOnAngle {:.2} -> {}", angle, name);
                    self.chassis
                        .triggers
                        .arm(TriggerCondition::Angle(angle), name);
                }
                Action::TriggerNow(ref name) => {
                    println!("Goal: TriggerNow {}", name);
                    let actions = self.chassis.triggers.trigger_now(name);
                    self.chassis.apply_trigger_actions(&actions);
                }
                Action::SetPose(pose) => {
                    println!("Goal: SetPose to {:?}", pose);
                    self.chassis.set_pose(pose);
                }
                Action::Wait(ms) => {
                    println!("Goal: Wait {} ms", ms);
                    let total = Duration::from_millis(ms);
                    let mut elapsed = Duration::from_millis(0);
                    let dt = self.chassis.config.dt;
                    while elapsed < total {
                        self.chassis.service_autonomous_intake();
                        let step = if total - elapsed > dt { dt } else { total - elapsed };
                        sleep(step).await;
                        elapsed += step;
                    }
                }
                Action::DriveFor(ms, voltage, reverse) => {
                    println!(
                        "Goal: DriveFor {} ms at {:.2} V (reverse={})",
                        ms, voltage, reverse
                    );
                    self.chassis.drive_for(ms, voltage, reverse).await;
                }
            }
        }
    }
}
