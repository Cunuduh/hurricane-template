extern crate alloc;
use alloc::{
    boxed::Box,
    string::String,
    vec::Vec,
};
use crate::odometry::Pose;
use crate::chassis::PoseSettings;
use crate::triggers::TriggerCondition;
use crate::Robot;
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
}

impl Robot {
    pub async fn run_plan(&mut self, plan: Vec<Action>) {
        for action in plan {
            match action {
                Action::DriveCurve(points) => {
                    self.chassis.drive_ramsete_catmull_rom(&points, 32, 0.005, 0.75).await;
                }
                Action::DrivePtp(points) => {
                    self.chassis.drive_ptp(&points).await;
                }
                Action::DriveToPoint(pose, settings) => {
                    self.chassis.drive_to_point(pose.x, pose.y, settings).await;
                }
                Action::DriveStraight(distance, settings) => {
                    self.chassis.drive_straight(distance, settings.max_voltage).await;
                }
                Action::TurnToPoint(pose, settings) => {
                    self.chassis.turn_to_point(pose.x, pose.y, settings.max_voltage).await;
                }
                Action::TurnToAngle(angle, settings) => {
                    self.chassis.turn_to_angle(angle, settings.max_voltage).await;
                }
                Action::TriggerOnIndex(index, name) => {
                    self.chassis.triggers.arm(TriggerCondition::Index(index), &name);
                }
                Action::TriggerOnDistance(distance, name) => {
                    self.chassis.triggers.arm(TriggerCondition::Distance(distance), &name);
                }
                Action::TriggerOnAngle(angle, name) => {
                    self.chassis.triggers.arm(TriggerCondition::Angle(angle), &name);
                }
            }
        }
    }
}
