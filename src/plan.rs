extern crate alloc;
use alloc::{
    sync::Arc,
    vec::Vec,
};
use crate::odometry::Pose;
use crate::control::PoseSettings;
use crate::triggers::TriggerCondition;
use crate::Robot;
pub enum Action {
    DriveCurve {
        points: Arc<[(Pose, PoseSettings)]>,
        b: f64,
        zeta: f64,
    },
    DrivePtp(Arc<[(Pose, PoseSettings)]>),
    DriveToPoint(Pose, PoseSettings),
    DriveStraight(f64, PoseSettings),
    TurnToPoint(Pose, PoseSettings),
    TurnToAngle(f64, PoseSettings),
    TriggerOnIndex(usize, &'static str),
    TriggerOnDistance(f64, &'static str),
    TriggerOnAngle(f64, &'static str),
}

impl Robot {
    pub async fn run_plan(&mut self, plan: Vec<Action>) {
        for action in plan {
            match action {
                Action::DriveCurve { points, b, zeta } => {
                    self.chassis.drive_ramsete_catmull_rom(&points, 32, b, zeta).await;
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
                    self.chassis.triggers.arm(TriggerCondition::Index(index), name);
                }
                Action::TriggerOnDistance(distance, name) => {
                    self.chassis.triggers.arm(TriggerCondition::Distance(distance), name);
                }
                Action::TriggerOnAngle(angle, name) => {
                    self.chassis.triggers.arm(TriggerCondition::Angle(angle), name);
                }
            }
        }
    }
}
