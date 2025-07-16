extern crate alloc;
use alloc::{
    sync::Arc,
    vec::Vec,
    boxed::Box,
};
use crate::odometry::Pose;
use crate::control::PoseSettings;
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
    TriggerOnIndex(usize, Box<dyn FnMut() + Send + Sync>),
    TriggerOnDistance(f64, Box<dyn FnMut() + Send + Sync>),
    TriggerOnAngle(f64, Box<dyn FnMut() + Send + Sync>),
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
                Action::TriggerOnIndex(index, trigger) => {
                    self.chassis.triggers.add_index_trigger(index, trigger);
                }
                Action::TriggerOnDistance(distance, trigger) => {
                    self.chassis.triggers.add_distance_trigger(distance, trigger);
                }
                Action::TriggerOnAngle(angle, trigger) => {
                    self.chassis.triggers.add_angle_trigger(angle, trigger);
                }
            }
        }
    }
}
