extern crate alloc;
use alloc::{string::String, vec::Vec};

use serde::{Deserialize, Serialize};

use crate::{chassis::PoseSettings, odometry::Pose, plan::Action};

#[derive(Serialize, Deserialize, Debug, Clone)]
struct IRPose {
    x: f64,
    y: f64,
    heading: Option<f64>,
}
#[derive(Serialize, Deserialize, Debug, Clone)]
struct IRPoseSettings {
    is_reversed: bool,
    max_voltage: f64,
}
#[derive(Serialize, Deserialize, Debug, Clone)]
enum IRAction {
    DriveCurve(Vec<(IRPose, IRPoseSettings)>),
    DriveBoomerang(Vec<(IRPose, IRPoseSettings)>),
    DrivePtp(Vec<(IRPose, IRPoseSettings)>),
    DriveToPoint(IRPose, IRPoseSettings),
    DriveStraight(f64, IRPoseSettings),
    TurnToPoint(IRPose, IRPoseSettings),
    TurnToAngle(f64, IRPoseSettings),
    TriggerOnIndex(usize, String),
    TriggerOnDistance(f64, String),
    TriggerOnAngle(f64, String),
    TriggerNow(String),
    SetPose(IRPose),
    Wait(u64),
    DriveFor(u64, f64, bool),
}
#[derive(Serialize, Deserialize, Debug, Clone)]
struct IRRoutine {
    name: String,
    actions: Vec<IRAction>,
}

include!(concat!(env!("OUT_DIR"), "/routines_index.rs"));

fn mirror_pose(pose: Pose) -> Pose {
    // rotate 180 degrees about the origin to map red to blue side
    Pose {
        x: -pose.x,
        y: -pose.y,
        heading: pose.heading + core::f64::consts::PI,
    }
}

fn map_pose(p: IRPose, is_red: bool) -> Pose {
    let pose = if let Some(h) = p.heading {
        Pose::with_heading(p.x, p.y, h)
    } else {
        Pose::new(p.x, p.y)
    };
    
    if is_red {
        pose
    } else {
        mirror_pose(pose)
    }
}
fn map_settings(s: IRPoseSettings) -> PoseSettings {
    PoseSettings {
        is_reversed: s.is_reversed,
        max_voltage: s.max_voltage,
    }
}

fn map_action(a: IRAction, is_red: bool) -> Action {
    match a {
        IRAction::DriveCurve(v) => {
            let pts: Vec<(Pose, PoseSettings)> = v
                .into_iter()
                .map(|(p, s)| (map_pose(p, is_red), map_settings(s)))
                .collect();
            Action::DriveCurve(pts.into_boxed_slice())
        }
        IRAction::DriveBoomerang(v) => {
            let pts: Vec<(Pose, PoseSettings)> = v
                .into_iter()
                .map(|(p, s)| (map_pose(p, is_red), map_settings(s)))
                .collect();
            Action::DriveBoomerang(pts.into_boxed_slice())
        }
        IRAction::DrivePtp(v) => {
            let pts: Vec<(Pose, PoseSettings)> = v
                .into_iter()
                .map(|(p, s)| (map_pose(p, is_red), map_settings(s)))
                .collect();
            Action::DrivePtp(pts.into_boxed_slice())
        }
        IRAction::DriveToPoint(p, s) => Action::DriveToPoint(map_pose(p, is_red), map_settings(s)),
        IRAction::DriveStraight(d, s) => Action::DriveStraight(d, map_settings(s)),
        IRAction::TurnToPoint(p, s) => Action::TurnToPoint(map_pose(p, is_red), map_settings(s)),
        IRAction::TurnToAngle(a, s) => {
            let angle = if is_red { a } else { a + 180.0 };
            Action::TurnToAngle(angle, map_settings(s))
        }
        IRAction::TriggerOnIndex(i, n) => Action::TriggerOnIndex(i, n),
        IRAction::TriggerOnDistance(d, n) => Action::TriggerOnDistance(d, n),
        IRAction::TriggerOnAngle(a, n) => Action::TriggerOnAngle(a, n),
        IRAction::TriggerNow(n) => Action::TriggerNow(n),
        IRAction::SetPose(p) => Action::SetPose(map_pose(p, is_red)),
        IRAction::Wait(ms) => Action::Wait(ms),
        IRAction::DriveFor(ms, v, r) => Action::DriveFor(ms, v, r),
    }
}

pub fn load_all(is_red: bool) -> Vec<(alloc::string::String, Vec<Action>)> {
    let mut out = Vec::new();
    for (name, bytes) in ROUTINE_BLOBS {
        if let Ok(ir) = postcard::from_bytes::<IRRoutine>(bytes) {
            let actions = ir.actions.into_iter().map(|a| map_action(a, is_red)).collect::<Vec<_>>();
            out.push((alloc::string::String::from(*name), actions));
        }
    }
    out
}

pub fn list_names() -> Vec<alloc::string::String> {
    let mut out = Vec::new();
    for (name, _) in ROUTINE_BLOBS {
        out.push(alloc::string::String::from(*name));
    }
    out
}
