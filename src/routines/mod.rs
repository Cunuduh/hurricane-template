extern crate alloc;
use alloc::{string::String, vec::Vec};

use serde::{Deserialize, Serialize};

use crate::{chassis::PoseSettings, odometry::Pose, plan::Action};

#[derive(Serialize, Deserialize, Debug, Clone)]
struct Routine {
    name: String,
    actions: Vec<Action>,
}

include!(concat!(env!("OUT_DIR"), "/routines_index.rs"));

fn mirror_pose(pose: Pose) -> Pose {
    Pose {
        x: -pose.x,
        y: -pose.y,
        heading: pose.heading + core::f64::consts::PI,
    }
}

fn mirror_action(action: Action) -> Action {
    match action {
        Action::DriveCurve(v) => {
            let pts: Vec<(Pose, PoseSettings)> = v
                .into_iter()
                .map(|(p, s)| (mirror_pose(p), s))
                .collect();
            Action::DriveCurve(pts)
        }
        Action::DrivePtp(v) => {
            let pts: Vec<(Pose, PoseSettings)> = v
                .into_iter()
                .map(|(p, s)| (mirror_pose(p), s))
                .collect();
            Action::DrivePtp(pts)
        }
        Action::DriveToPoint(p, s) => Action::DriveToPoint(mirror_pose(p), s),
        Action::TurnToPoint(p, s) => Action::TurnToPoint(mirror_pose(p), s),
        Action::TurnToAngle(a, s) => Action::TurnToAngle(a + core::f64::consts::PI, s),
        Action::DriveSwing(a, lv, rv, s) => Action::DriveSwing(a + core::f64::consts::PI, lv, rv, s),
        Action::SetPose(p) => Action::SetPose(mirror_pose(p)),
        other => other,
    }
}

pub fn load_all(is_red: bool) -> Vec<(String, Vec<Action>)> {
    let mut out = Vec::new();
    for (name, bytes) in ROUTINE_BLOBS {
        if let Ok(routine) = postcard::from_bytes::<Routine>(bytes) {
            let actions = if is_red {
                routine.actions
            } else {
                routine.actions.into_iter().map(mirror_action).collect()
            };
            out.push((String::from(*name), actions));
        }
    }
    out
}

pub fn list_names() -> Vec<String> {
    let mut out = Vec::new();
    for (name, _) in ROUTINE_BLOBS {
        out.push(String::from(*name));
    }
    out
}
