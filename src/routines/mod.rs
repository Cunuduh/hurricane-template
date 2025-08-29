extern crate alloc;
use alloc::{vec::Vec, string::String, boxed::Box};
use serde::{Deserialize, Serialize};

use crate::control::PoseSettings;
use crate::odometry::Pose;
use crate::plan::Action;

#[derive(Serialize, Deserialize, Debug, Clone)]
struct IRPose { x: f64, y: f64, heading: Option<f64> }
#[derive(Serialize, Deserialize, Debug, Clone)]
struct IRPoseSettings { is_reversed: bool, max_voltage: f64 }
#[derive(Serialize, Deserialize, Debug, Clone)]
enum IRAction {
	DriveCurve(Vec<(IRPose, IRPoseSettings)>),
	DrivePtp(Vec<(IRPose, IRPoseSettings)>),
	DriveToPoint(IRPose, IRPoseSettings),
	DriveStraight(f64, IRPoseSettings),
	TurnToPoint(IRPose, IRPoseSettings),
	TurnToAngle(f64, IRPoseSettings),
	TriggerOnIndex(usize, String),
	TriggerOnDistance(f64, String),
	TriggerOnAngle(f64, String),
}
#[derive(Serialize, Deserialize, Debug, Clone)]
struct IRRoutine { name: String, actions: Vec<IRAction> }

include!(concat!(env!("OUT_DIR"), "/routines_index.rs"));

fn map_pose(p: IRPose) -> Pose {
	if let Some(h) = p.heading { Pose::with_heading(p.x, p.y, h) } else { Pose::new(p.x, p.y) }
}
fn map_settings(s: IRPoseSettings) -> PoseSettings { PoseSettings { is_reversed: s.is_reversed, max_voltage: s.max_voltage } }

fn map_action(a: IRAction) -> Action {
	match a {
		IRAction::DriveCurve(v) => {
			let pts: Vec<(Pose, PoseSettings)> = v.into_iter().map(|(p,s)| (map_pose(p), map_settings(s))).collect();
			Action::DriveCurve(alloc::sync::Arc::from(pts.into_boxed_slice()))
		}
		IRAction::DrivePtp(v) => {
			let pts: Vec<(Pose, PoseSettings)> = v.into_iter().map(|(p,s)| (map_pose(p), map_settings(s))).collect();
			Action::DrivePtp(alloc::sync::Arc::from(pts.into_boxed_slice()))
		}
		IRAction::DriveToPoint(p, s) => Action::DriveToPoint(map_pose(p), map_settings(s)),
		IRAction::DriveStraight(d, s) => Action::DriveStraight(d, map_settings(s)),
		IRAction::TurnToPoint(p, s) => Action::TurnToPoint(map_pose(p), map_settings(s)),
		IRAction::TurnToAngle(a, s) => Action::TurnToAngle(a, map_settings(s)),
		IRAction::TriggerOnIndex(i, n) => Action::TriggerOnIndex(i, Box::leak(n.into_boxed_str())),
		IRAction::TriggerOnDistance(d, n) => Action::TriggerOnDistance(d, Box::leak(n.into_boxed_str())),
		IRAction::TriggerOnAngle(a, n) => Action::TriggerOnAngle(a, Box::leak(n.into_boxed_str())),
	}
}

pub fn load_all() -> Vec<(alloc::string::String, Vec<Action>)> {
	let mut out = Vec::new();
	for (name, bytes) in ROUTINE_BLOBS {
		if let Ok(ir) = postcard::from_bytes::<IRRoutine>(bytes) {
			let actions = ir.actions.into_iter().map(map_action).collect::<Vec<_>>();
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
