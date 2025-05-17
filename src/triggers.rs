extern crate alloc;
use alloc::{vec::Vec, boxed::Box};


pub enum TriggerCondition {
    Distance(f64),
    Index(usize),
    Angle(f64),
}

struct Trigger {
    cond: TriggerCondition,
    callback: Box<dyn FnMut()>,
}

pub struct TriggerManager {
    triggers: Vec<Trigger>,
}

impl TriggerManager {
    pub fn new() -> Self {
        TriggerManager { triggers: Vec::new() }
    }

    pub fn add_distance_trigger(&mut self, distance: f64, callback: impl FnMut() + 'static) {
        self.triggers.push(Trigger {
            cond: TriggerCondition::Distance(distance),
            callback: Box::new(callback),
        });
    }

    pub fn add_index_trigger(&mut self, index: usize, callback: impl FnMut() + 'static) {
        self.triggers.push(Trigger {
            cond: TriggerCondition::Index(index),
            callback: Box::new(callback),
        });
    }
    
    pub fn add_angle_trigger(&mut self, angle: f64, callback: impl FnMut() + 'static) {
        self.triggers.push(Trigger {
            cond: TriggerCondition::Angle(angle),
            callback: Box::new(callback),
        });
    }

    pub fn check(&mut self, mut matcher: impl FnMut(&TriggerCondition) -> bool) {
        self.triggers.retain_mut(|trigger| {
            if matcher(&trigger.cond) {
                (trigger.callback)();
                false
            } else {
                true
            }
        });
    }

    pub fn check_distance(&mut self, current: f64) {
        self.check(|cond| matches!(cond, TriggerCondition::Distance(d) if current.abs() >= *d));
    }

    pub fn check_index(&mut self, current: usize) {
        self.check(|cond| matches!(cond, TriggerCondition::Index(idx) if current >= *idx));
    }

    pub fn check_angle(&mut self, current: f64) {
        self.check(|cond| matches!(cond, TriggerCondition::Angle(a) if current.abs() >= *a));
    }
}
