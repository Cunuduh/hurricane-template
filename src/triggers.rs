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

    pub fn check_distance(&mut self, current: f64) {
        let mut i = 0;
        while i < self.triggers.len() {
            let remove = matches!(&self.triggers[i].cond, TriggerCondition::Distance(d) if current.abs() >= *d);
            if remove {
                let mut t = self.triggers.remove(i);
                (t.callback)();
            } else {
                i += 1;
            }
        }
    }

    pub fn check_index(&mut self, current: usize) {
        let mut i = 0;
        while i < self.triggers.len() {
            let remove = matches!(&self.triggers[i].cond, TriggerCondition::Index(idx) if current >= *idx);
            if remove {
                let mut t = self.triggers.remove(i);
                (t.callback)();
            } else {
                i += 1;
            }
        }
    }

    pub fn check_angle(&mut self, current: f64) {
        let mut i = 0;
        while i < self.triggers.len() {
            let remove = matches!(&self.triggers[i].cond, TriggerCondition::Angle(a) if current.abs() >= *a);
            if remove {
                let mut t = self.triggers.remove(i);
                (t.callback)();
            } else {
                i += 1;
            }
        }
    }
}
