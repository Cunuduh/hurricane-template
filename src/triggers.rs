extern crate alloc;
use alloc::{boxed::Box, collections::BTreeMap, format, string::String, vec::Vec};


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
    triggers: BTreeMap<String, Box<dyn FnMut()>>,
    armed_triggers: Vec<Trigger>,
}

impl TriggerManager {
    pub fn new(triggers: BTreeMap<String, Box<dyn FnMut()>>) -> Self {
        TriggerManager { triggers, armed_triggers: Vec::new() }
    }
    pub fn arm<S: AsRef<str>>(&mut self, condition: TriggerCondition, name: S) {
        let name_ref = name.as_ref();
        let callback = self.triggers.remove(name_ref)
            .unwrap_or_else(|| panic!("Trigger '{}' not found", name_ref));
        self.armed_triggers.push(Trigger {
            cond: condition,
            callback,
        });
    }

    pub fn check(&mut self, mut matcher: impl FnMut(&TriggerCondition) -> bool) {
        self.armed_triggers.retain_mut(|trigger| {
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
