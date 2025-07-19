extern crate alloc;
use alloc::vec::Vec;

pub enum TriggerCondition {
    Distance(f64),
    Index(usize),
    Angle(f64),
}

struct Trigger {
    cond: TriggerCondition,
    callback: fn(),
}

pub struct TriggerManager {
    triggers: &'static [(&'static str, fn())],
    armed_triggers: Vec<Trigger>,
}

impl TriggerManager {
    pub fn new(triggers: &'static [(&'static str, fn())]) -> Self {
        TriggerManager { triggers, armed_triggers: Vec::new() }
    }
    pub fn arm(&mut self, condition: TriggerCondition, name: &str) {
        let callback = self.triggers.iter()
            .find_map(|(n, f)| (*n == name).then_some(*f))
            .unwrap_or_else(|| panic!("Trigger '{}' not found", name));
        self.armed_triggers.push(Trigger { cond: condition, callback });
    }

    pub fn check(&mut self, mut matcher: impl FnMut(&TriggerCondition) -> bool) {
        self.armed_triggers.retain(|trigger| {
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
