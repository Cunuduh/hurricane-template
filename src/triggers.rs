extern crate alloc;
use alloc::vec::Vec;

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum TriggerCondition {
    Distance(f64),
    Index(usize),
    Angle(f64),
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum PneumaticTarget {
    Scraper,
    Hood,
    Wings,
    BlockPark,
    ColourSort,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum IntakeCommand {
    Intake,
    Outtake,
    OuttakeMiddle,
    Reverse,
    Stop,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TriggerAction {
    Intake(IntakeCommand),
    SetPneumatic {
        target: PneumaticTarget,
        state: bool,
    },
    TogglePneumatic(PneumaticTarget),
    SetColourSortEnabled(bool),
    SetAltColourSortEnabled(bool),
    ResetXY,
}

pub struct TriggerDefinition {
    pub name: &'static str,
    pub actions: &'static [TriggerAction],
}

struct Trigger {
    cond: TriggerCondition,
    actions: &'static [TriggerAction],
}

pub struct TriggerManager {
    triggers: &'static [TriggerDefinition],
    armed_triggers: Vec<Trigger>,
}

impl TriggerManager {
    pub fn new(triggers: &'static [TriggerDefinition]) -> Self {
        TriggerManager {
            triggers,
            armed_triggers: Vec::new(),
        }
    }

    fn find_actions(&self, name: &str) -> &'static [TriggerAction] {
        self.triggers
            .iter()
            .find_map(|def| (def.name == name).then_some(def.actions))
            .unwrap_or_else(|| panic!("Trigger '{}' not found", name))
    }

    pub fn arm(&mut self, condition: TriggerCondition, name: &str) {
        let actions = self.find_actions(name);
        self.armed_triggers.push(Trigger {
            cond: condition,
            actions,
        });
    }

    fn check_inner(
        &mut self,
        mut matcher: impl FnMut(&TriggerCondition) -> bool,
    ) -> Vec<TriggerAction> {
        let mut fired = Vec::new();
        self.armed_triggers.retain(|trigger| {
            if matcher(&trigger.cond) {
                fired.extend_from_slice(trigger.actions);
                false
            } else {
                true
            }
        });
        fired
    }

    pub fn check_distance(&mut self, current: f64) -> Vec<TriggerAction> {
        self.check_inner(
            |cond| matches!(cond, TriggerCondition::Distance(d) if current.abs() >= *d),
        )
    }

    pub fn check_index(&mut self, current: usize) -> Vec<TriggerAction> {
        self.check_inner(|cond| matches!(cond, TriggerCondition::Index(idx) if current >= *idx))
    }

    pub fn check_angle(&mut self, current: f64) -> Vec<TriggerAction> {
        self.check_inner(|cond| matches!(cond, TriggerCondition::Angle(a) if current.abs() >= *a))
    }

    pub fn trigger_now(&self, name: &str) -> Vec<TriggerAction> {
        self.find_actions(name).to_vec()
    }
}
