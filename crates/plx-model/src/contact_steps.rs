//! Contacts changed per step, Ansys' step controls of a contact region: switched off for
//! some steps, or sliding with another friction coefficient. The solver needs every contact
//! pair before the first step and changes it within a step by `*MODEL CHANGE, TYPE=CONTACT
//! PAIR` and `*CHANGE FRICTION`; a change holds from its step on, until another one.

use serde::{Deserialize, Serialize};

use crate::{ContactBehavior, ContactRegion, FeModel, Id, Step};

/// A change of a contact region or contact pair from a step on.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ContactChange {
    /// The contact region or contact pair.
    pub region: crate::Id,
    /// Switched on or off.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub active: Option<bool>,
    /// The new friction coefficient; for frictional regions only.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub friction: Option<f64>,
}

/// What a contact region is in a step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactState {
    pub active: bool,
    /// The friction coefficient of a frictional region.
    pub friction: Option<f64>,
}

impl ContactState {
    fn changed(mut self, change: &ContactChange) -> Self {
        if let Some(active) = change.active {
            self.active = active;
        }
        if let (Some(mu), Some(_)) = (change.friction, self.friction) {
            self.friction = Some(mu);
        }
        self
    }
}

impl ContactRegion {
    /// The state at the end of `steps`: as defined, then changed by each active step.
    pub fn state_after(&self, steps: &[Step]) -> ContactState {
        let state = ContactState {
            active: true,
            friction: match self.behavior {
                ContactBehavior::Frictional { mu } => Some(mu),
                _ => None,
            },
        };
        self.changes(steps).fold(state, ContactState::changed)
    }

    /// The state in `step`, which follows on `before`, the state at the end of the steps
    /// before it.
    pub fn state_in(&self, step: &Step, before: ContactState) -> ContactState {
        (step.contact_changes.iter())
            .filter(|c| c.region == self.id)
            .fold(before, ContactState::changed)
    }

    /// The changes of the region in the active steps.
    fn changes<'a>(&'a self, steps: &'a [Step]) -> impl Iterator<Item = &'a ContactChange> {
        (steps.iter().filter(|s| s.active))
            .flat_map(|s| &s.contact_changes)
            .filter(|c| c.region == self.id)
    }

    /// Whether a step switches the region off or on.
    pub fn switched(&self, steps: &[Step]) -> bool {
        self.changes(steps).any(|c| c.active.is_some())
    }

    /// Whether an active step prints contact values of the region, a contact history
    /// output of it (`*CONTACT PRINT`).
    pub fn prints_contact(&self, steps: &[Step]) -> bool {
        (steps.iter().filter(|s| s.active))
            .flat_map(|s| &s.history_outputs)
            .any(|o| {
                o.active
                    && matches!(&o.kind, crate::HistoryKind::Contact { pair } if *pair == Some(self.id))
            })
    }

    /// Whether a step changes the friction coefficient.
    pub fn friction_changed(&self, steps: &[Step]) -> bool {
        self.changes(steps).any(|c| c.friction.is_some())
    }
}

impl Step {
    /// Makes the contact region `region` be `wanted` in this step, which follows on a step
    /// where it was `before`: the step keeps only what differs.
    pub fn set_contact_state(&mut self, region: Id, before: ContactState, wanted: ContactState) {
        let active = (wanted.active != before.active).then_some(wanted.active);
        let friction = wanted.friction.filter(|&mu| Some(mu) != before.friction);
        self.contact_changes.retain(|c| c.region != region);
        if active.is_some() || friction.is_some() {
            self.contact_changes.push(ContactChange {
                region,
                active,
                friction,
            });
        }
    }
}

impl FeModel {
    /// Drops the changes of a contact region or pair that is deleted.
    pub fn remove_contact_changes(&mut self, region: Id) {
        for step in &mut self.steps {
            step.contact_changes.retain(|c| c.region != region);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_step_keeps_only_what_it_changes_and_changes_hold_on() {
        let region = ContactRegion::new("Region-1", ContactBehavior::Frictional { mu: 0.15 });
        let mut steps = vec![Step::new_static("Step-1"), Step::new_static("Step-2")];
        let before = region.state_after(&steps[..1]);
        assert_eq!(
            before,
            ContactState {
                active: true,
                friction: Some(0.15)
            }
        );
        let off = ContactState {
            active: false,
            ..before
        };
        steps[0].set_contact_state(region.id, before, off);
        assert_eq!(steps[0].contact_changes[0].friction, None);
        // Step-2 follows on the change of Step-1.
        assert_eq!(region.state_after(&steps), off);
        assert!(region.switched(&steps) && !region.friction_changed(&steps));
        // Back to what it was: no change left.
        steps[0].set_contact_state(region.id, before, before);
        assert!(steps[0].contact_changes.is_empty());
        // A deactivated step changes nothing.
        steps[1].set_contact_state(region.id, before, off);
        steps[1].active = false;
        assert_eq!(region.state_after(&steps), before);
    }
}
