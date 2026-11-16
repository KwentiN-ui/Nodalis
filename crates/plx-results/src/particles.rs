//! The results of a particle analysis in the container of the `.ndlr` files
//! ([`plxs_output::packed`]): the frames of the particles, their fields, the history of the
//! energies and wall loads, and the surfaces of the walls to draw them with. The arrays of
//! each frame (positions, velocities, one array per field) are datasets of their own, so
//! that the container compresses them as it does the fields of an FE run.

use plxs_core::particles::{ParticleResults, TriangleSurface};
use plxs_output::packed::{self, Array, Packable};
use serde::{Deserialize, Serialize};

/// The magic of a particle results container; that of FE results differs, so neither is
/// read as the other.
const MAGIC: &[u8; 8] = b"PLXPRT\x00\x01";

/// A finished particle run: its results and the walls it ran with.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ParticleRun {
    pub results: ParticleResults,
    /// The surfaces of the walls, by name, as the solver took them.
    pub walls: Vec<(String, TriangleSurface)>,
    /// What each body number of the field "Body" is: the name of its part.
    pub bodies: Vec<String>,
}

impl ParticleRun {
    pub fn to_bytes(&mut self) -> Vec<u8> {
        packed::pack_in_place(MAGIC, self)
    }

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, String> {
        packed::unpack(MAGIC, bytes).map_err(|e| e.0)
    }
}

impl Packable for ParticleRun {
    fn arrays(&mut self, visit: &mut dyn FnMut(Array<'_>)) {
        for frame in &mut self.results.frames {
            visit(Array::F32x3(&mut frame.positions));
            visit(Array::F32x3(&mut frame.velocities));
            for field in &mut frame.fields {
                visit(Array::F32(field));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use plxs_core::particles::ParticleFrame;

    #[test]
    fn round_trip() {
        let mut run = ParticleRun {
            results: ParticleResults {
                frames: vec![ParticleFrame {
                    time: 0.5,
                    positions: vec![[1.0, 2.0, 3.0], [4.0, 5.0, 6.5]],
                    velocities: vec![[0.0; 3], [1.0, 0.0, 0.0]],
                    fields: vec![vec![1.5, 2.5], vec![0.0, -1.0]],
                }],
                ..Default::default()
            },
            walls: vec![("Wall".into(), TriangleSurface::cuboid([0.0; 3], [1.0; 3]))],
            bodies: vec!["Paste".into()],
        };
        let bytes = run.to_bytes();
        assert_eq!(ParticleRun::from_bytes(&bytes).unwrap(), run);
    }
}
