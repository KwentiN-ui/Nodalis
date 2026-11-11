//! A face-to-face slave point far beside the master surface, whose closest point is a
//! corner on the master's boundary that several master faces share, touches nothing: the
//! distance along the normal there is no overclosure. (A pin through a bore longer than
//! the bore: its points beyond the bore's end projected onto a corner of the bore's end,
//! got springs with false overclosures of 0.1 mm and the run never converged.)
//!
//! The master is the top of a unit square split into two triangles (two C3D6) and tilted
//! by 5 % in x, so that its normal leans towards the slave; the slave block lies beyond
//! the shared corner (1, 1), 1.5 away in x and y and 0.01 above the corner's height.

use plxs_contact::ContactModel;

const INPUT: &str = "*Node
1, 0, 0, 0
2, 1, 0, 0
3, 1, 1, 0
4, 0, 1, 0
5, 0, 0, 1
6, 1, 0, 1.05
7, 1, 1, 1.05
8, 0, 1, 1
11, 2.5, 2.5, 1.06
12, 3.5, 2.5, 1.06
13, 3.5, 3.5, 1.06
14, 2.5, 3.5, 1.06
15, 2.5, 2.5, 2.06
16, 3.5, 2.5, 2.06
17, 3.5, 3.5, 2.06
18, 2.5, 3.5, 2.06
*Element, Type=C3D6, Elset=BASE
1, 1, 2, 3, 5, 6, 7
2, 1, 3, 4, 5, 7, 8
*Element, Type=C3D8, Elset=BLOCK
3, 11, 12, 13, 14, 15, 16, 17, 18
*Surface, Name=MASTER, Type=Element
BASE, S2
*Surface, Name=SLAVE, Type=Element
BLOCK, S1
*Material, Name=STEEL
*Elastic
210000, 0.3
*Solid section, Elset=BASE, Material=STEEL
*Solid section, Elset=BLOCK, Material=STEEL
*Surface interaction, Name=SOFT
*Surface behavior, Pressure-overclosure=Linear
1e6
*Contact pair, Interaction=SOFT, Type=Surface to surface
SLAVE, MASTER
*Step
*Static
*End step
";

#[test]
fn a_point_beside_a_shared_corner_on_the_master_boundary_has_no_spring() {
    let mut model = plxs_input::read_inp_str(INPUT, None)
        .unwrap_or_else(|e| panic!("{e:?}"))
        .analysis
        .model;
    let mut contact = ContactModel::new(&mut model).unwrap();
    let coords = model.nodes.coordinates.clone();
    let u = vec![0.0; 3 * coords.len()];
    contact.prepare(&coords, &u);
    let evaluation = contact.evaluate(&coords, &u, &[], true);
    assert_eq!(evaluation.active, 0, "{:?}", evaluation.points);
}
