//! A node-to-face slave node beside the master surface touches nothing, also where it lies
//! behind the plane of the border face it projects on. (The 16 points round the
//! circumference of a rope that left a sheave lay above the rim of the groove, behind the
//! plane of the flanks' last faces: they got springs with false overclosures of up to the
//! rope's radius, and a rope with contacts and a finely meshed sheave never converged.
//! The check for nodes beside the master compared the foot point, which the projection
//! clamps to the face, with the face and so never found one.)
//!
//! The master is the top of a unit cube tilted by 5 % in x, so that its normal leans
//! towards the slave; the slave block stands beside it, from 0.5 to 1.5 beyond its edge
//! x = 1, its bottom 0.01 above the edge but below the plane of the top face there.

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
11, 1.5, 0, 1.06
12, 2.5, 0, 1.06
13, 2.5, 1, 1.06
14, 1.5, 1, 1.06
15, 1.5, 0, 2.06
16, 2.5, 0, 2.06
17, 2.5, 1, 2.06
18, 1.5, 1, 2.06
*Element, Type=C3D8, Elset=BASE
1, 1, 2, 3, 4, 5, 6, 7, 8
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
*Contact pair, Interaction=SOFT, Type=Node to surface
SLAVE, MASTER
*Step
*Static
*End step
";

fn evaluate(input: &str) -> plxs_contact::Evaluation {
    let mut model = plxs_input::read_inp_str(input, None)
        .unwrap_or_else(|e| panic!("{e:?}"))
        .analysis
        .model;
    let mut contact = ContactModel::new(&mut model).unwrap();
    let coords = model.nodes.coordinates.clone();
    let u = vec![0.0; 3 * coords.len()];
    contact.prepare(&coords, &u);
    contact.evaluate(&coords, &u, &[], false)
}

#[test]
fn a_node_beside_the_master_behind_the_plane_of_its_border_face_has_no_spring() {
    let evaluation = evaluate(INPUT);
    assert_eq!(evaluation.active, 0, "{:?}", evaluation.points);
}

#[test]
fn a_node_pressed_into_the_master_keeps_its_spring() {
    // The same block moved over the master, its bottom 0.01 below the tilted top face.
    let moved = INPUT
        .replace("11, 1.5, 0, 1.06", "11, 0.2, 0.2, 1.0")
        .replace("12, 2.5, 0, 1.06", "12, 0.8, 0.2, 1.03")
        .replace("13, 2.5, 1, 1.06", "13, 0.8, 0.8, 1.03")
        .replace("14, 1.5, 1, 1.06", "14, 0.2, 0.8, 1.0")
        .replace("15, 1.5, 0, 2.06", "15, 0.2, 0.2, 2.0")
        .replace("16, 2.5, 0, 2.06", "16, 0.8, 0.2, 2.03")
        .replace("17, 2.5, 1, 2.06", "17, 0.8, 0.8, 2.03")
        .replace("18, 1.5, 1, 2.06", "18, 0.2, 0.8, 2.0");
    let evaluation = evaluate(&moved);
    assert_eq!(evaluation.active, 4, "{:?}", evaluation.points);
}
