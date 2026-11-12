//! Node groups of the project schematic, like those of Blender: blocks put together into one
//! block, which opens into a schematic of its own with a Group Input and a Group Output
//! block. The path at the top left of the schematic shows the group open and goes back out.
//!
//! Update Project does not see groups: [`ProjectApp::links`] keeps every link from the block
//! that gives to the block that takes, wherever they are. A group keeps in its sockets only
//! how the wires of those links run through it, to draw them at each level: an input socket
//! has a wire from outside to the group's pin (its `source`) and wires from the Group Input
//! block to blocks inside (its `targets`); an output socket the other way round. A wire
//! between two pins of sockets, such as from a Group Input block to a group inside, is kept
//! by both sockets. Once a wire completes the way from a block to another through sockets,
//! the link between them is made; once a wire of the way goes, the link goes too.

use super::*;
use file::{GroupPin, GroupSocket, NodeGroup};

/// The first element of the path of a pin of a socket, `["Socket", id]`.
pub(super) const SOCKET: &str = "Socket";
/// The last element of the path of an empty pin, which makes a new socket, choice or
/// column once a wire is dropped on it, as in Blender.
pub(super) const OPEN: &str = "+";
/// Depth of nested groups followed at most, against a loop in a damaged file.
const DEPTH: usize = 64;

fn socket_pin(node: u64, socket: u32) -> GroupPin {
    GroupPin {
        node,
        path: vec![SOCKET.to_owned(), socket.to_string()],
    }
}

/// The socket a pin path is of.
pub(super) fn socket_of(path: &[String]) -> Option<u32> {
    match path {
        [cell, id] if cell == SOCKET => id.parse().ok(),
        _ => None,
    }
}

/// Whether the path is that of an empty pin.
pub(super) fn is_open(path: &[String]) -> bool {
    path.last().is_some_and(|last| last == OPEN)
}

/// Which side of which socket a pin of a group or its Group Input or Group Output block is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Side {
    /// The input pin on the group's block: the wire into it is the source of the input
    /// socket.
    Into(u64, u32),
    /// The output pin on the group's block: its wires go to the targets of the output
    /// socket.
    OutOf(u64, u32),
    /// The pin of the Group Input block: its wires go to the targets of the input socket.
    Gives(u64, u32),
    /// The pin of the Group Output block: the wire into it is the source of the output
    /// socket.
    Takes(u64, u32),
}

impl Side {
    /// The group, whether the socket is an input, and the socket.
    fn socket(self) -> (u64, bool, u32) {
        match self {
            Self::Into(g, s) | Self::Gives(g, s) => (g, true, s),
            Self::OutOf(g, s) | Self::Takes(g, s) => (g, false, s),
        }
    }

    /// Whether the pin takes one wire, which is the socket's source.
    fn takes(self) -> bool {
        matches!(self, Self::Into(..) | Self::Takes(..))
    }
}

/// How a wire of the schematic is kept: by a link between two blocks, or by a socket, which
/// keeps one end of it; the other is the socket's pin.
#[derive(Clone, Debug)]
enum Store {
    Link,
    /// The source of the socket of the group, input or not.
    Source(u64, bool, u32),
    /// A target of the socket.
    Target(u64, bool, u32),
}

/// A wire of the schematic shown at one level.
#[derive(Clone, Debug)]
struct Wire {
    from: GroupPin,
    to: GroupPin,
    store: Store,
}

/// A name for a socket that the group has not yet: `name`, or with a number.
fn free_name(group: &NodeGroup, input: bool, name: &str) -> String {
    let taken = |n: &str| {
        let sockets = if input { &group.inputs } else { &group.outputs };
        sockets.iter().any(|s| s.name == n)
    };
    let name = if name.trim().is_empty() {
        if input { "Input" } else { "Output" }
    } else {
        name
    };
    if !taken(name) {
        return name.to_owned();
    }
    (2..)
        .map(|i| format!("{name} {i}"))
        .find(|n| !taken(n))
        .unwrap_or_default()
}

/// An id no socket of the group has; inputs and outputs share them, so that the pin
/// `["Socket", id]` of the group's block is one of either.
fn free_socket(group: &NodeGroup) -> u32 {
    (group.inputs.iter().chain(&group.outputs))
        .map(|s| s.id + 1)
        .max()
        .unwrap_or(1)
}

fn icon_of(kind: &[String]) -> NodeIcon {
    match links::DataType::of(kind) {
        Some(links::DataType::Geometry) => NodeIcon::Geometry,
        Some(links::DataType::Mesh) => NodeIcon::Mesh,
        Some(links::DataType::Results | links::DataType::Number) => NodeIcon::Results,
        Some(links::DataType::Analysis) => NodeIcon::Solution,
        _ => NodeIcon::Setup,
    }
}

fn socket_row(path: Vec<String>, text: String, icon: NodeIcon, tip: String) -> Row {
    Row {
        path,
        text,
        depth: 0,
        icon,
        state: None,
        expander: None,
        input: false,
        output: false,
        linked_from: None,
        collection: false,
        partial: false,
        widget: None,
        tip,
    }
}

/// What groups need of the blocks: their groups, sockets and the wires those keep.
trait Tree {
    fn group(&self, id: u64) -> Option<&NodeGroup>;

    fn group_mut(&mut self, id: u64) -> Option<&mut NodeGroup>;

    fn all_groups(&self) -> impl Iterator<Item = &NodeGroup>;

    fn all_groups_mut(&mut self) -> impl Iterator<Item = &mut NodeGroup>;

    fn socket_mut(&mut self, group: u64, input: bool, socket: u32) -> Option<&mut GroupSocket>;

    fn socket(&self, group: u64, input: bool, socket: u32) -> Option<&GroupSocket>;

    /// The group each block is right inside, by block.
    fn parents(&self) -> HashMap<u64, u64>;

    /// The group the block is right inside; `None` outside all groups.
    fn parent_of(&self, id: u64) -> Option<u64>;

    /// The Group Input and Group Output blocks of a group.
    fn ends(&self, group: u64) -> (Option<u64>, Option<u64>);

    /// The group whose Group Input or Group Output block `end` is.
    fn owner(&self, end: u64) -> Option<u64>;

    fn kind_of(&self, id: u64) -> Option<Kind>;

    /// Whether the pin is one of a block that gives or takes data itself: no group and no
    /// Group Input or Group Output block.
    fn is_real(&self, pin: &GroupPin) -> bool;

    /// Which socket the pin is of, if it is one of a group or its Group Input or Group
    /// Output block.
    fn side_of(&self, pin: &GroupPin) -> Option<Side>;

    /// Whether a wire at the pin is one of a group: of a socket, or of an empty pin of a
    /// Group Input or Group Output block.
    fn group_pin(&self, node: u64, path: &[String]) -> bool;

    /// The blocks a wire into the pin of a group brings data to, through the sockets; `None`
    /// for a pin of another block.
    fn group_targets(&self, node: u64, path: &[String]) -> Option<Vec<u64>>;

    /// The block and row a wire from the pin gives the data of, through the sockets it runs
    /// through; `None` while the way is not complete.
    fn resolve_source(&self, pin: &GroupPin, depth: usize) -> Option<GroupPin>;

    /// The blocks and rows that take what a wire into the pin brings, through the sockets
    /// it runs through.
    fn resolve_targets(&self, pin: &GroupPin, depth: usize) -> Vec<GroupPin>;

    /// The links the sockets of the groups make: from the block and row that gives to each
    /// that takes, where they are in different groups.
    fn routed(&self) -> BTreeSet<(GroupPin, GroupPin)>;

    /// Keeps the wire from `from` to `to` in the sockets at its ends.
    fn store_wire(&mut self, from: &GroupPin, to: &GroupPin);

    /// Removes `to` from the targets of the socket of the pin `from`, if it is one that
    /// gives.
    fn drop_target(&mut self, from: &GroupPin, to: &GroupPin);

    /// Forgets the wire from `from` to `to` in the sockets at its ends.
    fn drop_wire(&mut self, from: &GroupPin, to: &GroupPin);

    /// Forgets the ends of wires at the pins `gone` in all sockets.
    fn forget_pins(&mut self, gone: impl Fn(&GroupPin) -> bool);

    /// What a socket made from the pin is called.
    fn pin_name(&self, pin: &GroupPin) -> String;

    /// The type of the pin: the path of the row that gives it its colour and the links it
    /// fits; that of the pin a socket was made from for a pin of a socket.
    fn pin_kind(&self, pin: &GroupPin) -> Vec<String>;

    /// The wires shown with the group `level` open, or outside all groups, with how each
    /// is kept.
    fn level_wires(&self, links: &[Link], level: Option<u64>) -> Vec<Wire>;

    /// Removes the sockets without wires, and the ends of wires at pins that are gone.
    fn tidy_sockets(&mut self);

    /// The types of the rows of sockets of a block, by row path, for their colours.
    fn socket_kinds(&self, id: u64) -> HashMap<Vec<String>, Vec<String>>;

    /// A new socket of the group made from the pin `like`: named after it, and of its type.
    fn new_socket(&mut self, group: u64, input: bool, like: &GroupPin) -> Option<u32>;
    /// Makes a group of blocks, see [`ProjectApp::group_blocks`].
    fn regroup(
        &mut self,
        links: &[Link],
        level: Option<u64>,
        chosen: &[u64],
        ids: [u64; 3],
        name: String,
        position: [f32; 2],
    ) -> NodeGroup;

    /// Takes a group apart, see [`ProjectApp::ungroup`].
    fn take_apart(&mut self, id: u64, level: Option<u64>) -> Option<Vec<u64>>;
}

impl Tree for Vec<Node> {
    fn group(&self, id: u64) -> Option<&NodeGroup> {
        match &self.iter().find(|n| n.id == id)?.block {
            Block::Group(group) => Some(&**group),
            _ => None,
        }
    }

    fn group_mut(&mut self, id: u64) -> Option<&mut NodeGroup> {
        match &mut self.iter_mut().find(|n| n.id == id)?.block {
            Block::Group(group) => Some(&mut **group),
            _ => None,
        }
    }

    fn all_groups(&self) -> impl Iterator<Item = &NodeGroup> {
        self.iter().filter_map(|n| match &n.block {
            Block::Group(group) => Some(&**group),
            _ => None,
        })
    }

    fn all_groups_mut(&mut self) -> impl Iterator<Item = &mut NodeGroup> {
        self.iter_mut().filter_map(|n| match &mut n.block {
            Block::Group(group) => Some(&mut **group),
            _ => None,
        })
    }

    fn socket_mut(&mut self, group: u64, input: bool, socket: u32) -> Option<&mut GroupSocket> {
        let group = self.group_mut(group)?;
        let sockets = if input {
            &mut group.inputs
        } else {
            &mut group.outputs
        };
        sockets.iter_mut().find(|s| s.id == socket)
    }

    fn socket(&self, group: u64, input: bool, socket: u32) -> Option<&GroupSocket> {
        let group = self.group(group)?;
        let sockets = if input { &group.inputs } else { &group.outputs };
        sockets.iter().find(|s| s.id == socket)
    }

    fn parents(&self) -> HashMap<u64, u64> {
        let mut parents = HashMap::new();
        for group in self.all_groups() {
            for &node in &group.nodes {
                parents.insert(node, group.id);
            }
        }
        parents
    }

    fn parent_of(&self, id: u64) -> Option<u64> {
        self.all_groups()
            .find(|g| g.nodes.contains(&id))
            .map(|g| g.id)
    }

    fn ends(&self, group: u64) -> (Option<u64>, Option<u64>) {
        let Some(group) = self.group(group) else {
            return (None, None);
        };
        let find = |kind: Kind| {
            (group.nodes.iter().copied())
                .find(|id| self.iter().any(|n| n.id == *id && n.kind() == kind))
        };
        (find(Kind::GroupInput), find(Kind::GroupOutput))
    }

    fn owner(&self, end: u64) -> Option<u64> {
        self.parent_of(end)
    }

    fn kind_of(&self, id: u64) -> Option<Kind> {
        self.iter().find(|n| n.id == id).map(Node::kind)
    }

    fn is_real(&self, pin: &GroupPin) -> bool {
        self.kind_of(pin.node)
            .is_some_and(|k| !matches!(k, Kind::Group | Kind::GroupInput | Kind::GroupOutput))
    }

    fn side_of(&self, pin: &GroupPin) -> Option<Side> {
        let socket = socket_of(&pin.path)?;
        match self.kind_of(pin.node)? {
            Kind::Group => {
                let group = self.group(pin.node)?;
                if group.inputs.iter().any(|s| s.id == socket) {
                    Some(Side::Into(pin.node, socket))
                } else if group.outputs.iter().any(|s| s.id == socket) {
                    Some(Side::OutOf(pin.node, socket))
                } else {
                    None
                }
            }
            Kind::GroupInput => {
                let group = self.owner(pin.node)?;
                (self.socket(group, true, socket)).map(|_| Side::Gives(group, socket))
            }
            Kind::GroupOutput => {
                let group = self.owner(pin.node)?;
                (self.socket(group, false, socket)).map(|_| Side::Takes(group, socket))
            }
            _ => None,
        }
    }

    fn group_pin(&self, node: u64, path: &[String]) -> bool {
        let pin = GroupPin {
            node,
            path: path.to_vec(),
        };
        self.side_of(&pin).is_some()
            || (is_open(path)
                && matches!(
                    self.kind_of(node),
                    Some(Kind::GroupInput | Kind::GroupOutput)
                ))
    }

    fn group_targets(&self, node: u64, path: &[String]) -> Option<Vec<u64>> {
        let pin = GroupPin {
            node,
            path: path.to_vec(),
        };
        self.side_of(&pin)?;
        Some(
            self.resolve_targets(&pin, 0)
                .into_iter()
                .map(|p| p.node)
                .collect(),
        )
    }

    fn resolve_source(&self, pin: &GroupPin, depth: usize) -> Option<GroupPin> {
        if depth > DEPTH {
            return None;
        }
        match self.side_of(pin) {
            Some(side @ (Side::OutOf(..) | Side::Gives(..))) => {
                let (g, input, s) = side.socket();
                let source = self.socket(g, input, s)?.source.clone()?;
                self.resolve_source(&source, depth + 1)
            }
            Some(_) => None,
            None => self.is_real(pin).then(|| pin.clone()),
        }
    }

    fn resolve_targets(&self, pin: &GroupPin, depth: usize) -> Vec<GroupPin> {
        if depth > DEPTH {
            return Vec::new();
        }
        match self.side_of(pin) {
            Some(side @ (Side::Into(..) | Side::Takes(..))) => {
                let (g, input, s) = side.socket();
                (self.socket(g, input, s).into_iter())
                    .flat_map(|s| s.targets.clone())
                    .flat_map(|t| self.resolve_targets(&t, depth + 1))
                    .collect()
            }
            Some(_) => Vec::new(),
            None if self.is_real(pin) => vec![pin.clone()],
            None => Vec::new(),
        }
    }

    fn routed(&self) -> BTreeSet<(GroupPin, GroupPin)> {
        let parents = self.parents();
        let mut pairs = BTreeSet::new();
        for group in self.all_groups() {
            for socket in group.inputs.iter().chain(&group.outputs) {
                let Some(source) = (socket.source.as_ref()).and_then(|s| self.resolve_source(s, 0))
                else {
                    continue;
                };
                for target in &socket.targets {
                    for target in self.resolve_targets(target, 0) {
                        if parents.get(&source.node) != parents.get(&target.node) {
                            pairs.insert((source.clone(), target));
                        }
                    }
                }
            }
        }
        pairs
    }

    fn store_wire(&mut self, from: &GroupPin, to: &GroupPin) {
        if let Some(side) = self.side_of(to).filter(|s| s.takes()) {
            let (g, input, s) = side.socket();
            let old = (self.socket_mut(g, input, s)).and_then(|s| s.source.replace(from.clone()));
            if let Some(old) = old.filter(|old| old != from) {
                self.drop_target(&old, to);
            }
        }
        if let Some(side) = self.side_of(from).filter(|s| !s.takes()) {
            let (g, input, s) = side.socket();
            if let Some(socket) = self.socket_mut(g, input, s)
                && !socket.targets.contains(to)
            {
                socket.targets.push(to.clone());
            }
        }
    }

    fn drop_target(&mut self, from: &GroupPin, to: &GroupPin) {
        if let Some(side) = self.side_of(from).filter(|s| !s.takes()) {
            let (g, input, s) = side.socket();
            if let Some(socket) = self.socket_mut(g, input, s) {
                socket.targets.retain(|t| t != to);
            }
        }
    }

    fn drop_wire(&mut self, from: &GroupPin, to: &GroupPin) {
        if let Some(side) = self.side_of(to).filter(|s| s.takes()) {
            let (g, input, s) = side.socket();
            if let Some(socket) = self.socket_mut(g, input, s)
                && socket.source.as_ref() == Some(from)
            {
                socket.source = None;
            }
        }
        self.drop_target(from, to);
    }

    fn forget_pins(&mut self, gone: impl Fn(&GroupPin) -> bool) {
        for group in self.all_groups_mut() {
            for socket in group.inputs.iter_mut().chain(&mut group.outputs) {
                if socket.source.as_ref().is_some_and(&gone) {
                    socket.source = None;
                }
                socket.targets.retain(|t| !gone(t));
            }
        }
    }

    fn pin_name(&self, pin: &GroupPin) -> String {
        if let Some(side) = self.side_of(pin) {
            let (g, input, s) = side.socket();
            return self
                .socket(g, input, s)
                .map_or_else(String::new, |s| s.name.clone());
        }
        match pin.path.as_slice() {
            [cell, _] if cell == links::PARAMETERS => parameter_label(self, pin.node, &pin.path),
            [] => String::new(),
            // The result of a block is best known by the name of the block.
            [.., last] if last == "Result" => self
                .iter()
                .find(|n| n.id == pin.node)
                .map_or_else(|| last.clone(), |n| n.name.clone()),
            [.., last] => last.clone(),
        }
    }

    fn pin_kind(&self, pin: &GroupPin) -> Vec<String> {
        match self.side_of(pin) {
            Some(side) => {
                let (g, input, s) = side.socket();
                self.socket(g, input, s)
                    .map_or_else(Vec::new, |s| s.kind.clone())
            }
            None => pin.path.clone(),
        }
    }

    fn level_wires(&self, links: &[Link], level: Option<u64>) -> Vec<Wire> {
        let parents = self.parents();
        let here = |id: u64| parents.get(&id).copied() == level;
        let mut wires = Vec::new();
        for link in links {
            let (from, to) = (
                GroupPin {
                    node: link.source,
                    path: links::output_path(link),
                },
                GroupPin {
                    node: link.target,
                    path: links::slot(link).to_vec(),
                },
            );
            if here(link.source) && here(link.target) && self.is_real(&from) && self.is_real(&to) {
                wires.push(Wire {
                    from,
                    to,
                    store: Store::Link,
                });
            }
        }
        for group in self.all_groups() {
            let g = group.id;
            if here(g) {
                for s in &group.inputs {
                    if let Some(source) = &s.source {
                        wires.push(Wire {
                            from: source.clone(),
                            to: socket_pin(g, s.id),
                            store: Store::Source(g, true, s.id),
                        });
                    }
                }
                for s in &group.outputs {
                    for target in &s.targets {
                        wires.push(Wire {
                            from: socket_pin(g, s.id),
                            to: target.clone(),
                            store: Store::Target(g, false, s.id),
                        });
                    }
                }
            }
            if level == Some(g) {
                let (input, output) = self.ends(g);
                if let Some(input) = input {
                    for s in &group.inputs {
                        for target in &s.targets {
                            wires.push(Wire {
                                from: socket_pin(input, s.id),
                                to: target.clone(),
                                store: Store::Target(g, true, s.id),
                            });
                        }
                    }
                }
                if let Some(output) = output {
                    for s in &group.outputs {
                        if let Some(source) = &s.source {
                            wires.push(Wire {
                                from: source.clone(),
                                to: socket_pin(output, s.id),
                                store: Store::Source(g, false, s.id),
                            });
                        }
                    }
                }
            }
        }
        wires
    }

    fn tidy_sockets(&mut self) {
        let exists: BTreeSet<u64> = self.iter().map(|n| n.id).collect();
        let mut sockets: BTreeSet<(u64, u32)> = BTreeSet::new();
        for group in self.all_groups() {
            for socket in group.inputs.iter().chain(&group.outputs) {
                sockets.insert((group.id, socket.id));
            }
        }
        let ends: HashMap<u64, u64> = (self.iter())
            .filter(|n| matches!(n.kind(), Kind::GroupInput | Kind::GroupOutput))
            .filter_map(|n| Some((n.id, self.owner(n.id)?)))
            .collect();
        let valid = |p: &GroupPin| {
            exists.contains(&p.node)
                && socket_of(&p.path).is_none_or(|s| {
                    let group = ends.get(&p.node).copied().unwrap_or(p.node);
                    sockets.contains(&(group, s))
                })
        };
        for group in self.all_groups_mut() {
            for socket in group.inputs.iter_mut().chain(&mut group.outputs) {
                if socket.source.as_ref().is_some_and(|p| !valid(p)) {
                    socket.source = None;
                }
                socket.targets.retain(&valid);
            }
            let used = |s: &GroupSocket| s.source.is_some() || !s.targets.is_empty();
            group.inputs.retain(used);
            group.outputs.retain(used);
        }
    }

    fn socket_kinds(&self, id: u64) -> HashMap<Vec<String>, Vec<String>> {
        let group = match self.kind_of(id) {
            Some(Kind::Group) => self.group(id),
            Some(Kind::GroupInput | Kind::GroupOutput) => {
                self.owner(id).and_then(|g| self.group(g))
            }
            _ => None,
        };
        (group.into_iter())
            .flat_map(|g| g.inputs.iter().chain(&g.outputs))
            .map(|s| (vec![SOCKET.to_owned(), s.id.to_string()], s.kind.clone()))
            .collect()
    }

    fn new_socket(&mut self, group: u64, input: bool, like: &GroupPin) -> Option<u32> {
        let name = self.pin_name(like);
        let kind = self.pin_kind(like);
        let group = self.group_mut(group)?;
        let id = free_socket(group);
        let name = free_name(group, input, &name);
        let socket = GroupSocket {
            id,
            name,
            kind,
            source: None,
            targets: Vec::new(),
        };
        if input {
            group.inputs.push(socket);
        } else {
            group.outputs.push(socket);
        }
        Some(id)
    }

    /// Makes the group `ids` (the group, its Group Input and its Group Output block) of the
    /// blocks `chosen` at the level `level`: the wires that went in or out of them run
    /// through its new sockets, and the sockets of other groups keep their ends at the
    /// group's pins. Returns the group, not yet among the blocks.
    fn regroup(
        &mut self,
        links: &[Link],
        level: Option<u64>,
        chosen: &[u64],
        [g, gi, go]: [u64; 3],
        name: String,
        position: [f32; 2],
    ) -> NodeGroup {
        let wires = self.level_wires(links, level);
        let mut group = NodeGroup {
            id: g,
            name,
            position,
            nodes: chosen.iter().copied().chain([gi, go]).collect(),
            inputs: Vec::new(),
            outputs: Vec::new(),
        };
        let inside = |id: u64| chosen.contains(&id);
        // The other blocks of the group, to rewrite how their sockets keep the wires.
        let mut rewrites: Vec<(Store, GroupPin, GroupPin)> = Vec::new();
        for wire in wires {
            let (from_in, to_in) = (inside(wire.from.node), inside(wire.to.node));
            if from_in == to_in {
                continue;
            }
            let input = to_in;
            let found = {
                let sockets = if input { &group.inputs } else { &group.outputs };
                (sockets.iter())
                    .find(|s| s.source.as_ref() == Some(&wire.from))
                    .map(|s| s.id)
            };
            let socket = match found {
                Some(id) => id,
                None => {
                    let id = free_socket(&group);
                    let name = free_name(&group, input, &self.pin_name(&wire.from));
                    let socket = GroupSocket {
                        id,
                        name,
                        kind: self.pin_kind(&wire.from),
                        source: Some(wire.from.clone()),
                        targets: Vec::new(),
                    };
                    if input {
                        group.inputs.push(socket);
                    } else {
                        group.outputs.push(socket);
                    }
                    id
                }
            };
            let sockets = if input {
                &mut group.inputs
            } else {
                &mut group.outputs
            };
            if let Some(s) = sockets.iter_mut().find(|s| s.id == socket)
                && !s.targets.contains(&wire.to)
            {
                s.targets.push(wire.to.clone());
            }
            // The end a socket keeps becomes the group's pin if it is inside, that of its
            // Group Input or Group Output block if it is outside.
            let kept = match wire.store {
                Store::Link => continue,
                Store::Source(..) => &wire.from,
                Store::Target(..) => &wire.to,
            };
            let new = if inside(kept.node) {
                socket_pin(g, socket)
            } else {
                socket_pin(if input { gi } else { go }, socket)
            };
            rewrites.push((wire.store.clone(), kept.clone(), new));
        }
        for (store, old, new) in rewrites {
            match store {
                Store::Link => {}
                Store::Source(h, input, s) => {
                    if let Some(socket) = self.socket_mut(h, input, s) {
                        socket.source = Some(new);
                    }
                }
                Store::Target(h, input, s) => {
                    if let Some(socket) = self.socket_mut(h, input, s) {
                        for target in &mut socket.targets {
                            if *target == old {
                                *target = new.clone();
                            }
                        }
                        let mut seen = BTreeSet::new();
                        socket.targets.retain(|t| seen.insert(t.clone()));
                    }
                }
            }
        }
        if let Some(parent) = level.and_then(|l| self.group_mut(l)) {
            parent.nodes.retain(|n| !chosen.contains(n));
            parent.nodes.push(g);
        }
        group
    }

    /// Takes the group `id` at the level `level` apart: the wires through its sockets run
    /// straight again, kept by the sockets around, and its blocks go to the level. Returns
    /// its blocks; the group and its Group Input and Group Output blocks are left empty, to
    /// be removed.
    fn take_apart(&mut self, id: u64, level: Option<u64>) -> Option<Vec<u64>> {
        let group = self.group(id)?.clone();
        let (gi, go) = self.ends(id);
        let replace_target = |app: &mut Self, old: &GroupPin, new: &[GroupPin]| {
            for g in app.all_groups_mut() {
                for socket in g.inputs.iter_mut().chain(&mut g.outputs) {
                    if socket.targets.contains(old) {
                        socket.targets.retain(|t| t != old);
                        for pin in new {
                            if !socket.targets.contains(pin) {
                                socket.targets.push(pin.clone());
                            }
                        }
                    }
                }
            }
        };
        let replace_source = |app: &mut Self, old: &GroupPin, new: Option<GroupPin>| {
            for g in app.all_groups_mut() {
                for socket in g.inputs.iter_mut().chain(&mut g.outputs) {
                    if socket.source.as_ref() == Some(old) {
                        socket.source = new.clone();
                    }
                }
            }
        };
        for input in &group.inputs {
            let Some(now) = self.socket(id, true, input.id).cloned() else {
                continue;
            };
            replace_target(self, &socket_pin(id, input.id), &now.targets);
            if let Some(gi) = gi {
                replace_source(self, &socket_pin(gi, input.id), now.source.clone());
            }
        }
        for output in &group.outputs {
            let Some(now) = self.socket(id, false, output.id).cloned() else {
                continue;
            };
            replace_source(self, &socket_pin(id, output.id), now.source.clone());
            if let Some(go) = go {
                replace_target(self, &socket_pin(go, output.id), &now.targets);
            }
        }
        let members: Vec<u64> = (group.nodes.iter().copied())
            .filter(|n| Some(*n) != gi && Some(*n) != go)
            .collect();
        if let Some(parent) = level.and_then(|l| self.group_mut(l)) {
            parent.nodes.extend(members.iter().copied());
        }
        if let Some(own) = self.group_mut(id) {
            own.nodes.clear();
        }
        Some(members)
    }
}

impl ProjectApp {
    /// Whether a wire at the pin is one of a group, see [`Tree::group_pin`].
    pub(super) fn group_pin(&self, node: u64, path: &[String]) -> bool {
        self.nodes.group_pin(node, path)
    }

    /// The blocks a wire into the pin of a group brings data to, see
    /// [`Tree::group_targets`].
    pub(super) fn group_targets(&self, node: u64, path: &[String]) -> Option<Vec<u64>> {
        self.nodes.group_targets(node, path)
    }

    /// The types of the rows of sockets of a block, see [`Tree::socket_kinds`].
    pub(super) fn socket_kinds(&self, id: u64) -> HashMap<Vec<String>, Vec<String>> {
        self.nodes.socket_kinds(id)
    }
}

impl ProjectApp {
    /// The group shown in the schematic; `None` for the blocks outside all groups.
    pub(super) fn shown_group(&self) -> Option<u64> {
        self.level.last().copied()
    }

    /// The blocks the schematic shows: those right in the group shown.
    pub(super) fn here(&self) -> BTreeSet<u64> {
        let level = self.shown_group();
        let parents = self.nodes.parents();
        (self.nodes.iter())
            .map(|n| n.id)
            .filter(|id| parents.get(id).copied() == level)
            .collect()
    }

    /// Makes and removes the links that the sockets make now and not `before`, or the other
    /// way round.
    fn route(&mut self, before: BTreeSet<(GroupPin, GroupPin)>) {
        let after = self.nodes.routed();
        for (source, target) in before.difference(&after) {
            self.unlink(|l| {
                l.source == source.node
                    && links::covers(&source.path, &links::output_path(l))
                    && l.target == target.node
                    && links::ends_at(&target.path, l)
            });
        }
        for (source, target) in after.difference(&before) {
            self.link(
                source.node,
                source.path.clone(),
                target.node,
                target.path.clone(),
            );
        }
        self.sync_due = true;
    }

    /// Takes the block out of the groups and their sockets, as it is deleted; the group
    /// shown is left if it is the block.
    pub(super) fn leave_groups(&mut self, id: u64) {
        for group in self.nodes.all_groups_mut() {
            group.nodes.retain(|n| *n != id);
        }
        self.nodes.forget_pins(|p| p.node == id);
        if let Some(at) = self.level.iter().position(|g| *g == id) {
            self.level.truncate(at);
        }
    }

    /// The rows of a group, its Group Input or its Group Output block: a row with a pin for
    /// each socket, and on the blocks inside an empty pin at the bottom for a new one.
    pub(super) fn group_rows(&self, node: &Node) -> Vec<Row> {
        // A row of a socket, an input of the group or not, with its pin on the right or left.
        let row = |socket: &GroupSocket, input: bool, output: bool, tip: &str| Row {
            input: !output,
            output,
            ..socket_row(
                vec![SOCKET.to_owned(), socket.id.to_string()],
                socket.name.clone(),
                icon_of(&socket.kind),
                format!(
                    "{} of the group. Double-click to rename it.\n{tip}",
                    if input { "An input" } else { "An output" }
                ),
            )
        };
        let open = |input: bool, tip: &str| Row {
            input,
            output: !input,
            ..socket_row(
                vec![SOCKET.to_owned(), OPEN.to_owned()],
                String::new(),
                NodeIcon::Link,
                tip.to_owned(),
            )
        };
        match &node.block {
            Block::Group(group) => {
                let mut rows: Vec<Row> = (group.outputs.iter())
                    .map(|s| row(s, false, true, "Its wires go on from here."))
                    .chain(
                        (group.inputs.iter())
                            .map(|s| row(s, true, false, "A wire here goes on inside.")),
                    )
                    .collect();
                if rows.is_empty() {
                    rows.push(socket_row(
                        Vec::new(),
                        format!("{} blocks", group.nodes.len().saturating_sub(2)),
                        NodeIcon::Group,
                        "Tab or a double-click on the header opens the group. Its inputs and \
                         outputs are made inside, at the empty pins of its Group Input and \
                         Group Output blocks."
                            .into(),
                    ));
                }
                rows
            }
            Block::GroupInput => {
                let Some(group) = self.nodes.owner(node.id).and_then(|g| self.nodes.group(g))
                else {
                    return Vec::new();
                };
                let mut rows: Vec<Row> = (group.inputs.iter())
                    .map(|s| row(s, true, true, "Its wires go to the blocks of the group."))
                    .collect();
                rows.push(open(
                    false,
                    "Drag a wire from this empty pin to a block of the group: the group gets an \
                     input for it, and the next empty pin comes below.",
                ));
                rows
            }
            Block::GroupOutput => {
                let Some(group) = self.nodes.owner(node.id).and_then(|g| self.nodes.group(g))
                else {
                    return Vec::new();
                };
                let mut rows: Vec<Row> = (group.outputs.iter())
                    .map(|s| row(s, false, false, "What the group gives out there."))
                    .collect();
                rows.push(open(
                    true,
                    "Drag a wire from a block of the group to this empty pin: the group gets an \
                     output for it, and the next empty pin comes below.",
                ));
                rows
            }
            _ => Vec::new(),
        }
    }

    /// Why a wire from the pin `from` cannot end at `to`, where one of them is a pin of a
    /// group: checked between the blocks the wire would link, else by the types of the pins.
    pub(super) fn group_refusal(
        &self,
        from: (u64, &[String]),
        to: (u64, &[String]),
    ) -> Option<String> {
        let from = GroupPin {
            node: from.0,
            path: from.1.to_vec(),
        };
        let to = GroupPin {
            node: to.0,
            path: to.1.to_vec(),
        };
        if from.node == to.node {
            return Some("a block cannot take data from itself.".into());
        }
        let (from_open, to_open) = (is_open(&from.path), is_open(&to.path));
        if from_open && to_open {
            return Some("two empty pins give nothing to link.".into());
        }
        if from_open || to_open {
            // A new socket takes the type of the other end.
            return None;
        }
        let source = self.nodes.resolve_source(&from, 0);
        let targets = self.nodes.resolve_targets(&to, 0);
        if let Some(source) = &source
            && !targets.is_empty()
        {
            return (targets.iter()).find_map(|target| {
                refusal(
                    &self.nodes,
                    &self.links,
                    source.node,
                    &source.path,
                    target.node,
                    &target.path,
                )
            });
        }
        let (given, taken) = (self.nodes.pin_kind(&from), self.nodes.pin_kind(&to));
        let data = |path: &[String]| links::DataType::of(path);
        match (data(&given), data(&taken)) {
            (Some(a), Some(b)) if a != b && !links::compatible(&given, &taken) => Some(format!(
                "{} gives {} and {} takes {}.",
                self.nodes.pin_name(&from),
                with_article(a.label()),
                self.nodes.pin_name(&to),
                with_article(b.label())
            )),
            _ => None,
        }
    }

    /// A wire from the pin `from` to `to` was drawn, one of them a pin of a group: kept in
    /// the sockets, with the links it completes.
    pub(super) fn link_pins(&mut self, from: GroupPin, to: GroupPin) {
        let before = self.nodes.routed();
        let mut from = from;
        let mut to = to;
        if is_open(&from.path) && is_open(&to.path) {
            return;
        }
        // An empty pin becomes a new socket of the type of the other end.
        if is_open(&from.path)
            && self.nodes.kind_of(from.node) == Some(Kind::GroupInput)
            && let Some(group) = self.nodes.owner(from.node)
        {
            let Some(socket) = self.nodes.new_socket(group, true, &to) else {
                return;
            };
            from = socket_pin(from.node, socket);
        }
        if is_open(&to.path)
            && self.nodes.kind_of(to.node) == Some(Kind::GroupOutput)
            && let Some(group) = self.nodes.owner(to.node)
        {
            let Some(socket) = self.nodes.new_socket(group, false, &from) else {
                return;
            };
            to = socket_pin(to.node, socket);
        }
        if let Some(why) = self.group_refusal((from.node, &from.path), (to.node, &to.path)) {
            self.output.push(format!("Not linked: {why}"));
            self.nodes.tidy_sockets();
            return;
        }
        // A pin of a single value takes one wire, the new one.
        let assembly =
            links::assembles(&to.path) && self.nodes.kind_of(to.node) == Some(Kind::Analysis);
        if self.nodes.is_real(&to) && !links::is_collection(&to.path) && !assembly {
            let pin = to.clone();
            self.nodes.forget_pins(|p| *p == pin);
            let here = self.nodes.parents().get(&to.node).copied();
            let parents = self.nodes.parents();
            self.unlink(|l| {
                l.target == pin.node
                    && links::ends_at(&pin.path, l)
                    && parents.get(&l.source).copied() == here
            });
        }
        self.nodes.store_wire(&from, &to);
        self.route(before);
    }

    /// Removes the wire between two pins, or all wires of one pin, where a wire of a group
    /// is among them, with the links that went through them; `false` if it is no business
    /// of groups.
    pub(super) fn unlink_pins(
        &mut self,
        source: Option<&(u64, Vec<String>)>,
        target: Option<&(u64, Vec<String>)>,
    ) -> bool {
        let pin = |(node, path): &(u64, Vec<String>)| GroupPin {
            node: *node,
            path: path.clone(),
        };
        let (source, target) = (source.map(pin), target.map(pin));
        let grouped = |p: &Option<GroupPin>| {
            p.as_ref()
                .is_some_and(|p| self.nodes.group_pin(p.node, &p.path))
        };
        let in_sockets = |p: &Option<GroupPin>| {
            p.as_ref().is_some_and(|p| {
                self.nodes.all_groups().any(|g| {
                    (g.inputs.iter().chain(&g.outputs))
                        .any(|s| s.source.as_ref() == Some(p) || s.targets.contains(p))
                })
            })
        };
        let any_grouped = grouped(&source) || grouped(&target);
        if !any_grouped && !in_sockets(&source) && !in_sockets(&target) {
            return false;
        }
        let before = self.nodes.routed();
        match (&source, &target) {
            (Some(from), Some(to)) => self.nodes.drop_wire(from, to),
            (Some(from), None) => {
                // The wires the pin gives here: those kept by its socket, and those into
                // sockets it is the source of.
                if let Some(side) = self.nodes.side_of(from).filter(|s| !s.takes()) {
                    let (g, input, s) = side.socket();
                    let targets = (self.nodes.socket_mut(g, input, s))
                        .map(|s| std::mem::take(&mut s.targets))
                        .unwrap_or_default();
                    for to in targets {
                        self.nodes.drop_wire(from, &to);
                    }
                }
                let from = from.clone();
                for group in self.nodes.all_groups_mut() {
                    for socket in group.inputs.iter_mut().chain(&mut group.outputs) {
                        if socket.source.as_ref() == Some(&from) {
                            socket.source = None;
                        }
                    }
                }
            }
            (None, Some(to)) => {
                if let Some(side) = self.nodes.side_of(to).filter(|s| s.takes()) {
                    let (g, input, s) = side.socket();
                    let old = self
                        .nodes
                        .socket_mut(g, input, s)
                        .and_then(|s| s.source.take());
                    if let Some(old) = old {
                        self.nodes.drop_target(&old, to);
                    }
                }
                let to = to.clone();
                for group in self.nodes.all_groups_mut() {
                    for socket in group.inputs.iter_mut().chain(&mut group.outputs) {
                        socket.targets.retain(|t| *t != to);
                    }
                }
            }
            (None, None) => {}
        }
        self.route(before);
        // The wires between blocks right here, as without groups.
        if !any_grouped {
            let parents = self.nodes.parents();
            let same = |l: &Link| parents.get(&l.source) == parents.get(&l.target);
            self.unlink(|link| {
                same(link)
                    && source.as_ref().is_none_or(|p| {
                        link.source == p.node && links::covers(&p.path, &links::output_path(link))
                    })
                    && target
                        .as_ref()
                        .is_none_or(|p| link.target == p.node && links::ends_at(&p.path, link))
            });
        }
        self.nodes.tidy_sockets();
        true
    }

    /// Lets go of the wires of sockets whose link is gone, such as one refused or removed
    /// with a block: each wire into a block from a socket whose way starts at a block is
    /// kept only while a link between the two is there. A socket left without wires goes.
    pub(super) fn prune_sockets(&mut self) {
        let mut gone: Vec<(u64, bool, u32, GroupPin)> = Vec::new();
        for group in self.nodes.all_groups() {
            for (input, socket) in (group.inputs.iter().map(|s| (true, s)))
                .chain(group.outputs.iter().map(|s| (false, s)))
            {
                let Some(source) =
                    (socket.source.as_ref()).and_then(|s| self.nodes.resolve_source(s, 0))
                else {
                    continue;
                };
                for target in socket.targets.iter().filter(|t| self.nodes.is_real(t)) {
                    if !(self.links.iter())
                        .any(|l| l.source == source.node && l.target == target.node)
                    {
                        gone.push((group.id, input, socket.id, target.clone()));
                    }
                }
            }
        }
        for (group, input, socket, target) in gone {
            if let Some(socket) = self.nodes.socket_mut(group, input, socket) {
                socket.targets.retain(|t| *t != target);
            }
        }
        self.nodes.tidy_sockets();
    }

    /// The wires of sockets the schematic shows now: from block and row to block and row.
    pub(super) fn socket_wires(&self) -> Vec<(GroupPin, GroupPin)> {
        (self
            .nodes
            .level_wires(&self.links, self.shown_group())
            .into_iter())
        .filter(|w| !matches!(w.store, Store::Link))
        .map(|w| (w.from, w.to))
        .collect()
    }

    /// Puts the blocks into a new group, Ctrl+G: its block takes their place, and wires
    /// that went in or out of them run through its new sockets. The links stay as they are.
    pub(super) fn group_blocks(&mut self, ids: &[u64]) {
        let here = self.here();
        let chosen: Vec<u64> = (ids.iter().copied())
            .filter(|id| here.contains(id))
            .filter(|id| {
                !matches!(
                    self.nodes.kind_of(*id),
                    Some(Kind::GroupInput | Kind::GroupOutput) | None
                )
            })
            .collect();
        if chosen.is_empty() {
            return;
        }
        let level = self.shown_group();
        // Where the blocks are, to put the group there and its ends left and right of them.
        let bounds = (chosen.iter())
            .filter_map(|id| {
                let pos = self.graph.get_node_info(self.graph_node(*id)?)?.pos;
                let size = (self.block_sizes.get(id).copied()).unwrap_or(egui::vec2(220.0, 60.0));
                Some(egui::Rect::from_min_size(pos, size))
            })
            .fold(egui::Rect::NOTHING, egui::Rect::union);
        let bounds = if bounds.is_finite() {
            bounds
        } else {
            egui::Rect::from_min_size(egui::pos2(0.0, 0.0), egui::vec2(220.0, 60.0))
        };
        let g = self.next_id();
        let ids = [g, g + 1, g + 2];
        let (gi, go) = (g + 1, g + 2);
        let position = [bounds.center().x - 110.0, bounds.center().y - 30.0];
        let group = self.nodes.regroup(
            &self.links,
            level,
            &chosen,
            ids,
            block_name("Group"),
            position,
        );
        self.leave_frames(&chosen);
        let name = group.name.clone();
        let position = egui::pos2(group.position[0], group.position[1]);
        self.nodes
            .push(Node::new(g, name.clone(), Block::Group(Box::new(group))));
        self.graph.insert_node(position, g);
        self.nodes
            .push(Node::new(gi, "Group Input".into(), Block::GroupInput));
        self.graph.insert_node(
            egui::pos2(bounds.left() - 320.0, bounds.center().y - 30.0),
            gi,
        );
        self.nodes
            .push(Node::new(go, "Group Output".into(), Block::GroupOutput));
        self.graph.insert_node(
            egui::pos2(bounds.right() + 80.0, bounds.center().y - 30.0),
            go,
        );
        self.known.extend([g, gi, go]);
        self.selected = vec![g];
        self.output.push(format!(
            "{name} made of {} blocks; Tab opens it.",
            chosen.len()
        ));
    }

    /// Puts the blocks of the group back where the group is, Ctrl+Alt+G, and removes it: the
    /// wires through its sockets run straight again.
    pub(super) fn ungroup(&mut self, id: u64) {
        let Some(group) = self.nodes.group(id).cloned() else {
            return;
        };
        if !self.here().contains(&id) {
            return;
        }
        let (gi, go) = self.nodes.ends(id);
        let level = self.shown_group();
        let Some(members) = self.nodes.take_apart(id, level) else {
            return;
        };
        for end in [gi, go].into_iter().flatten().chain([id]) {
            if let Some(node) = self.detach(end) {
                self.keep_deleted(node);
            }
        }
        self.nodes.tidy_sockets();
        self.selected = members;
        self.output.push(format!("{} ungrouped", group.name));
    }

    /// Duplicates a group with the blocks in it and the links between them, Shift+D; the
    /// copy has the sockets of the group, without wires outside.
    pub(super) fn duplicate_group(&mut self, id: u64, offset: egui::Vec2) -> Option<u64> {
        let group = self.nodes.group(id)?.clone();
        // The blocks inside, at every depth.
        let mut inner = Vec::new();
        let mut open = vec![id];
        while let Some(g) = open.pop() {
            for &n in &self.nodes.group(g)?.nodes {
                inner.push(n);
                if self.nodes.kind_of(n) == Some(Kind::Group) {
                    open.push(n);
                }
            }
        }
        let map: HashMap<u64, u64> = std::iter::once(id)
            .chain(inner.iter().copied())
            .zip(self.next_id()..)
            .collect();
        let remap = |pin: &GroupPin| -> Option<GroupPin> {
            Some(GroupPin {
                node: *map.get(&pin.node)?,
                path: pin.path.clone(),
            })
        };
        let mut added = Vec::new();
        for &old in std::iter::once(&id).chain(&inner) {
            let new = map[&old];
            let position = (self.graph_node(old))
                .and_then(|g| self.graph.get_node_info(g))
                .map_or(egui::pos2(40.0, 40.0), |info| info.pos);
            let node = self.nodes.iter_mut().find(|n| n.id == old)?;
            let name = if old == id {
                format!("Copy of {}", node.name)
            } else {
                node.name.clone()
            };
            let copy = match &node.block {
                Block::Group(g) => {
                    let mut g = (**g).clone();
                    g.id = new;
                    g.nodes = g.nodes.iter().filter_map(|n| map.get(n).copied()).collect();
                    for socket in g.inputs.iter_mut().chain(&mut g.outputs) {
                        socket.source = socket.source.as_ref().and_then(remap);
                        socket.targets = socket.targets.iter().filter_map(remap).collect();
                    }
                    Node::new(new, name, Block::Group(Box::new(g)))
                }
                Block::GroupInput => Node::new(new, name, Block::GroupInput),
                Block::GroupOutput => Node::new(new, name, Block::GroupOutput),
                _ => node.copied(new, name)?,
            };
            let at = if old == id {
                position + offset
            } else {
                position
            };
            added.push((copy, at));
        }
        for (node, at) in added {
            let new = node.id;
            self.nodes.push(node);
            self.graph.insert_node(at, new);
            self.known.insert(new);
        }
        let _ = group;
        // The links between the blocks inside.
        let copied: Vec<Link> = (self.links.iter())
            .filter_map(|l| {
                Some(Link {
                    source: *map.get(&l.source)?,
                    target: *map.get(&l.target)?,
                    ..l.clone()
                })
            })
            .collect();
        self.links.extend(copied);
        let copy = map[&id];
        if let Some(parent) = self.shown_group().and_then(|l| self.nodes.group_mut(l)) {
            parent.nodes.push(copy);
        }
        self.nodes.tidy_sockets();
        self.sync_due = true;
        Some(copy)
    }

    /// Opens the group in the schematic, with the groups around it.
    pub(super) fn enter_group(&mut self, id: u64) {
        if self.nodes.kind_of(id) != Some(Kind::Group) {
            return;
        }
        let mut path = vec![id];
        let mut at = id;
        while let Some(parent) = self.nodes.parent_of(at) {
            if path.contains(&parent) || path.len() > DEPTH {
                break;
            }
            path.push(parent);
            at = parent;
        }
        path.reverse();
        self.show_level(path);
    }

    /// Shows the level of `path`: the groups open, the outermost first.
    pub(super) fn show_level(&mut self, path: Vec<u64>) {
        if path == self.level {
            return;
        }
        // The group left is selected outside, as in Blender.
        let left = (path.len() < self.level.len()).then(|| self.level[path.len()]);
        self.level = path;
        self.graph_generation += 1;
        self.show_all = true;
        self.selected = left.into_iter().collect();
        self.picker = None;
    }

    /// Goes out of the group shown, to the level around it.
    pub(super) fn leave_group(&mut self) {
        let mut path = self.level.clone();
        if path.pop().is_some() {
            self.show_level(path);
        }
    }

    /// At the start of every frame: the groups shown are still there, each inside the one
    /// before, and blocks added since the last frame go into the group shown.
    pub(super) fn keep_level(&mut self) {
        let mut valid = 0;
        let mut parent = None;
        for &g in &self.level {
            if self.nodes.kind_of(g) != Some(Kind::Group) || self.nodes.parent_of(g) != parent {
                break;
            }
            parent = Some(g);
            valid += 1;
        }
        if valid < self.level.len() {
            let path = self.level[..valid].to_vec();
            self.show_level(path);
        }
        let ids: BTreeSet<u64> = self.nodes.iter().map(|n| n.id).collect();
        if let Some(level) = self.shown_group() {
            let parents = self.nodes.parents();
            let new: Vec<u64> = (ids.iter().copied())
                .filter(|id| !self.known.contains(id) && *id != level)
                .filter(|id| !parents.contains_key(id))
                .collect();
            if !new.is_empty()
                && let Some(group) = self.nodes.group_mut(level)
            {
                group.nodes.extend(new);
            }
        }
        self.known = ids;
    }

    /// The path of the groups open at the top left of the schematic: Project, then each
    /// group; a click goes back to that level.
    pub(super) fn group_path(&mut self, ui: &mut egui::Ui, area: egui::Rect) {
        let names: Vec<(u64, String)> =
            (self.level.iter()).map(|&g| (g, self.name_of(g))).collect();
        let mut chosen = None;
        // In an area of its own: the schematic takes the clicks on its own layer otherwise.
        let ctx = ui.ctx().clone();
        egui::Area::new(crate::ids::window(&ctx, "project group path"))
            .order(egui::Order::Foreground)
            .fixed_pos(area.min + egui::vec2(10.0, 8.0))
            .show(&ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 4.0;
                    let crumb = |ui: &mut egui::Ui, text: &str, current: bool| {
                        let text = egui::RichText::new(text);
                        let text = if current { text.strong() } else { text };
                        ui.add(egui::Button::new(text).frame(false))
                    };
                    let top = crumb(ui, &self.project_name(), names.is_empty())
                        .on_hover_text("The blocks outside all groups");
                    if top.clicked() {
                        chosen = Some(0);
                    }
                    for (depth, (_, name)) in names.iter().enumerate() {
                        ui.weak("›");
                        let current = depth + 1 == names.len();
                        let tip = if current {
                            "The group shown; Ctrl+Tab goes out of it"
                        } else {
                            "Shows this group"
                        };
                        if crumb(ui, name, current).on_hover_text(tip).clicked() {
                            chosen = Some(depth + 1);
                        }
                    }
                });
            });
        if let Some(depth) = chosen {
            let path = self.level[..depth].to_vec();
            self.show_level(path);
        }
    }
}

impl ProjectApp {
    /// The blocks of `ids` that can be deleted: all but the Group Input and Group Output
    /// blocks, which belong to their group.
    pub(super) fn deletable(&mut self, ids: Vec<u64>) -> Vec<u64> {
        let (ends, rest): (Vec<u64>, Vec<u64>) = ids.into_iter().partition(|id| {
            matches!(
                self.nodes.kind_of(*id),
                Some(Kind::GroupInput | Kind::GroupOutput)
            )
        });
        if !ends.is_empty() {
            self.output.push(
                "The Group Input and Group Output blocks belong to their group and are not \
                 deleted; Ctrl+Alt+G on the group outside puts its blocks back."
                    .into(),
            );
        }
        rest
    }

    /// The input of a Switch block that a wire dropped on its empty pin takes: a new choice,
    /// which is added; `None` for any other pin.
    pub(super) fn new_choice(&mut self, target: u64, slot: &[String]) -> Option<Vec<String>> {
        let [cell, key] = slot else {
            return None;
        };
        if cell != links::PARAMETERS || key != OPEN {
            return None;
        }
        let (number, true) = self.node(target).and_then(Node::as_number_mut)? else {
            return None;
        };
        let count = number.values.len().saturating_sub(1) + 1;
        self.set_choices(target, count);
        Some(vec![links::PARAMETERS.to_owned(), count.to_string()])
    }

    /// The pin a wire dropped on the empty pin `slot` of a block is checked against: the
    /// last choice of a Switch block, as all choices are of one kind.
    pub(super) fn checked_slot(&self, target: u64, slot: &[String]) -> Vec<String> {
        match slot {
            [cell, key] if cell == links::PARAMETERS && key == OPEN => {
                vec![links::PARAMETERS.to_owned(), "1".to_owned()]
            }
            _ => {
                let _ = target;
                slot.to_vec()
            }
        }
    }

    /// Asks for a new name of the socket of the row of `path` of block `id`.
    pub(super) fn ask_socket_name(&mut self, id: u64, path: &[String]) {
        let pin = GroupPin {
            node: id,
            path: path.to_vec(),
        };
        let Some(side) = self.nodes.side_of(&pin) else {
            return;
        };
        let (group, input, socket) = side.socket();
        if let Some(s) = self.nodes.socket(group, input, socket) {
            self.rename_socket = Some((group, socket, s.name.clone()));
        }
    }

    /// The window to rename a socket of a group.
    pub(super) fn socket_name_window(&mut self, ctx: &egui::Context) {
        let Some((group, socket, mut name)) = self.rename_socket.take() else {
            return;
        };
        let mut open = true;
        let (mut done, mut cancelled) = (false, false);
        let valid = !name.trim().is_empty();
        egui::Window::new("Rename Socket")
            .id(crate::ids::window(ctx, "rename socket"))
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .pivot(egui::Align2::CENTER_CENTER)
            .default_pos(ctx.content_rect().center())
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Name:");
                    let edit = ui.text_edit_singleline(&mut name);
                    if !edit.has_focus() && !edit.lost_focus() {
                        edit.request_focus();
                    }
                    if edit.lost_focus() {
                        if ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                            done = valid;
                        } else if ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                            cancelled = true;
                        }
                    }
                });
                ui.horizontal(|ui| {
                    if ui.add_enabled(valid, egui::Button::new("OK")).clicked() {
                        done = true;
                    }
                    if ui.button("Cancel").clicked() {
                        cancelled = true;
                    }
                });
            });
        if cancelled {
            open = false;
        }
        if done {
            self.checkpoint(&[]);
            let name = name.trim().to_owned();
            for input in [true, false] {
                if let Some(s) = self.nodes.socket_mut(group, input, socket) {
                    s.name = name.clone();
                }
            }
        } else if open {
            self.rename_socket = Some((group, socket, name));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn path(parts: &[&str]) -> Vec<String> {
        parts.iter().map(|p| (*p).to_owned()).collect()
    }

    fn pin(node: u64, parts: &[&str]) -> GroupPin {
        GroupPin {
            node,
            path: path(parts),
        }
    }

    fn link(source: u64, item: &[&str], target: u64, slot: &[&str]) -> Link {
        Link {
            source,
            target,
            item: path(item),
            slot: path(slot),
            ..Default::default()
        }
    }

    fn math(id: u64) -> Node {
        Node::read(ProjectNode::Math(file::MathBlock {
            id,
            name: format!("Math {id}"),
            ..Default::default()
        }))
    }

    /// A parameter `1` linked to the Math block `2`, whose result goes to the Math block
    /// `3`.
    fn chain() -> (Vec<Node>, Vec<Link>) {
        let parameters = Node::read(ProjectNode::Parameters(file::ParameterSet {
            id: 1,
            name: "Parameters".into(),
            ..Default::default()
        }));
        let nodes = vec![parameters, math(2), math(3)];
        let links = vec![
            link(1, &["Value", "x"], 2, &["Parameters", "a"]),
            link(2, &["Value", "Result"], 3, &["Parameters", "a"]),
        ];
        (nodes, links)
    }

    /// Makes the group `id` of `chosen` at `level` and adds it with its ends to `nodes`.
    fn make_group(
        nodes: &mut Vec<Node>,
        links: &[Link],
        level: Option<u64>,
        chosen: &[u64],
        id: u64,
    ) {
        let group = nodes.regroup(
            links,
            level,
            chosen,
            [id, id + 1, id + 2],
            "Group".into(),
            [0.0; 2],
        );
        nodes.push(Node::new(id, "Group".into(), Block::Group(Box::new(group))));
        nodes.push(Node::new(id + 1, "Group Input".into(), Block::GroupInput));
        nodes.push(Node::new(id + 2, "Group Output".into(), Block::GroupOutput));
    }

    fn wires(
        nodes: &Vec<Node>,
        links: &[Link],
        level: Option<u64>,
    ) -> BTreeSet<(GroupPin, GroupPin)> {
        (nodes.level_wires(links, level).into_iter())
            .map(|w| (w.from, w.to))
            .collect()
    }

    #[test]
    fn a_group_takes_the_wires_through_its_sockets_and_keeps_the_links() {
        let (mut nodes, links) = chain();
        make_group(&mut nodes, &links, None, &[2], 10);
        let group = nodes.group(10).unwrap();
        assert_eq!(group.nodes, vec![2, 11, 12]);
        let [input] = &group.inputs[..] else {
            panic!("one input: {:?}", group.inputs);
        };
        assert_eq!(input.name, "x");
        assert_eq!(input.kind, path(&["Value", "x"]));
        assert_eq!(input.source, Some(pin(1, &["Value", "x"])));
        assert_eq!(input.targets, vec![pin(2, &["Parameters", "a"])]);
        let [output] = &group.outputs[..] else {
            panic!("one output: {:?}", group.outputs);
        };
        assert_eq!(output.source, Some(pin(2, &["Value", "Result"])));
        assert_eq!(output.targets, vec![pin(3, &["Parameters", "a"])]);
        assert_ne!(input.id, output.id);
        let (i, o) = (input.id, output.id);
        // Outside, the wires run to and from the group's pins.
        assert_eq!(
            wires(&nodes, &links, None),
            BTreeSet::from([
                (pin(1, &["Value", "x"]), socket_pin(10, i)),
                (socket_pin(10, o), pin(3, &["Parameters", "a"])),
            ])
        );
        // Inside, from the Group Input block and to the Group Output block.
        assert_eq!(
            wires(&nodes, &links, Some(10)),
            BTreeSet::from([
                (socket_pin(11, i), pin(2, &["Parameters", "a"])),
                (pin(2, &["Value", "Result"]), socket_pin(12, o)),
            ])
        );
        // The links the sockets make are the links there were.
        assert_eq!(
            nodes.routed(),
            BTreeSet::from([
                (pin(1, &["Value", "x"]), pin(2, &["Parameters", "a"])),
                (pin(2, &["Value", "Result"]), pin(3, &["Parameters", "a"])),
            ])
        );
    }

    #[test]
    fn a_group_in_a_group_passes_the_wires_on() {
        let (mut nodes, links) = chain();
        make_group(&mut nodes, &links, None, &[2], 10);
        // Inside the group, the Math block goes into a group of its own.
        make_group(&mut nodes, &links, Some(10), &[2], 20);
        if let Some(outer) = nodes.group_mut(10) {
            outer.nodes.retain(|n| *n != 2);
            outer.nodes.push(20);
        }
        let outer = nodes.group(10).unwrap().clone();
        let inner = nodes.group(20).unwrap().clone();
        let (i, o) = (outer.inputs[0].id, outer.outputs[0].id);
        let (ii, io) = (inner.inputs[0].id, inner.outputs[0].id);
        // The outer group's Group Input block gives to the inner group, which takes from it.
        assert_eq!(outer.inputs[0].targets, vec![socket_pin(20, ii)]);
        assert_eq!(inner.inputs[0].source, Some(socket_pin(11, i)));
        assert_eq!(outer.outputs[0].source, Some(socket_pin(20, io)));
        assert_eq!(inner.outputs[0].targets, vec![socket_pin(12, o)]);
        assert_eq!(
            wires(&nodes, &links, Some(10)),
            BTreeSet::from([
                (socket_pin(11, i), socket_pin(20, ii)),
                (socket_pin(20, io), socket_pin(12, o)),
            ])
        );
        assert_eq!(nodes.routed().len(), 2);
        // Taken apart, the inner group's block goes and its Math block is in the outer group
        // again, wired to the Group Input and Group Output blocks.
        let members = nodes.take_apart(20, Some(10)).unwrap();
        assert_eq!(members, vec![2]);
        nodes.retain(|n| ![20, 21, 22].contains(&n.id));
        nodes.tidy_sockets();
        assert_eq!(
            wires(&nodes, &links, Some(10)),
            BTreeSet::from([
                (socket_pin(11, i), pin(2, &["Parameters", "a"])),
                (pin(2, &["Value", "Result"]), socket_pin(12, o)),
            ])
        );
        assert_eq!(nodes.routed().len(), 2);
    }

    #[test]
    fn a_group_taken_apart_leaves_the_links_as_wires_between_its_blocks() {
        let (mut nodes, links) = chain();
        make_group(&mut nodes, &links, None, &[2], 10);
        assert_eq!(nodes.take_apart(10, None), Some(vec![2]));
        nodes.retain(|n| ![10, 11, 12].contains(&n.id));
        nodes.tidy_sockets();
        assert!(nodes.routed().is_empty());
        assert_eq!(
            wires(&nodes, &links, None),
            BTreeSet::from([
                (pin(1, &["Value", "x"]), pin(2, &["Parameters", "a"])),
                (pin(2, &["Value", "Result"]), pin(3, &["Parameters", "a"])),
            ])
        );
    }

    #[test]
    fn a_wire_of_a_socket_makes_the_link_once_its_way_is_complete() {
        let (mut nodes, links) = chain();
        make_group(&mut nodes, &links, None, &[2], 10);
        let i = nodes.group(10).unwrap().inputs[0].id;
        // Without the wire into the group, the input socket leads nowhere.
        nodes.drop_wire(&pin(1, &["Value", "x"]), &socket_pin(10, i));
        assert_eq!(nodes.routed().len(), 1);
        assert!(nodes.group(10).unwrap().inputs[0].source.is_none());
        // A wire from another block completes it again.
        nodes.store_wire(&pin(3, &["Value", "Result"]), &socket_pin(10, i));
        assert!(
            nodes
                .routed()
                .contains(&(pin(3, &["Value", "Result"]), pin(2, &["Parameters", "a"])))
        );
        // A second wire from the Group Input block goes to another block inside too.
        nodes.store_wire(&socket_pin(11, i), &pin(2, &["Parameters", "b"]));
        assert_eq!(nodes.group(10).unwrap().inputs[0].targets.len(), 2);
        assert!(
            nodes
                .routed()
                .contains(&(pin(3, &["Value", "Result"]), pin(2, &["Parameters", "b"])))
        );
    }

    #[test]
    fn a_socket_without_wires_goes() {
        let (mut nodes, links) = chain();
        make_group(&mut nodes, &links, None, &[2], 10);
        let i = nodes.group(10).unwrap().inputs[0].id;
        nodes.drop_wire(&pin(1, &["Value", "x"]), &socket_pin(10, i));
        nodes.tidy_sockets();
        assert_eq!(nodes.group(10).unwrap().inputs.len(), 1);
        nodes.drop_wire(&socket_pin(11, i), &pin(2, &["Parameters", "a"]));
        nodes.tidy_sockets();
        assert!(nodes.group(10).unwrap().inputs.is_empty());
        assert_eq!(nodes.group(10).unwrap().outputs.len(), 1);
    }

    #[test]
    fn pins_of_groups_are_told_apart() {
        let (mut nodes, links) = chain();
        make_group(&mut nodes, &links, None, &[2], 10);
        let group = nodes.group(10).unwrap().clone();
        let (i, o) = (group.inputs[0].id, group.outputs[0].id);
        assert_eq!(nodes.side_of(&socket_pin(10, i)), Some(Side::Into(10, i)));
        assert_eq!(nodes.side_of(&socket_pin(10, o)), Some(Side::OutOf(10, o)));
        assert_eq!(nodes.side_of(&socket_pin(11, i)), Some(Side::Gives(10, i)));
        assert_eq!(nodes.side_of(&socket_pin(12, o)), Some(Side::Takes(10, o)));
        assert_eq!(nodes.side_of(&pin(2, &["Parameters", "a"])), None);
        assert!(nodes.group_pin(11, &path(&[SOCKET, OPEN])));
        assert!(!nodes.group_pin(2, &path(&["Parameters", OPEN])));
        assert_eq!(nodes.parents().get(&2), Some(&10));
        assert_eq!(nodes.ends(10), (Some(11), Some(12)));
    }

    #[test]
    fn a_group_and_its_ends_are_kept_in_the_project_file() {
        let (mut nodes, links) = chain();
        make_group(&mut nodes, &links, None, &[2], 10);
        for node in &mut nodes {
            let written = node.written([1.0, 2.0]).unwrap();
            let read = Node::read(written.clone());
            assert_eq!(read.kind(), node.kind());
            if let ProjectNode::Group(group) = &written {
                assert_eq!(Some(group), nodes_group(&read).as_ref());
            }
        }
    }

    fn nodes_group(node: &Node) -> Option<NodeGroup> {
        match &node.block {
            Block::Group(g) => Some(NodeGroup {
                position: [1.0, 2.0],
                ..(**g).clone()
            }),
            _ => None,
        }
    }
}
