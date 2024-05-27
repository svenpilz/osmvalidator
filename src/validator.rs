use nalgebra::geometry::Rotation2;
use nalgebra::Vector2;
use osmpbfreader::objects::{OsmId, OsmObj};
use osmpbfreader::{NodeId, WayId};
use petgraph::graphmap::DiGraphMap;
use std::collections::BTreeMap;
use std::collections::HashSet;
use std::fmt;
use std::str::FromStr;
use serde::{Deserialize, Serialize};

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum Direction {
    Left,
    Right,
}

#[derive(Debug, PartialEq, Eq)]
pub struct Turn {
    pub direction: Option<Direction>,
    pub through: Option<bool>,
}

impl Turn {
    pub fn from_angle(s: f64, base: &Option<f64>) -> Turn {
        // Heuristic angle for left/right classification.
        let min_angle_for_turn = 0.2;

        if s > base.unwrap_or(min_angle_for_turn) + f64::EPSILON {
            return Turn {
                direction: Some(Direction::Left),
                through: Some(false),
            };
        } else if s < base.unwrap_or(-min_angle_for_turn) - f64::EPSILON {
            return Turn {
                direction: Some(Direction::Right),
                through: Some(false),
            };
        }

        return Turn {
            direction: None,
            through: Some(true),
        };
    }
}

impl fmt::Display for Turn {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.through == Some(true) {
            write!(f, "through")?;
        } else if self.through.is_none() {
            return write!(f, "none");
        }

        if let Some(d) = self.direction {
            if self.through == Some(true) {
                write!(f, ";")?;
            }

            return match d {
                Direction::Left => write!(f, "left"),
                Direction::Right => write!(f, "right"),
            };
        }

        Ok(())
    }
}

impl FromStr for Turn {
    type Err = osmpbfreader::error::Error;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let mut result = Turn {
            direction: None,
            through: None,
        };

        for value in s.split(";") {
            if value == "through" {
                result.through = Some(true);
            } else if value == "left" || value == "slight_left" || value == "sharp_left" {
                result.direction = Some(Direction::Left);
            } else if value == "right" || value == "slight_right" || value == "sharp_right" {
                result.direction = Some(Direction::Right);
            } else if value.is_empty()
                || value == "none"
                || value == "merge_to_left"
                || value == "merge_to_right"
                || value == "reverse"
            {
                // skip
            } else {
                return Err(osmpbfreader::error::Error::InvalidData);
            }
        }

        return Ok(result);
    }
}

pub struct Turns {
    pub forward: Vec<Turn>,
    pub backward: Vec<Turn>,
}

#[derive(Debug)]
pub struct OutgoingTurn {
    pub angle: f64,
    pub classification: Turn,
    pub turn_lanes: Result<Vec<Turn>, osmpbfreader::error::Error>,
    pub way_id: WayId,
}

pub struct Segment {
    pub start: NodeId,
    pub end: NodeId,
}

pub type WayGraph = DiGraphMap<osmpbfreader::objects::NodeId, osmpbfreader::objects::WayId>;
pub type Objects = BTreeMap<OsmId, OsmObj>;

fn to_point(objects: &Objects, nodeid: NodeId) -> Result<Vector2<f64>, osmpbfreader::error::Error> {
    objects
        .get(&OsmId::Node(nodeid))
        .map(|obj| match &obj {
            OsmObj::Node(node) => Some(node),
            _ => None,
        })
        .flatten()
        .map(|node| Vector2::new(node.lon(), node.lat()))
        .ok_or(osmpbfreader::error::Error::InvalidData)
}

fn tangent(
    objects: &Objects,
    point: NodeId,
    origin: NodeId,
) -> Result<Vector2<f64>, osmpbfreader::error::Error> {
    Ok(to_point(objects, point)? - to_point(objects, origin)?)
}

pub fn is_oneway(objects: &Objects, way_id: WayId) -> bool {
    objects
        .get(&OsmId::Way(way_id))
        .map(|way| way.tags().contains("oneway", "yes"))
        == Some(true)
}

pub fn oriented_connected_ways(
    graph: &WayGraph,
    objects: &Objects,
    edge: &Segment,
    node: NodeId,
) -> Vec<(osmpbfreader::WayId, f64, petgraph::Direction, bool)> {
    let lookup_direction = if node == edge.end {
        petgraph::Direction::Outgoing
    } else {
        petgraph::Direction::Incoming
    };

    let current_tangent = tangent(
        objects,
        node,
        match lookup_direction {
            petgraph::Direction::Outgoing => edge.start,
            petgraph::Direction::Incoming => edge.end,
        },
    )
    .expect("Failed to compute tangent.");

    graph
        .edges_directed(node, petgraph::Direction::Outgoing)
        .map(|edge| (edge.0, edge.1, edge.2, petgraph::Direction::Outgoing, true))
        .chain(
            graph
                .edges_directed(node, petgraph::Direction::Incoming)
                .map(|edge| {
                    (
                        edge.0,
                        edge.1,
                        edge.2,
                        petgraph::Direction::Incoming,
                        !is_oneway(objects, *edge.2),
                    )
                }),
        )
        .filter(|(start_node, end_node, _, _, _)| {
            !(start_node == &edge.start && end_node == &edge.end)
        })
        .map(|(start, end, id, direction, accessible)| {
            // Assume coordinates are euclidean on local scale.
            let angle = Rotation2::rotation_between(
                &current_tangent,
                &tangent(
                    objects,
                    match direction {
                        petgraph::Direction::Outgoing => end,
                        petgraph::Direction::Incoming => start,
                    },
                    match direction {
                        petgraph::Direction::Outgoing => start,
                        petgraph::Direction::Incoming => end,
                    },
                )
                .expect("Tangent needed to determine left/right."),
            )
            .angle();

            (*id, angle, direction, accessible)
        })
        .collect()
}

pub fn turn_lanes_annotation(
    objects: &Objects,
    way_id: WayId,
) -> Result<Turns, osmpbfreader::error::Error> {
    if let Some(OsmObj::Way(way)) = objects.get(&OsmId::Way(way_id)) {
        let parse =
            |lanes: &str| -> Result<Vec<_>, _> { lanes.split("|").map(Turn::from_str).collect() };

        if way.tags.contains("oneway", "yes") {
            if way.tags.get("turn:lanes:forward").is_some()
                || way.tags.get("turn:lanes:backward").is_some()
            {
                return Err(osmpbfreader::error::Error::InvalidData);
            }

            if let Some(turn_lanes_tag) = way.tags.get("turn:lanes") {
                return Ok(Turns {
                    forward: parse(turn_lanes_tag.as_str())?,
                    backward: Vec::new(),
                });
            }
        } else {
            if way.tags.get("turn:lanes").is_some() {
                return Err(osmpbfreader::error::Error::InvalidData);
            }

            return Ok(Turns {
                forward: match way.tags.get("turn:lanes:forward") {
                    Some(tag) => parse(tag.as_str())?,
                    _ => Vec::new(),
                },
                backward: match way.tags.get("turn:lanes:backward") {
                    Some(tag) => parse(tag.as_str())?,
                    _ => Vec::new(),
                },
            });
        }

        return Ok(Turns {
            forward: Vec::new(),
            backward: Vec::new(),
        });
    }

    Err(osmpbfreader::error::Error::InvalidData)
}

#[derive(Serialize, Deserialize)]
pub struct Issue {
    pub id: OsmId,
    pub message: String,
    pub details: String,
}

pub fn analyse_way_direction(
    graph: &WayGraph,
    objects: &Objects,
    turn_lanes: Vec<Turn>,
    edge: &Segment,
    node: NodeId,
    current_way_id: WayId,
) -> Vec<Issue> {
    let mut connected_ways = oriented_connected_ways(graph, objects, edge, node);

    let mut result = Vec::new();

    if connected_ways.is_empty()
        || connected_ways.len() == 1
            && connected_ways
                .first()
                .filter(|(way, _, _, _)| *way == current_way_id)
                .is_some()
    {
        return result;
    }

    connected_ways.sort_by(|a, b| a.1.abs().partial_cmp(&b.1.abs()).unwrap());
    let through_angle: Option<f64> = match connected_ways.len() {
        2 => {
            // If there is a connection with small angle, assume it to be through.
            connected_ways
                .first()
                .map(|(_, angle, _, _)| *angle)
                .filter(|angle| angle.abs() < 0.1)
        }
        3 => connected_ways
            .first()
            .map(|(_, angle, _, _)| *angle)
            .filter(|angle| angle.abs() < 0.5),
        _ => None,
    };

    if connected_ways.len() > 3 {
        println!(
            "- Skipping complex junction from way {} ({}).",
            current_way_id.0,
            connected_ways
                .iter()
                .map(|(way_id, _, _, _)| way_id.0.to_string())
                .collect::<Vec<_>>()
                .join(", ")
        );

        return result;
    }

    let accessible_ways: Vec<_> = connected_ways
        .iter()
        .filter(|(_, _, _, accesible)| *accesible)
        .map(|(way_id, angle, direction, _)| OutgoingTurn {
            angle: *angle,
            classification: Turn::from_angle(*angle, &through_angle),
            turn_lanes: turn_lanes_annotation(objects, *way_id).map(|turn| match direction {
                petgraph::Direction::Outgoing => turn.forward,
                petgraph::Direction::Incoming => turn.backward,
            }),
            way_id: *way_id,
        })
        .collect();

    let junction_details = accessible_ways
        .iter()
        .map(|turn| {
            format!(
                "(way={}, angle={}, classification={})",
                turn.way_id.0, turn.angle, turn.classification
            )
        })
        .collect::<Vec<_>>()
        .join(", ");

    let through = accessible_ways
        .iter()
        .find(|turn| turn.classification.through == Some(true));

    let through_has_direction = |direction| {
        if let Some(way) = &through {
            if let Ok(turns) = &way.turn_lanes {
                return turns.iter().any(|turn| turn.direction == Some(direction));
            }
        }
        false
    };

    let is_needed = |direction| {
        turn_lanes
            .iter()
            .any(|turn| turn.direction == Some(direction))
    };

    let direction_available = |direction| {
        accessible_ways
            .iter()
            .any(|turn| turn.classification.direction == Some(direction))
    };

    if is_needed(Direction::Left) && !direction_available(Direction::Left) {
        if !through_has_direction(Direction::Left) {
            result.push(Issue {
                id: OsmId::Way(current_way_id),
                message: "Left turn cannot flow out.".to_string(),
                details: junction_details.clone(),
            });
        }
    }

    if is_needed(Direction::Right) && !direction_available(Direction::Right) {
        if !through_has_direction(Direction::Right) {
            result.push(Issue {
                id: OsmId::Way(current_way_id),
                message: "Right turn cannot flow out.".to_string(),
                details: junction_details.clone(),
            });
        }
    }

    return result;
}

pub fn validate_turn_lanes<Reader: std::io::Read + std::io::Seek>(
    reader: &mut osmpbfreader::OsmPbfReader<Reader>,
) -> Result<Vec<Issue>, osmpbfreader::error::Error> {
    let mut turn_lane_endpoints = HashSet::new();

    for obj in reader.iter() {
        if let Ok(OsmObj::Way(way)) = &obj {
            if ["turn:lanes", "turn:lanes:forward", "turn:lanes:backward"]
                .iter()
                .any(|tag| way.tags.contains_key(*tag))
            {
                if let Some(first) = way.nodes.first() {
                    if let Some(last) = way.nodes.last() {
                        turn_lane_endpoints.insert(first.clone());
                        turn_lane_endpoints.insert(last.clone());
                    }
                }
            }
        }
    }

    let highway_types = HashSet::from([
        "motorway",
        "trunk",
        "primary",
        "secondary",
        "tertiary",
        "unclassified ",
        "unclassified",
        "residential",
        "motorway_link",
        "trunk_link",
        "primary_link",
        "secondary_link",
        "tertiary_link",
    ]);

    let turn_adjacent_objs = reader.get_objs_and_deps(|obj| {
        if let OsmObj::Way(way) = &obj {
            return way
                .tags
                .get("highway")
                .filter(|value| highway_types.contains(value.as_str()))
                .is_some()
                && way.nodes.iter().any(|id| turn_lane_endpoints.contains(id));
        }
        false
    })?;

    let mut graph = WayGraph::new();

    for (_, obj) in &turn_adjacent_objs {
        if let OsmObj::Way(way) = &obj {
            // Add all segments to ease tangent calculation.
            for nodes in way.nodes.windows(2) {
                for node in nodes {
                    graph.add_node(*node);
                }

                if let Some(start) = nodes.first() {
                    if let Some(end) = nodes.last() {
                        graph.add_edge(*start, *end, way.id);
                    }
                }
            }
        }
    }

    let mut issues = Vec::new();

    for (start, end, &way_id) in graph.all_edges() {
        if let Ok(current_turn_lanes) = turn_lanes_annotation(&turn_adjacent_objs, way_id) {
            let way = Segment { start, end };

            if !current_turn_lanes.forward.is_empty() {
                issues.extend(analyse_way_direction(
                    &graph,
                    &turn_adjacent_objs,
                    current_turn_lanes.forward,
                    &way,
                    end,
                    way_id,
                ));
            }

            if !current_turn_lanes.backward.is_empty() {
                issues.extend(analyse_way_direction(
                    &graph,
                    &turn_adjacent_objs,
                    current_turn_lanes.backward,
                    &way,
                    start,
                    way_id,
                ));
            }
        } else {
            issues.push(Issue {
                id: OsmId::Way(way_id),
                message: "Cannot parse turn:lanes".to_string(),
                details: "None".to_string(),
            });
        }
    }

    Ok(issues)
}
