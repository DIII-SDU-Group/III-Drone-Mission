from pathlib import Path
import re
import xml.etree.ElementTree as ET


TREES = Path(__file__).parents[1] / "behavior_trees"
PACKAGE = TREES.parent


def _tree_actions(path: str, tree_id: str):
    root = ET.parse(TREES / path).getroot()
    tree = next(node for node in root.findall("BehaviorTree") if node.attrib["ID"] == tree_id)
    return list(tree.iter())


def test_reach_tree_uses_explicit_boundary_and_nonrepeating_clearance_path():
    nodes = _tree_actions("reach_cable_tree.xml", "FlyToUnderCables")
    provider = next(node for node in nodes if node.attrib.get("ID") == "PowerlineWaypointProvider")
    assert provider.attrib["depart_outside_boundary_index"] == "{outside_boundary_index}"
    partition = next(node for node in nodes if node.attrib.get("ID") == "PartitionPointQueue")
    assert partition.attrib["boundary_index"] == "{outside_boundary_index}"
    follow = next(node for node in nodes if node.attrib.get("ID") == "FollowWaypointPath")
    assert follow.attrib["repeat"] == "false"
    assert follow.attrib["waypoints"] == "{route_through_boundary}"
    assert any(node.tag == "IfThenElse" for node in nodes)


def test_initial_powerline_detection_retry_fits_retained_hover_reference():
    nodes = _tree_actions("reach_cable_tree.xml", "LandOnCableCoreTree")
    detection = next(
        node for node in nodes
        if node.attrib.get("name") == "detect_powerline_retry_until_successful"
    )
    delay = next(
        node for node in detection.iter()
        if node.attrib.get("name") == "delay_while_waiting_for_powerline"
    )
    hover = next(
        node for node in nodes
        if node.attrib.get("name") == "hover_while_waiting_for_powerline"
    )

    attempts = int(detection.attrib["num_attempts"])
    interval_sec = int(delay.attrib["delay_msec"]) / 1000.0
    retained_reference_sec = int(hover.attrib["stop_maneuver_after_timeout_ms"]) / 1000.0

    assert attempts >= 20
    assert attempts * interval_sec < retained_reference_sec


def test_leave_tree_stops_at_first_outside_point_then_follows_saved_return_suffix():
    nodes = _tree_actions("leave_cable_tree.xml", "FlyToInitPosition")
    partition = next(node for node in nodes if node.attrib.get("ID") == "PartitionPointQueue")
    assert partition.attrib["boundary_index"] == "{@return_outside_boundary_index}"
    actions = [node for node in nodes if node.attrib.get("ID") == "FlyToPosition"]
    boundary_move = next(node for node in actions if node.attrib.get("name") == "fly_to_first_outside_return_waypoint")
    assert boundary_move.attrib.get("blend_to_next", "false") == "false"
    follow = next(node for node in nodes if node.attrib.get("ID") == "FollowWaypointPath")
    assert follow.attrib["repeat"] == "false"
    assert follow.attrib["waypoints"] == "{return_outside_suffix}"
    assert partition.attrib["boundary"] == "{return_outside_boundary}"
    assert partition.attrib["after"] == "{return_outside_suffix}"
    assert not any(node.attrib.get("name") == "fly_to_final_initial_position" for node in nodes)
    direct = next(node for node in nodes if node.attrib.get("name") == "fly_to_direct_return_waypoint")
    assert direct.attrib.get("blend_to_next", "false") == "false"
    final_direct = next(node for node in nodes if node.attrib.get("name") == "fly_to_direct_return_initial_position")
    assert final_direct.attrib["completion_position_tolerance_m"] == "0.2"


def test_powerline_provider_parameter_reads_are_declared_in_its_runtime_scope():
    provider = (
        PACKAGE / "src/behavior/action_nodes/powerline_waypoint_provider_action_node.cpp"
    ).read_text()
    provider = re.sub(r"//[^\n]*", "", provider)
    reads = set(re.findall(r'GetParameter\(\s*"([^"]+)"', provider))
    optional_reads = set(re.findall(r'HasParameter\(\s*"([^"]+)"', provider))
    required_reads = reads - optional_reads

    tree_provider = (PACKAGE / "src/behavior/trees/tree_provider.cpp").read_text()
    scope = tree_provider.split(
        'CreateConfiguration("powerline_waypoint_provider_action_node", {', 1
    )[1].split("});", 1)[0]
    entries = set(re.findall(r'ConfigurationEntry\("([^"]+)"', scope))
    declarations = set(re.findall(r'DeclareParameter\("([^"]+)"', tree_provider))

    assert required_reads <= entries, (
        f"provider reads absent from scoped configuration: {required_reads - entries}"
    )
    assert required_reads <= declarations, (
        f"provider reads absent from node declarations: {required_reads - declarations}"
    )


def test_reach_inside_approach_waypoints_stop_individually():
    nodes = _tree_actions("reach_cable_tree.xml", "FlyToUnderCables")
    actions = [node for node in nodes if node.attrib.get("ID") == "FlyToPosition"]
    assert actions
    assert all(node.attrib.get("blend_to_next", "false") == "false" for node in actions)
