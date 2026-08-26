from __future__ import annotations

import hashlib
import json
import shutil
import subprocess
from pathlib import Path

import pytest

from iii_drone_mission.mission_catalog import (
    CatalogError,
    canonical_json,
    content_hash,
    generate_catalogs,
    materialize_field_catalog,
    source_drift,
    verify_catalog,
)


def _node_contract(path: Path) -> Path:
    payload = {
        "nodes": [
            {"builtin": True, "id": "Sequence", "ports": [], "type": "CONTROL"},
            {
                "builtin": False,
                "id": "TestAction",
                "ports": [
                    {
                        "default": "",
                        "description": "target",
                        "direction": "INPUT",
                        "name": "target",
                        "type": "std::string",
                    }
                ],
                "type": "ACTION",
            },
        ],
        "schema": "iii.behavior-node-contract/v1",
    }
    payload["contract_hash"] = content_hash(payload)
    path.write_bytes(canonical_json(payload) + b"\n")
    return path


def _tree(path: Path, *, node: str = "TestAction", attributes: str = 'target="tower"') -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        f'<root BTCPP_format="4" main_tree_to_execute="Main">'
        f'<BehaviorTree ID="Main"><Sequence><Action ID="{node}" {attributes}/></Sequence></BehaviorTree>'
        f'</root>\n'
    )


def _spec(path: Path, tree: str = "behavior_trees/main.xml") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "executor_owned_mode: inspect\n"
        "entries:\n"
        "  - key: inspect\n"
        "    mode_name: Inspect\n"
        f"    behavior_tree_xml_file: {tree}\n"
        "    allow_activate_when_disarmed: false\n"
    )


def _fixture(tmp_path: Path) -> dict[str, Path]:
    source = tmp_path / "source"
    source.mkdir(parents=True)
    _tree(source / "behavior_trees/main.xml")
    _spec(source / "mission_specification/production.yaml")
    _spec(source / "mission_specification/experimental.yaml")
    _spec(source / "mission_specification/test.yaml")
    registrations = tmp_path / "registrations.tsv"
    registrations.write_text(
        "inspection-production\t"
        f"{source / 'mission_specification/production.yaml'}\tproduction\tactive\t"
        "hil,opti_track,real,sim\topti_track,real,sim\n"
        "inspection-experimental\t"
        f"{source / 'mission_specification/experimental.yaml'}\texperimental\tactive\t"
        "hil,opti_track,real,sim\t\n"
        "inspection-test\t"
        f"{source / 'mission_specification/test.yaml'}\ttest\tactive\tsim\t\n"
    )
    interface = tmp_path / "MissionModeStatus.msg"
    interface.write_text("string active_catalog_id\n")
    runtime = source / "config/runtime.json"
    runtime.parent.mkdir(parents=True)
    runtime.write_text(
        json.dumps({"schema": "iii.mission-runtime-contract/v1", "selection": "catalog-id"}) + "\n"
    )
    models = tmp_path / "models.xml"
    models.write_text("<root><TreeNodesModel/></root>\n")
    return {
        "source": source,
        "registrations": registrations,
        "node": _node_contract(tmp_path / "nodes.json"),
        "interface": interface,
        "runtime": runtime,
        "models": models,
        "local": tmp_path / "out/local",
        "qualified": tmp_path / "out/qualified",
        "field": tmp_path / "out/field-candidates",
    }


def _generate(paths: dict[str, Path]) -> dict[str, str]:
    return generate_catalogs(
        source_root=paths["source"],
        registrations_path=paths["registrations"],
        node_contract_path=paths["node"],
        interface_contract_paths=[paths["interface"]],
        runtime_contract_path=paths["runtime"],
        models_xml_path=paths["models"],
        build_contract_paths=[paths["runtime"]],
        local_output=paths["local"],
        qualified_output=paths["qualified"],
        field_candidates_output=paths["field"],
    )


def _configure_registration_project(tmp_path: Path, declarations: str):
    source = tmp_path / "cmake-source"
    source.mkdir()
    (source / "mission.yaml").write_text("executor_owned_mode: inspect\nentries: []\n")
    module = Path(__file__).resolve().parents[1] / "cmake/IIIRegisterMission.cmake"
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.16)\n"
        "project(mission_registration_contract NONE)\n"
        f'include("{module}")\n'
        f"{declarations}\n"
        'iii_write_mission_registrations("${CMAKE_BINARY_DIR}/registrations.tsv")\n'
    )
    build = tmp_path / "cmake-build"
    process = subprocess.run(
        ["cmake", "-S", str(source), "-B", str(build)],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    return process, build


def test_cmake_registration_contract_is_stable_and_sorted(tmp_path: Path):
    process, build = _configure_registration_project(
        tmp_path,
        "iii_register_mission(ID z-mission SPECIFICATION mission.yaml CLASSIFICATION test PROFILES sim)\n"
        "iii_register_mission(ID a-mission SPECIFICATION CMakeLists.txt CLASSIFICATION production "
        "PROFILES sim real DEFAULT_FOR sim real)",
    )
    assert process.returncode == 0, process.stdout
    lines = (build / "registrations.tsv").read_text().splitlines()
    assert lines[0].startswith("a-mission\t")
    assert lines[0].endswith("\tproduction\tactive\treal,sim\treal,sim")
    assert lines[1].startswith("z-mission\t")


@pytest.mark.parametrize(
    ("declarations", "message"),
    [
        (
            "iii_register_mission(ID duplicate SPECIFICATION mission.yaml CLASSIFICATION production PROFILES sim)\n"
            "iii_register_mission(ID duplicate SPECIFICATION CMakeLists.txt CLASSIFICATION test PROFILES sim)",
            "duplicate iii_register_mission ID",
        ),
        (
            "iii_register_mission(ID missing SPECIFICATION absent.yaml "
            "CLASSIFICATION production PROFILES sim)",
            "specification is missing",
        ),
        (
            "iii_register_mission(ID missing-class SPECIFICATION mission.yaml PROFILES sim)",
            "requires CLASSIFICATION",
        ),
        (
            "iii_register_mission(ID bad-profile SPECIFICATION mission.yaml "
            "CLASSIFICATION production PROFILES moon)",
            "unknown profile moon",
        ),
        (
            "iii_register_mission(ID bad-default SPECIFICATION mission.yaml CLASSIFICATION production PROFILES real DEFAULT_FOR sim)",
            "default for disallowed profile sim",
        ),
    ],
)
def test_cmake_registration_contract_fails_closed(tmp_path: Path, declarations: str, message: str):
    process, _build = _configure_registration_project(tmp_path, declarations)
    assert process.returncode != 0
    assert message in process.stdout


def test_valid_closure_is_content_addressed_deterministic_and_source_independent(tmp_path: Path):
    paths = _fixture(tmp_path)
    first = _generate(paths)
    first_bytes = (paths["local"] / "catalog.json").read_bytes()
    second = _generate(paths)
    assert second == first
    assert (paths["local"] / "catalog.json").read_bytes() == first_bytes

    catalog = verify_catalog(paths["local"], expected_scope="local")
    dependencies = catalog["entries"][0]["dependencies"]
    assert len(dependencies) == 2
    shutil.rmtree(paths["source"])
    assert verify_catalog(paths["local"])["catalog_hash"] == catalog["catalog_hash"]


def test_source_state_detects_asset_contract_interface_and_unclassified_drift(tmp_path: Path):
    paths = _fixture(tmp_path)
    _generate(paths)
    interface_root = tmp_path / "interfaces/msg"
    interface_root.mkdir(parents=True)
    shutil.copy2(paths["interface"], interface_root / paths["interface"].name)
    assert source_drift(
        source_root=paths["source"],
        catalog_directory=paths["local"],
        interface_directory=tmp_path / "interfaces",
    ) == []

    _tree(paths["source"] / "behavior_trees/main.xml", attributes='target="changed"')
    _spec(paths["source"] / "mission_specification/new.yaml")
    (interface_root / paths["interface"].name).write_text("string changed_contract\n")
    reasons = source_drift(
        source_root=paths["source"],
        catalog_directory=paths["local"],
        interface_directory=tmp_path / "interfaces",
    )
    assert "changed source file: behavior_trees/main.xml" in reasons
    assert "new or unclassified mission specification: mission_specification/new.yaml" in reasons
    assert "changed installed interface contract: MissionModeStatus.msg" in reasons


@pytest.mark.parametrize("reference", ["/tmp/tree.xml", "../tree.xml", "$TREE/tree.xml", "missing.xml"])
def test_absolute_escaping_expanded_and_missing_references_fail(tmp_path: Path, reference: str):
    paths = _fixture(tmp_path)
    _spec(paths["source"] / "mission_specification/production.yaml", reference)
    with pytest.raises(CatalogError, match="package-relative|missing|escapes"):
        _generate(paths)


def test_malformed_yaml_and_xml_fail(tmp_path: Path):
    paths = _fixture(tmp_path)
    (paths["source"] / "mission_specification/production.yaml").write_text("entries: [\n")
    with pytest.raises(CatalogError, match="malformed YAML"):
        _generate(paths)
    paths = _fixture(tmp_path / "xml")
    (paths["source"] / "behavior_trees/main.xml").write_text("<root>")
    with pytest.raises(CatalogError, match="malformed XML"):
        _generate(paths)


def test_duplicate_and_unclassified_specifications_fail(tmp_path: Path):
    paths = _fixture(tmp_path)
    paths["registrations"].write_text(
        paths["registrations"].read_text()
        + paths["registrations"].read_text().splitlines()[0]
        + "\n"
    )
    with pytest.raises(CatalogError, match="duplicate mission catalog ID"):
        _generate(paths)
    paths = _fixture(tmp_path / "unclassified")
    _spec(paths["source"] / "mission_specification/forgotten-test.yaml")
    with pytest.raises(CatalogError, match="unclassified mission specifications"):
        _generate(paths)


@pytest.mark.parametrize(
    ("node", "attributes", "message"),
    [
        ("Unavailable", 'target="tower"', "unavailable node"),
        ("TestAction", 'unknown_port="tower"', "unavailable ports"),
    ],
)
def test_unavailable_node_and_port_fail(tmp_path: Path, node: str, attributes: str, message: str):
    paths = _fixture(tmp_path)
    _tree(paths["source"] / "behavior_trees/main.xml", node=node, attributes=attributes)
    with pytest.raises(CatalogError, match=message):
        _generate(paths)


def test_node_and_runtime_compatibility_mismatch_fail(tmp_path: Path):
    paths = _fixture(tmp_path)
    document = json.loads(paths["node"].read_text())
    document["nodes"][1]["ports"][0]["name"] = "changed"
    paths["node"].write_bytes(canonical_json(document) + b"\n")
    with pytest.raises(CatalogError, match="identity mismatch"):
        _generate(paths)
    paths = _fixture(tmp_path / "runtime")
    paths["runtime"].write_text(json.dumps({"schema": "wrong"}))
    with pytest.raises(CatalogError, match="runtime contract schema"):
        _generate(paths)


def test_qualified_and_field_catalog_classification_is_fail_closed(tmp_path: Path):
    paths = _fixture(tmp_path)
    _generate(paths)
    qualified = verify_catalog(paths["qualified"], expected_scope="qualified")
    assert [(entry["id"], entry["classification"]) for entry in qualified["entries"]] == [
        ("inspection-production", "production")
    ]
    assert set(qualified["profiles"]) == {"hil", "opti_track", "real"}
    assert qualified["entries"][0]["profiles"] == ["hil", "opti_track", "real"]

    selected = tmp_path / "selected-field"
    field = materialize_field_catalog(
        candidates_directory=paths["field"],
        output=selected,
        include_experimental=["inspection-experimental"],
    )
    assert field["scope"] == "field"
    assert field["field_selection"]["included_experimental"] == ["inspection-experimental"]
    assert "EXPERIMENTAL" in field["field_selection"]["warning"]
    with pytest.raises(CatalogError, match="unknown or non-experimental"):
        materialize_field_catalog(
            candidates_directory=paths["field"],
            output=tmp_path / "invalid-field",
            include_experimental=["inspection-production"],
        )


def test_catalog_and_asset_tamper_are_detected(tmp_path: Path):
    paths = _fixture(tmp_path)
    _generate(paths)
    catalog_path = paths["local"] / "catalog.json"
    original = catalog_path.read_bytes()
    catalog_path.write_bytes(original + b" ")
    with pytest.raises(CatalogError, match="canonical JSON|byte checksum"):
        verify_catalog(paths["local"])
    catalog_path.write_bytes(original)
    catalog = json.loads(original)
    asset = paths["local"] / "assets/sha256" / catalog["assets"][0]["asset_id"].removeprefix("sha256:")
    asset.write_bytes(asset.read_bytes() + b"tamper")
    with pytest.raises(CatalogError, match="asset content hash mismatch"):
        verify_catalog(paths["local"])


def test_derived_model_and_groot_project_tamper_are_detected(tmp_path: Path):
    paths = _fixture(tmp_path)
    _generate(paths)
    models = paths["local"] / "models.xml"
    models.write_text("<root>tampered</root>\n")
    with pytest.raises(CatalogError, match="behavior-node model differs"):
        verify_catalog(paths["local"])

    _generate(paths)
    project_path = paths["local"] / "groot2-project.json"
    project = json.loads(project_path.read_text())
    project["catalog_hash"] = "sha256:" + "0" * 64
    project_path.write_bytes(canonical_json(project) + b"\n")
    with pytest.raises(CatalogError, match="not bound"):
        verify_catalog(paths["local"])


def test_catalog_identity_changes_for_asset_only_edit_without_interface_change(tmp_path: Path):
    paths = _fixture(tmp_path)
    first = _generate(paths)
    interface_digest = hashlib.sha256(paths["interface"].read_bytes()).hexdigest()
    _tree(paths["source"] / "behavior_trees/main.xml", attributes='target="different"')
    second = _generate(paths)
    assert second["local"] != first["local"]
    assert hashlib.sha256(paths["interface"].read_bytes()).hexdigest() == interface_digest


def test_invalid_profile_defaults_fail(tmp_path: Path):
    paths = _fixture(tmp_path)
    content = paths["registrations"].read_text().replace(
        "hil,opti_track,real,sim\topti_track,real,sim",
        "hil,opti_track,real,sim\treal,sim",
    )
    paths["registrations"].write_text(content)
    with pytest.raises(CatalogError, match="exactly one default.*opti_track"):
        _generate(paths)
