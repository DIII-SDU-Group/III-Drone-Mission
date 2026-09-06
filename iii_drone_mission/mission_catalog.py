"""Deterministic mission-catalog generation, reduction, and verification.

This module is intentionally ROS-free so CMake, release builders, and tests use
the same fail-closed implementation. Runtime code consumes only its generated
catalog contract and content-addressed assets from the package install space.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence

import yaml


CATALOG_SCHEMA = "iii.mission-catalog/v1"
ENTRY_SCHEMA = "iii.mission-catalog-entry/v1"
NODE_CONTRACT_SCHEMA = "iii.behavior-node-contract/v1"
FIELD_POLICY_SCHEMA = "iii.mission-field-policy/v1"
SOURCE_STATE_SCHEMA = "iii.mission-source-state/v1"
GROOT_PROJECT_SCHEMA = "iii.groot2-project/v1"
KNOWN_PROFILES = ("hil", "opti_track", "real", "sim")
COMMISSIONED_PROFILES = frozenset({"hil", "opti_track", "real", "sim"})
ONBOARD_PROFILES = frozenset({"hil", "opti_track", "real"})
CLASSIFICATIONS = frozenset({"production", "experimental", "test", "legacy"})
STATUSES = frozenset({"active", "deprecated"})
ID_PATTERN = re.compile(r"[a-z0-9](?:[a-z0-9.-]{0,126}[a-z0-9])?")
HASH_PATTERN = re.compile(r"sha256:[a-f0-9]{64}")
STANDARD_XML_ATTRIBUTES = frozenset(
    {
        "ID",
        "name",
        "_failureIf",
        "_successIf",
        "_skipIf",
        "_while",
        "_onFailure",
        "_onSuccess",
        "_post",
        "__shared_blackboard",
    }
)
GENERIC_NODE_TAGS = {
    "Action": "ACTION",
    "Condition": "CONDITION",
    "Control": "CONTROL",
    "Decorator": "DECORATOR",
}


class CatalogError(RuntimeError):
    """Raised when mission content cannot produce a trustworthy catalog."""


@dataclass(frozen=True)
class Registration:
    catalog_id: str
    specification: Path
    classification: str
    status: str
    profiles: tuple[str, ...]
    default_for: tuple[str, ...]


def canonical_json(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")


def sha256_bytes(value: bytes) -> str:
    return "sha256:" + hashlib.sha256(value).hexdigest()


def content_hash(value: Any) -> str:
    return sha256_bytes(canonical_json(value))


def _read_regular_file(path: Path, *, label: str, allow_symlink: bool = False) -> bytes:
    if (path.is_symlink() and not allow_symlink) or not path.is_file():
        raise CatalogError(f"{label} is missing, linked, or not a regular file: {path}")
    return path.read_bytes()


def _load_json(path: Path, *, label: str, allow_symlink: bool = False) -> Any:
    raw = _read_regular_file(path, label=label, allow_symlink=allow_symlink)
    try:
        return json.loads(raw)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise CatalogError(f"{label} is not valid JSON: {path}: {exc}") from exc


def _validate_relative_reference(value: str, *, label: str) -> PurePosixPath:
    if not value or "\\" in value or "$" in value or "~" in value:
        raise CatalogError(f"{label} must be a non-empty package-relative POSIX path: {value!r}")
    reference = PurePosixPath(value)
    if reference.is_absolute() or any(part in {"", ".", ".."} for part in reference.parts):
        raise CatalogError(f"{label} escapes or is not package-relative: {value!r}")
    return reference


def _resolve_source_asset(source_root: Path, reference: str, *, label: str) -> Path:
    relative = _validate_relative_reference(reference, label=label)
    candidate = source_root.joinpath(*relative.parts)
    try:
        resolved = candidate.resolve(strict=True)
        resolved.relative_to(source_root)
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        raise CatalogError(f"{label} is missing or escapes the package root: {reference}") from exc
    if candidate.is_symlink() or not resolved.is_file():
        raise CatalogError(f"{label} must resolve to a regular in-package file: {reference}")
    return resolved


def read_registrations(path: Path, source_root: Path) -> list[Registration]:
    raw = _read_regular_file(path, label="mission registration manifest")
    try:
        lines = raw.decode("utf-8").splitlines()
    except UnicodeDecodeError as exc:
        raise CatalogError("mission registration manifest is not UTF-8") from exc
    registrations: list[Registration] = []
    ids: set[str] = set()
    specs: set[Path] = set()
    for line_number, line in enumerate(lines, start=1):
        if not line:
            continue
        fields = line.split("\t")
        if len(fields) != 6:
            raise CatalogError(f"registration manifest line {line_number} must have six tab-separated fields")
        catalog_id, specification, classification, status, profiles_raw, defaults_raw = fields
        if not ID_PATTERN.fullmatch(catalog_id):
            raise CatalogError(f"invalid mission catalog ID: {catalog_id!r}")
        if catalog_id in ids:
            raise CatalogError(f"duplicate mission catalog ID: {catalog_id}")
        ids.add(catalog_id)
        if classification not in CLASSIFICATIONS:
            raise CatalogError(f"mission {catalog_id} has unknown classification: {classification!r}")
        if status not in STATUSES:
            raise CatalogError(f"mission {catalog_id} has unknown status: {status!r}")
        profiles = tuple(sorted(filter(None, profiles_raw.split(","))))
        defaults = tuple(sorted(filter(None, defaults_raw.split(","))))
        if not profiles:
            raise CatalogError(f"mission {catalog_id} has no profile allowlist")
        unknown = sorted(set(profiles) - set(KNOWN_PROFILES))
        if unknown:
            raise CatalogError(f"mission {catalog_id} has unknown profiles: {', '.join(unknown)}")
        if len(profiles) != len(set(profiles)) or len(defaults) != len(set(defaults)):
            raise CatalogError(f"mission {catalog_id} repeats a profile/default declaration")
        invalid_defaults = sorted(set(defaults) - set(profiles))
        if invalid_defaults:
            raise CatalogError(
                f"mission {catalog_id} is default for disallowed profiles: {', '.join(invalid_defaults)}"
            )
        spec_path = Path(specification)
        try:
            resolved_spec = spec_path.resolve(strict=True)
            resolved_spec.relative_to(source_root)
        except (FileNotFoundError, RuntimeError, ValueError) as exc:
            raise CatalogError(f"mission {catalog_id} specification is missing or outside the package") from exc
        if spec_path.is_symlink() or not resolved_spec.is_file():
            raise CatalogError(f"mission {catalog_id} specification must be a regular source file")
        if resolved_spec in specs:
            raise CatalogError(f"specification is registered more than once: {resolved_spec.relative_to(source_root)}")
        specs.add(resolved_spec)
        registrations.append(
            Registration(catalog_id, resolved_spec, classification, status, profiles, defaults)
        )
    if not registrations:
        raise CatalogError("mission registration manifest is empty")

    specification_root = source_root / "mission_specification"
    if specification_root.is_dir():
        unclassified = sorted(
            path.relative_to(source_root).as_posix()
            for path in specification_root.iterdir()
            if path.is_file() and not path.is_symlink() and path.resolve() not in specs
        )
        if unclassified:
            raise CatalogError("unclassified mission specifications: " + ", ".join(unclassified))
    return sorted(registrations, key=lambda item: item.catalog_id)


def _node_contract(path: Path) -> tuple[dict[str, Any], dict[str, dict[str, Any]], str]:
    contract = _load_json(path, label="behavior-node contract")
    if not isinstance(contract, dict) or contract.get("schema") != NODE_CONTRACT_SCHEMA:
        raise CatalogError(f"behavior-node contract must use {NODE_CONTRACT_SCHEMA}")
    declared_hash = contract.get("contract_hash")
    payload = {key: value for key, value in contract.items() if key != "contract_hash"}
    if declared_hash != content_hash(payload):
        raise CatalogError("behavior-node contract identity mismatch")
    nodes = contract.get("nodes")
    if not isinstance(nodes, list) or not nodes:
        raise CatalogError("behavior-node contract contains no nodes")
    by_id: dict[str, dict[str, Any]] = {}
    for node in nodes:
        if not isinstance(node, dict) or not isinstance(node.get("id"), str):
            raise CatalogError("behavior-node contract contains a malformed node")
        node_id = node["id"]
        if node_id in by_id:
            raise CatalogError(f"behavior-node contract repeats node ID: {node_id}")
        ports = node.get("ports")
        if not isinstance(ports, list) or any(not isinstance(port, dict) for port in ports):
            raise CatalogError(f"behavior node {node_id} has malformed ports")
        port_names = [port.get("name") for port in ports]
        if any(not isinstance(name, str) or not name for name in port_names) or len(port_names) != len(set(port_names)):
            raise CatalogError(f"behavior node {node_id} has invalid or duplicate port names")
        by_id[node_id] = node
    return contract, by_id, declared_hash


def _interface_contract(paths: Sequence[Path], source_root: Path) -> tuple[dict[str, str], str]:
    if not paths:
        raise CatalogError("at least one mission interface contract is required")
    hashes: dict[str, str] = {}
    for path in sorted((item.resolve() for item in paths), key=str):
        raw = _read_regular_file(path, label="mission interface contract", allow_symlink=True)
        logical = path.name
        if logical in hashes:
            logical = path.as_posix()
        hashes[logical] = sha256_bytes(raw)
    return hashes, content_hash(hashes)


def _runtime_contract(path: Path) -> tuple[dict[str, Any], str]:
    contract = _load_json(path, label="mission runtime contract")
    if not isinstance(contract, dict) or contract.get("schema") != "iii.mission-runtime-contract/v1":
        raise CatalogError("mission runtime contract schema is invalid")
    return contract, content_hash(contract)


def _yaml_mapping(path: Path, *, catalog_id: str) -> dict[str, Any]:
    raw = _read_regular_file(path, label=f"mission {catalog_id} specification")
    try:
        document = yaml.safe_load(raw)
    except yaml.YAMLError as exc:
        raise CatalogError(f"mission {catalog_id} specification is malformed YAML: {exc}") from exc
    if not isinstance(document, dict):
        raise CatalogError(f"mission {catalog_id} specification must be a YAML mapping")
    return document


def _validate_xml(
    path: Path,
    *,
    catalog_id: str,
    node_by_id: Mapping[str, Mapping[str, Any]],
) -> tuple[list[str], list[str]]:
    raw = _read_regular_file(path, label=f"mission {catalog_id} behavior tree")
    try:
        root = ET.fromstring(raw)
    except ET.ParseError as exc:
        raise CatalogError(f"mission {catalog_id} behavior tree is malformed XML: {path.name}: {exc}") from exc
    if root.tag != "root" or root.get("BTCPP_format") != "4":
        raise CatalogError(f"mission {catalog_id} behavior tree {path.name} must be a BTCPP format 4 root")
    trees = [item for item in root if item.tag == "BehaviorTree"]
    tree_ids = [item.get("ID", "") for item in trees]
    if not tree_ids or any(not value for value in tree_ids) or len(tree_ids) != len(set(tree_ids)):
        raise CatalogError(f"mission {catalog_id} behavior tree {path.name} has missing or duplicate tree IDs")
    main_tree = root.get("main_tree_to_execute")
    if main_tree not in tree_ids:
        raise CatalogError(f"mission {catalog_id} behavior tree {path.name} has an unavailable main tree")
    used_nodes: set[str] = set()
    for tree in trees:
        for element in tree.iter():
            if element is tree:
                continue
            if element.tag == "SubTree":
                subtree = element.get("ID", "")
                if subtree not in tree_ids:
                    raise CatalogError(
                        f"mission {catalog_id} behavior tree {path.name} references unavailable SubTree {subtree!r}"
                    )
                continue
            expected_type = GENERIC_NODE_TAGS.get(element.tag)
            node_id = element.get("ID") if expected_type else element.tag
            node = node_by_id.get(node_id or "")
            if node is None:
                raise CatalogError(
                    f"mission {catalog_id} behavior tree {path.name} uses unavailable node {node_id!r}"
                )
            if expected_type and node.get("type") != expected_type:
                raise CatalogError(
                    f"mission {catalog_id} behavior tree {path.name} uses {node_id} as {expected_type}, "
                    f"but the runtime registers {node.get('type')}"
                )
            ports = {port["name"] for port in node["ports"]}
            unknown_attributes = sorted(set(element.attrib) - STANDARD_XML_ATTRIBUTES - ports)
            if unknown_attributes:
                raise CatalogError(
                    f"mission {catalog_id} behavior tree {path.name} uses unavailable ports on {node_id}: "
                    + ", ".join(unknown_attributes)
                )
            used_nodes.add(node_id or "")
    return sorted(tree_ids), sorted(used_nodes)


def _validate_specification(
    registration: Registration,
    source_root: Path,
    node_by_id: Mapping[str, Mapping[str, Any]],
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    document = _yaml_mapping(registration.specification, catalog_id=registration.catalog_id)
    executor_owned_mode = document.get("executor_owned_mode")
    entries = document.get("entries")
    if not isinstance(executor_owned_mode, str) or not executor_owned_mode:
        raise CatalogError(f"mission {registration.catalog_id} has no executor_owned_mode")
    if not isinstance(entries, list) or not entries:
        raise CatalogError(f"mission {registration.catalog_id} has no mission entries")
    mode_keys: set[str] = set()
    resolved_entries: list[dict[str, Any]] = []
    tree_assets: dict[str, dict[str, Any]] = {}
    for index, value in enumerate(entries):
        if not isinstance(value, dict):
            raise CatalogError(f"mission {registration.catalog_id} entry {index} is not a mapping")
        key = value.get("key")
        name = value.get("mode_name")
        tree_reference = value.get("behavior_tree_xml_file")
        if not isinstance(key, str) or not key or key in mode_keys:
            raise CatalogError(f"mission {registration.catalog_id} has a missing or duplicate mode key: {key!r}")
        if not isinstance(name, str) or not name:
            raise CatalogError(f"mission {registration.catalog_id} mode {key} has no display name")
        if not isinstance(tree_reference, str):
            raise CatalogError(f"mission {registration.catalog_id} mode {key} has no behavior tree reference")
        mode_keys.add(key)
        tree_path = _resolve_source_asset(
            source_root,
            tree_reference,
            label=f"mission {registration.catalog_id} mode {key} behavior tree",
        )
        logical = tree_path.relative_to(source_root).as_posix()
        if logical not in tree_assets:
            tree_ids, nodes = _validate_xml(
                tree_path,
                catalog_id=registration.catalog_id,
                node_by_id=node_by_id,
            )
            raw = tree_path.read_bytes()
            tree_assets[logical] = {
                "kind": "behavior_tree",
                "logical_name": logical,
                "content_hash": sha256_bytes(raw),
                "asset_id": sha256_bytes(raw),
                "tree_ids": tree_ids,
                "behavior_nodes": nodes,
                "source_path": tree_path,
            }
        resolved = {
            "key": key,
            "mode_name": name,
            "behavior_tree_asset_id": tree_assets[logical]["asset_id"],
            "allow_activate_when_disarmed": bool(value.get("allow_activate_when_disarmed", False)),
        }
        if "next_mode" in value:
            if not isinstance(value["next_mode"], str) or not value["next_mode"]:
                raise CatalogError(f"mission {registration.catalog_id} mode {key} has an invalid next_mode")
            resolved["next_mode"] = value["next_mode"]
        resolved_entries.append(resolved)
    if executor_owned_mode not in mode_keys:
        raise CatalogError(f"mission {registration.catalog_id} executor_owned_mode is unavailable")
    for value in resolved_entries:
        if value.get("next_mode") not in mode_keys and "next_mode" in value:
            raise CatalogError(
                f"mission {registration.catalog_id} mode {value['key']} references unavailable next_mode "
                f"{value['next_mode']!r}"
            )
    intents = document.get("intent_services", [])
    if not isinstance(intents, list):
        raise CatalogError(f"mission {registration.catalog_id} intent_services must be a list")
    resolved_intents: list[dict[str, Any]] = []
    service_names: set[str] = set()
    for value in intents:
        if not isinstance(value, dict):
            raise CatalogError(f"mission {registration.catalog_id} has a malformed intent service")
        service_name = value.get("service_name")
        flag_name = value.get("flag_name")
        value_type = value.get("type")
        valid_modes = value.get("valid_modes")
        if (
            not isinstance(service_name, str)
            or not service_name.startswith("/")
            or service_name in service_names
            or not isinstance(flag_name, str)
            or not flag_name
            or value_type != "bool"
            or not isinstance(valid_modes, list)
            or not valid_modes
            or any(mode not in mode_keys for mode in valid_modes)
        ):
            raise CatalogError(f"mission {registration.catalog_id} has an invalid intent service declaration")
        service_names.add(service_name)
        resolved_intents.append(
            {
                "service_name": service_name,
                "flag_name": flag_name,
                "type": value_type,
                "valid_modes": sorted(set(valid_modes)),
            }
        )
    specification_raw = registration.specification.read_bytes()
    specification = {
        "kind": "mission_specification",
        "logical_name": registration.specification.relative_to(source_root).as_posix(),
        "content_hash": sha256_bytes(specification_raw),
        "asset_id": sha256_bytes(specification_raw),
        "source_path": registration.specification,
    }
    resolved_specification = {
        "executor_owned_mode": executor_owned_mode,
        "entries": resolved_entries,
        "intent_services": resolved_intents,
    }
    return resolved_specification, [specification, *sorted(tree_assets.values(), key=lambda item: item["logical_name"])]


def _copy_asset(asset: Mapping[str, Any], output: Path) -> None:
    digest = str(asset["asset_id"]).removeprefix("sha256:")
    destination = output / "assets" / "sha256" / digest
    destination.parent.mkdir(parents=True, exist_ok=True)
    source = Path(asset["source_path"])
    raw = _read_regular_file(source, label="catalog source asset")
    if sha256_bytes(raw) != asset["content_hash"]:
        raise CatalogError(f"catalog source asset changed during generation: {source}")
    if destination.exists() and destination.read_bytes() != raw:
        raise CatalogError(f"content-addressed asset collision: {asset['asset_id']}")
    destination.write_bytes(raw)


def _public_asset(asset: Mapping[str, Any]) -> dict[str, Any]:
    return {key: value for key, value in asset.items() if key != "source_path"}


def _source_state(
    *,
    source_root: Path,
    registrations_path: Path,
    node_contract_path: Path,
    interface_contract_paths: Sequence[Path],
    runtime_contract_path: Path,
    models_xml_path: Path | None,
    build_contract_paths: Sequence[Path],
    source_asset_paths: Mapping[str, Path],
) -> dict[str, Any]:
    source_files: dict[str, str] = {}

    def add_source(path: Path, *, label: str) -> None:
        try:
            resolved = path.resolve(strict=True)
            logical = resolved.relative_to(source_root).as_posix()
        except (FileNotFoundError, RuntimeError, ValueError) as exc:
            raise CatalogError(f"{label} is missing or outside the mission package: {path}") from exc
        raw = _read_regular_file(resolved, label=label)
        digest = sha256_bytes(raw)
        if logical in source_files and source_files[logical] != digest:
            raise CatalogError(f"source-state collision for {logical}")
        source_files[logical] = digest

    for path in source_asset_paths.values():
        add_source(path, label="mission source asset")
    for path in build_contract_paths:
        add_source(path, label="mission build contract")
    add_source(runtime_contract_path, label="mission runtime contract")

    interface_contracts: dict[str, str] = {}
    for path in interface_contract_paths:
        name = path.name
        if name in interface_contracts:
            raise CatalogError(f"duplicate interface contract filename in source state: {name}")
        interface_contracts[name] = sha256_bytes(
            _read_regular_file(path, label="mission interface contract", allow_symlink=True)
        )

    state: dict[str, Any] = {
        "schema": SOURCE_STATE_SCHEMA,
        "state_hash": "",
        "source_files": dict(sorted(source_files.items())),
        "interface_contracts": dict(sorted(interface_contracts.items())),
        "registration_manifest_sha256": sha256_bytes(
            _read_regular_file(registrations_path, label="mission registration manifest")
        ),
        "behavior_node_contract_sha256": sha256_bytes(
            _read_regular_file(node_contract_path, label="behavior-node contract")
        ),
        "models_xml_sha256": (
            sha256_bytes(_read_regular_file(models_xml_path, label="generated behavior-node model"))
            if models_xml_path is not None
            else None
        ),
    }
    state["state_hash"] = content_hash({key: value for key, value in state.items() if key != "state_hash"})
    return state


def _catalog_for_scope(
    *,
    entries: Sequence[dict[str, Any]],
    scope: str,
    compatibility: Mapping[str, str],
    field_policy: Mapping[str, Any],
) -> dict[str, Any]:
    if scope == "local":
        selected = [json.loads(json.dumps(entry)) for entry in entries]
    elif scope == "qualified":
        selected = [
            json.loads(json.dumps(entry))
            for entry in entries
            if entry["classification"] == "production"
        ]
    elif scope == "field-candidates":
        selected = [
            json.loads(json.dumps(entry))
            for entry in entries
            if entry["classification"] in {"production", "experimental"}
        ]
    else:
        raise CatalogError(f"unknown catalog scope: {scope}")
    if scope != "local":
        selected = [entry for entry in selected if set(entry["profiles"]) & ONBOARD_PROFILES]
        for entry in selected:
            entry["profiles"] = sorted(set(entry["profiles"]) & ONBOARD_PROFILES)
            entry["default_for"] = sorted(set(entry["default_for"]) & ONBOARD_PROFILES)
            entry["entry_hash"] = content_hash(
                {key: value for key, value in entry.items() if key != "entry_hash"}
            )
    assets_by_id: dict[str, dict[str, Any]] = {}
    for entry in selected:
        for asset in entry["assets"]:
            assets_by_id[asset["asset_id"]] = asset
    defaults: dict[str, str] = {}
    catalog_profiles = set(KNOWN_PROFILES) if scope == "local" else set(ONBOARD_PROFILES)
    for profile in sorted(COMMISSIONED_PROFILES & catalog_profiles):
        candidates = [entry["id"] for entry in selected if profile in entry["default_for"]]
        if len(candidates) != 1:
            raise CatalogError(
                f"catalog scope {scope} requires exactly one default for commissioned profile {profile}; "
                f"found {len(candidates)}"
            )
        defaults[profile] = candidates[0]
    profile_descriptors = {
        profile: {
            "commissioned": profile in COMMISSIONED_PROFILES,
            "onboard": profile in ONBOARD_PROFILES,
            "default_entry_id": defaults.get(profile),
        }
        for profile in KNOWN_PROFILES
        if profile in catalog_profiles
    }
    catalog: dict[str, Any] = {
        "schema": CATALOG_SCHEMA,
        "catalog_hash": "",
        "scope": scope,
        "compatibility": dict(sorted(compatibility.items())),
        "profiles": profile_descriptors,
        "entries": sorted(selected, key=lambda item: item["id"]),
        "assets": sorted(assets_by_id.values(), key=lambda item: item["asset_id"]),
    }
    if scope == "field-candidates":
        catalog["field_policy"] = field_policy
    catalog["catalog_hash"] = content_hash({key: value for key, value in catalog.items() if key != "catalog_hash"})
    return catalog


def _write_catalog(
    output: Path,
    catalog: Mapping[str, Any],
    assets: Mapping[str, Mapping[str, Any]],
    source_state: Mapping[str, Any],
) -> None:
    staging = Path(tempfile.mkdtemp(prefix=f".{output.name}.", dir=output.parent))
    try:
        for asset in catalog["assets"]:
            full = assets[asset["asset_id"]]
            _copy_asset(full, staging)
        catalog_bytes = canonical_json(catalog) + b"\n"
        (staging / "catalog.json").write_bytes(catalog_bytes)
        (staging / "catalog.sha256").write_text(hashlib.sha256(catalog_bytes).hexdigest() + "  catalog.json\n")
        (staging / "source-state.json").write_bytes(canonical_json(source_state) + b"\n")
        if output.exists():
            shutil.rmtree(output)
        os.replace(staging, output)
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def generate_catalogs(
    *,
    source_root: Path,
    registrations_path: Path,
    node_contract_path: Path,
    interface_contract_paths: Sequence[Path],
    runtime_contract_path: Path,
    models_xml_path: Path | None,
    build_contract_paths: Sequence[Path],
    local_output: Path,
    qualified_output: Path,
    field_candidates_output: Path,
) -> dict[str, str]:
    source_root = source_root.resolve(strict=True)
    registrations = read_registrations(registrations_path, source_root)
    _node_document, node_by_id, node_hash = _node_contract(node_contract_path)
    interface_files, interface_hash = _interface_contract(interface_contract_paths, source_root)
    _runtime_document, runtime_hash = _runtime_contract(runtime_contract_path)
    compatibility = {
        "behavior_nodes_sha256": node_hash,
        "interfaces_sha256": interface_hash,
        "mission_runtime_sha256": runtime_hash,
    }
    entries: list[dict[str, Any]] = []
    all_assets: dict[str, dict[str, Any]] = {}
    all_source_asset_paths: dict[str, Path] = {}
    for registration in registrations:
        resolved, assets = _validate_specification(registration, source_root, node_by_id)
        public_assets = [_public_asset(asset) for asset in assets]
        for asset in assets:
            all_source_asset_paths[asset["logical_name"]] = Path(asset["source_path"])
            existing = all_assets.get(asset["asset_id"])
            if existing and Path(existing["source_path"]).read_bytes() != Path(asset["source_path"]).read_bytes():
                raise CatalogError(f"content-addressed asset collision: {asset['asset_id']}")
            all_assets[asset["asset_id"]] = asset
        entry: dict[str, Any] = {
            "schema": ENTRY_SCHEMA,
            "id": registration.catalog_id,
            "entry_hash": "",
            "classification": registration.classification,
            "status": registration.status,
            "profiles": list(registration.profiles),
            "default_for": list(registration.default_for),
            "experimental_warning": (
                "EXPERIMENTAL mission: not qualified for production release or flight."
                if registration.classification == "experimental"
                else None
            ),
            "compatibility": dict(compatibility),
            "specification": resolved,
            "assets": public_assets,
            "dependencies": sorted(asset["asset_id"] for asset in public_assets),
        }
        entry["entry_hash"] = content_hash({key: value for key, value in entry.items() if key != "entry_hash"})
        entries.append(entry)

    source_state = _source_state(
        source_root=source_root,
        registrations_path=registrations_path,
        node_contract_path=node_contract_path,
        interface_contract_paths=interface_contract_paths,
        runtime_contract_path=runtime_contract_path,
        models_xml_path=models_xml_path,
        build_contract_paths=build_contract_paths,
        source_asset_paths=all_source_asset_paths,
    )
    compatibility["source_state_sha256"] = source_state["state_hash"]
    for entry in entries:
        entry["compatibility"] = dict(compatibility)
        entry["entry_hash"] = content_hash({key: value for key, value in entry.items() if key != "entry_hash"})

    for profile in COMMISSIONED_PROFILES:
        defaults = [entry["id"] for entry in entries if profile in entry["default_for"]]
        if len(defaults) != 1:
            raise CatalogError(
                f"local catalog requires exactly one default for commissioned profile {profile}; found {len(defaults)}"
            )
    field_policy = {
        "schema": FIELD_POLICY_SCHEMA,
        "qualified_classifications": ["production"],
        "field_classifications": ["production", "experimental"],
        "experimental_default_included": False,
        "experimental_requires_explicit_include": True,
        "experimental_warning_required": True,
        "onboard_profiles": sorted(ONBOARD_PROFILES),
        "excluded_classifications": ["legacy", "test"],
    }
    catalogs = {
        "local": _catalog_for_scope(
            entries=entries, scope="local", compatibility=compatibility, field_policy=field_policy
        ),
        "qualified": _catalog_for_scope(
            entries=entries, scope="qualified", compatibility=compatibility, field_policy=field_policy
        ),
        "field-candidates": _catalog_for_scope(
            entries=entries, scope="field-candidates", compatibility=compatibility, field_policy=field_policy
        ),
    }
    for output in (local_output, qualified_output, field_candidates_output):
        output.parent.mkdir(parents=True, exist_ok=True)
    _write_catalog(local_output, catalogs["local"], all_assets, source_state)
    _write_catalog(qualified_output, catalogs["qualified"], all_assets, source_state)
    _write_catalog(field_candidates_output, catalogs["field-candidates"], all_assets, source_state)
    if models_xml_path is not None:
        models_xml = _read_regular_file(models_xml_path, label="generated behavior-node model")
        for output in (local_output, qualified_output, field_candidates_output):
            (output / "models.xml").write_bytes(models_xml)
            project = {
                "schema": GROOT_PROJECT_SCHEMA,
                "catalog": "catalog.json",
                "node_model": "models.xml",
                "catalog_hash": json.loads((output / "catalog.json").read_bytes())["catalog_hash"],
            }
            (output / "groot2-project.json").write_bytes(canonical_json(project) + b"\n")
    result = {scope: catalog["catalog_hash"] for scope, catalog in catalogs.items()}
    result["interfaces_sha256"] = interface_hash
    result["interface_files"] = content_hash(interface_files)
    return result


def verify_catalog(directory: Path, *, expected_scope: str | None = None) -> dict[str, Any]:
    directory = directory.resolve(strict=True)
    catalog_path = directory / "catalog.json"
    raw = _read_regular_file(catalog_path, label="mission catalog", allow_symlink=True)
    catalog = _load_json(catalog_path, label="mission catalog", allow_symlink=True)
    if raw != canonical_json(catalog) + b"\n":
        raise CatalogError("mission catalog is not canonical JSON")
    if not isinstance(catalog, dict) or catalog.get("schema") != CATALOG_SCHEMA:
        raise CatalogError(f"mission catalog must use {CATALOG_SCHEMA}")
    if expected_scope is not None and catalog.get("scope") != expected_scope:
        raise CatalogError(f"mission catalog scope is {catalog.get('scope')!r}, expected {expected_scope!r}")
    expected_catalog_hash = content_hash({key: value for key, value in catalog.items() if key != "catalog_hash"})
    if catalog.get("catalog_hash") != expected_catalog_hash:
        raise CatalogError("mission catalog content identity mismatch")
    source_state_path = directory / "source-state.json"
    source_state_raw = _read_regular_file(
        source_state_path, label="mission source state", allow_symlink=True
    )
    source_state = _load_json(source_state_path, label="mission source state", allow_symlink=True)
    if source_state_raw != canonical_json(source_state) + b"\n":
        raise CatalogError("mission source state is not canonical JSON")
    if not isinstance(source_state, dict) or source_state.get("schema") != SOURCE_STATE_SCHEMA:
        raise CatalogError(f"mission source state must use {SOURCE_STATE_SCHEMA}")
    expected_state_hash = content_hash(
        {key: value for key, value in source_state.items() if key != "state_hash"}
    )
    if source_state.get("state_hash") != expected_state_hash:
        raise CatalogError("mission source-state identity mismatch")
    if catalog.get("compatibility", {}).get("source_state_sha256") != expected_state_hash:
        raise CatalogError("mission catalog is not bound to its source-state identity")
    digest_line = _read_regular_file(
        directory / "catalog.sha256", label="mission catalog checksum", allow_symlink=True
    ).decode("ascii")
    expected_file_digest = hashlib.sha256(raw).hexdigest() + "  catalog.json\n"
    if digest_line != expected_file_digest:
        raise CatalogError("mission catalog byte checksum mismatch")
    assets = catalog.get("assets")
    entries = catalog.get("entries")
    if not isinstance(assets, list) or not isinstance(entries, list):
        raise CatalogError("mission catalog assets/entries are malformed")
    asset_ids: set[str] = set()
    for asset in assets:
        if not isinstance(asset, dict) or not HASH_PATTERN.fullmatch(str(asset.get("asset_id", ""))):
            raise CatalogError("mission catalog contains a malformed asset")
        asset_id = asset["asset_id"]
        if asset_id in asset_ids or asset.get("content_hash") != asset_id:
            raise CatalogError(f"mission catalog contains duplicate or inconsistent asset {asset_id}")
        asset_ids.add(asset_id)
        path = directory / "assets" / "sha256" / asset_id.removeprefix("sha256:")
        raw_asset = _read_regular_file(path, label=f"mission asset {asset_id}", allow_symlink=True)
        if sha256_bytes(raw_asset) != asset_id:
            raise CatalogError(f"mission asset content hash mismatch: {asset_id}")
    entry_ids: set[str] = set()
    for entry in entries:
        if not isinstance(entry, dict) or not ID_PATTERN.fullmatch(str(entry.get("id", ""))):
            raise CatalogError("mission catalog contains a malformed entry")
        if entry["id"] in entry_ids:
            raise CatalogError(f"mission catalog repeats entry {entry['id']}")
        entry_ids.add(entry["id"])
        expected_entry_hash = content_hash({key: value for key, value in entry.items() if key != "entry_hash"})
        if entry.get("entry_hash") != expected_entry_hash:
            raise CatalogError(f"mission catalog entry identity mismatch: {entry['id']}")
        dependencies = entry.get("dependencies")
        if not isinstance(dependencies, list) or any(item not in asset_ids for item in dependencies):
            raise CatalogError(f"mission catalog entry has unavailable dependencies: {entry['id']}")
        if catalog["scope"] == "qualified" and entry.get("classification") != "production":
            raise CatalogError("qualified mission catalog contains non-production content")
        if catalog["scope"] != "local" and not set(entry.get("profiles", ())) & ONBOARD_PROFILES:
            raise CatalogError(f"drone mission catalog contains a non-onboard entry: {entry['id']}")
    disk_assets = {
        "sha256:" + path.name
        for path in (directory / "assets" / "sha256").iterdir()
        if path.is_file()
    } if (directory / "assets" / "sha256").is_dir() else set()
    if disk_assets != asset_ids:
        raise CatalogError("mission catalog asset directory has missing or extra content")
    models_hash = source_state.get("models_xml_sha256")
    if models_hash is not None:
        models_raw = _read_regular_file(
            directory / "models.xml", label="mission behavior-node model", allow_symlink=True
        )
        if sha256_bytes(models_raw) != models_hash:
            raise CatalogError("mission behavior-node model differs from its source-state identity")
        project_path = directory / "groot2-project.json"
        project_raw = _read_regular_file(
            project_path, label="mission Groot project", allow_symlink=True
        )
        project = _load_json(project_path, label="mission Groot project", allow_symlink=True)
        if project_raw != canonical_json(project) + b"\n":
            raise CatalogError("mission Groot project is not canonical JSON")
        expected_project = {
            "schema": GROOT_PROJECT_SCHEMA,
            "catalog": "catalog.json",
            "node_model": "models.xml",
            "catalog_hash": catalog["catalog_hash"],
        }
        if project != expected_project:
            raise CatalogError("mission Groot project is not bound to the catalog and node model")
    return catalog


def source_drift(
    *,
    source_root: Path,
    catalog_directory: Path,
    interface_directory: Path | None = None,
) -> list[str]:
    """Return deterministic source/install drift reasons after catalog verification."""

    verify_catalog(catalog_directory)
    state = _load_json(
        catalog_directory / "source-state.json", label="mission source state", allow_symlink=True
    )
    reasons: list[str] = []
    expected_files = state.get("source_files", {})
    if not isinstance(expected_files, dict):
        raise CatalogError("mission source state has malformed source_files")
    for logical, expected_hash in sorted(expected_files.items()):
        try:
            relative = _validate_relative_reference(logical, label="source-state logical name")
            path = source_root.joinpath(*relative.parts)
            raw = _read_regular_file(path, label=f"mission source {logical}")
        except (CatalogError, OSError):
            reasons.append(f"missing source file: {logical}")
            continue
        if sha256_bytes(raw) != expected_hash:
            reasons.append(f"changed source file: {logical}")

    specification_root = source_root / "mission_specification"
    current_specifications = {
        path.relative_to(source_root).as_posix()
        for path in specification_root.iterdir()
        if path.is_file() and not path.is_symlink()
    } if specification_root.is_dir() else set()
    expected_specifications = {
        logical for logical in expected_files if logical.startswith("mission_specification/")
    }
    for logical in sorted(current_specifications - expected_specifications):
        reasons.append(f"new or unclassified mission specification: {logical}")

    expected_interfaces = state.get("interface_contracts", {})
    if interface_directory is not None:
        for name, expected_hash in sorted(expected_interfaces.items()):
            candidates = sorted(interface_directory.glob(f"**/{name}"))
            if len(candidates) != 1:
                reasons.append(f"installed interface contract unavailable or ambiguous: {name}")
            elif sha256_bytes(
                _read_regular_file(candidates[0], label=f"interface contract {name}", allow_symlink=True)
            ) != expected_hash:
                reasons.append(f"changed installed interface contract: {name}")
    return reasons


def materialize_field_catalog(
    *, candidates_directory: Path,
    output: Path,
    include_experimental: Sequence[str],
) -> dict[str, Any]:
    source = verify_catalog(candidates_directory, expected_scope="field-candidates")
    requested = set(include_experimental)
    experimental = {
        entry["id"]: entry for entry in source["entries"] if entry["classification"] == "experimental"
    }
    unknown = sorted(requested - set(experimental))
    if unknown:
        raise CatalogError("unknown or non-experimental field mission IDs: " + ", ".join(unknown))
    entries = [
        entry
        for entry in source["entries"]
        if entry["classification"] == "production" or entry["id"] in requested
    ]
    assets_by_id = {
        asset["asset_id"]: asset
        for entry in entries
        for asset in entry["assets"]
    }
    catalog = {
        **source,
        "catalog_hash": "",
        "scope": "field",
        "entries": entries,
        "assets": sorted(assets_by_id.values(), key=lambda item: item["asset_id"]),
        "field_selection": {
            "included_experimental": sorted(requested),
            "warning": (
                "EXPERIMENTAL missions are included in this field-development catalog and are not qualified."
                if requested
                else None
            ),
        },
    }
    catalog["catalog_hash"] = content_hash({key: value for key, value in catalog.items() if key != "catalog_hash"})
    full_assets = {
        asset_id: {
            **asset,
            "source_path": candidates_directory / "assets" / "sha256" / asset_id.removeprefix("sha256:"),
        }
        for asset_id, asset in assets_by_id.items()
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    source_state = _load_json(
        candidates_directory / "source-state.json", label="mission source state", allow_symlink=True
    )
    _write_catalog(output, catalog, full_assets, source_state)
    models_hash = source_state.get("models_xml_sha256")
    if models_hash is not None:
        models_raw = _read_regular_file(
            candidates_directory / "models.xml",
            label="field-candidate behavior-node model",
            allow_symlink=True,
        )
        if sha256_bytes(models_raw) != models_hash:
            raise CatalogError("field-candidate behavior-node model differs from source state")
        (output / "models.xml").write_bytes(models_raw)
        project = {
            "schema": GROOT_PROJECT_SCHEMA,
            "catalog": "catalog.json",
            "node_model": "models.xml",
            "catalog_hash": catalog["catalog_hash"],
        }
        (output / "groot2-project.json").write_bytes(canonical_json(project) + b"\n")
    return verify_catalog(output, expected_scope="field")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    generate = subparsers.add_parser("generate")
    generate.add_argument("--source-root", type=Path, required=True)
    generate.add_argument("--registrations", type=Path, required=True)
    generate.add_argument("--node-contract", type=Path, required=True)
    generate.add_argument("--interface-contract", type=Path, action="append", default=[])
    generate.add_argument("--runtime-contract", type=Path, required=True)
    generate.add_argument("--models-xml", type=Path)
    generate.add_argument("--build-contract", type=Path, action="append", default=[])
    generate.add_argument("--local-output", type=Path, required=True)
    generate.add_argument("--qualified-output", type=Path, required=True)
    generate.add_argument("--field-candidates-output", type=Path, required=True)
    verify = subparsers.add_parser("verify")
    verify.add_argument("directory", type=Path)
    verify.add_argument("--scope")
    field = subparsers.add_parser("materialize-field")
    field.add_argument("--candidates", type=Path, required=True)
    field.add_argument("--output", type=Path, required=True)
    field.add_argument("--include-experimental", action="append", default=[])
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "generate":
            result = generate_catalogs(
                source_root=args.source_root,
                registrations_path=args.registrations,
                node_contract_path=args.node_contract,
                interface_contract_paths=args.interface_contract,
                runtime_contract_path=args.runtime_contract,
                models_xml_path=args.models_xml,
                build_contract_paths=args.build_contract,
                local_output=args.local_output,
                qualified_output=args.qualified_output,
                field_candidates_output=args.field_candidates_output,
            )
        elif args.command == "verify":
            catalog = verify_catalog(args.directory, expected_scope=args.scope)
            result = {"scope": catalog["scope"], "catalog_hash": catalog["catalog_hash"]}
        else:
            catalog = materialize_field_catalog(
                candidates_directory=args.candidates,
                output=args.output,
                include_experimental=args.include_experimental,
            )
            result = {"scope": catalog["scope"], "catalog_hash": catalog["catalog_hash"]}
    except (CatalogError, OSError) as exc:
        print(f"mission catalog error: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
