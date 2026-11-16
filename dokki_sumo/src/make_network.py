from __future__ import annotations

import ast
import math
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import geopandas as gpd
import pandas as pd
from pyproj import Transformer
from shapely.geometry import LineString


# RouteX / Dokki SUMO network builder
# Source nodes and edge attributes: Model A Excel
# Source edge geometry: cleaned Model A GeoPackage
# CRS: EPSG:4326 -> EPSG:32636 (UTM Zone 36N)

SUMO_DIR = Path(__file__).resolve().parents[1]
DATA_DIR = SUMO_DIR / "data" / "model_a"
NETWORK_DIR = SUMO_DIR / "network"

EXCEL_FILE = DATA_DIR / "SOURCE_RouteX_Road_Data_Calculated.xlsx"
GEOMETRY_FILE = DATA_DIR / "dokki_graph_cleaned.gpkg"

NODES_OUT = NETWORK_DIR / "dokki.nod.xml"
EDGES_OUT = NETWORK_DIR / "dokki.edg.xml"
TYPES_OUT = NETWORK_DIR / "dokki.typ.xml"
NET_OUT = NETWORK_DIR / "dokki.net.xml"
EDGE_MAP_OUT = NETWORK_DIR / "edge_mapping.csv"
NODE_MAP_OUT = NETWORK_DIR / "node_mapping.csv"
LOG_OUT = NETWORK_DIR / "netconvert.log"

SOURCE_CRS = "EPSG:4326"
TARGET_CRS = "EPSG:32636"

# The source lanes tag is copied as-is to the SUMO directed edge.
# This preserves the recorded source value but does NOT assert that the
# original OSM lanes tag is direction-specific. Validate this assumption
# before calibrated traffic-volume/capacity studies.

# SUMO edge-type defaults are deliberately neutral. Every edge overrides
# numLanes and speed with the reviewed Model A values. Priority is kept at 1
# because no reviewed right-of-way priority field exists in Model A.
NEUTRAL_PRIORITY = 1
DEFAULT_LANES = 1
DEFAULT_SPEED_MPS = 8.333333


class BuildError(RuntimeError):
    pass


def is_missing(value) -> bool:
    if value is None:
        return True
    try:
        return bool(pd.isna(value))
    except (TypeError, ValueError):
        return False


def parse_sequence_or_text(value) -> list[str]:
    """Parse OSM list-like values safely; otherwise return one text value."""
    if is_missing(value):
        return []
    if isinstance(value, (list, tuple, set)):
        return [str(item).strip() for item in value if not is_missing(item)]

    text = str(value).strip()
    if not text:
        return []
    if text.startswith("[") and text.endswith("]"):
        try:
            parsed = ast.literal_eval(text)
            if isinstance(parsed, (list, tuple, set)):
                return [str(item).strip() for item in parsed if not is_missing(item)]
        except (ValueError, SyntaxError):
            pass
    return [text]


def numeric_id(value, field: str) -> int:
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise BuildError(f"Invalid {field} value: {value!r}")
    if not math.isfinite(number) or not number.is_integer():
        raise BuildError(f"{field} must be an integer ID; got {value!r}")
    return int(number)


def positive_integer(value, field: str, edge_label: str) -> int:
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise BuildError(f"Edge {edge_label}: invalid {field}={value!r}")
    if not math.isfinite(number) or not number.is_integer() or number < 1:
        raise BuildError(
            f"Edge {edge_label}: {field} must be a positive integer; got {value!r}")
    return int(number)


def positive_float(value, field: str, edge_label: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise BuildError(f"Edge {edge_label}: invalid {field}={value!r}")
    if not math.isfinite(number) or number <= 0:
        raise BuildError(
            f"Edge {edge_label}: {field} must be > 0; got {value!r}")
    return number


def parse_bool(value, field: str, edge_label: str) -> bool:
    if isinstance(value, bool):
        return value
    if is_missing(value):
        raise BuildError(f"Edge {edge_label}: missing {field}")
    text = str(value).strip().lower()
    if text in {"true", "1", "yes", "y"}:
        return True
    if text in {"false", "0", "no", "n"}:
        return False
    raise BuildError(
        f"Edge {edge_label}: cannot interpret {field}={value!r} as boolean")


def normalize_highway(value) -> str:
    classes = parse_sequence_or_text(value)
    if not classes:
        return "unclassified"
    # The first class is retained when classes have equal ranking.
    rank = {
        "motorway": 6,
        "motorway_link": 6,
        "trunk": 5,
        "trunk_link": 5,
        "primary": 4,
        "primary_link": 4,
        "secondary": 3,
        "secondary_link": 3,
        "tertiary": 2,
        "tertiary_link": 2,
        "residential": 1,
        "unclassified": 1,
        "living_street": 1,
        "service": 1,
    }
    return max(classes, key=lambda item: rank.get(item, 0)).replace(" ", "_") or "unclassified"


def safe_type_id(highway_class: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9.-]+", "_", highway_class).strip("_.-")
    return "h_" + (cleaned or "unclassified")


def xml_write(root: ET.Element, output_path: Path) -> None:
    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ")
    tree.write(output_path, encoding="utf-8", xml_declaration=True)


def main() -> int:
    NETWORK_DIR.mkdir(parents=True, exist_ok=True)

    for required in (EXCEL_FILE, GEOMETRY_FILE):
        if not required.exists():
            raise BuildError(f"Required input file not found: {required}")

    print("[1/7] Reading Model A source data...")
    model_nodes = pd.read_excel(EXCEL_FILE, sheet_name="Nodes")
    model_edges = pd.read_excel(EXCEL_FILE, sheet_name="Edges")

    required_node_columns = {"osmid", "x", "y"}
    required_edge_columns = {
        "u", "v", "key", "name", "highway", "length_m",
        "lanes_osm", "speed_kph_review", "oneway",
    }
    if missing := required_node_columns - set(model_nodes.columns):
        raise BuildError(f"Nodes sheet missing columns: {sorted(missing)}")
    if missing := required_edge_columns - set(model_edges.columns):
        raise BuildError(f"Edges sheet missing columns: {sorted(missing)}")

    model_nodes = model_nodes.copy()
    model_edges = model_edges.copy()

    model_nodes["osmid"] = model_nodes["osmid"].map(
        lambda v: numeric_id(v, "node osmid"))
    for column in ("u", "v", "key"):
        model_edges[column] = model_edges[column].map(
            lambda v: numeric_id(v, f"edge {column}"))

    if model_nodes["osmid"].duplicated().any():
        raise BuildError("Duplicate node IDs found in Model A.")
    if model_edges.duplicated(subset=["u", "v", "key"]).any():
        raise BuildError("Duplicate (u, v, key) rows found in Model A Edges.")

    if model_nodes[["x", "y"]].isna().any().any():
        raise BuildError("Model A contains missing node coordinates.")
    if not model_nodes["x"].between(-180, 180).all() or not model_nodes["y"].between(-90, 90).all():
        raise BuildError("Model A contains invalid longitude/latitude values.")

    node_ids = set(model_nodes["osmid"].tolist())
    referenced_nodes = set(model_edges["u"]).union(set(model_edges["v"]))
    missing_nodes = referenced_nodes - node_ids
    if missing_nodes:
        raise BuildError(
            f"Edges reference missing Model A nodes: {sorted(missing_nodes)[:20]}")

    print("[2/7] Reading and matching cleaned edge geometries...")
    geometry_edges = gpd.read_file(GEOMETRY_FILE, layer="edges")
    if geometry_edges.crs is None:
        raise BuildError("Cleaned GPKG edge layer has no CRS metadata.")
    geometry_edges = geometry_edges.to_crs(SOURCE_CRS)
    for column in ("u", "v", "key"):
        geometry_edges[column] = geometry_edges[column].map(
            lambda v: numeric_id(v, f"geometry {column}"))

    if geometry_edges.duplicated(subset=["u", "v", "key"]).any():
        raise BuildError("Duplicate (u, v, key) rows found in cleaned GPKG.")

    geometry_columns = ["u", "v", "key", "geometry"]
    if "junction" in geometry_edges.columns:
        geometry_columns.append("junction")

    merged = model_edges.merge(
        geometry_edges[geometry_columns],
        on=["u", "v", "key"],
        how="left",
        validate="one_to_one",
        indicator=True,
    )
    unmatched = merged[merged["_merge"] != "both"]
    if not unmatched.empty:
        sample = unmatched[["u", "v", "key"]].head(10).to_dict("records")
        raise BuildError(
            f"Model A edges without matching geometry: {len(unmatched)}; sample={sample}")
    if len(merged) != len(model_edges):
        raise BuildError(
            "Unexpected row-count change while joining geometries.")

    print("[3/7] Projecting nodes and edge shapes to EPSG:32636...")
    transformer = Transformer.from_crs(SOURCE_CRS, TARGET_CRS, always_xy=True)
    projected_nodes: dict[int, tuple[float, float]] = {}
    node_mapping_rows = []

    for row in model_nodes.itertuples(index=False):
        node_id = int(row.osmid)
        lon = float(row.x)
        lat = float(row.y)
        x_m, y_m = transformer.transform(lon, lat)
        if not (math.isfinite(x_m) and math.isfinite(y_m)):
            raise BuildError(f"Projection failed for node {node_id}.")
        projected_nodes[node_id] = (x_m, y_m)
        node_mapping_rows.append({
            "node_id": node_id,
            "osm_ids_original": getattr(row, "osmid_original", ""),
            "latitude": lat,
            "longitude": lon,
            "x_utm36_m": x_m,
            "y_utm36_m": y_m,
        })

    edge_rows_for_xml = []
    edge_mapping_rows = []
    geometry_diffs = []
    endpoint_snaps = []
    type_classes = set()

    for row in merged.itertuples(index=False):
        u, v, key = int(row.u), int(row.v), int(row.key)
        source_edge_key = f"{u}-{v}-{key}"
        # No underscores: SUMO reserves '_' in lane IDs.
        edge_id = f"e{u}to{v}k{key}"

        if u == v:
            raise BuildError(f"Self-loop edge found: {source_edge_key}")

        lanes = positive_integer(row.lanes_osm, "lanes_osm", source_edge_key)
        speed_kph = positive_float(
            row.speed_kph_review, "speed_kph_review", source_edge_key)
        length_m = positive_float(row.length_m, "length_m", source_edge_key)
        speed_mps = speed_kph / 3.6
        is_oneway = parse_bool(row.oneway, "oneway", source_edge_key)

        geom = row.geometry
        if geom is None or geom.is_empty or not isinstance(geom, LineString):
            raise BuildError(
                f"Edge {source_edge_key}: geometry must be a non-empty LineString.")
        if len(geom.coords) < 2:
            raise BuildError(
                f"Edge {source_edge_key}: geometry has fewer than two coordinates.")

        projected_shape = [transformer.transform(
            float(lon), float(lat)) for lon, lat, *rest in geom.coords]
        start_node_xy = projected_nodes[u]
        end_node_xy = projected_nodes[v]
        start_snap = math.dist(projected_shape[0], start_node_xy)
        end_snap = math.dist(projected_shape[-1], end_node_xy)
        endpoint_snaps.extend([start_snap, end_snap])

        # SUMO requires the edge shape to start/end at its from/to nodes.
        # Replace only the first and last shape points with exact Model A node positions.
        if start_snap > 2.0 or end_snap > 2.0:
            raise BuildError(
                f"Edge {source_edge_key}: geometry endpoint mismatch exceeds 2 m "
                f"(start={start_snap:.3f} m, end={end_snap:.3f} m)."
            )
        projected_shape[0] = start_node_xy
        projected_shape[-1] = end_node_xy

        shape_length_m = LineString(projected_shape).length
        length_diff = abs(shape_length_m - length_m)
        geometry_diffs.append(length_diff)
        if length_diff > 2.0:
            raise BuildError(
                f"Edge {source_edge_key}: geometry length differs from Model A length by "
                f"{length_diff:.3f} m (>2 m)."
            )

        shape_text = " ".join(f"{x:.3f},{y:.3f}" for x, y in projected_shape)
        highway_class = normalize_highway(row.highway)
        type_id = safe_type_id(highway_class)
        type_classes.add((type_id, highway_class))
        name_values = parse_sequence_or_text(row.name)
        edge_name = " / ".join(name_values) if name_values else ""

        edge_attrs = {
            "id": edge_id,
            "from": str(u),
            "to": str(v),
            "type": type_id,
            "numLanes": str(lanes),
            "speed": f"{speed_mps:.6f}",
            "length": f"{length_m:.6f}",
            "shape": shape_text,
            # For a one-way road, center the lanes on the supplied road geometry.
            # For two-way arcs, SUMO's default right-side spread separates opposite directions.
            "spreadType": "center" if is_oneway else "right",
        }
        if edge_name:
            edge_attrs["name"] = edge_name
        edge_rows_for_xml.append(edge_attrs)

        junction_tag = "" if not hasattr(row, "junction") or is_missing(
            row.junction) else str(row.junction)
        edge_mapping_rows.append({
            "edge_id": edge_id,
            "source_edge_key": source_edge_key,
            "u": u,
            "v": v,
            "key": key,
            "name": edge_name,
            "highway_raw": str(row.highway),
            "highway_class_used": highway_class,
            "junction_tag_from_cleaned_gpkg": junction_tag,
            "lanes_osm": lanes,
            "sumo_numLanes": lanes,
            "speed_kph_review": speed_kph,
            "sumo_speed_mps": speed_mps,
            "length_m_model_a": length_m,
            "geometry_length_m_projected": shape_length_m,
            "geometry_length_abs_difference_m": length_diff,
            "oneway_osm_tag": is_oneway,
            "geometry_start_snap_m": start_snap,
            "geometry_end_snap_m": end_snap,
            "spreadType": "center" if is_oneway else "right",
        })

    if len({row["id"] for row in edge_rows_for_xml}) != len(edge_rows_for_xml):
        raise BuildError("Generated SUMO edge IDs are not unique.")

    print("[4/7] Writing plain XML input files...")
    nodes_root = ET.Element("nodes")
    for node in node_mapping_rows:
        ET.SubElement(nodes_root, "node", {
            "id": str(node["node_id"]),
            "x": f"{node['x_utm36_m']:.3f}",
            "y": f"{node['y_utm36_m']:.3f}",
        })
    xml_write(nodes_root, NODES_OUT)

    edges_root = ET.Element("edges")
    for attrs in edge_rows_for_xml:
        ET.SubElement(edges_root, "edge", attrs)

    # Preserve known roundabout membership from the cleaned GPKG explicitly.
    # Connected components are formed using only edges carrying junction=roundabout.
    roundabout_rows = [
        row for row in edge_mapping_rows
        if str(row.get("junction_tag_from_cleaned_gpkg", "")).strip().lower() == "roundabout"
    ]
    roundabout_parent: dict[int, int] = {}

    def rb_find(node_id: int) -> int:
        roundabout_parent.setdefault(node_id, node_id)
        if roundabout_parent[node_id] != node_id:
            roundabout_parent[node_id] = rb_find(roundabout_parent[node_id])
        return roundabout_parent[node_id]

    def rb_union(a: int, b: int) -> None:
        root_a, root_b = rb_find(a), rb_find(b)
        if root_a != root_b:
            roundabout_parent[root_b] = root_a

    for record in roundabout_rows:
        rb_union(int(record["u"]), int(record["v"]))

    roundabout_groups: dict[int, list[str]] = {}
    for record in roundabout_rows:
        root_id = rb_find(int(record["u"]))
        roundabout_groups.setdefault(
            root_id, []).append(str(record["edge_id"]))

    for _, roundabout_edge_ids in sorted(roundabout_groups.items()):
        if roundabout_edge_ids:
            ET.SubElement(edges_root, "roundabout", {
                          "edges": " ".join(sorted(roundabout_edge_ids))})

    xml_write(edges_root, EDGES_OUT)

    types_root = ET.Element("types")
    for type_id, highway_class in sorted(type_classes):
        ET.SubElement(types_root, "type", {
            "id": type_id,
            "priority": str(NEUTRAL_PRIORITY),
            "numLanes": str(DEFAULT_LANES),
            "speed": f"{DEFAULT_SPEED_MPS:.6f}",
        })
    xml_write(types_root, TYPES_OUT)

    pd.DataFrame(edge_mapping_rows).to_csv(
        EDGE_MAP_OUT, index=False, encoding="utf-8-sig")
    pd.DataFrame(node_mapping_rows).to_csv(
        NODE_MAP_OUT, index=False, encoding="utf-8-sig")

    print("[5/7] Running SUMO netconvert...")
    netconvert = shutil.which("netconvert")
    if not netconvert:
        raise BuildError(
            "netconvert was not found on PATH. Check SUMO installation and PATH.")

    command = [
        netconvert,
        f"--node-files={NODES_OUT}",
        f"--edge-files={EDGES_OUT}",
        f"--type-files={TYPES_OUT}",
        f"--output-file={NET_OUT}",
        "--no-turnarounds",
        "--geometry.avoid-overlap", "false",
        "--roundabouts.guess", "false",
        "--tls.guess", "false",
    ]
    result = subprocess.run(command, capture_output=True,
                            text=True, encoding="utf-8", errors="replace")
    log_text = (result.stdout or "") + \
        ("\n" if result.stdout and result.stderr else "") + (result.stderr or "")
    LOG_OUT.write_text(log_text, encoding="utf-8")
    if log_text.strip():
        print(log_text.rstrip())
    if result.returncode != 0:
        raise BuildError(
            f"netconvert failed with exit code {result.returncode}. See {LOG_OUT}")
    if not NET_OUT.exists():
        raise BuildError(
            "netconvert returned success but dokki.net.xml was not created.")

    print("[6/7] Checking generated SUMO network...")
    root = ET.parse(NET_OUT).getroot()
    output_edges = {
        element.get("id")
        for element in root.findall("edge")
        if element.get("function") != "internal" and not (element.get("id") or "").startswith(":")
    }
    expected_edges = {row["id"] for row in edge_rows_for_xml}
    output_nodes = {element.get("id") for element in root.findall("junction")}
    expected_nodes = {str(node_id) for node_id in node_ids}

    missing_edge_ids = expected_edges - output_edges
    unexpected_edge_ids = output_edges - expected_edges
    missing_node_ids = expected_nodes - output_nodes

    if missing_edge_ids or unexpected_edge_ids or missing_node_ids:
        raise BuildError(
            "Network validation mismatch: "
            f"expected nodes={len(expected_nodes)}, output nodes={len(output_nodes)}, "
            f"missing nodes={len(missing_node_ids)}; "
            f"expected edges={len(expected_edges)}, output edges={len(output_edges)}, "
            f"missing edges={len(missing_edge_ids)}, unexpected edges={len(unexpected_edge_ids)}. "
            f"See {LOG_OUT}."
        )

    print("[7/7] Build completed.")
    print(f"Nodes: {len(output_nodes)} / {len(expected_nodes)}")
    print(f"Edges: {len(output_edges)} / {len(expected_edges)}")
    print(f"Highway types: {len(type_classes)}")
    print(
        f"Explicit roundabout groups: {len(roundabout_groups)} ({len(roundabout_rows)} tagged edges)")
    print(
        f"Mean |geometry length - Model A length|: {sum(geometry_diffs)/len(geometry_diffs):.4f} m")
    print(
        f"Max |geometry length - Model A length|: {max(geometry_diffs):.4f} m")
    print(f"Max endpoint snap: {max(endpoint_snaps):.4f} m")
    print("Generated files:")
    for path in (NODES_OUT, EDGES_OUT, TYPES_OUT, NET_OUT, EDGE_MAP_OUT, NODE_MAP_OUT, LOG_OUT):
        print(f"  {path.name} ({path.stat().st_size:,} bytes)")
    print("NOTE: lanes_osm values were copied as per-edge lane counts. Validate direction-specific lane semantics before capacity calibration.")
    print("NOTE: bridge/tunnel and original traffic-signal tags are not present in the Model A network source and were not added.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except BuildError as exc:
        print(f"BUILD ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
