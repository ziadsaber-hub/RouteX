
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[4]

SOURCE = ROOT / "dokki_sumo/simulation/dokki_random.rou.xml"
OUTPUT = ROOT / "dokki_sumo/experiments/scenario_0/demand"

OUTPUT.mkdir(parents=True, exist_ok=True)

SCENARIOS = {
    "normal": 200,
    "moderate": 400,
    "heavy": 600,
}


def main():
    tree = ET.parse(SOURCE)
    root = tree.getroot()

    vehicles = root.findall("vehicle")

    if len(vehicles) != 600:
        raise ValueError(
            f"Expected 600 source vehicles, found {len(vehicles)}"
        )

    # Keep the first N vehicles in their original order.
    for name, count in SCENARIOS.items():
        new_root = ET.Element(root.tag, root.attrib)

        for child in root:
            if child.tag != "vehicle":
                new_root.append(child)

        for vehicle in vehicles[:count]:
            new_root.append(vehicle)

        output_file = OUTPUT / f"{name}.rou.xml"

        ET.ElementTree(new_root).write(
            output_file,
            encoding="UTF-8",
            xml_declaration=True,
        )

        print(f"{name}: {count} vehicles -> {output_file}")


if __name__ == "__main__":
    main()
