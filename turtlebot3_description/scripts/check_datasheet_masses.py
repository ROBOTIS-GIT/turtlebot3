#!/usr/bin/env python3
"""Sum the link masses of each URDF listed in config/datasheet_masses.yaml and
exit 1 if any sum differs from the vendor's datasheet weight by more than the
tolerance. Needs only PyYAML. Usage: python3 scripts/check_datasheet_masses.py
"""
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

PKG = Path(__file__).resolve().parent.parent


def urdf_mass(path):
    root = ET.parse(path).getroot()
    return sum(float(m.get("value")) for m in root.iterfind("./link/inertial/mass"))


def main():
    cfg = yaml.safe_load((PKG / "config" / "datasheet_masses.yaml").read_text())
    tol = cfg["tolerance"]
    bad = 0
    for name, e in cfg["robots"].items():
        got, want = urdf_mass(PKG / e["urdf"]), e["datasheet_kg"]
        delta = (got - want) / want
        ok = abs(delta) <= tol
        bad += not ok
        print(f"{'OK  ' if ok else 'FAIL'} {name}: urdf {got:.4f} kg, datasheet {want} kg, {delta:+.1%} (tolerance {tol:.0%})")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
