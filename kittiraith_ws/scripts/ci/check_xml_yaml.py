#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_xml_yaml.py — validate every XML (package.xml, *.launch) and YAML file
in the repository. Used by CI and scripts/verify_all.sh.

Exit code: 0 = all valid, 1 = at least one invalid.
"""

import os
import sys
import xml.dom.minidom

try:
    import yaml
    HAVE_YAML = True
except ImportError:  # pragma: no cover
    HAVE_YAML = False

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


def main():
    errors = []
    xml_count = 0
    yaml_count = 0

    for dirpath, dirnames, filenames in os.walk(ROOT):
        dirnames[:] = [d for d in dirnames
                       if d not in (".git", ".venv", "build", "devel",
                                    "install", "node_modules", "__pycache__")]
        for name in filenames:
            path = os.path.join(dirpath, name)
            if name.endswith(".xml") or name.endswith(".launch"):
                xml_count += 1
                try:
                    xml.dom.minidom.parse(path)
                except Exception as exc:  # noqa: BLE001
                    errors.append("invalid XML {}: {}".format(path, exc))
            elif name.endswith((".yaml", ".yml")) and HAVE_YAML:
                yaml_count += 1
                try:
                    with open(path, "r") as handle:
                        yaml.safe_load(handle)
                except Exception as exc:  # noqa: BLE001
                    errors.append("invalid YAML {}: {}".format(path, exc))

    if errors:
        for e in errors:
            print("FAIL:", e)
        print("RESULT: {} XML, {} YAML — {} error(s)".format(
            xml_count, yaml_count, len(errors)))
        return 1
    print("OK: {} XML and {} YAML files valid".format(xml_count, yaml_count))
    return 0


if __name__ == "__main__":
    sys.exit(main())
