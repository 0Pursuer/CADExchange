"""Ensure an untrusted STEP boolean result cannot supply displayed difference solids."""

import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("result", type=Path)
    args = parser.parse_args()
    result = json.loads(args.result.read_text(encoding="utf-8-sig"))
    consistency = result["boolean_consistency"]
    if consistency["boolean_result_valid"]:
        print("Boolean evidence is valid")
        return
    failures = []
    for artifact_id in result["artifacts"]:
        if artifact_id.startswith(("missing_material", "added_material")):
            failures.append(f"Untrusted difference artifact: {artifact_id}")
    for check in result["checks"]:
        if check["id"] == "boolean_cut_residual" and (check["available"] or check["passed"]):
            failures.append("Untrusted residual is available or passed")
    if failures:
        parser.exit(1, "\n".join(failures) + "\n")
    print("Untrusted boolean evidence is excluded from displayed differences")


if __name__ == "__main__":
    main()
