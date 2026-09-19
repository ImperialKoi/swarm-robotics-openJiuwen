#!/usr/bin/env python
"""Pack what a Kaggle session needs to run the simulator, and nothing else.

    uv run python scripts/kaggle_bundle.py        # -> runs/kaggle/swarmmind-rl-src.zip

Upload the zip as a private Kaggle Dataset (Kaggle unpacks it), attach it to
`swarmmind/training/notebooks/unit_policy.ipynb`, and run. No git, no commit: the zip is
built from the working tree as it stands, so what trains is exactly what is on disk.

Contents: the `swarmmind` package, the scenario YAMLs (including the evolved roster, which
`World` loads for the demo), and `pyproject.toml` + `uv.lock` so the session installs the
same locked dependencies. The dashboard, docs, models and tests stay behind -- none of them
is needed to run a headless mission, and the GGUF alone would dwarf everything else.
"""

from __future__ import annotations

import hashlib
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "runs" / "kaggle" / "swarmmind-rl-src.zip"


def files() -> list[Path]:
    out = [ROOT / "pyproject.toml", ROOT / "uv.lock", ROOT / "README.md"]
    out += sorted(p for p in (ROOT / "swarmmind").rglob("*.py") if "__pycache__" not in p.parts)
    out += sorted((ROOT / "assets" / "scenarios").glob("*.yaml"))
    return out


def main() -> int:
    OUT.parent.mkdir(parents=True, exist_ok=True)
    digest = hashlib.sha256()
    with zipfile.ZipFile(OUT, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for p in files():
            rel = p.relative_to(ROOT).as_posix()
            data = p.read_bytes()
            digest.update(rel.encode() + b"\0" + data)
            zf.writestr(rel, data)
    n = len(files())
    print(f"  {n} files -> {OUT.relative_to(ROOT)}  ({OUT.stat().st_size / 1e6:.2f} MB)")
    print(f"  content sha256 {digest.hexdigest()[:16]}  (compare against the notebook's log)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
