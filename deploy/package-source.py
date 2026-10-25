#!/usr/bin/env python3
"""Export the committed Wild Sync source and its software version as a ZIP."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "rig"))
import buildinfo


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--output", required=True, type=Path,
                    help="new ZIP file to create, outside the checkout")
    args = ap.parse_args()
    output = args.output.expanduser().resolve()
    if output == ROOT or ROOT in output.parents:
        ap.error("choose an output outside the checkout")
    if not output.parent.is_dir():
        ap.error("the output directory does not exist")
    if output.exists():
        ap.error("the output already exists; choose a new filename")
    stamp = buildinfo.from_git(str(ROOT))
    if not stamp or stamp.get("dirty") is not False:
        ap.error("export requires a Git checkout with no tracked-file changes")
    commit = stamp["commit"]
    record = {k: stamp[k] for k in ("version", "commit", "dirty", "describe")}
    # Build aside; publish only a complete archive and never replace a file.
    with tempfile.TemporaryDirectory(prefix="wildsync-source-",
                                     dir=str(output.parent)) as temp:
        archive = Path(temp) / "source.zip"
        subprocess.run(["git", "-C", str(ROOT), "archive", "--format=zip",
                        "--prefix=wildsync/", "--output=" + str(archive),
                        commit], check=True)
        with zipfile.ZipFile(archive, "a", compression=zipfile.ZIP_DEFLATED) as z:
            z.writestr("wildsync/BUILD_INFO.json",
                       json.dumps(record, indent=2) + "\n")
            bad = z.testzip()
            if bad:
                raise RuntimeError("archive verification failed: " + bad)
        checksum = hashlib.sha256()
        with archive.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                checksum.update(block)
        digest = checksum.hexdigest()
        os.link(archive, output)
    print("Source commit: " + commit)
    print("Archive: " + str(output))
    print("SHA256: " + digest)
    return 0


if __name__ == "__main__":
    sys.exit(main())
