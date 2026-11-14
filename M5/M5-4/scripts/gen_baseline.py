"""Package a complete scene and its models with an offline integrity check."""
import argparse
from datetime import date
from pathlib import Path
import shutil
import tarfile
import tempfile

import yaml

from exp import ROOT, digest, model_info, read_config, scene_path, tested_scene, write_config

VERIFY = """#!/bin/sh
set -eu
cd "$(dirname "$0")"
for tool in sha256sum find wc; do
    command -v "$tool" >/dev/null 2>&1 || { echo "FAIL: install coreutils/findutils ($tool missing)"; exit 1; }
done
fail() { echo "FAIL: $*"; exit 1; }
[ -s baseline.yaml ] || fail "baseline.yaml missing or empty"
[ -d models ] || fail "models directory missing"
[ "$(find models -type f | wc -l)" -eq MODEL_COUNT ] || fail "model count mismatch"
[ -z "$(find models -type f -size 0c -print)" ] || fail "empty model"
sha256sum -c SHA256SUMS || fail "checksum mismatch"
echo PASS
"""


def package(scene, output):
    models = model_info(scene)
    if not models:
        raise ValueError("基线需要至少一个模型文件")
    if output.exists():
        raise ValueError(f"输出已存在：{output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=output.parent) as temporary:
        root = Path(temporary)
        (root / "models").mkdir()
        artifacts = ["baseline.yaml"]
        for index, model in enumerate(models.values(), 1):
            source = (ROOT / model["path"]).resolve()
            if source.stat().st_size <= 0:
                raise ValueError(f"模型为空：{source}")
            name = f"models/{index:02d}" + "".join(source.suffixes)
            if any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789./-_" for c in name):
                raise ValueError(f"模型扩展名不受支持：{source.name}")
            shutil.copyfile(source, root / name)
            if digest(root / name) != model["sha256"]:
                raise ValueError(f"模型在复制期间变化：{source}")
            model["path"] = name
            artifacts.append(name)
        scene["models"] = models
        write_config(root / "baseline.yaml", scene)
        (root / "SHA256SUMS").write_text("".join(f"{digest(root / name)}  {name}\n" for name in artifacts))
        (root / "verify_baseline.sh").write_text(VERIFY.replace("MODEL_COUNT", str(len(models))))
        (root / "verify_baseline.sh").chmod(0o755)
        (root / "README.md").write_text(
            "# M5-4 基线包\n\n解压后进入该目录并执行 ./verify_baseline.sh。\n"
            "需要 POSIX shell、GNU coreutils 和 findutils。通过条件为模型数精确匹配、"
            "每个模型大于零字节、场景和各模型 SHA-256 完全一致。\n"
            "基线参数和模型路径列于 baseline.yaml，路径相对于解压目录。"
            "自验覆盖文件完整性，不执行模型推理或 ROS 上车测试。\n"
        )
        archive_path = root / "archive.tar.gz"
        with tarfile.open(archive_path, "w:gz") as archive:
            for name in ["baseline.yaml", "models", "verify_baseline.sh", "README.md", "SHA256SUMS"]:
                archive.add(root / name, arcname=name)
        archive_path.replace(output)
    print(f"{output} · {output.stat().st_size} bytes")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--exp")
    source.add_argument("--scene")
    parser.add_argument("--out", type=Path, default=Path(f"baseline_package_{date.today():%Y%m%d}.tar.gz"))
    args = parser.parse_args()
    try:
        scene = tested_scene(args.exp)[0] if args.exp else read_config(scene_path(args.scene))
        package(scene, args.out)
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        parser.exit(1, f"基线包生成失败：{exc}\n")


if __name__ == "__main__":
    main()
