import argparse
import json
from pathlib import Path
import yaml

def parse_args():
    parser = argparse.ArgumentParser(
        description="Task scheduler simulator"
    )

    parser.add_argument(
        "--config",
        required=True,
        help="path to tasks.yaml, tasks.yml, or tasks.json",
    )

    return parser.parse_args()
    
def load_config(path):
    path = Path(path)

    suffix = path.suffix.lower()

    if suffix in [".yaml", ".yml"]:
        with open(path, "r") as f:
            config = yaml.safe_load(f)

    elif suffix == ".json":
        with open(path, "r") as f:
            config = json.load(f)

    else:
        raise ValueError(
            "unsupported config format: use .yaml, .yml, or .json"
        )

    return config
    
def main():
    args = parse_args()

    config = load_config(args.config)

    print(config)
    
if __name__ == "__main__":
    main()
