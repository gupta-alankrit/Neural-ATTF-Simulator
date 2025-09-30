#!/usr/bin/env python3
import os, re, time, json, yaml, shutil, argparse
from pathlib import Path
from scipy.io import savemat

def to_mat(out_dir: Path, env_p: Path, sched_p: Path, cfg_p: Path):
    with open(env_p, "r", encoding="utf-8") as f: env = json.load(f)
    with open(sched_p, "r", encoding="utf-8") as f: schedule = json.load(f)
    with open(cfg_p, "r", encoding="utf-8") as f: config = yaml.safe_load(f)
    savemat(out_dir / "env.mat",      {"env": env})
    savemat(out_dir / "schedule.mat", {"schedule": schedule})
    savemat(out_dir / "config.mat",   {"config": config})

def ensure_imports(p: Path):
    s = p.read_text(encoding="utf-8")
    if not re.search(r'(?m)^\s*import\s+os\b', s): s = "import os\n" + s
    if "loadmat" not in s: s = "from scipy.io import loadmat\n" + s
    p.write_text(s, encoding="utf-8")

def strip_helpers(p: Path):
    """Remove load_json_data(...) and load_yaml_data(...) defs, and prune unused imports json/yaml."""
    s = p.read_text(encoding="utf-8")
    # delete function blocks (non-greedy up to next def/if __main__/EOF)
    s = re.sub(r'(?s)\n\s*def\s+load_json_data\([^)]*\):.*?(?=\n\s*def\b|\nif __name__ ==|$)', "\n", s)
    s = re.sub(r'(?s)\n\s*def\s+load_yaml_data\([^)]*\):.*?(?=\n\s*def\b|\nif __name__ ==|$)', "\n", s)
    # drop now-unused imports if present
    s = re.sub(r'(?m)^\s*import\s+json\s*\n', "", s)
    s = re.sub(r'(?m)^\s*import\s+yaml\s*\n', "", s)
    p.write_text(s, encoding="utf-8")

def patch_main(p: Path):
    s = p.read_text(encoding="utf-8")
    # switch loads to MAT
    s = re.sub(r'env_dict\s*=\s*load_yaml_data\([^)]*\)|env_dict\s*=\s*load_json_data\([^)]*\)',
               'env_dict = loadmat(os.path.join(script_dir,"env.mat"), simplify_cells=True)["env"]', s, count=1)
    s = re.sub(r'schedule_dict\s*=\s*load_json_data\([^)]*\)',
               'schedule_dict = loadmat(os.path.join(script_dir,"schedule.mat"), simplify_cells=True)["schedule"]', s, count=1)
    s = re.sub(r'config_dict\s*=\s*load_yaml_data\([^)]*\)',
               'config_dict = loadmat(os.path.join(script_dir,"config.mat"), simplify_cells=True)["config"]', s, count=1)
    # force background filename
    s = re.sub(r'utility\.load_background\(\s*script_dir\s*,\s*"[^"]*"\s*\)',
               'utility.load_background(script_dir, "background.png")', s, count=1)
    p.write_text(s, encoding="utf-8")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default="", help="suffix for output dir")
    ap.add_argument("--main", default="local_script_test1.py")
    ap.add_argument("--utility", default="utility.py")
    ap.add_argument("--env", default="env.json")
    ap.add_argument("--schedule", default="schedule.json")
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--image", default="warehouse_image.png")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    base = Path.cwd()  # create output where you run the script
    main_p  = base / args.main
    util_p  = base / args.utility
    env_p   = base / args.env
    sched_p = base / args.schedule
    cfg_p   = base / args.config
    img_p   = base / args.image
    for req in [main_p, util_p, env_p, sched_p, cfg_p]:
        if not req.exists(): raise FileNotFoundError(f"Missing: {req}")

    out = base / (args.out if args.out else f"robotarium_submission{args.name or int(time.time())}")
    out.mkdir(parents=True, exist_ok=True)

    to_mat(out, env_p, sched_p, cfg_p)
    shutil.copy2(main_p, out / "main.py")
    shutil.copy2(util_p, out / "utility.py")
    if img_p.exists(): shutil.copy2(img_p, out / "background.png")

    ensure_imports(out / "main.py")
    patch_main(out / "main.py")
    strip_helpers(out / "main.py")

    print("Files written to", out)
    for fn in ["main.py","utility.py","env.mat","schedule.mat","config.mat","background.png"]:
        fp = out / fn
        if fp.exists(): print(" -", fn)

if __name__ == "__main__":
    main()