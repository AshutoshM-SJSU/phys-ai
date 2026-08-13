from __future__ import annotations
import json
from pathlib import Path
from gazebo_trust_experiments.config import load_config
from gazebo_trust_experiments.paths import resolve_from_config
from gazebo_trust_experiments.movingai_map import load_movingai_map

def load_runtime(config_path: str):
    p=Path(config_path).expanduser().resolve(); cfg=load_config(p)
    return p,cfg,load_movingai_map(resolve_from_config(cfg.map.file,p))

def event_payload(event_type: str, sim_time: float, **details):
    return json.dumps({'event_type':event_type,'sim_time':sim_time,'details':details},separators=(',',':'))
