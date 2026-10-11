"""Audit logs only. Archived telemetry has no beams and cannot reconstruct a map."""
from collections import Counter
import json
from pathlib import Path
import zipfile


def audit_archive(path):
    path=Path(path)
    if path.suffix.lower()=='.zip':
        with zipfile.ZipFile(path) as archive:
            names=[n for n in archive.namelist() if n.endswith('.jsonl')]
            if len(names)!=1: raise ValueError('Archive must contain exactly one JSONL log')
            if archive.getinfo(names[0]).file_size>20_000_000: raise ValueError('Log too large')
            text=archive.read(names[0]).decode('utf-8')
    else:
        if path.stat().st_size>20_000_000: raise ValueError('Log too large')
        text=path.read_text()
    rows=[json.loads(line) for line in text.splitlines() if line.strip()]
    return {'rows':len(rows),'states':dict(Counter(r.get('state','unknown') for r in rows)),
            'nonzero_encoder_rows':sum(any(r.get('counts',[])) for r in rows),
            'nonzero_command_rows':sum(any(r.get('wheel_command_mps',[])) for r in rows),
            'raw_scans_available':False,
            'limitation':'Telemetry audit only: no raw LiDAR beams; original mapping accuracy cannot be replayed'}
