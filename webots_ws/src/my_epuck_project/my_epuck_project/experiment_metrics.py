"""Pure deterministic metrics for the passive cooperative observer."""
from __future__ import annotations
import csv, json, math, os, re, tempfile
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds').replace('+00:00','Z')
def finite(value):
    if isinstance(value,float): return value if math.isfinite(value) else None
    if isinstance(value,dict): return {str(k):finite(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)): return [finite(v) for v in value]
    return value
def atomic_json(path:Path,value):
    path.parent.mkdir(parents=True,exist_ok=True); fd,tmp=tempfile.mkstemp(prefix=f'.{path.name}.',suffix='.tmp',dir=path.parent)
    try:
        with os.fdopen(fd,'w',encoding='utf-8') as stream:
            json.dump(finite(value),stream,indent=2,sort_keys=True,allow_nan=False); stream.write('\n'); stream.flush()
        os.replace(tmp,path)
    except BaseException:
        try: os.unlink(tmp)
        except OSError: pass
        raise
def allocate_run_directory(root:Path,requested:str):
    root.mkdir(parents=True,exist_ok=True); safe=re.sub(r'[^A-Za-z0-9_.-]','_',requested).strip('._')
    if not safe: raise ValueError('run_id has no filename-safe characters')
    for suffix in ['',*[f'-{n:02d}' for n in range(1,100)]]:
        run_id=safe+suffix; path=root/run_id
        try: path.mkdir(); return run_id,path
        except FileExistsError: pass
    raise FileExistsError(f'no free run directory for {safe}')

_FLOAT=re.compile(r'(?<![\w.])[-+]?(?:\d+\.\d+|\d+)(?:[eE][-+]?\d+)?'); _STAMP=re.compile(r'\b\d{10}(?:\.\d{1,9})?\b')
def normalize_warning(message): return _FLOAT.sub('<num>',_STAMP.sub('<stamp>',message)).strip()
def warning_category(message):
    """Return the legacy warning category for a complete rosout message."""
    lower = str(message).lower()
    return next((value for key, value in (
        ('costmap', 'COSTMAP_WARNING'),
        ('controller', 'CONTROLLER_WARNING'),
        ('slam', 'SLAM_WARNING'),
        ('scan', 'SCAN_WARNING'),
        ('transform', 'TF_WARNING'),
        (' tf', 'TF_WARNING'),
    ) if key in lower), 'PROCESS_WARNING')


def rosout_diagnostic_category(message):
    """Return the legacy Nav2/rosout diagnostic category, if any."""
    lower = str(message).lower()
    rules = (
        ('TF_FAILURE', r'unable to transform robot pose into global plan|'
         r'transform.*global plan|tf error|lookup would require'),
        ('MISSED_RATE_WARNING', r'missed its desired rate|current loop rate'),
        ('FOLLOW_PATH', r'\[follow_path\]|followpath'),
        ('COMPUTE_PATH', r'compute_path_to_pose|computepathtopose'),
        ('RECOVERY', r'recovery|clear_(local|global|entirely)|\bspin\b|'
         r'back.?up|\bwait\b'),
        ('COLLISION_MONITOR', r'collision.?monitor|stop.?zone|emergency stop'),
    )
    category = next((name for name, pattern in rules
                     if re.search(pattern, lower)), None)
    controller_or_planner = (
        re.search(r'controller|planner', lower) and
        re.search(r'failed|failure|abort|progress checker|no valid control|'
                  r'timeout', lower))
    return category or ('CONTROLLER_OR_PLANNER_ERROR'
                        if controller_or_planner else None)
@dataclass
class WarningRecord:
    node_name:str; severity:str; representative_message:str; normalized_message:str; first_occurrence:str; last_occurrence:str; occurrence_count:int=1; category:str='PROCESS_WARNING'
class WarningDeduplicator:
    def __init__(self): self.records={}
    def add(self,node,severity,message,wall_time,category):
        normalized=normalize_warning(message); key=(node,severity,normalized); record=self.records.get(key)
        if record is None:
            record=WarningRecord(node,severity,message,normalized,wall_time,wall_time,category=category); self.records[key]=record; return record,True
        record.last_occurrence=wall_time; record.occurrence_count+=1; return record,False

@dataclass
class MotionSample:
    time_s:float; x:float; y:float; distance_remaining:float|None; command_linear:float; command_angular:float
@dataclass
class DetectorState:
    no_progress:bool=False; stuck:bool=False; oscillating:bool=False
class MotionDetector:
    """Windowed, edge-triggered passive navigation anomaly detector."""

    def __init__(self,progress_window=10.,min_improvement=.03,min_displacement=.02,stuck_window=6.,command_linear=.02,command_angular=.15,stuck_displacement=.015,oscillation_window=10.,sign_changes=4,oscillation_displacement=.04):
        self.progress_window=progress_window; self.min_improvement=min_improvement; self.min_displacement=min_displacement; self.stuck_window=stuck_window; self.command_linear=command_linear; self.command_angular=command_angular; self.stuck_displacement=stuck_displacement; self.oscillation_window=oscillation_window; self.sign_changes=sign_changes; self.oscillation_displacement=oscillation_displacement; self.samples=deque(); self.state=DetectorState()
    @staticmethod
    def _disp(samples): return math.hypot(samples[-1].x-samples[0].x,samples[-1].y-samples[0].y) if len(samples)>1 else 0.
    @staticmethod
    def _improve(samples):
        values=[s.distance_remaining for s in samples if s.distance_remaining is not None and math.isfinite(s.distance_remaining)]; return values[0]-min(values) if len(values)>1 else math.inf
    @staticmethod
    def _changes(samples):
        signs=[1 if s.command_angular>0 else -1 for s in samples if abs(s.command_angular)>=.05]; return sum(a!=b for a,b in zip(signs,signs[1:]))
    def update(self,sample,active,accepted=True,near_goal=False,terminating=False):
        self.samples.append(sample); cutoff=sample.time_s-max(self.progress_window,self.stuck_window,self.oscillation_window)
        while self.samples and self.samples[0].time_s<cutoff: self.samples.popleft()
        eligible=active and accepted and not near_goal and not terminating; events=[]
        p=[s for s in self.samples if s.time_s>=sample.time_s-self.progress_window]; np=eligible and len(p)>1 and p[-1].time_s-p[0].time_s>=self.progress_window and self._improve(p)<self.min_improvement and self._disp(p)<self.min_displacement
        s=[x for x in self.samples if x.time_s>=sample.time_s-self.stuck_window]; commanded=bool(s) and all(abs(x.command_linear)>=self.command_linear or abs(x.command_angular)>=self.command_angular for x in s); stuck=eligible and len(s)>1 and s[-1].time_s-s[0].time_s>=self.stuck_window and commanded and self._disp(s)<self.stuck_displacement
        o=[x for x in self.samples if x.time_s>=sample.time_s-self.oscillation_window]; osc=eligible and len(o)>1 and o[-1].time_s-o[0].time_s>=self.oscillation_window and self._changes(o)>=self.sign_changes and self._disp(o)<self.oscillation_displacement and self._improve(o)<self.min_improvement
        for attr,name,value in [('no_progress','NO_PROGRESS',np),('stuck','STUCK',stuck),('oscillating','OSCILLATION',osc)]:
            if getattr(self.state,attr)!=value: events.append(name+('_STARTED' if value else '_CLEARED')); setattr(self.state,attr,value)
        return events
    def reset(self):
        events=[]
        for attr,name in [('no_progress','NO_PROGRESS'),('stuck','STUCK'),('oscillating','OSCILLATION')]:
            if getattr(self.state,attr): events.append(name+'_CLEARED'); setattr(self.state,attr,False)
        self.samples.clear(); return events

@dataclass(frozen=True)
class Grid:
    width:int; height:int; resolution:float; origin_x:float; origin_y:float; origin_yaw:float; data:object
def known_counts(grid):
    values=np.asarray(grid.data,dtype=np.int16)
    return int(np.count_nonzero((values>=0)&(values<50))),int(np.count_nonzero(values>=50)),int(np.count_nonzero(values<0))
def known_world_cells(grid,output_resolution,transform=(0.,0.,0.)):
    values=np.asarray(grid.data,dtype=np.int16); indices=np.flatnonzero(values>=0)
    if not indices.size: return set()
    tx,ty,tyaw=transform; co,so=math.cos(grid.origin_yaw),math.sin(grid.origin_yaw); ct,st=math.cos(tyaw),math.sin(tyaw)
    cols=indices%grid.width; rows=indices//grid.width; dx=(cols.astype(np.float64)+.5)*grid.resolution; dy=(rows.astype(np.float64)+.5)*grid.resolution
    ox=grid.origin_x+co*dx-so*dy; oy=grid.origin_y+so*dx+co*dy
    xs=np.floor((ct*ox-st*oy+tx)/output_resolution).astype(np.int64); ys=np.floor((st*ox+ct*oy+ty)/output_resolution).astype(np.int64)
    return set(zip(xs.tolist(),ys.tolist()))
class CoverageAttribution:
    def __init__(self,simultaneous_window_s=2.): self.window=simultaneous_window_s; self.first={}; self.seen={}; self.unique={}; self.later={}; self.simultaneous=set(); self.duplicated=set()
    def observe(self,robot,cells,time_s):
        mine=self.seen.setdefault(robot,set())
        for cell in set(cells)-mine:
            mine.add(cell); first=self.first.get(cell)
            if first is None: self.first[cell]=(robot,time_s); self.unique[robot]=self.unique.get(robot,0)+1
            elif first[0]!=robot:
                self.duplicated.add(cell)
                if time_s-first[1]<=self.window: self.simultaneous.add(cell)
                else: self.later[robot]=self.later.get(robot,0)+1
    def summary(self):
        total=len(self.first); return {'unique_first_seen_cells':dict(self.unique),'later_duplicated_cells':dict(self.later),'simultaneously_observed_cells':len(self.simultaneous),'total_known_union_cells':total,'duplicated_known_fraction':len(self.duplicated)/total if total else 0.}
class TrajectoryOverlap:
    def __init__(self,bin_size=.05,exclusion_radius=.15): self.bin_size=bin_size; self.exclusion_radius=exclusion_radius; self.bins={}; self.starts={}; self.repeated_distance={}; self.total_distance={}; self.last={}
    def add(self,robot,x,y):
        self.starts.setdefault(robot,(x,y)); previous=self.last.get(robot); distance=planar_step_distance(previous, (x, y)); cell=(math.floor(x/self.bin_size),math.floor(y/self.bin_size)); visited=self.bins.setdefault(robot,set())
        if cell in visited: self.repeated_distance[robot]=self.repeated_distance.get(robot,0.)+distance
        self.total_distance[robot]=self.total_distance.get(robot,0.)+distance
        if all(math.hypot(x-sx,y-sy)>self.exclusion_radius for sx,sy in self.starts.values()): visited.add(cell)
        self.last[robot]=(x,y)
    def summary(self):
        sets=list(self.bins.values()); shared=set.intersection(*sets) if len(sets)>=2 else set(); union=set.union(*sets) if sets else set(); return {'bins_per_robot':{k:len(v) for k,v in self.bins.items()},'cross_robot_bins':len(shared),'cross_robot_overlap_fraction':len(shared)/len(union) if union else 0.,'repeated_visit_distance_m':dict(self.repeated_distance),'distance_travelled_m':dict(self.total_distance)}
class LocalTrajectory:
    """Per-robot route metric in that robot's odometry frame.

    Independent odometry frames are valid for measuring each robot's own
    travel and revisits.  They are not valid for cross-robot overlap, which is
    why this class intentionally exposes no cross-robot comparison.
    """
    def __init__(self,bin_size=.05,exclusion_radius=.15):
        self.bin_size=bin_size; self.exclusion_radius=exclusion_radius
        self.bins={}; self.starts={}; self.repeated_distance={}
        self.total_distance={}; self.last={}; self.samples={}
    def add(self,robot,x,y):
        x=float(x); y=float(y)
        self.samples[robot]=self.samples.get(robot,0)+1
        self.starts.setdefault(robot,(x,y)); previous=self.last.get(robot)
        distance=planar_step_distance(previous, (x, y))
        cell=(math.floor(x/self.bin_size),math.floor(y/self.bin_size))
        visited=self.bins.setdefault(robot,set())
        if cell in visited:
            self.repeated_distance[robot]=self.repeated_distance.get(robot,0.)+distance
        self.total_distance[robot]=self.total_distance.get(robot,0.)+distance
        if math.hypot(x-self.starts[robot][0],y-self.starts[robot][1])>self.exclusion_radius:
            visited.add(cell)
        self.last[robot]=(x,y)
    def summary(self):
        return {
            'valid': bool(self.samples),
            'reason': 'runtime odometry samples' if self.samples else
                      'no runtime odometry samples',
            'source_frame_by_robot': {robot: f'{robot}/odom'
                                     for robot in self.samples},
            'bins_per_robot': {k: len(v) for k,v in self.bins.items()},
            'repeated_visit_distance_m': dict(self.repeated_distance),
            'distance_travelled_m': dict(self.total_distance),
            'sample_count_by_robot': dict(self.samples),
        }


def planar_step_distance(previous, current):
    """Return the legacy planar step distance for two poses."""
    if previous is None:
        return 0.0
    return math.hypot(current[0] - previous[0],
                     current[1] - previous[1])


class LiveDistanceAccumulator:
    """Minimal live state retained for telemetry and cycle accounting.

    Full bins/revisit state belongs to deferred ``LocalTrajectory`` replay.
    This object preserves the legacy live consumers that need cumulative
    distance before finalization without retaining a second full trajectory.
    """

    def __init__(self):
        self.total_distance = {}
        self.last = {}

    def add(self, robot, x, y):
        point = (float(x), float(y))
        previous = self.last.get(robot)
        self.total_distance[robot] = (
            self.total_distance.get(robot, 0.0) +
            planar_step_distance(previous, point))
        self.last[robot] = point


def replay_local_trajectory_csv(path, robot, bin_size=.05,
                                exclusion_radius=.15):
    """Replay the legacy local trajectory metric from forensic odometry.

    The live observer feeds ``LocalTrajectory.add`` in odometry callback
    order.  ``ForensicEvidenceWriter.record_odom`` writes the same callback
    order to one CSV per robot, so the deferred path deliberately preserves
    file order instead of sorting by message timestamps.  No metric logic is
    duplicated: the replay uses the same ``LocalTrajectory`` implementation.

    Missing or malformed rows fail closed rather than being silently skipped.
    That preserves the distinction between an empty valid stream and damaged
    evidence before this result is allowed to replace the live accumulator.
    """
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(path)
    trajectory = LocalTrajectory(
        bin_size=bin_size, exclusion_radius=exclusion_radius)
    with path.open(newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        required = {'robot_id', 'pose_x', 'pose_y'}
        if not required.issubset(set(reader.fieldnames or ())):
            missing = sorted(required - set(reader.fieldnames or ()))
            raise ValueError(
                f'local trajectory evidence missing columns: {missing}')
        for row_number, row in enumerate(reader, start=2):
            if str(row.get('robot_id', '')) != str(robot):
                raise ValueError(
                    f'local trajectory robot mismatch at row {row_number}')
            try:
                x = float(row['pose_x'])
                y = float(row['pose_y'])
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(
                    f'invalid local trajectory pose at row {row_number}') from exc
            if not math.isfinite(x) or not math.isfinite(y):
                raise ValueError(
                    f'non-finite local trajectory pose at row {row_number}')
            trajectory.add(robot, x, y)
    return trajectory


def equivalent_frontiers(a,b,centroid_tolerance=.15,bbox_margin=.05):
    centroid=math.hypot(a['centroid_x']-b['centroid_x'],a['centroid_y']-b['centroid_y']); overlap=not(a['max_x']+bbox_margin<b['min_x'] or b['max_x']+bbox_margin<a['min_x'] or a['max_y']+bbox_margin<b['min_y'] or b['max_y']+bbox_margin<a['min_y']); return centroid<=centroid_tolerance and overlap
def duplicate_goal(a,b,tolerance=.15): return math.hypot(a[0]-b[0],a[1]-b[1])<=tolerance
