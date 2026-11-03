"""Open-loop pulse calibration. JY61 measurements never enter this conversion."""
from dataclasses import dataclass,asdict
from pathlib import Path
import math,json

@dataclass(frozen=True)
class Axis:
    zero_us: int=1500
    us_per_degree: float=2000/180
    direction: int=1
    minimum_us: int=1000
    maximum_us: int=2000
    calibrated: bool=False
    def validate(self):
        if not (1000<=self.minimum_us<=self.zero_us<=self.maximum_us<=2000):raise ValueError('脉宽设置必须在 1000–2000 μs 内')
        if not math.isfinite(self.us_per_degree) or not 1<=self.us_per_degree<=100:raise ValueError('每度脉宽需在 1–100 μs 内')
        if self.direction not in (-1,1):raise ValueError('方向只能是 +1 或 -1')
        return self
    def pulse(self,degrees):
        self.validate();degrees=float(degrees)
        if not math.isfinite(degrees):raise ValueError('角度必须是有限数值')
        value=self.zero_us+self.direction*degrees*self.us_per_degree
        if not self.minimum_us<=value<=self.maximum_us:raise ValueError('目标角度超出当前保守行程，请减小角度')
        return int(math.floor(value+.5))
    def angle(self,pulse):return (pulse-self.zero_us)/(self.direction*self.us_per_degree)

def default_profile():return Axis(),Axis(us_per_degree=2000/270)
def load_profile(path):
    if not Path(path).exists():return default_profile()
    data=json.loads(Path(path).read_text(encoding='utf-8'))
    return Axis(**data['pan']).validate(),Axis(**data['tilt']).validate()
def save_profile(path,pan,tilt):
    pan.validate();tilt.validate();path=Path(path);tmp=path.with_suffix('.tmp')
    tmp.write_text(json.dumps(dict(pan=asdict(pan),tilt=asdict(tilt)),ensure_ascii=False,indent=2),encoding='utf-8');tmp.replace(path)
def speed(degrees_per_second,pan,tilt):
    value=float(degrees_per_second)
    if not math.isfinite(value) or not .1<=value<=30:raise ValueError('速度请输入 0.1–30 °/s')
    # Shared pulse rate. Both nominal angular speeds are <= requested speed.
    target=value*min(pan.us_per_degree,tilt.us_per_degree)
    if target>=50:step=max(1,min(20,int(target//50)));frames=1
    else:step=1;frames=math.ceil(50/target)
    if frames>100:raise ValueError('当前标定下速度太低，硬件最慢为每 2 秒 1 μs')
    rate=50*step/frames
    return step,frames,rate/pan.us_per_degree,rate/tilt.us_per_degree
