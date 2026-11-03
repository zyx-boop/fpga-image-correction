"""User-observed gimbal home positions; these are pulse widths, not angles."""
from dataclasses import asdict, dataclass
import json
from pathlib import Path
from protocol import (PAN_MIN_US, PAN_MAX_US, PAN_CENTER_US,
                      TILT_MIN_US, TILT_MAX_US, TILT_CENTER_US)


@dataclass(frozen=True)
class HomeCalibration:
    pan_us: int = PAN_CENTER_US
    tilt_us: int = TILT_CENTER_US
    pan_low_us: int = PAN_MIN_US
    pan_high_us: int = PAN_MAX_US
    tilt_low_us: int = TILT_MIN_US
    tilt_high_us: int = TILT_MAX_US
    pan_recorded: bool = False
    tilt_recorded: bool = False

    def validate(self):
        if any(type(value) is not int for value in
               (self.pan_us, self.tilt_us, self.pan_low_us, self.pan_high_us,
                self.tilt_low_us, self.tilt_high_us)):
            raise ValueError('回中脉宽和范围必须是整数')
        if not (PAN_MIN_US <= self.pan_low_us < PAN_CENTER_US < self.pan_high_us <= PAN_MAX_US):
            raise ValueError('水平调试范围须在 500–2500 μs 内且包含 1500 μs')
        if not (TILT_MIN_US <= self.tilt_low_us < TILT_CENTER_US < self.tilt_high_us <= TILT_MAX_US):
            raise ValueError('俯仰调试范围须在 500–2000 μs 内且包含 1250 μs')
        if not (self.pan_low_us <= self.pan_us <= self.pan_high_us and
                self.tilt_low_us <= self.tilt_us <= self.tilt_high_us):
            raise ValueError('每个轴的调试范围须覆盖其已记录回中值')
        if type(self.pan_recorded) is not bool or type(self.tilt_recorded) is not bool:
            raise ValueError('无效的回中记录标志')
        return self

    def save(self, path: Path):
        self.validate()
        path = Path(path)
        temporary = path.with_name(path.name + '.tmp')
        temporary.write_text(json.dumps({'version': 2, **asdict(self)},
                                        ensure_ascii=False, indent=2), encoding='utf-8')
        temporary.replace(path)

    @classmethod
    def load(cls, path: Path):
        data = json.loads(Path(path).read_text(encoding='utf-8'))
        if not isinstance(data, dict):
            raise ValueError('无效的回中配置')
        required = set(cls.__dataclass_fields__)
        if data.get('version') == 2 and set(data) == required | {'version'}:
            return cls(**{name: data[name] for name in required}).validate()
        legacy = {'pan_us','tilt_us','low_us','high_us','pan_recorded','tilt_recorded','version'}
        if data.get('version') == 1 and set(data) == legacy:
            if type(data['low_us']) is not int or type(data['high_us']) is not int:
                raise ValueError('旧版调试范围必须是整数')
            if not (500 <= data['low_us'] < 1500 < data['high_us'] <= 19999):
                raise ValueError('旧版调试范围无效')
            # Preserve homes and intersect the old shared span with each axis's
            # confirmed limits. Loading never rewrites the existing file.
            return cls(pan_us=data['pan_us'],tilt_us=data['tilt_us'],
                       pan_low_us=data['low_us'],pan_high_us=min(data['high_us'],PAN_MAX_US),
                       tilt_low_us=min(data['low_us'],TILT_CENTER_US-1),
                       tilt_high_us=min(data['high_us'],TILT_MAX_US),
                       pan_recorded=data['pan_recorded'],tilt_recorded=data['tilt_recorded']).validate()
        raise ValueError('回中配置版本或字段不支持')
