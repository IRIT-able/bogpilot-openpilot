"""Drop radar tracks that report byte-identical values over time (Pre-AP Bosch radar).

Route 00000019 held tracks at exactly 1.1, 5.9 and 8.4 m at every speed with no car present,
which radard took as a lead. A real target jitters between frames even when both cars are stopped.
"""
FROZEN_AFTER_S = 2.0


class FrozenTrackFilter:
  def __init__(self, frozen_after_s: float = FROZEN_AFTER_S):
    self.frozen_after_s = frozen_after_s
    self.last: dict[int, tuple[tuple[float, ...], float]] = {}

  def keep(self, index: int, values: tuple[float, ...], now_s: float) -> bool:
    prev = self.last.get(index)
    if prev is None or prev[0] != values:
      self.last[index] = (values, now_s)
      return True
    return now_s - prev[1] < self.frozen_after_s

  def forget(self, index: int) -> None:
    self.last.pop(index, None)
