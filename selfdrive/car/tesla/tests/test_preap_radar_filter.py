from openpilot.selfdrive.car.tesla.preap.radar_filter import FROZEN_AFTER_S, FrozenTrackFilter

STUCK = (1.1, 0.0, 0.0, 0.0)


def test_identical_track_is_dropped_after_window():
  f = FrozenTrackFilter()
  assert f.keep(0, STUCK, 0.0) and f.keep(0, STUCK, FROZEN_AFTER_S - 0.1)
  assert not f.keep(0, STUCK, FROZEN_AFTER_S)


def test_jittering_track_is_kept_even_at_same_distance():
  f = FrozenTrackFilter()
  assert all(f.keep(0, (20.0, 0.1 * (i % 2), 0.0, 0.0), i * 0.5) for i in range(40))


def test_frozen_track_returns_when_it_changes():
  f = FrozenTrackFilter()
  f.keep(0, STUCK, 0.0)
  assert not f.keep(0, STUCK, 5.0)
  assert f.keep(0, (1.2, 0.0, 0.0, 0.0), 5.1)


def test_tracks_are_independent_and_forgettable():
  f = FrozenTrackFilter()
  f.keep(0, STUCK, 0.0)
  assert f.keep(1, STUCK, 5.0) and not f.keep(0, STUCK, 5.0)
  f.forget(0)
  assert f.keep(0, STUCK, 6.0)
