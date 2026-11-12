import pytest

from scripts.build_highway_flow_rl_v05_heldout_bank import DESIGNS, UNSUPPORTED, TRAINED_FAMILY, geometry


@pytest.mark.parametrize('ident,description,options', DESIGNS, ids=[d[0] for d in DESIGNS])
def test_design_source_geometry(ident, description, options):
    nodes, edges, connections, placement = geometry(**options)
    lanes = {e.get('id'): int(e.get('numLanes')) for e in edges}
    through = options['lanes']
    assert lanes['sb1'] == through and lanes['sb5'] == through
    aux = [f'sb{i}' for i in range(1, 6) if lanes[f'sb{i}'] == through + 1]
    assert aux == [f'sb{i}' for i in range(2, options['aux_end'] + 1)]
    # The auxiliary lane is fed only by the ramp, and every through lane continues.
    into_aux = [c.get('from') for c in connections if c.get('to') == 'sb2' and c.get('toLane') == '0']
    assert into_aux == ['mlk_on']
    for edge in range(1, 5):
        here, there = lanes[f'sb{edge}'], lanes[f'sb{edge+1}']
        targets = sorted(int(c.get('toLane')) for c in connections
                         if c.get('from') == f'sb{edge}' and c.get('to') == f'sb{edge+1}')
        # Lanes present on both edges continue; an added lane starts empty and a
        # dropped auxiliary lane has no successor.
        continuing = min(here, there)
        assert targets == list(range(there - continuing, there))
    widths = {float(e.get('width')) for e in edges}
    assert widths == {options.get('lane_width', 3.2)}


def test_every_design_differs_from_trained_family():
    for ident, _, options in DESIGNS:
        novel = [options['lanes'] not in TRAINED_FAMILY['through_lanes'],
                 options.get('lane_width', 3.2) != TRAINED_FAMILY['lane_width_m'],
                 options['speed_mph'] not in TRAINED_FAMILY['speed_limit_mph'],
                 options.get('ramp_speed_mph', 30) != TRAINED_FAMILY['ramp_speed_mph'],
                 options['ramp_upstream'] not in TRAINED_FAMILY['ramp_upstream_m'],
                 options['aux_end'] != 3,
                 abs(options.get('bend', 0.)) > TRAINED_FAMILY['max_abs_bend_m'],
                 len(set(options['segments'])) > 1]
        assert any(novel), ident


def test_unsupported_designs_are_explicit():
    assert {'left_side_entry', 'no_acceleration_lane', 'weaving_with_exit'} <= set(UNSUPPORTED)


def test_invalid_parameters_rejected():
    with pytest.raises(ValueError):
        geometry(lanes=3, segments=[400, 100, 400, 400, 400], speed_mph=55, aux_end=3, ramp_upstream=160.)
