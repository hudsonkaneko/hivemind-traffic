import pytest
from traffic.trip_accounting import TripLedger, TripRequest, read_requests


def test_hand_calculated_external_queue_and_unfinished_groups():
    ledger = TripLedger([TripRequest('av',0,'controlled','mainline'),
        TripRequest('ramp',2,'human_proxy','ramp'), TripRequest('waiting',3,'human_proxy','ramp'),
        TripRequest('future',20)])
    ledger.observe(1,departed={'av':0},active=['av'])
    ledger.observe(5,departed={'ramp':4},active=['ramp'],arrived=['av'])
    ledger.observe(10,active=['ramp'])
    report=ledger.snapshot(); total=report['totals']
    assert total['requested']==3 and total['scheduled']==1
    assert total['completed']==1 and total['unfinished']==2
    assert total['boundary_time_s']==20  # 5 + (10-2) + (10-3)
    assert total['external_wait_s']==9  # 0 + (4-2) + (10-3)
    assert report['groups']['origin:ramp']['boundary_time_s']==15


def test_removal_and_teleport_cannot_earn_completion_or_reduce_cost():
    ledger=TripLedger([TripRequest('removed',0),TripRequest('teleported',0)])
    ledger.observe(1,departed={'removed':0,'teleported':0},active=['removed','teleported'])
    ledger.observe(2,teleport_started=['teleported'])
    ledger.observe(3,teleport_ended=['teleported'],active=['teleported'])
    ledger.observe(4,arrived=['teleported'])
    ledger.observe(10)
    r=ledger.snapshot()
    assert r['totals']['completed']==0 and r['totals']['removed']==2
    assert r['totals']['boundary_time_s']==20 and not r['comparison_valid']


def test_initial_and_same_tick_departure_arrival():
    ledger=TripLedger([TripRequest('initial',0,initial=True),TripRequest('short',0)])
    ledger.observe(.2,departed={'short':0},arrived=['short'],active=['initial'])
    ledger.observe(1,arrived=['initial'])
    assert ledger.snapshot()['totals']['boundary_time_s']==pytest.approx(1.2)
    assert ledger.snapshot()['totals']['initial']==1


def test_demand_and_event_errors_fail_closed(tmp_path):
    path=tmp_path/'routes.xml';path.write_text('<routes><flow id="hidden"/></routes>')
    with pytest.raises(ValueError,match='Expand'): read_requests(path)
    with pytest.raises(ValueError,match='Duplicate'): TripLedger([TripRequest('x',0),TripRequest('x',1)])
    ledger=TripLedger([TripRequest('x',0)])
    with pytest.raises(ValueError,match='outside'):ledger.observe(1,active=['unknown'])
    with pytest.raises(ValueError,match='strictly'):ledger.observe(0)
    with pytest.raises(ValueError,match='lifecycle'):ledger.observe(1,active=['x'])
