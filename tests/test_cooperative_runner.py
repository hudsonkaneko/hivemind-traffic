"""Exercise the actual nested capture writer without starting the GPU runtime."""
import ast
from pathlib import Path
from types import SimpleNamespace
import time

import numpy as np
import pytest


@pytest.fixture
def capture():
    source=Path(__file__).resolve().parents[1]/'scripts/cooperative_lidar_drive.py'
    tree=ast.parse(source.read_text())
    classes=[node for node in ast.walk(tree) if isinstance(node,ast.ClassDef) and node.name=='Capture']
    assert len(classes)==1
    annotator=object();errors=[]
    rep=SimpleNamespace(Writer=type('Writer',(),{}),annotators=SimpleNamespace(get=lambda name:annotator))
    namespace=dict(rep=rep,np=np,time=time,errors=errors,parse_generic_model_output_data=lambda raw:raw)
    isolated=ast.Module(body=[classes[0]],type_ignores=[])
    exec(compile(ast.fix_missing_locations(isolated),str(source),'exec'),namespace)
    return namespace['Capture'],annotator,errors


def packet(x,timestamp=100):
    return SimpleNamespace(numElements=2,scanComplete=1,elementsCoordsType='CARTESIAN',frameOfReference='WORLD',
        motionCompensationState='NONCOMPENSATED',timestampNs=timestamp,
        x=np.array([x,x+1]),y=np.array([-4.8,-4.8]),z=np.array([1.,1.]),
        flags=np.array([64,64]),timeOffsetNs=np.array([0,10]),
        frameStart=SimpleNamespace(posM=np.array([x,0,1.8]),timestampNs=timestamp),
        frameEnd=SimpleNamespace(posM=np.array([x+1,0,1.8]),timestampNs=timestamp+10))


def payload(data):
    return {'renderProducts':{'synthetic_product':{'GenericModelOutput':{'data':data}}}}


def create_writer(cls,vid,sink):
    # Match Replicator's registry construction: user __init__ is skipped.
    writer=cls.__new__(cls)
    writer.initialize(vehicle_id=vid,sink=sink)
    return writer


def test_registry_initialization_establishes_annotator_and_layout(capture):
    cls,annotator,_=capture
    writer=create_writer(cls,'ego',{})
    assert writer.data_structure=='renderProduct'
    assert writer.annotators==[annotator]


def test_each_writer_owns_packet_sink_and_arrays(capture):
    cls,_,errors=capture;ego={};peer={}
    one=create_writer(cls,'ego',ego);two=create_writer(cls,'peer',peer)
    source=packet(40)
    one.write(payload(source));snapshot=ego.copy()
    two.write(payload(packet(34,200)))
    assert ego['vehicle_id']=='ego' and peer['vehicle_id']=='peer'
    assert ego['timestamp']==100 and peer['timestamp']==200
    np.testing.assert_array_equal(ego['xyz'][:,0],[40,41])
    np.testing.assert_array_equal(peer['xyz'][:,0],[34,35])
    assert ego['xyz'] is snapshot['xyz']
    source.x[:]=-100;source.flags[:]=0;source.frameEnd.posM[:]=0
    np.testing.assert_array_equal(ego['xyz'][:,0],[40,41])
    np.testing.assert_array_equal(ego['flags'],[64,64])
    np.testing.assert_array_equal(ego['end_position'],[41,0,1.8])
    assert not errors


def test_bad_coordinate_packets_preserve_last_good_scan_and_bound_errors(capture):
    cls,_,errors=capture;sink={};writer=create_writer(cls,'peer',sink)
    writer.write(payload(packet(34)))
    previous=sink['xyz'];bad=packet(500,999);bad.frameOfReference='SENSOR'
    for _ in range(20):writer.write(payload(bad))
    assert sink['timestamp']==100 and sink['xyz'] is previous
    assert len(errors)==8
    assert all(message.startswith('peer: ') and 'coordinate contract' in message for message in errors)


def test_missing_or_empty_output_does_not_erase_packet(capture):
    cls,_,errors=capture;sink={};writer=create_writer(cls,'ego',sink)
    writer.write(payload(packet(40)))
    empty=packet(99);empty.numElements=0
    writer.write(payload(empty));writer.write({'renderProducts':{}})
    writer.write({'renderProducts':{'p':{'GenericModelOutput':None}}})
    assert sink['timestamp']==100 and not errors
