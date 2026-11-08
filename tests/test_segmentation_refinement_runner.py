"""Contract guards for the saved-input-only refinement runner."""
import copy
import json
import pytest
from experiments.preview_segmentation_refinement import CONFIG, validate_refinement


def test_declared_config_is_inference_only():
    cfg=json.loads(CONFIG.read_text())
    validate_refinement(cfg)
    assert cfg['training'] is False and cfg['new_simulator_runs'] is False


@pytest.mark.parametrize('field,value',[
    ('training',True),('new_simulator_runs',True),('variants',['letterbox1024']),
    ('boundary_tolerance_px',10),('base_config','../../outside.json')])
def test_refuses_changed_experiment_contract(field,value):
    cfg=copy.deepcopy(json.loads(CONFIG.read_text()));cfg[field]=value
    with pytest.raises(ValueError):validate_refinement(cfg)
