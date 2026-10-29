"""Compare every field of a fixed original evaluation without changing tolerance."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil

ROOT=Path(__file__).resolve().parents[1]


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def differences(reference,actual,atol=1e-9,rtol=1e-9,path='$'):
    found=[]
    if type(reference) is not type(actual):
        return [dict(path=path,reference=reference,actual=actual,within_tolerance=False,reason='type')]
    if isinstance(reference,dict):
        for key in sorted(reference.keys()|actual.keys()):
            if key not in reference or key not in actual:
                found.append(dict(path=path+'.'+key,within_tolerance=False,reason='missing_key'))
            else:found.extend(differences(reference[key],actual[key],atol,rtol,path+'.'+key))
    elif isinstance(reference,list):
        if len(reference)!=len(actual):
            found.append(dict(path=path,within_tolerance=False,reason='length'))
        for i,(x,y) in enumerate(zip(reference,actual)):
            found.extend(differences(x,y,atol,rtol,f'{path}[{i}]'))
    elif isinstance(reference,float):
        if not math.isfinite(reference) or not math.isfinite(actual):
            found.append(dict(path=path,within_tolerance=False,reason='nonfinite'))
        elif reference!=actual:
            found.append(dict(path=path,reference=reference,actual=actual,absolute_difference=abs(actual-reference),
                within_tolerance=math.isclose(actual,reference,rel_tol=rtol,abs_tol=atol),reason='float'))
    elif reference!=actual:
        found.append(dict(path=path,reference=reference,actual=actual,within_tolerance=False,reason='value'))
    return found


def compare(reference,actual,output):
    reference=Path(reference).resolve();actual=Path(actual).resolve();output=Path(output).resolve()
    plan_path=ROOT/'experiments/configs/hudson_delivered_reproduction_v1.json'
    plan=json.loads(plan_path.read_text(encoding='utf-8'));contract=plan['comparison']
    if sha(reference)!=plan['expected_reference_sha256']:raise ValueError('Unrecognized reference edition')
    if contract['float_absolute_tolerance']!=1e-9 or contract['float_relative_tolerance']!=1e-9:
        raise ValueError('Changed frozen comparison tolerance')
    r=json.loads(reference.read_text(encoding='utf-8'));a=json.loads(actual.read_text(encoding='utf-8'))
    seeds=list(range(410001,410021))
    for value in (r,a):
        if value['controller']!='learned' or [e['seed'] for e in value['episodes']]!=seeds:
            raise ValueError('Wrong controller, count, order or seed set')
    output.mkdir(parents=True,exist_ok=False)
    shutil.copyfile(__file__,output/'compare-source.py');shutil.copyfile(plan_path,output/'plan.json')
    mismatch=differences(r,a)
    result=dict(reference=str(reference),actual=str(actual),reference_sha256=sha(reference),actual_sha256=sha(actual),
        compared_episodes=20,all_fields_exact=not mismatch,
        all_fields_within_declared_tolerance=all(m['within_tolerance'] for m in mismatch),
        exact_differences=mismatch,original_reference_acceptance=r['summary']['accepted'],
        local_original_acceptance=a['summary']['accepted'],local_summary=a['summary'],
        tolerance=dict(absolute=1e-9,relative=1e-9),training=False,
        limits='All supplied per-seed and summary metrics compared. Equality does not prove every unprovided state/action is identical, erase runtime differences, or establish I110/physical acceptance.')
    (output/'comparison.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    (output/'manifest.json').write_text(json.dumps(dict(status='completed',inputs={str(reference):sha(reference),str(actual):sha(actual),str(plan_path):sha(plan_path)},
        outputs={p.name:sha(p) for p in output.iterdir() if p.is_file()}),indent=2)+'\n',encoding='utf-8')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for arg in ('reference','actual','output'):parser.add_argument('--'+arg,type=Path,required=True)
    result=compare(**vars(parser.parse_args()))
    print(json.dumps({k:v for k,v in result.items() if k!='exact_differences'},indent=2))
    raise SystemExit(0 if result['all_fields_within_declared_tolerance'] else 1)
