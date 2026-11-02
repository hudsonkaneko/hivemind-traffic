"""Retrieve a bounded, date-pinned public NAIP orthophoto for I-110 inspection.

USDA FPAC-BC GEO service: illustrative mapping, not surveying/engineering.
No account, credential or PeMS dependency. Preserve receipt and credit source.
"""
import argparse
import hashlib
import json
from pathlib import Path
import urllib.parse
import urllib.request

SERVICE='https://apps.geo.fpac.usda.gov/geo-imagery/rest/services/naip/conus_naip/ImageServer'
ORIGIN=[381687.546870016,3764108.929151129]


def post(operation,parameters):
    request=urllib.request.Request(SERVICE+'/'+operation,data=urllib.parse.urlencode(parameters).encode())
    with urllib.request.urlopen(request,timeout=45) as response:
        data=response.read(12_000_001)
    if len(data)>12_000_000:raise RuntimeError('Bounded imagery response exceeded 12MB')
    return data


def acquire(output):
    output.mkdir(parents=True,exist_ok=False)
    query=dict(f='json',where="ST='CA' AND (Name='m_3411862_se_11_060_20240506' OR Name='m_3311806_ne_11_060_20240506')",
               outFields='*',returnGeometry='false')
    metadata=post('query',query);(output/'footprints.json').write_bytes(metadata)
    d=json.loads(metadata);features=d.get('features',[])
    if len(features)!=2 or any(f['attributes']['year_ts']!=2024 for f in features):
        raise RuntimeError('Exact dated NAIP source records unavailable; do not silently change epoch')
    ids=[f['attributes']['OBJECTID'] for f in features]
    x,y=ORIGIN
    parameters=dict(f='image',bbox=','.join(map(str,[x-85,y-940,x+85,y+45])),bboxSR=6340,imageSR=6340,
                    size='340,1970',format='png',bandIds='0,1,2',
                    mosaicRule=json.dumps(dict(mosaicMethod='esriMosaicLockRaster',lockRasterIds=ids)))
    image=post('exportImage',parameters)
    if not image.startswith(b'\x89PNG\r\n\x1a\n'):raise RuntimeError('Imagery export did not return PNG')
    (output/'naip-2024.png').write_bytes(image)
    receipt=dict(service=SERVICE,parameters=parameters,origin_6340=ORIGIN,local_extent=[-85,-940,85,45],
                 native_gsd_m=.6,export_pixel_m=.5,source_ids=ids,acquisition_date='2024-05-06',
                 sha256=hashlib.sha256(image).hexdigest(),
                 limits='Not survey/engineering accuracy; exported pixel spacing does not improve native resolution')
    (output/'receipt.json').write_text(json.dumps(receipt,indent=2))
    print(json.dumps(receipt,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if not args.output.resolve().is_relative_to(Path(__file__).resolve().parents[1]):p.error('Output outside checkout')
    acquire(args.output)
