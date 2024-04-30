import os
XIAOMI_ACCESS_KEY_ID = "AKDWWR6VNFNH6LBMHV"
XIAOMI_SECRET_ACCESS_KEY = "wNlcgtjZCrc5uOt3I1xXLMKuMvz9PYLVDLicA6PU"
os.environ["XIAOMI_ACCESS_KEY_ID"] = XIAOMI_ACCESS_KEY_ID
os.environ["XIAOMI_SECRET_ACCESS_KEY"] = XIAOMI_SECRET_ACCESS_KEY
os.environ["XIAOMI_IAM_ACCESS_KEY_ID"] = 'CAKXBT0VC1HT7WWPDRK'
os.environ["XIAOMI_IAM_SECRET_ACCESS_KEY"] = 'zMoA83wo49IRxsXIT5pUXQLuqtTpQLN1i2l8f0IR'
os.environ["FRAME_DATA_CACHE_PATH"] = "/mnt/frame-data-cache"
os.environ["DATA_MOUNT_PATH"] = "/mnt/output-data-storage"
os.environ["AD_CLOUD_DATASEEKER_FILESYSTEM"] = "local"
os.environ["AD_CLOUD_DATASEEKER_DATA_PATH"] = "/mnt/output-data-storage"
os.environ["AD_CLOUD_DATASEEKER_FRAME_CACHE_PATH"] = "/mnt/frame-data-cache"
os.environ["AD_CLOUD_DATASEEKER_PROD_DATA_PATH"] = "/mnt/pro-output-data-storage"
os.environ["XIAOMI_USERNAME"] = "fengguoyang"
os.environ["AD_CLOUD_AUTH_HOST"] = "http://king-metaflow-service-prod.data.evad.mioffice.cn"
os.environ["AD_CLOUD_DATASEEKER_INDEX_FILE_PATH"] = "/mnt/mcap-index/raw"
os.environ["AD_CLOUD_DATASEEKER_KS3_ENDPOINT"] = "http://ks3-cn-tianjin-xm01-internal.ksyuncs.com"
os.environ['AD_CLOUD_XIAOMI_DEPARTMENT'] = "perception"
os.environ['AD_CLOUD_DATASET_CALL_ENV'] = "tjv1"
from ad_cloud.event import seeker
from ad_cloud.adrn.data_seeker.records import DataclipReader, RecordReader
# from ad_cloud.adrn.data_seeker.dataclip import build_clip
# from ad_cloud.adrn.data_seeker.frame import read_frame


filesystem_kws = {
    'protocol': 's3',
    'key': f"{XIAOMI_ACCESS_KEY_ID}",
    'secret': f"{XIAOMI_SECRET_ACCESS_KEY}",
    'client_kwargs': {'endpoint_url': "https://s3-cnbj1.mi-fds.net"},
    'default_block_size': 20 * 1024 * 1024,
    'default_fill_cache': False,
    'default_cache_type': 'readahead'
}

watch_channels = [
    "/localization/kinetic_localization",
]

clipAdrnStr = "adr::dataclip:eng.bhd.0001::1695003208091000000:1695004092490000000"
reader = DataclipReader(f"{clipAdrnStr}")