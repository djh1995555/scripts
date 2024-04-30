# !/usr/bin/env python
from utils import *

set_fds_config()
from ad_cloud.adrn.data_seeker.records import DataclipReader
adrn = "adr::dataclip:eng.mo1-vp1.0102::1699252694000000000:1699252719000000000"
adrn = "adr::dataclip:prod.mo1-mt.0227::1712821800000000000:1712821860000000000"
filesystem_kws = {
    "protocol": "s3",
    "key": f"{os.getenv('XIAOMI_ACCESS_KEY_ID')}",
    "secret": f"{os.getenv('XIAOMI_SECRET_ACCESS_KEY')}",
    "client_kwargs": {"endpoint_url": "https://s3-cnbj1.mi-fds.net"},
    "default_block_size": 20 * 1024 * 1024,
    "default_fill_cache": False,
    "default_cache_type": "readahead",
}
reader = DataclipReader(adrn, filesystem_kws=filesystem_kws)
print(len(reader.record_files))
for file in reader.record_files:
    print(file.path) 

# set_ks3_config()
# from ad_cloud.adrn.data_seeker.records import DataclipReader
# adrn = "adr::dataclip:prod.mo1-mt.0060:..:1638949057006882000:1638949378006498000"
# adrn = "adr::dataclip:eng.mo1-vp1.0102::1699252694000000000:1699252719000000000"
# adrn = "adr::dataclip:prod.mo5-mt.0257::1713973210000000000:1713973810000000000"
# filesystem_kws = {
#     "protocol": "s3",
#     "key": f"{os.getenv('XIAOMI_ACCESS_KEY_ID')}",
#     "secret": f"{os.getenv('XIAOMI_SECRET_ACCESS_KEY')}",
#     # "key": f"{os.getenv('AD_CLOUD_DATASEEKER_KS3_ACCESS_KEY')}",
#     # "secret": f"{os.getenv('AD_CLOUD_DATASEEKER_KS3_SECRET_KEY')}",
#     "client_kwargs": {"endpoint_url": "https://test-raw-data-storage.ks3-cn-tianjin-xm01.ksyuncs.com"},
#     "default_block_size": 20 * 1024 * 1024,
#     "default_fill_cache": False,
#     "default_cache_type": "readahead",
# }
# reader = DataclipReader(adrn, filesystem_kws=filesystem_kws)
# print(reader.record_files) 
