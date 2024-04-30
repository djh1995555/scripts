#!/usr/bin/env python
from utils import *

set_fds_config()
from ad_cloud.adrn.data_seeker.records import RecordReader
filesystem_kws = {
    "protocol": "s3",
    "key": f"{os.getenv('XIAOMI_ACCESS_KEY_ID')}",
    "secret": f"{os.getenv('XIAOMI_SECRET_ACCESS_KEY')}",
    "client_kwargs": {"endpoint_url": "https://s3-cnbj1.mi-fds.net"},
    "default_block_size": 20 * 1024 * 1024,
    "default_fill_cache": False,
    "default_cache_type": "readahead",
}
path = "data-input/car/prod/mo1-mt/0227/2024-04-11/14-56-54/full_record.2024-04-11-14-59-19.00001"
reader = RecordReader(path, filesystem_kws=filesystem_kws)
for meta in reader.iter_metadata():
    print(meta)  # meta 是上述数据模型中定义的 Metadata 类
    break

# set_ks3_config()
# from ad_cloud.adrn.data_seeker.records import RecordReader
# filesystem_kws = {
#     "protocol": "s3",
#     "key": f"{os.getenv('AD_CLOUD_DATASEEKER_KS3_ACCESS_KEY')}",
#     "secret": f"{os.getenv('AD_CLOUD_DATASEEKER_KS3_SECRET_KEY')}",
#     "client_kwargs": {"endpoint_url": "https://test-raw-data-storage.ks3-cn-tianjin-xm01.ksyuncs.com"},
#     "default_block_size": 20 * 1024 * 1024,
#     "default_fill_cache": False,
#     "default_cache_type": "readahead",
# }
# path = "test-raw-data-storage/car/prod/mo5-mt/0257/2024-04-24/23-31-19/full_record.2024-04-24-23-56-24.00029"
# reader = RecordReader(path, filesystem_kws=filesystem_kws)
# for meta in reader.iter_metadata():
#     print(meta)  # meta 是上述数据模型中定义的 Metadata 类
#     break