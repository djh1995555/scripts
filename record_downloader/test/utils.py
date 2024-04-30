#!/usr/bin/env python
import os

def set_output_dir():
    os.environ["XIAOMI_USERNAME"] = "dongjianhao"
    os.environ["AD_CLOUD_XIAOMI_DEPARTMENT"] = "泊车功能"
    os.environ["AD_CLOUD_DATASET_CALL_ENV"] = "tjv1"

    os.environ["XIAOMI_IAM_ACCESS_KEY_ID"] = "AKQDGKPFZXGMBNPGSA"
    os.environ["XIAOMI_IAM_SECRET_ACCESS_KEY"] = "rtA5wNkdE9wQSu9EopZCiVk+t6hYX9S3AT+o7CEd"

    # os.environ["XIAOMI_ACCESS_KEY_ID"] = "AK3VFUU4SNOMLQE345"
    # os.environ["XIAOMI_SECRET_ACCESS_KEY"] = "g/KmXjETpeigjbDc1PmuIqscCniVpfG9NbAwT6Fe"

    os.environ["XIAOMI_ACCESS_KEY_ID"] = "AKXX2XVJ5ECBCLGLFH"
    os.environ["XIAOMI_SECRET_ACCESS_KEY"] = "Wxt8JCuiM/TlkHVVICBs5iQAlxGG2cCEVkpPwCz9"

    os.environ["AD_CLOUD_DATASEEKER_KS3_ACCESS_KEY"] = "AKLTH0w8rIHDRJO6uIv0bBCr"
    os.environ["AD_CLOUD_DATASEEKER_KS3_SECRET_KEY"] = "OOdbSAfokuQENo5eTEadPz7jK5hLUZIGz7ofXQjP"

def set_fds_config():
    set_output_dir()
    # os.environ["XIAOMI_FDS_ENDPOINT"] = "cnbj1-fds.api.xiaomi.net"
    # os.environ["AD_CLOUD_DATASEEKER_DATA_PATH"] = "/tmp"
    # os.environ["AD_CLOUD_DATASEEKER_FRAME_CACHE_PATH"] = "/tmp"
    # os.environ["AD_CLOUD_DATASEEKER_FILESYSTEM"] = "s3"
    # os.environ["AD_CLOUD_DATASEEKER_INDEX_FILE_PATH"] = "data-input"

def set_ks3_config():
    set_output_dir()
    # os.environ["AD_CLOUD_DATASEEKER_KS3_ENDPOINT"] = "ks3-cn-tianjin-xm01.ksyuncs.com"
    os.environ["AD_CLOUD_DATASEEKER_KS3_ENDPOINT"] = "http://ks3-cn-tianjin-xm01-internal.ksyuncs.com"

    # os.environ["AD_CLOUD_DATASEEKER_DATA_PATH"] = "/tmp"
    os.environ["AD_CLOUD_DATASEEKER_DATA_PATH"] = "/mnt/output-data-storage"

    # os.environ["AD_CLOUD_DATASEEKER_FRAME_CACHE_PATH"] = "/tmp"
    os.environ["AD_CLOUD_DATASEEKER_FRAME_CACHE_PATH"] = "/mnt/frame-data-cache"

    
    # os.environ["AD_CLOUD_DATASEEKER_INDEX_FILE_PATH"] = "test-raw-data-storage"
    os.environ["AD_CLOUD_DATASEEKER_INDEX_FILE_PATH"] = "/mnt/mcap-index/raw"

    os.environ["FRAME_DATA_CACHE_PATH"] = "/mnt/frame-data-cache"
    os.environ["DATA_MOUNT_PATH"] = "/mnt/output-data-storage"
    
    os.environ["AD_CLOUD_DATASEEKER_FILESYSTEM"] = "s3"
    os.environ["AD_CLOUD_DATASEEKER_PROD_DATA_PATH"] = "/mnt/pro-output-data-storage"
    os.environ["AD_CLOUD_AUTH_HOST"] = "http://king-metaflow-service-prod.data.evad.mioffice.cn"
    
    


def set_config2():
    t_AK = "AKXX2XVJ5ECBCLGLFH"
    t_SK = "Wxt8JCuiM/TlkHVVICBs5iQAlxGG2cCEVkpPwCz9"
    os.environ["XIAOMI_ACCESS_KEY_ID"] = t_AK
    os.environ["XIAOMI_SECRET_ACCESS_KEY"] = t_SK

    os.environ["XIAOMI_FDS_ENDPOINT"] = "cnbj1-fds.api.xiaomi.net"
    os.environ["AD_CLOUD_DATASEEKER_DATA_PATH"] = "/tmp"
    os.environ["AD_CLOUD_DATASEEKER_FRAME_CACHE_PATH"] = "/tmp"
    os.environ["AD_CLOUD_DATASEEKER_FILESYSTEM"] = "s3"
    os.environ["AD_CLOUD_DATASEEKER_INDEX_FILE_PATH"] = "data-input"

    os.environ["XIAOMI_IAM_ACCESS_KEY_ID"] = "CAKCSUULUDMC4NFYSWN"
    os.environ["XIAOMI_IAM_SECRET_ACCESS_KEY"] = "zzWoAAYqRTnttLHWJ4MsQeUlWtWFUnlNlUDAkLxK"
    os.environ["AD_CLOUD_XIAOMI_DEPARTMENT"] = "ad-infra"
    os.environ["XIAOMI_USERNAME"] = "zhanglazhuan"

    os.environ["AD_CLOUD_DATASET_CALL_ENV"] = "tjv1"

    os.environ["EVENT_TYPE"] = "prod"