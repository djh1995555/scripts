import requests

url = "https://test-raw-data-storage.ks3-cn-tianjin-xm01.ksyuncs.com/car/prod/lm5-vp1/0094/2025-01-26/10-11-51/full_record.2025-01-26-10-55-32.00000"
output_path = "./full_record.2025-01-26-10-55-32.00000"

# try:
#     response = requests.get(url, stream=True)
#     with open(output_path, "wb") as f:
#         for chunk in response.iter_content(chunk_size=1024):
#             if chunk:
#                 f.write(chunk)
#     print("下载完成!")
# except Exception as e:
#     print(f"下载失败: {e}")
    
import wget
try:
    filename = wget.download(url, out=output_path)
    print(f"\n文件已保存至: {filename}")
except Exception as e:
    print(f"\n下载失败: {e}")