import requests

# class get_lidar_config:
#     def __init__(self):
#         pass


def get_request(LiDAR_IP, LiDAR_TCP_PORT, command):
    try:
        # 发送 GET 请求并获取SDK版本信息，设置超时时间为2秒
        print("HTTP {} 请求".format(command))
        response = requests.get('http://{}:{}/command/?{}'.format(LiDAR_IP, LiDAR_TCP_PORT, command), timeout=3)
        # 检查响应状态码是否为200
        if response.status_code == 200:
            return response.text
        else:
            print("HTTP 请求失败，状态码:", response.status_code)
            return None
    except requests.exceptions.Timeout:
        # 处理超时异常
        print("HTTP {} 请求超时".format(command))
    except requests.exceptions.RequestException as e:
        # 处理其他请求异常
        print("HTTP 请求失败:", e)
    #     raise



# def get_sn(LiDAR_IP, LiDAR_TCP_PORT):
#     # 发送 GET 请求并获取sn内容
#     LiDAR_sn = requests.get('http://172.168.1.10:8010/command/?get_sdk_version').text
#     return LiDAR_sn
#

if __name__ == '__main__':
    LiDAR_IP = '172.168.1.10'
    LiDAR_TCP_PORT = '8010'
    res = [None] * 3
    res[0] = get_request(LiDAR_IP, LiDAR_TCP_PORT, command='get_sn')
    print(res)
    print("1111111111111111")
