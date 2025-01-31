#!/usr/bin/env python3
import rospy
import json
from tts_msgs.srv import TTS, TTSRequest


def test_service():
    # 等待服务可用
    rospy.wait_for_service("/srv__tts")

    try:
        # 创建服务代理
        tts_proxy = rospy.ServiceProxy("/srv__tts", TTS)

        # 构造请求
        request = TTSRequest()
        request.input_json = '{"query": "你好，我是雨田"}'

        # 调用服务
        response = tts_proxy(request)
        print("Response:", json.loads(response.output_json))

    except rospy.ServiceException as e:
        print("Service call failed:", e)


if __name__ == "__main__":
    test_service()
