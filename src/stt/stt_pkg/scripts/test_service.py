#!/usr/bin/env python3
import rospy
from stt_msgs.srv import STT, STTRequest


def test_service():
    # 等待服务可用
    rospy.wait_for_service("/srv_paddle_speech_stt")

    try:
        # 创建服务代理
        stt_proxy = rospy.ServiceProxy("/srv_paddle_speech_stt", STT)

        # 构造请求
        request = STTRequest()
        request.input_json = '{"query": "yutian.wav"}'

        # 调用服务
        response = stt_proxy(request)
        print("Response:", response.output_json)

    except rospy.ServiceException as e:
        print("Service call failed:", e)


if __name__ == "__main__":
    test_service()
