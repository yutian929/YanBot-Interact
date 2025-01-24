import requests

# 发送请求到容器
response = requests.post(
    "http://localhost:8888/query", json={"query": "How to use PaddleSpeech, yutian?"}
)

print("Status Code:", response.status_code)
print("Response:", response.json())
