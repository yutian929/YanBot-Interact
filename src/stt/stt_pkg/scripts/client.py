import requests

# run in ROS
response = requests.post("http://localhost:8888/query", json={"query": "yutian.wav"})

print("Status Code:", response.status_code)
print("Response:", response.json())
