import requests
import json

url = "https://wcode.net/api/gpt/v1/chat/completions"

payload = {
  "model": "qwen-vl-plus",
  "messages": [
    {
      "role": "user",
      "content": "你好"
    }
  ]
}

headers = {
  "Authorization": "xxx",     # TODO: 这里的 API_KEY 需要替换，获取 API Key 入口：https://wcode.net/get-apikey
  "content-type": "application/json"
}

response = requests.post(url, json=payload, headers=headers).json()
print(response["choices"][0]["message"]["content"])
