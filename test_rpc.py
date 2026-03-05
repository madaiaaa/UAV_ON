import msgpackrpc

client = msgpackrpc.Client(msgpackrpc.Address("127.0.0.1", 30000))

try:
    result = client.call('ping') 
    print("RPC 测试成功，Server 响应了 ping！")
except Exception as e:
    print(f"RPC 测试失败: {e}")