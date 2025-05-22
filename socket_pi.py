import socket
from threading import Thread

HOST = '0.0.0.0'
PORT = 8888

with socket.socket() as server:
    server.bind((HOST, PORT))
    server.listen(5)
    print("服务端已启动，等待连接...")
    
    while True:
        client, addr = server.accept()
        print(f"客户端 {addr} 已接入")
        
        def handle_client(c):
            while True:
                data = c.recv(1024)
                if not data: break
                print("收到数据:", data.decode())
                c.send(b'Feeling good and requesting to take off!')  # 返回确认
            c.close()
        
        Thread(target=handle_client, args=(client,)).start()