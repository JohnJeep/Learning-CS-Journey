from socket import *

serverName = "hostName"
serverPort = 12000
clientSocket = socket(AF_INET, SOCK_DGRAM)
message = input("Input uppercase char:")
clientSocket.sendto(message.encode(), (serverName, serverPort))
modifiedMessage, serveraddress = clientSocket.recvfrom(2048)
print(modifiedMessage.decode())
clientSocket.close()



