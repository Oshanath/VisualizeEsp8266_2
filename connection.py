import socket
import sys
import threading
import time
# import datetime
# from mpl_toolkits import mplot3d
# import numpy as np
# from matplotlib import pyplot as plt
# from IPython.display import clear_output
# from matplotlib.animation import FuncAnimation
import json

# --- constants ---

HOST = ''   
PORT = 17

x = []
y = []
z = []

# --- functions ---

# def update(data):
#     ax.cla()
#     ax.scatter(x, y, z, cmap='Greens')
#     # ax.plot(x, y, z)

def handle_client(conn, addr):
    try:

        while True:
            data = conn.recv(1024)
            dataString = data.decode('ascii')

            try:
                dataJson = json.loads(dataString)
            except Exception:
                continue

            # print('recv:', data.decode('ascii'))
            
            if dataJson['type'] == 'pos':
                # x.append(dataJson['x'])
                # y.append(dataJson['y'])
                # z.append(dataJson['z'])
                print(dataJson['x'], '\t', dataJson['y'], '\t', dataJson['z'])

            if dataJson['type'] == 'angle':
                print(dataJson['angle'])

            time.sleep(0.01)

    except BrokenPipeError:
        print('[DEBUG] addr:', addr, 'Connection closed by client?')
    except Exception as ex:
        print('[DEBUG] addr:', addr, 'Exception:', ex, )
    finally:
        conn.close()
        # plt.close()

# --- main ---

# fig = plt.figure()
# ax = plt.axes(projection='3d')

# ax.scatter(x, y, z, cmap='Greens')
# # ax.plot(x, y, z)

# plt.title("test plot")
# plt.xlabel("x")
# plt.ylabel("y")
# ax.set_zlabel('z')

# plt.legend(["x"])

# ani = FuncAnimation(fig, update, interval=100)

try:
    print('[DEBUG] create socket')    
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    print('[DEBUG] bind:', (HOST, PORT))
    s.bind((HOST, PORT))
    print('[DEBUG] listen')
    s.listen(1)

    while True:
        print('[DEBUG] accept ... waiting')
        conn, addr = s.accept()
        print('[DEBUG] addr:', addr)
        t = threading.Thread(target=handle_client, args=(conn, addr))
        t.start()
        # plt.show() 


except Exception as ex:
    print(ex)
except KeyboardInterrupt as ex:
    print(ex)
except:
    print(sys.exc_info())
finally:
    print('[DEBUG] close socket')
    s.close()
