# Autonomous Target Scoring Drone
# Libraries for GPS, IMU, and Bash/Camera operation
import socket
import sys
import time
import board
import busio
import adafruit_gps
import subprocess
import math
import select

# Initialize GPS serial connection
import serial
uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=3000)
gps = adafruit_gps.GPS(uart, debug=False)
gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
gps.send_command(b'PMTK220,1000')

while True:

    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Bind the socket to the port
    server_address = ('192.168.1.10', 50007)
    print ('starting up on %s port %s' % server_address)
    sock.bind(server_address)

    # Listen for incoming connections
    sock.listen(1)

    while True:
        # Wait for a connection
        print('waiting for a connection')
        connection, client_address = sock.accept()

        is_readable = [connection]
        is_writable = []
        is_error = []

        try:
            print ('connection from', client_address)
            last_update = time.monotonic()

            # Message transmission
            while True:
                
                # Recieve image request, if available
                readable, w, e = select.select(is_readable, is_writable, is_error, 1.0)
                if readable:
                    bufsize = 2048
                    req = connection.recv(bufsize)
                    reqDe = req.decode('utf-8')
                else:
                    reqDe = ''

                # If an image was requested, take picture and send image
                if reqDe == 'p':
                    subprocess.call("./sendpng.sh")

                # Must wait at least 1 second between gps module updates
                current = time.monotonic()
                if current - last_update >= 1:     # (seconds)
                
                    # Update GPS Data
                    gps.update()

                    # Update time of last update
                    last_update = current

                    # Create string with current latitude and longitude
                    toSend = ''
                    # If GPS has no fix, send NA (Not Available)
                    if not gps.has_fix:
                        toSend += 'cLatNA;'
                        toSend += 'cLonNA;'
                    # Else append cLat and cLon to the toSend string
                    else:
                        toSend += 'cLat{0:.8f};'.format(gps.latitude)
                        toSend += 'cLon{0:.8f};'.format(gps.longitude)

                    # Send data string to client
                    connection.sendall(toSend.encode('utf-8'))

                    # Print data in nice readable format to the Pi terminal
                    print('=' * 40)
                    print(toSend)
                    
                    if not gps.has_fix:
                        print('GPS waiting for fix')
                    else:
                        print('Latitude: {0:.6f} degrees'.format(gps.latitude))
                        print('Longitude: {0:.6f} degrees'.format(gps.longitude))
                else:
                    print('time condition not met')
                    print('current = ', current)
                    print('last_update = ', last_update)
        except:
            break
        finally:
            # Clean up the connection
            print('closing connection')
            connection.close()
            print('closing socket')
            sock.close()

