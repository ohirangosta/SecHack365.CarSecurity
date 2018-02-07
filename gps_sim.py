#! /usr/bin/python

import socket
import os
from time import *
import time
import threading
 
localhost = "127.0.0.1"
localport = 4989
client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
 
if __name__ == '__main__':
  try:
    while True:
      #It may take a second or two to get good data
      #print gpsd.fix.latitude,', ',gpsd.fix.longitude,'  Time: ',gpsd.utc
 
      gps_send_data = str(34.639606) + ":" + str(135.423169) + "\n"
      client.sendto(gps_send_data, (localhost, localport))
      print 'send packet : ', gps_send_data
      time.sleep(1)
 
  except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
    print "\nKilling Thread..."
  print "Done.\nExiting."
