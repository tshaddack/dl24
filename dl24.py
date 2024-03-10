#!/usr/bin/python3

import socket
import errno
from struct import pack
from time import sleep,monotonic
from sys import argv,exit,stdin,stderr
from select import select
# other imports are placed where they are needed, to avoid crashing whole software instead of a single function on a missing dependency


#DEFAULT_SERPORT='/dev/ttyUSB0'
DEFAULT_SERPORT='/dev/rfcomm0'
DEFAULT_BAUDRATE=9600
DEFAULT_TCPPORT=8888

CURRENT_LIMIT=24.1


#DEFAULT_MODBUS_ADDR=1

stdlog=stderr







####################
##
##  LOW LEVEL SERIAL
##
####################


class LowLevelSerPort:
  import serial
  serport='/dev/ttyUSB0'            # target serial port
  baudrate=0
  port=None                         # physical port instance
  verbconn=False
  verbport=False # DEBUG
  verblnk=False
  #verbconn=True
  #verbport=True # DEBUG
  timeout=3
  connretries=5
  connected=False

  def __init__(self,portname='/dev/ttyUSB0',baudrate=DEFAULT_BAUDRATE):
    self.serport=portname
    self.baudrate=baudrate
    pass

  def connect(self):
    if self.verbconn: print('SERPORT:connecting to',self.serport,'@',self.baudrate,file=stdlog)
    for t in range(0,self.connretries):
      try:
        if t>0: print('retrying...',t)
        #self.port=self.serial.Serial(self.serport,self.baudrate, timeout=self.timeout)
        self.port=self.serial.serial_for_url(self.serport,self.baudrate, timeout=self.timeout)
        self.connected=True
        break
      except Exception as e:
        print('PORTCONNERR:',e)
        sleep(1)
    if not self.connected:
      print('ERRPORTCONN: cannot connect to',self.serport,'- too many retries. Aborting.',file=stdlog)
      exit(12)
    if self.verbconn: print('SERPORT:connected',file=stdlog)
    return None

  def close(self):
    if self.port==None: print('SERPORT:neverOpened',file=stdlog);return None
    self.port.close()
    if self.verbconn: print('SERPORT:closed',file=stdlog)
    return None

  def send(self,raw,showpacket=None):
    if showpacket!=None and self.verbconn: showpacket(raw,name='SERPORT:SEND',check=False)
    self.port.write(raw)
    return False

  def recv(self,l,showpacket=None):
    res=self.port.read(l)
    if showpacket!=None and self.verbconn: showpacket(res,name='SERPORT:RECV',check=False)
    return res

  def recvflush(self):
    #n=self.port.in_waiting
    self.port.reset_input_buffer()
    return 0

  def avail(self):
    return self.port.in_waiting



####################
##
##  LOW LEVEL TCP/IP
##
####################

class LowLevelTcpPort:
  ipaddr=None             # target IP
  ipport=None             # target port
  sock=None
  verbconn=False
  verbport=False
  timeout=3
  flushtimeout=0.02
  connretries=5
  connected=False

  reconnect=True
  default_timeout=5 # reconnect timeout for data loss
  time_lastread=-1

  buf=None

  def __init__(self,addr,port):
    self.ipaddr=addr
    self.ipport=port

  def connect(self):
    if self.verbconn: print('SOCK:connecting to',self.ipaddr,':',self.ipport,file=stdlog)
    self.sock=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.sock.settimeout(self.timeout)
    for t in range(0,self.connretries):
      try:
        if t>0: print('SOCK:connection retrying...',t,file=stdlog)
        self.sock.connect( (self.ipaddr,self.ipport) )
        self.connected=True
        break
      except Exception as e:
        print('SOCKCONNERR:',e)
        sleep(min(0.5+t,5)) # increase retries delay, max. 5s
    if not self.connected:
      print('ERRSOCKCONN: cannot connect to',self.ipaddr,':',self.ipport,'- too many retries. Aborting.',file=stdlog)
      exit(12)
    self.sock.setblocking(False) # nonblocking
    if self.verbconn: print('SOCK:connected',file=stdlog)
    self.time_lastread=monotonic()
    return self.sock

  def close(self):
    if self.verbconn: print('SOCK:closed',file=stdlog)
    self.sock.close()


  def send(self,raw,showpacket=None):
    if showpacket!=None and self.verbport: showpacket(raw,name='SOCK:SEND',check=False,file=stdlog)
    try:  return self.sock.sendall(raw)
    except socket.error as e:
      print('SOCK:SEND:ERR:',e,file=stderr)
      if self.reconnect: self.connect()

# WARN: ignores l
  def recv(self,l,showpacket=None):
#    res=self.sock.recv(l)
    res=self.buf
    if showpacket!=None and self.verbport: showpacket(res,name='SOCK:RECV',check=False,file=stdlog)
    return res

  def recvflush(self):
    #print('flush')
    self.sock.settimeout(self.flushtimeout)
    try: n=len(self.sock.recv(1024))
    except socket.timeout: n=0
    self.sock.settimeout(self.timeout)
    return n

  # actual reading happens here, do recv() if result nonzero!
  def avail(self):
    #return self.port.in_waiting
    try: self.buf=self.sock.recv(256)
    except socket.error as e:
      err=e.args[0]
      if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
        if monotonic()-self.time_lastread>self.default_timeout:
          print('SOCK:RECV:TIMEOUT',file=stderr)
          try: self.close()
          except Exception as e: print('SOCK:CLOSE:ERR:',e,file=stderr)
          self.connect()
        return 0
      print('SOCK:RECV:ERR:',e,file=stderr)
      if self.reconnect: self.connect()
    self.time_lastread=monotonic()
    return len(self.buf)



############################
##
##  ATORCH OVER SERIAL OR IP
##
############################

def getint32(l,n):
  return (l[n]<<24) + (l[n+1]<<16) + (l[n+2]<<8) + l[n+3]

def getint24(l,n):
  return (l[n]<<16) + (l[n+1]<<8) + l[n+2]

def getint16(l,n):
  return (l[n]<<8) + l[n+1]





class Instr_Atorch:
  #verbcmd=True
  #verbcom=True
  verblnk=False
  verbcom=False
  verbcomsr=False
  #verblnk=False

  dodelay=False

  usesocket=False
  comm=None

  timeout=3
  retries=10

  retries=3 # command send retries, low level
  retriescmd=3 # setting command retries
#  waitretries=20 # response wait retries
  waitretries=50 # response wait retries
  retrydelay=0.05 # response wait

  default_minimize=True  # True for wireless, False for wired


  expectshort=False   # expect short single-byte reply confirmation
  expectans=False
  packet=[]
  pacletlong=[]
  longpacketcnt=0
  longpacketcntold=0

  offoff=False        # switch output off at program close

  out=None
  stopoff=False

  instrtype=None
  ADU=2 # read from instrtype, this is default; possibly specify in config

  PROTO_SHORTACK=0x6F

  CMD_A_CLRALL=0x01
  CMD_A_CLRCAP=0x02
  CMD_A_CLRTIME=0x03
  CMD_A_SETBACKLIGHT=0x21
  CMD_A_SETCOST=0x22
  CMD_A_BUTTON_SET=0x31
  CMD_A_BUTTON_OK=0x32
  CMD_A_BUTTON_RIGHT=0x33
  CMD_A_BUTTON_LEFT=0x34

  BUTTON_SETUP=CMD_A_BUTTON_SET
  BUTTON_ONOFF=CMD_A_BUTTON_OK
  BUTTON_PLUS=CMD_A_BUTTON_RIGHT
  BUTTON_MINUS=CMD_A_BUTTON_LEFT

  CMD_ONOFF=0x01
  CMD_SETCURRENT=0x02
  CMD_SETCUTOFF=0x03
  CMD_SETTIMEOUT=0x04
  CMD_RESET=0x05

  CMD_GETONOFF=0x10
  CMD_GETV=0x11
  CMD_GETA=0x12
  CMD_GETTIMER=0x13
  CMD_GETMAH=0x14
  CMD_GETMWH=0x15
  CMD_GETTEMP=0x16
  CMD_GETSETCUTOFF=0x18
  CMD_GETSETCURRENT=0x17
  CMD_GETSETTIMER=0x19


  def __init__(self):
    #self.buf=Queue()
    self.buf=[]
    self.packet=[]
    self.packetlong=[]
    self.state={}
    pass

  def initport(self,comm):
    #print('INITPORT')
    self.comm=comm

  def connect(self):
    self.comm.connect()

#  def disconnect(self):
#    #print('xxx',self.comm)
#    self.comm.close()

  def close(self):
    if self.offoff: self.setOFF();
    self.comm.close()



  def strpacket(self,p):
    return ':'.join(f'{x:02x}' for x in p)

  # show hexdump of packet, optionally check crc
  def showpacket(self,p,name='',check=False,force=False):
    if self.verbcom or force:
      print(name,':'.join(f'{x:02x}' for x in p),end='',file=stdlog)
      if p==b'': print(name,'(empty)')
      elif check:
        #print(' CRC:',f'{self.atorch_get_crc(p[2:-1]):02x}',file=stdlog)
        if self.atorch_check_crc(p): print(' (correct)',file=stdlog)
        else: print(' (CRC FAIL)',file=stdlog)
      else: print(file=stdlog)


  def atorch_get_crc(self, data):
    sum=0
    #for x in range(2,len(data)-1): sum+=data[x]
    for x in range(0,len(data)): sum+=data[x]
    sum=(sum^0x44) & 0xff
    return sum # dummy function

  def atorch_check_crc(self,data):
    sum=self.atorch_get_crc(data[2:-1])
    #print('[',sum,']')
    if sum==data[-1]: return True;return False



# Bluetooth initialization:
# AT+BMDL24_BLE



# https://github.com/devanlai/webvoltmeter/blob/master/REVERSE.md
# DL24P protocol, 1-per-second, 36-byte: (pfct=power factor, bk=backlight)
#                  x4               x8               xc               x10              x14              x18              x1c              x20
#                  4                8                12               16               20               24               28               32
# [FF][55][01][02] [00][00][00][00] [00][00][00][00] [12][00][00][00] [00][00][00][00] [00][00][00][00] [00][17][00][00] [0A][33][3c][00] [00][00][00][E1]
# [FF][55][01][02] [00][00][33][00] [00][00][00][00] [12][00][00][00] [00][00][00][00] [00][00][00][00] [00][17][00][00] [0A][33][3c][00] [00][00][00][9C]
#              01   -voltage--  -milliamps-  ---power---  ----energy-----  --price?--   -freq-  -pfct-   -temp-                   bk
#              02   -voltage--  -milliamps-  -amphours--  ----energy-----  --price?--           -pfct-   -temp-  --hhhh---mm--ss  bk
#              03   -voltage--  -milliamps-  -amphours--  ----energy-----   usbd+   usbd-   -temp-  --hhhh---mm--ss  bk
#             ADU      0.1v       0.001a       0.01Ah

# command packet: (unspec=00, except ADU=1..3)
# FF 55 11 <adu> <a2> <a3> 00 <a4> <a5> <checksum>
# clr_all         01
# clr_cap         02
# clr_time        03
# set_bklit       21                val
# set_cost        22 [23:16] 00 [15:8][7:0]
# butt_set        31
# butt_ok         32
# butt_right      33
# butt_left       34


# https://github.com/misdoro/Electronic_load_px100/blob/master/protocol_PX-100_2_70.md
# PX100 command packet:
#            B1 B2 [cmd] [d1] [d2] B6
# on/off            01    xx   00       xx=01 for on, 00 for off
# set current       02    xx   yy       xx=integer, yy=decimal (00..99d)
# set cutoff v      03    xx   yy       ""
# set timeout       04    xx   yy       xxyy as unsigned int in seconds
# reset counters    05    00   00
#
# command response: 0x6F = self.PROTO_SHORTACK
#
# query response data:
#               cmd:   CA CB [d1] [d2] [d3] CE CF
# load enabled   10           00   00   xx          xx=00 or 01
# measured mV    11           xx   yy   zz          xxyyzz, 24bit int
# measured mA    12           xx   yy   zz
# timer value    13           hh   mm   ss
# cap mAh        14           xx   yy   zz
# cap mWh        15           xx   yy   zz
# mosfet 'c      16           xx   yy   zz
# set current    17           xx   yy   zz          10s mA
# set cutoff     18           xx   yy   zz          10s mV
# set timer      19           hh   mm   ss

#  buf=bytearray()




  def flushbuf(self,discard=False,cmt=''):
    disc=False # was anything discarded?
    if discard:
      disc=True
      print(f'discard: {cmt} {self.buf.pop(0):02x}',end='',file=stdlog)
    while len(self.buf)>0:
      if (self.expectshort and self.buf[0]==self.PROTO_SHORTACK) \
      or (self.expectans and self.buf[0]==0xCA) \
      or (self.buf[0]==0xff):
         if disc: print(file=stdlog)
         return True
      if not disc: disc=True;print('discard:',end='',file=stdlog)
      print(f' {self.buf.pop(0):02x}',end='',file=stdlog)
    if disc: print(file=stdlog)
    return False

  def clearbuf(self):
    self.buf=[]

  def recvpacket(self):#
    if not self.flushbuf(): return False

    if self.buf[0]==self.PROTO_SHORTACK:
      self.packet=[self.buf.pop(0)]
      self.expectshort=False
      self.showpacket(self.packet,name='short ANS')
      return True

    while len(self.buf)>0:
      if self.buf[0]==0xCA:
        if len(self.buf)<7: return False # too short
        if self.buf[1]!=0xCB or self.buf[5]!=0xCE or self.buf[6]!=0xCF:
          self.flushbuf(discard=True,cmt='in_shortreply');continue
        self.packet=self.buf[:7].copy()
        self.buf=self.buf[7:]
        self.showpacket(self.packet,name='ANS: ')
        return True
      if self.buf[0]==0xFF:
        if len(self.buf)<3: return False
        if self.buf[1]==0x55: #self.flushbuf(discard=True,cmt='2');continue
          if self.buf[2]==0x01: # FF 55 01 - repeated status message
            if len(self.buf)<36: return False
            self.packetlong=self.buf[:36].copy()
            self.buf=self.buf[36:]
#            self.showpacket(self.packetlong,name='long status',check=True)
            if self.atorch_check_crc(self.packetlong): return True
            self.packetlong=[]
            return False
          if self.buf[2]==0x02: # FF 55 02 - 8-byte command response
            if len(self.buf)<8: return False
            self.packet=self.buf[:8].copy()
            self.buf=self.buf[8:]
            if self.packet[3]==1: s='reply'
            elif self.packet[3]==3: s='reply:UNSUPPORTED'
            else: s=f'reply:UNKNOWN:{self.packet[3]:02x}'
            self.showpacket(self.packet,name=s,check=True)
            if self.atorch_check_crc(self.packet): return True
            self.packet=[]
            return False
      self.flushbuf(discard=True,cmt='in_statusmsg')
    return False


  def handlelongpacket(self):
#                  4                8                12               16               20               24               28               32
# [FF][55][01][02] [00][00][00][00] [00][00][00][00] [12][00][00][00] [00][00][00][00] [00][00][00][00] [00][17][00][00] [0A][33][3c][00] [00][00][00][E1]
# [FF][55][01][02] [00][00][33][00] [00][00][00][00] [12][00][00][00] [00][00][00][00] [00][00][00][00] [00][17][00][00] [0A][33][3c][00] [00][00][00][9C]
#              01   -voltage--  -milliamps-  ---power---  ----energy-----  --price?--   -freq-  -pfct-   -temp-                   bk
#              02   -voltage--  -milliamps-  -amphours--  ----energy-----  --price?--           -pfct-   -temp-  --hhhh---mm--ss  bk
#              03   -voltage--  -milliamps-  -amphours--  ----energy-----   usbd+   usbd-   -temp-  --hhhh---mm--ss  bk
#             ADU      0.1v       0.001a       0.01Ah
    self.longpacketcnt+=1
    a=self.state
    l=self.packetlong
    self.instrtype=l[3]
    self.ADU=self.instrtype
    # for ADU=2; todo, other decoding
    a['V']=getint24(l,4)/10
    a['A']=getint24(l,7)/1000
    a['temp']=getint16(l,24)
    #a['aH']=getint24(l,10)/100
    #a['energy']=getint24(l,13)
    #a['price']=getint24(l,16)/100
    #print('state',a)
    self.packetlong=[]

  def gotupdate(self):
    if self.longpacketcnt==self.longpacketcntold: return False
    self.longpacketcntold=self.longpacketcnt
    return True

  def recvdata(self):
    avail=self.comm.avail()
    #print(avail)
    if avail==0: return False
    r=self.comm.recv(avail)
    self.showpacket(r,name='RECV:',force=self.verbcomsr)
    for x in r: self.buf.append(x)
    r=self.recvpacket()
    if r and self.packetlong!=[]: self.handlelongpacket();r=False
    if self.verbcom: print('receivedAns:',r,file=stdlog)
    return r


  def waitreply(self,expectshort=False,retries=0):
    #if retries<1: retries=self.waitretries
    if retries<1: retries=self.waitretries
    for t in range(0,retries):
      sleep(self.retrydelay)
      self.packet=[]
      self.recvdata()
      if self.packet!=[]:
        #self.showpacket(self.packet,name='wait:',force=True)
        if self.packet[0]==self.PROTO_SHORTACK and not expectshort: continue
        return True
    print('REPLY TIMEOUT',file=stdlog)
    return False

  def send_px100cmd_raw(self,cmd,d=[0,0]):
    packet=pack('>BBBBBB',0xb1,0xb2,cmd,d[0],d[1],0xb6)
    for t in range(0,self.retries):
      if cmd<0x10: self.expectshort=True
      else:        self.expectans=True
      self.showpacket(packet,name='SEND:',force=self.verbcomsr)
      sleep(self.retrydelay)
      self.clearbuf()
      self.comm.send(packet)
      if self.waitreply(expectshort=(cmd<0x10), retries=self.waitretries*(t+1)): return True
    return False

  def send_atorch_raw(self,cmd,d=[0,0,0,0]): # second byte, d[1], seems to always be 0
    # FF 55 11 <adu> <a2> <a3> 00 <a4> <a5> <checksum>
    for t in range(0,self.retries):
      packet=pack('>BBBBBBBBB',0xff,0x55,0x11,self.ADU, cmd,d[0],d[1],d[2],d[3])
      sum=self.atorch_get_crc(packet[2:])
      packet=packet+pack('>B',sum)
      self.showpacket(packet,name='SEND:',force=self.verbcomsr)
      sleep(self.retrydelay)
      self.clearbuf()
      self.comm.send(packet)
      if self.waitreply(): return True
    return False

  def px100_query(self,cmd,id='',div=1):
    if self.verbcom: print(f'sending PX100 query ({id})',file=stdlog)
    r=self.send_px100cmd_raw(cmd)
    if not r:
      print(f'ERR: no PX100 response ({id})',file=stdlog)
      return None
    if self.packet[0]!=0xCA or self.packet[1]!=0xCB or self.packet[5]!=0xCE or self.packet[6]!=0xCF:
      self.showpacket(self.packet,name=f'ERR: bad PX100 response ({id})',force=True)
      self.packet=[]
      return None
    self.showpacket(self.packet[2:5],name=f'PX100-value ({id})')
    val=getint24(self.packet,2)
    self.packet=[]
    if div!=1: return val/div
    return val


  def float2pair(self,f):
    int1=int(f)
    int2=int((f-int1)*100)
    return [int1,int2]

  def cmd_setcurrent(self,val=0):
    r=self.send_px100cmd_raw(self.CMD_SETCURRENT,self.float2pair(val))
    if not r: print('ERR: cannot send command!',file=stdlog)

  def cmd_setcutoff(self,val=0):
    r=self.send_px100cmd_raw(self.CMD_SETCUTOFF,self.float2pair(val))
    if not r: print('ERR: cannot send command!',file=stdlog)

  def cmd_setonoff(self,val=0):
    r=self.send_px100cmd_raw(self.CMD_ONOFF,[val,0])
    if not r: print('ERR: cannot send command!',file=stdlog)

  def cmd_resetcounters(self):
    r=self.send_px100cmd_raw(self.CMD_RESET)
    if not r: print('ERR: cannot send command!',file=stdlog)

  def cmd_getonoff(self):
    res=self.px100_query(self.CMD_GETONOFF,id='onoff')
    self.out=res
    return res

  def cmd_getvolt(self,div=1000):
    return self.px100_query(self.CMD_GETV,id='setcurrent',div=div)

  def cmd_getamp(self,div=1000):
    return self.px100_query(self.CMD_GETA,id='setcurrent',div=div)

  def cmd_getsetcurrent(self):
    return self.px100_query(self.CMD_GETSETCURRENT,id='setcurrent',div=100)

  def cmd_getsetcutoff(self):
    return self.px100_query(self.CMD_GETSETCUTOFF,id='setcutoff',div=100)

  def cmd_getah(self,div=1000):
    return self.px100_query(self.CMD_GETMAH,id='Ah',div=div)

  def cmd_getwh(self,div=1000):
    return self.px100_query(self.CMD_GETMWH,id='Wh',div=div)

  def cmd_gettemp(self):
    return self.px100_query(self.CMD_GETTEMP,id='temp')

  def cmd_button(self,butt):
    return self.send_atorch_raw(butt,d=[0,0,0,0])


  def cmd_readstate(self,energy=True,limits=True,temp=True,timestr=None,short=True,listenonly=False):
    #return self.state
    a={}

    if timestr!=None: a['time']=timestr
    if not listenonly:
      a['out']=self.cmd_getonoff()
      self.out=a['out']

    a.update(self.state)
    if listenonly: return a

    a['V']=self.cmd_getvolt()
    if short: return a

    a['A']=self.cmd_getamp()
    if energy:
      a['Ah']=self.cmd_getah()
      a['Wh']=self.cmd_getwh()
    if limits:
      a['Iset']=self.cmd_getsetcurrent()
      a['Vcut']=self.cmd_getsetcutoff()
    if temp:
      a['temp']=self.cmd_gettemp()
    return a


  def setOnOff(self,val,verify=True):
    if val!=0 and val!=1: val=0 # todo, error
    for x in range(0,self.retriescmd):
      self.cmd_setonoff(val)
      if not verify: return True
      res=self.cmd_getonoff()
      if res==val: return True
      print(f'ERR: cannot set output, desired={val}, actual={res}')
    print('ERR: output set failed')
    return False

  def setON(self):
    self.setOnOff(1)

  def setOFF(self):
    self.setOnOff(0)

  def setTOGGLE(self):
    if self.cmd_getonoff()==1: self.cmd_setonoff(0)
    else: self.cmd_setonoff(1)


  def setamp(self,val,rel,verify=True):
    if rel: val=val+self.cmd_getsetcurrent()
    val=round(val,2) # for readback
    if val<0: val=0
    if val>CURRENT_LIMIT: val=CURRENT_LIMIT
    for x in range(0,self.retriescmd):
      self.cmd_setcurrent(val)
      if not verify: return True
      res=self.cmd_getsetcurrent()
      if res==val: return True
      print(f'ERR: cannot set current, desired={val}, actual={res}')
    print('ERR: current set failed')
    return False

  def setcutoff(self,val,verify=True):
    val=round(val,2) # for readback
    if val<0: val=0
    if val>255.2: val=255.2
    for x in range(0,self.retriescmd):
      self.cmd_setcutoff(val)
      if not verify: return True
      res=self.cmd_getsetcutoff()
      if res==val: return True
      print(f'ERR: cannot set cutoff voltage, desired={val}, actual={res}')
    print('ERR: cutoff voltage set failed')
    return False

  def printtype(self):
    typeid={1:'AC sensor',2:'DC sensor',3:'DC/USB sensor'}
    while self.instrtype==None:
      self.recvdata()
    print(f'TYPE: {self.instrtype},',typeid[self.instrtype] if self.instrtype in typeid else 'unknown')

  statopts=''
  def printstate(self,opts='',help=False):
    if help:
      print('          opts:  J=JSON, S=short (V/A only), T=show time, U=show UTC time, A=show all, B=force battery, M=minimize queries, L=listen-only');return
    opts=opts.upper()
    if opts=='': opts=self.statopts
    else: self.statopts=opts
    energy=True
    limits=True
    temp=True
    showtime=False
    showtimeutc=False
    json=False
    listenonly=False
    minimize=True
    if 'S' in opts: energy=False;limits=False;temp=False;minimize=False
    if 'A' in opts: energy=True;limits=True;temp=True;minimize=False
    if 'B' in opts: energy=True
    if 'T' in opts: showtime=True
    if 'U' in opts: showtimeutc=True;showtime=True
    if 'J' in opts: json=True
    if 'L' in opts: listenonly=True
    if 'M' in opts: minimize=True
    timestr=None
    if showtime:
      from datetime import datetime
      if showtimeutc: timestr=datetime.utcnow().isoformat()[:23]
      else: timestr=datetime.now().isoformat()[:23]
    a=self.cmd_readstate(energy=energy,limits=limits,temp=temp,timestr=timestr,short=minimize,listenonly=listenonly)
    if json:
      from json import dumps
      print(dumps(a))
    else:
      print(a) # TODO: formatting



###############################
##
##  HIGH LEVEL COMMAND HANDLING
##
###############################



class PowerLoad:

  verbcmd=False
  verbcomm=False
  verbrun=False

  qend='\n'
  #qend=' '
  lastcmd=''

  def __init__(self):
    self.instr=Instr_Atorch()

  strin=''

  # console input with timeout, returns '' when timeout, False when stdin closed
  def timeout_stdin(self,timeout=0.1, default=""):
    inputs, outputs, errors = select([stdin], [], [], timeout)
    if not inputs: return ''
    s=stdin.readline()
    if len(s)==0: return False # stdin closed
    return s.strip()


  # return float and if it is absolute or relative
  def floatrel(self,val):
    rel=False
    if val[0]=='+' or val[0]=='-': rel=True
    f=float(val)
    return f,rel


  def xint(self,n):
    try: return int(n)
    except: return n

  # handle individual command; dryrun only checks validity, help only prints command description
  def handlecommand(self,cmdorig,dryrun,help=False,verb=verbcmd,mainhelp=False,singlelisten=False):
    #if verb or self.verbcmd: print('cmd:',cmdorig,dryrun,file=stdlog)
    if verb or (self.verbcmd and not dryrun): print('CMD:',cmdorig,file=stdlog)
    cmdarr=(cmdorig+':::').split(':') # enforce minimal length of parameters array
    cmd=cmdarr[0].upper()
    self.lastcmd=cmd
    if cmd=='': return True

    # help
    elif cmd in ['HELP','LIST','?','-H','--HELP']:
      if help: print(' HELP            list commands');return False
      if mainhelp:
        self.helpcommands()
        exit(0)
      elif not dryrun: self.helpcommands()
    elif cmd=='CFGFILE':
      if help: print('  CFGFILE        generate config file template to stdout');return False
      self.makecfgfile()
    elif cmd=='-':
      if help or not dryrun: print();return True

    elif cmd[:4]=='TCP=':
      if help: print('  TCP=addr[:port]           set connection via TCP');return False
      if dryrun:
        a=(cmdorig[4:]+':'+str(DEFAULT_TCPPORT)).split(':')
        self.conf['host']=a[0]
        self.conf['port']=int(a[1])
        for x in ['serport','baudrate']:
          if x in self.conf: self.conf.pop(x)

    elif cmd[:5]=='PORT=':
      if help: print('  PORT=/dev/ttyport[@baud]  set connection via serial port');return False
      if dryrun:
        a=(cmdorig[5:]+'@'+str(DEFAULT_BAUDRATE)).split('@')
        self.conf['serport']=DEFAULT_SERPORT if a[0]=='' else a[0]
        self.conf['baudrate']=a[1]
        for x in ['host','port']:
          if x in self.conf: self.conf.pop(x)


    # single-word queries
    elif cmd=='QV':
      if help: print('  QV             query actual voltage');return False
      if not dryrun: print(self.instr.cmd_getvolt(),end=self.qend)
    elif cmd=='QA' or cmd=='QI':
      if help: print('  QA             query actual current');return False
      if not dryrun: print(self.instr.cmd_getamp(),end=self.qend)
#    elif cmd=='QBV' or cmd=='QVB':
#      if help: print('  QBV            query battery voltage');return False
#      if not dryrun: self.instr.sync();print(self.instr.getreg('V_BAT')/self.instr.vmult,end=self.qend)
    elif cmd=='QMV':
      if help: print('  QMV            query actual voltage, integer millivolts');return False
      if not dryrun: print(self.instr.cmd_getvolt(div=1),end=self.qend)
    elif cmd=='QMA':
      if help: print('  QMA            query actual current, integer milliamps');return False
      if not dryrun: print(self.instr.cmd_getamp(div=1),end=self.qend)
    elif cmd=='QVCUT':
      if help: print('  QVCUT          query cutoff voltage');return False
      if not dryrun: print(self.instr.cmd_getsetcutoff(),end=self.qend)
    elif cmd=='QOUT':
      if help: print('  QOUT           query output state');return False
      if not dryrun: print(self.instr.cmd_getonoff(),end=self.qend)
#    elif cmd=='QTE':
#      if help: print('  QTE            query external temperature');return False
#      if not dryrun: self.instr.sync();print(self.instr.getreg('EXT_C'),end=self.qend)
    elif cmd=='QTI':
      if help: print('  QTI            query internal temperature');return False
      if not dryrun: print(self.instr.cmd_gettemp(),end=self.qend)

    # energy counters
    elif cmd=='RESET':
      if help: print('  RESET          reset energy counters');return False
      if not dryrun: self.instr.cmd_resetcounters()
    elif cmd=='QAH':
      if help: print('  QAH            query amp-hour counter');return False
      if not dryrun: print(self.instr.cmd_getah(div=1000),end=self.qend)
    elif cmd=='QMAH':
      if help: print('  QMAH           query amp-hour counter in integer mAh');return False
      if not dryrun: print(self.instr.cmd_getah(div=1),end=self.qend)
    elif cmd=='QWH':
      if help: print('  QWH            query watt-hour counter');return False
      if not dryrun: print(self.instr.cmd_getwh(div=1000),end=self.qend)
    elif cmd=='QMWH':
      if help: print('  QMWH           query watt-hour counter in integer mWh');return False
      if not dryrun: print(self.instr.cmd_getwh(div=1),end=self.qend)


    # output switch control
    elif cmd=='ON':
      if help: print('  ON             enable output');return False
      if not dryrun: self.instr.setON()
    elif cmd=='OFF':
      if help: print('  OFF            disable output');return False
      if not dryrun: self.instr.setOFF()
    elif cmd=='TOGGLE':
      if help: print('  TOGGLE         toggle output');return False
      if not dryrun: self.instr.setTOGGLE()


    # volt/amp settings
    elif cmd[-4:]=='VCUT':
      if help: print('  nn.nnVCUT      set cutoff voltage');return False
      try: val,rel=self.floatrel(cmd[:-4])
      except: print('[Unknown voltage to set:',cmd,']',file=stderr);return False
      if rel: print('[No relative value for voltage cutoff!]');return False
      if not dryrun: self.instr.setcutoff(val)
    elif cmd[-2:]=='MA':
      if help: print('  nn.nnMA        set output current');return False
      try: val,rel=self.floatrel(cmd[:-2])
      except: print('[Unknown current to set:',cmd,']',file=stderr);return False
      if not dryrun: self.instr.setamp(val/1000,rel=rel)
    elif cmd[-1:]=='A':
      if help: print('  nn.nnA         set output current');return False
      try: val,rel=self.floatrel(cmd[:-1])
      except: print('[Unknown current to set:',cmd,']',file=stderr);return False
      if not dryrun: self.instr.setamp(val,rel=rel)


    # take commands from stdin, help-only here
    elif cmd in ['STDIN']:
      if help: print('  STDIN          read commands from stdin');return False
      #self.instr.bat=True

    elif cmd=='LISTEN':
      if help: print('  LISTEN[:opts[:count]]  listen to status reports, query data, handle stdin')
      if help: print('  LISTEN[:opts[:off]]    listen, until off')
      if help: self.instr.printstate(help=True);return False
      stopoff=False;cnt=-1
      if cmdarr[2]=='': pass
      elif cmdarr[2].upper()=='OFF': stopoff=True
      else:
        try: cnt=int(cmdarr[2])
        except: print('[Unknown count:',cmdorig,']');return False
        if cmdarr[2].upper()=='OFF': stopoff=True
      if not dryrun:
        while True:
          if not singlelisten:
            s=self.timeout_stdin()
            if s:
              #print('STDIN:',s)
              if s.upper()[:6]=='LISTEN': print('ERR: cannot recurse LISTEN',file=stderr);continue
              if self.handlecommand(s,True): self.handlecommand(s,False) # check validity, execute if good
          self.instr.recvdata()
          if self.instr.gotupdate():
            #print(self.instr.state,self.instr.cmd_readstate())
            #d=self.instr.state.copy()
            #d.update(self.instr.cmd_readstate())
            #print(d)
            self.instr.printstate(opts=cmdarr[1])
            if singlelisten: break
            cnt-=1
            if cnt==0: break
            if (stopoff or self.instr.stopoff) and self.instr.out==0: break

    # output formatting
    elif cmd in ['LINE','ONELINE']:
      if help: print('  LINE           output the Q-queries as space-separated instead of newline-separated');return False
      self.qend=' '


    elif cmd in ['WAIT']:
      if help: print('  WAIT           wait for communication from device');return False

    elif cmd in ['ROBUST']:
      if help: print('  ROBUST         increase timeouts and retries');return False
      self.instr.robust=True

    elif cmd in ['OFFOFF']:
      if help: print('  OFFOFF         switch output off on program exit');return False
      self.instr.offoff=True

    elif cmd in ['STOPOFF']:
      if help: print('  STOPOFF        stop loop on output off');return False
      self.instr.stopoff=True

    # verbosity
    elif cmd in ['VERB']:
      if help: print('  VERB[:opts]    list operations')
      if help: self.setverb(help=True);return False
      if not dryrun: self.setverb(opts=cmdarr[1])

    # force delay in processing
    elif cmd[:5]=='SLEEP':
      if help: print('  SLEEPxx        sleep for xx seconds');return False
      s=cmd[5:]+cmdarr[1]
      if s!='':
        try: secs=float(s)
        except: print('['+cmd+': Unknown delay to set:',s,']');return False
      else: secs=1
      if not dryrun:
        for x in range(0,int(secs/0.1)):
          sleep(0.1)
          self.instr.recvdata()

    # perform loop, help and validation only here
    elif cmd=='LOOP':
      if help: print('  LOOP:[xx]      loop for xx time or endless if not specified');return False
      no=cmdarr[1]
      if no!='':
        try: int(no)
        except: print('[Unknown loops count:',cmd,' seen as "'+no+'" ]');return False

    # show device type
    elif cmd=='TYPE':
      if help: print('  TYPE           print detected device type');return False
      if not dryrun: self.instr.printtype()

    # show device type
    elif cmd=='BUTTON':
      if help: print('  BUTTON:[+-SO]  press button (Plus, Minus, Setup, OnOff)');return False
      butt=cmdarr[1][:1].upper()
      if   butt=='+': b=self.instr.BUTTON_PLUS
      elif butt=='-': b=self.instr.BUTTON_MINUS
      elif butt=='S': b=self.instr.BUTTON_SETUP
      elif butt=='O': b=self.instr.BUTTON_ONOFF
      else: print('[Unknown button code:'+cmdorig+' ]');return False
      if not dryrun: self.instr.cmd_button(b)

    elif cmd=='RAWPROTO':
      if help: print('  RAWPROTO:xx[:xx:xx:xx:xx]   raw Atorch protocol send, cmd + 4 payloads');return False
      try: barr=bytearray.fromhex((cmdorig[9:]+' 00 00 00 00 00').replace(':',' '))[:5]
      except: print('[cannot interpret hex string: "'+cmdorig+'" ]');return False
      if not dryrun: self.instr.send_atorch_raw(barr[0],d=barr[1:])

    elif cmd=='RAWPX100':
      if help: print('  RAWPX100:xx[:xx:xx]         raw PX100 protocol send, cmd + 2 payloads');return False
      try: barr=bytearray.fromhex((cmdorig[9:]+' 00 00 00').replace(':',' '))[:3]
      except: print('[cannot interpret hex string: "'+cmdorig+'" ]');return False
      if not dryrun: self.instr.send_px100cmd_raw(barr[0],d=barr[1:])

    elif cmd=='RAWSEND':
      if help: print('  RAWSEND:xx[:xx:xx:...]      raw serial protocol data send');return False
      try: barr=bytearray.fromhex(cmdorig[8:].replace(':',' '))
      except: print('[cannot interpret hex string: "'+cmdorig+'" ]');return False
      if not dryrun:
        self.instr.showpacket(barr,name='SEND:',force=self.instr.verbcomsr)
        self.instr.comm.send(barr)
        sleep(0.1)
        self.instr.waitreply(retries=30)

    elif cmd=='NORETRY':
      if help: print('  NORETRY                     do not retry timeouted commands');return False
      self.instr.retriescmd=1
      self.instr.retries=1

    elif cmd in ['STATE','STAT','STATUS']:
      if help: print('  STATE[:opts]   print setting state in JSON format')
      if not dryrun: self.instr.printstate(opts=cmdarr[1])

    elif cmd in ['JSTATE','JSTAT','JSTATUS','STATEJ','STATJ','STATUSJ']:
      if help: print('  STATEJ[:opts]  print setting state in JSON format, like opts=J')
      if help: self.instr.printstate(help=True);return False
      if not dryrun: self.instr.printstate(opts='J'+cmdarr[1])
      #if self.instr.stopoff and self.instr.out==0: 

    else: print('[Unknown command:',cmdorig,']',file=stderr);return False

    return True



  # list help of commands
  def helpcommands(self):
    helparr=['ON','OFF',
             '-','xVCUT','xMA','xA',
             '-','QV','QMV','QA','QMA','QTI','QVCUT',
             '-','QAH','QMAH','QWH','QMWH','RESET',
             '-','STAT','JSTAT','LISTEN',
             '-','TCP=','PORT=','WAIT','ROBUST','OFFOFF','STOPOFF',
             '-','STDIN','LOOP:','SLEEP','VERB','LINE','TYPE','CFGFILE',
             '-','RAWPROTO','RAWPX100','RAWSEND','NORETRY']
    print('Atorch DL24 artificial control')
    print('Usage:',argv[0],'<command> [command]...')
    print('Commands:')
    print()
    for x in helparr: self.handlecommand(x.upper(),dryrun=True,help=True)
    print()
    print('For volt and amp setting, prefixing the value with + or - marks it as relative, to be added/subtracted to the current value')
    print('Commands are executed in sequence. Writes are cached and grouped together to minimize bus transactions.')
    print('Commands are case-insensitive.')
    print('Command "-" forces a newline into output.')


  # dryrun of commands list
  def verifycommands(self,cmds):
    ok=True
    for x in cmds:
      if not self.handlecommand(x,True,mainhelp=True): ok=False
    return ok

  # dryrun of commands to check validity, then run live
  def handlecommands(self,cmds):
    #print("COMMANDS:",cmds)
#    for x in cmds: self.handlecommand(x,False)
    for t in range(0,len(cmds)):
      cmd=cmds[t].upper()
      #print('CMD:',cmd)
      counter=-1 # endless
      # recursive running of loop
      if cmd[:5]=='LOOP:':
        if len(cmd)>5: counter=int(cmd[5:]) # error check already done in dryrun parse
        while counter!=0:
          counter-=1
          self.handlecommands(cmds[t+1:])
          #if counter==0: break
        break

      # from now, everything comes from stdin
      elif cmd=='STDIN':
        #print('STDIN')
        while True:
          self.instr.recvdata()
          s=self.timeout_stdin()
          if s==False: break # pipe closed
          if s!='':
            #print('STDIN:',s)
            #if s.upper()[:6]=='LISTEN': print('ERR: cannot recurse LISTEN',file=stderr);continue
            if s.upper()=='STDIN':      print('ERR: cannot recurse STDIN',file=stderr);continue
            if self.handlecommand(s,True): self.handlecommand(s,False,singlelisten=True) # check validity, execute if good

      else:
        if self.qend==' ' and cmd[:5]=='SLEEP': print()
        self.handlecommand(cmd,False,verb=False)







  conf={}
  configfilename=None

  def getprocessbarename(self):
    cmdn=('/'+argv[0]).split('/')[-1]
    if cmdn[-3:]=='.py': cmdn=cmdn[:-3]
    return cmdn

  # generate configuration file name from running file name or from name or direct filename
  # TODO: windows compatibility
  def setconfigfilename(self,name=None,filename=None):
    if filename==None:
      if name==None: name=self.getprocessbarename()
      filename='~/.'+name+'.cfg'
    self.configfilename=filename

  # output configfile template to stdout
  def makecfgfile(self):
    if self.configfilename==None: self.setconfigfilename()
    print('# config file goes to '+self.configfilename)
    print("""
# physical serial TTY
# serport=/dev/ttyUSB0
# baudrate=115200
## wait for communication from device before sending data
# waitcomm=1

# plain TCP socket
# host=dt24p.local
# port=8888

# physical port takes precedence if both are defined
""")
    exit(0)

  # integer from string, with error handling
  def cfgint(self,s,msg='Configfile error:',default=-1):
    try: return int(s)
    except:
      print(msg,'Invalid value "'+s+'"',file=stderr)
      print('substituting',default,file=stderr)
      return default

  def isparm(self,parm):
    for x in argv:
      if x.upper()==parm.upper(): return True
    return False

  # read configfile - name=process name, filename=direct config file name
  def readconf(self,name=None,filename=None):
    from os.path import expanduser
    self.setconfigfilename(name=name,filename=filename)
    fn=expanduser(self.configfilename)
    if self.verbrun: print('CONFIGFILE:filename:',fn,file=stdlog)
    try:
      with open(fn,'r') as f:
        all=f.read().split('\n')
        for s in all:
          if '#' in s: continue
          if '=' not in s: continue
          a=(s+'=').split('=')
          self.conf[a[0].strip()]=a[1].strip()
    except Exception as e:
      if self.verbrun: print('CONFIGFILE:FAIL:',e,file=stderr)
      return False
    if self.verbrun: print('CONFIGFILE:',self.conf,file=stdlog)
    return True


  # initialize port from configuration in self.conf
  def initport(self):
    #global comm,port,self.instr
    if 'serport' in self.conf:
      port=self.conf['serport']
      baud=DEFAULT_BAUDRATE if 'baudrate' not in self.conf else self.cfgint(self.conf['baudrate'],default=DEFAULT_BAUDRATE)
      self.instr.initport(LowLevelSerPort(port,baud))
      self.default_minimize=False
    elif 'host' in self.conf:
      host=self.conf['host']
      port=DEFAULT_TCPPORT if 'port' not in self.conf else self.cfgint(self.conf['port'],default=DEFAULT_TCPPORT)
      self.instr.initport(LowLevelTcpPort(host,port))
      self.default_minimize=True
    else:
      print('ERROR: unknown serial port or TCP host',file=stderr)
      print('Use TCP=<host>[:port] or PORT=[/dev/tty...]',file=stderr)
      exit(1)



  def setverb(self,opts='',help=False):
    def setverbopts(opts):
      #if opts=='': opts='L'
      pload.verbrun=True
      if 'C' in opts: self.instr.verbcomsr=True
      if 'D' in opts: self.instr.verbcom=True
      if 'M' in opts: self.verbcmd=True
      if self.instr.comm!=None:
        if 'P' in opts: self.instr.comm.verbconn=True;pload.instr.comm.verbport=True

    if help:
      print('          opts:  P=port, C=communication, D=dataflow, M=commands');return
    if opts!='': setverbopts(opts);return
    for x in argv:
      if x.upper()[:4]=='VERB':
        a=(x.upper()+':').split(':')
        setverbopts(a[1])


#######################################
##
##  user code, no more objects
##
#######################################


if __name__=="__main__":
  cmds=argv[1:]

  pload=PowerLoad()
  pload.setverb() # set verbose flags
  pload.readconf() # set config file

  if len(cmds)==1 and (cmds[0][:4].upper()=='TCP=' or cmds[0][:5].upper()=='PORT='): cmds.append('STATE')
  elif cmds==[]: cmds=['STATE']

  if not pload.verifycommands(cmds):
    print(cmds)
    print('Command error.',file=stderr)
    exit(1)

  # now we read the config, processed parameters by a dry run, and know the port to use
  pload.initport()
  pload.setverb() # set verbose flags again, now for port


  #if pload.isparm('VERB'):    pload.instr.verblnk=True
  #if pload.isparm('VERBCOM'): pload.instr.verbcom=True
  #if pload.isparm('VERBSR'):  pload.instr.verbcomsr=True
  #if pload.isparm('VERBPORT'): pload.instr.comm.verbconn=True;pload.instr.comm.verbport=True

  #if pload.instr.verblnk: print('opening port',file=stdlog)
  pload.instr.connect()
  if pload.isparm('WAIT') or ('waitcomm' in pload.conf and pload.conf['waitcomm']=='1'):
    if pload.verbrun:
      print('waiting for incoming data',file=stdlog)
    while not pload.instr.gotupdate(): sleep(0.05);pload.instr.recvdata()

  if False:
    pload.handlecommands(cmds)
    pload.instr.close() # no exception catching, for debug
  else:
    try:
      pload.handlecommands(cmds)
    except KeyboardInterrupt: pass # clean break
    finally:
      if pload.lastcmd[:5]=='SLEEP' and pload.qend==' ': print()
      pload.instr.close()


