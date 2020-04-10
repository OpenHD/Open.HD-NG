
import serial
import socket
import threading
import select
import queue
import logging

from libc.errno cimport errno, EINTR, EINVAL
from libc.string cimport memset, memcpy, strerror
from libc.stdlib cimport malloc, calloc, free
from libc.stdint cimport uint8_t, uint16_t, uint32_t, uint64_t, int8_t, int16_t, int32_t, int64_t
from libc.stddef cimport size_t

cdef extern from 'mavlink/common/mavlink.h':
    pass

cdef extern from 'mavlink/mavlink_types.h':
    cdef struct __mavlink_message:
        uint16_t checksum
        uint8_t magic
        uint8_t len
        uint8_t incompat_flags
        uint8_t compat_flags
        uint8_t seq
        uint8_t sysid
        uint8_t compid
        uint32_t msgid
        uint64_t payload64
        uint8_t *ck;
        uint8_t *signature
    ctypedef __mavlink_message mavlink_message_t
    cdef struct mavlink_signing_t:
        pass
    cdef struct mavlink_signing_streams_t:
        pass
    ctypedef enum mavlink_parse_state_t:
        MAVLINK_PARSE_STATE_UNINIT "MAVLINK_PARSE_STATE_UNINIT"
        MAVLINK_PARSE_STATE_IDLE "MAVLINK_PARSE_STATE_IDLE"
        MAVLINK_PARSE_STATE_GOT_STX "MAVLINK_PARSE_STATE_GOT_STX"
        MAVLINK_PARSE_STATE_GOT_LENGTH "MAVLINK_PARSE_STATE_GOT_LENGTH"
        MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS "MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS"
        MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS "MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS"
        MAVLINK_PARSE_STATE_GOT_SEQ "MAVLINK_PARSE_STATE_GOT_SEQ"
        MAVLINK_PARSE_STATE_GOT_SYSID "MAVLINK_PARSE_STATE_GOT_SYSID"
        MAVLINK_PARSE_STATE_GOT_COMPID "MAVLINK_PARSE_STATE_GOT_COMPID"
        MAVLINK_PARSE_STATE_GOT_MSGID1 "MAVLINK_PARSE_STATE_GOT_MSGID1"
        MAVLINK_PARSE_STATE_GOT_MSGID2 "MAVLINK_PARSE_STATE_GOT_MSGID2"
        MAVLINK_PARSE_STATE_GOT_MSGID3 "MAVLINK_PARSE_STATE_GOT_MSGID3"
        MAVLINK_PARSE_STATE_GOT_PAYLOAD "MAVLINK_PARSE_STATE_GOT_PAYLOAD"
        MAVLINK_PARSE_STATE_GOT_CRC1 "MAVLINK_PARSE_STATE_GOT_CRC1"
        MAVLINK_PARSE_STATE_GOT_BAD_CRC1 "MAVLINK_PARSE_STATE_GOT_BAD_CRC1"
        MAVLINK_PARSE_STATE_SIGNATURE_WAIT "MAVLINK_PARSE_STATE_SIGNATURE_WAIT"
    ctypedef enum mavlink_channel_t:
        MAVLINK_COMM_0 "MAVLINK_COMM_0"
        MAVLINK_COMM_1 "MAVLINK_COMM_1"
        MAVLINK_COMM_2 "MAVLINK_COMM_2"
        MAVLINK_COMM_3 "MAVLINK_COMM_3"
    cdef struct __mavlink_status:
        uint8_t msg_received
        uint8_t buffer_overrun
        uint8_t parse_error
        mavlink_parse_state_t parse_state
        uint8_t packet_idx
        uint8_t current_rx_seq
        uint8_t current_tx_seq
        uint16_t packet_rx_success_count
        uint16_t packet_rx_drop_count
        uint8_t flags
        uint8_t signature_wait
        mavlink_signing_t *signing
        mavlink_signing_streams_t *signing_streams
    ctypedef __mavlink_status mavlink_status_t

cdef extern from 'mavlink/protocol.h':
    cdef uint8_t mavlink_parse_char(uint8_t chan, uint8_t c, mavlink_message_t* r_message,
                                    mavlink_status_t* r_mavlink_status)
    cdef uint16_t mavlink_msg_get_send_buffer_length(mavlink_message_t* msg)
    cdef uint16_t mavlink_msg_to_send_buffer(uint8_t *buffer, mavlink_message_t *msg)

cdef extern from 'mavlink/common/mavlink_msg_radio_status.h':
    cdef struct __mavlink_radio_status_t:
        uint16_t rxerrors
        uint16_t fixed
        uint8_t rssi
        uint8_t remrssi
        uint8_t txbuf
        uint8_t noise
        uint8_t remnoise
    ctypedef __mavlink_radio_status_t mavlink_radio_status_t
    cdef uint16_t mavlink_msg_radio_status_pack(uint8_t system_id, uint8_t component_id,
                                                mavlink_message_t* msg,
                                                uint8_t rssi, uint8_t remrssi, uint8_t txbuf,
                                                uint8_t noise, uint8_t remnoise, uint16_t rxerrors,
                                                uint16_t fixed)
    cdef uint16_t mavlink_msg_radio_status_encode(uint8_t system_id, uint8_t component_id,
                                                  mavlink_message_t* msg, const mavlink_radio_status_t* radio_status)

cdef class MavlinkMsg:
    cdef mavlink_message_t m_msg
    cdef mavlink_status_t m_status

    def __init__(self, sysid=1, compid=1):
        self.m_msg.sysid = sysid
        self.m_msg.compid = compid

    def len(self):
        return self.m_msg.len

    def seq(self):
        return self.m_msg.seq

    def msgid(self):
        return self.m_msg.msgid

    def sysid(self):
        return self.m_msg.sysid

    def compid(self):
        return self.m_msg.compid

    def parse_ch(self, c):
        if mavlink_parse_char(MAVLINK_COMM_0, c, &self.m_msg, &self.m_status) != 0:
            return True
        return False

    def serialize(self):
        # Get the length of the output message
        msglen = mavlink_msg_get_send_buffer_length(&self.m_msg)
        # Allocate a bytearry to store the message
        arr = bytearray(msglen)
        # Encode the message into the buffer
        mavlink_msg_to_send_buffer(arr, &self.m_msg)
        return arr

cdef class RadioStatusMsg(MavlinkMsg):

    def __init__(self, sysid, compid, rssi, remrssi, txbuf, noise, remnoise, rxerrors, fixed):
        super().__init__(sysid, compid)
        mavlink_msg_radio_status_pack(sysid, compid, &self.m_msg, rssi, remrssi, txbuf,
                                      noise, remnoise, rxerrors, fixed)

class Mavlink:

    def __init__(self, sysid=0, compid=0):
        self.m_msg = MavlinkMsg()
        self.sysid = sysid
        self.compid = compid

    def get_sysid(self):
        return self.sysid

    def compid(self):
        return self.comp_id

    def parse_ch(self, uint8_t c):
        if self.m_msg.parse_char(c):
            ret = [self.m_msg]
            self.m_msg = MavlinkMsg()
            return ret
        return []

    def parse_buf(self, buf):
        ret = []
        for c in buf:
            if self.m_msg.parse_ch(c):
                ret.append(self.m_msg)
                self.m_msg = MavlinkMsg()
        return ret


class SerialEndpoint:
    
    def __init__(self, out_queue, uart, baudrate, timeout=0.01, blocksize=1024, max_queue_size=100):
        threading.Thread.__init__(self)
        self.mav = Mavlink()
        self.out_queue = out_queue
        self.send_queue = queue.Queue(maxsize=max_queue_size)
        self.uart = uart
        self.ser = serial.Serial(uart, baudrate, timeout=timeout)
        self.blocksize = blocksize
        self.sysid = 0

        # Start the reader and writer threads
        self.running = True
        self.reader = threading.Thread(target=self.read_thread)
        self.reader.start()
        self.writer = threading.Thread(target=self.write_thread)
        self.writer.start()

    def queue_msg(self, msg):
        self.send_queue.put(msg)

    def join(self):
        self.reader.join()

    def terminate(self):
        self.running = False

    def read_thread(self):
        while self.running:
            try:
                buf = self.ser.read(self.blocksize)
            except:
                buf = None
            for msg in self.mav.parse_buf(buf):
                if msg is not None:
                    logging.debug("Read message from UART: %s of length %d" % (self.uart, msg.len()))
                    if self.sysid != msg.sysid():
                        self.sysid = msg.sysid()
                        logging.info("New connection from id: %d on UART: %s",
                                     msg.sysid(), self.uart)
                    self.out_queue.put(msg)

    def write_thread(self):
        while self.running:
            try:
                msg = self.send_queue.get(timeout=1)
                self.ser.write(msg.serialize())
            except:
                pass # Timeout?

class UDPEndpoint(object):
    
    def __init__(self, out_queue, local_host, local_port, target_host, target_port,
                 bufsize=1024, max_queue_size=100):
        threading.Thread.__init__(self)
        self.mav = Mavlink()
        self.out_queue = out_queue
        self.send_queue = queue.Queue(maxsize=max_queue_size)
        self.local_host = local_host
        self.local_port = int(local_port)
        self.target_host = target_host
        self.target_port = int(target_port)
        self.sysid = 0

        # Create and initialize the socket as required
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

        # Default the socket to broadcast
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        # If a local port wasn't specified, find one that's available
        if self.local_port == 0:
            self.local_port = self.target_port + 1
            while True:
                try:
                    self.sock.bind((self.local_host, self.local_port))
                except:
                    continue
                break
        else:
            try:
                self.sock.bind((self.local_host, self.local_port))
            except:
                logging.fatal("Error binding to local port: (%s:%d)" % \
                              (self.local_host, self.local_port))
            self.sock.setblocking(0)
        logging.debug("Bound to: %s:%d" % (self.local_host, self.local_port))

        # Start the reader and writer threads
        self.running = True
        self.reader = threading.Thread(target=self.read_thread)
        self.reader.start()
        self.writer = threading.Thread(target=self.write_thread)
        self.writer.start()

    def queue_msg(self, msg):
        self.send_queue.put(msg)

    def join(self):
        self.reader.join()

    def terminate(self):
        self.running = False

    def read_thread(self):

        while self.running:

            # Wait until a message is received so we don't block for too long
            ready = select.select([self.sock], [], [], 1)
            if ready[0]:

                # Receive the message
                data, addr = self.sock.recvfrom(1024)
                logging.debug("Received patcket: (length=%d)" % (len(data)))

                # Parse the message
                for msg in self.mav.parse_buf(data):

                    # Relay the message
                    if msg is not None:

                        # Is this a new connection?
                        if self.sysid != msg.sysid():
                            self.sysid = msg.sysid()
                            logging.info("New connection from id: %d on UDP (%s:%d) from UDP (%s:%d)",
                                         msg.sysid(), self.local_host, self.local_port, addr[0], addr[1])

                            # We should ensure that we're sending to that host
                            self.target_host = addr[0]
                            self.target_port = addr[1]

                            # Turn off socket
                            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 0)

                        # Pass on the message to the router
                        self.out_queue.put(msg)

    def write_thread(self):
        while self.running:
            try:
                msg = self.send_queue.get(timeout=1)
                if self.target_host != '' and self.target_port > 0:
                    buf = msg.serialize()
                    logging.debug("Sending message of length %d to (%s:%d)",
                                  len(buf), self.target_host, self.target_port)
                    self.sock.sendto(buf, (self.target_host, self.target_port))
            except:
                pass # Timeout?


class UDPSource(UDPEndpoint):
    '''
    A UDP endpoint that broadcasts to the specified port until a response
    message is received, then it switches to sending directly to the host/ip
    of the sender
    '''

    def __init__(self, out_queue, source_port, bufsize=1024, max_queue_size=100):
        super().__init__(out_queue, '0.0.0.0', 0, '<broadcast>', source_port)


class UDPSink(UDPEndpoint):
    '''
    A UDP endpoint that waits to receive a message on the specified port, then
    uses the address of the received message to communicate back with the sender
    '''

    def __init__(self, out_queue, sink_port, sink_host='0.0.0.0', bufsize=1024, max_queue_size=100):
        super().__init__(out_queue, sink_host, sink_port, '', 0)


class Router:

    def __init__(self, max_queue_size=500):
        self.endpoints = []
        self.queue = queue.Queue(maxsize=500)

        # Start the processing thread
        self.running = True
        self.process = threading.Thread(target=self.process_thread)
        self.process.start()

    def join(self):
        self.process.join()

    def terminate(self):
        self.running = False

    def get_queue(self):
        return self.queue

    def put_queue(self, msg):
        self.queue.put(msg)

    def add_endpoint(self, endpoint):
        self.endpoints.append(endpoint)

    def process_thread(self):
        while self.running:
            msg = None
            try:
                msg = self.queue.get(timeout=1)
            except:
                pass # Timeout?
            if msg is not None:
                self.route_message(msg)

    def route_message(self, msg):
        # At this point we will just broadcast all messages to everyone but the sender.
        for ep in self.endpoints:
            if ep.sysid != msg.sysid:
                ep.queue_msg(msg)
