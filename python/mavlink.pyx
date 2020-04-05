
import serial
import socket
import threading
import select
import queue

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


cdef class MavlinkMsg:
    cdef mavlink_message_t m_msg
    cdef mavlink_status_t m_status

    def len(self):
        return self.m_msg.len

    def seq(self):
        return self.m_msg.seq

    def sysid(self):
        return self.m_msg.sysid

    def compid(self):
        return self.m_msg.compid

    def msgid(self):
        return self.m_msg.msgid

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

class Mavlink:

    def __init__(self):
        self.m_msg = MavlinkMsg()

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
        self.ser = serial.Serial(uart, baudrate, timeout=timeout)
        self.blocksize = blocksize
        self.sysid = 0

        # Start the reader and writer threads
        self.running = True
        self.reader = threading.Thread(target=self.read_thread)
        self.reader.start()
        self.writer = threading.Thread(target=self.write_thread)
        self.writer.start()

    def get_sysid(self):
        return self.sysid

    def queue_msg(self, msg):
        self.send_queue.put(msg)

    def join(self):
        self.reader.join()

    def terminate(self):
        self.running = False

    def read_thread(self):
        while self.running:
            buf = self.ser.read(self.blocksize)
            for msg in self.mav.parse_buf(buf):
                if msg is not None:
                    self.sysid = msg.sysid()
                    self.out_queue.put(msg)

    def write_thread(self):
        while self.running:
            try:
                msg = self.send_queue.get(timeout=1)
                self.ser.write(msg.serialize())
            except:
                pass # Timeout?

class UDPEndpoint:
    
    def __init__(self, out_queue, source_host, source_port, target_host, target_port,
                 bufsize=1024, max_queue_size=100):
        threading.Thread.__init__(self)
        self.mav = Mavlink()
        self.out_queue = out_queue
        self.send_queue = queue.Queue(maxsize=max_queue_size)
        self.target_host = target_host
        self.target_port = target_port
        self.sysid = 0

        # Create and initialize the socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((source_host, source_port))
        self.sock.setblocking(0)

        # Start the reader and writer threads
        self.running = True
        self.reader = threading.Thread(target=self.read_thread)
        self.reader.start()
        self.writer = threading.Thread(target=self.write_thread)
        self.writer.start()

    def get_sysid(self):
        return self.sysid

    def queue_msg(self, msg):
        self.send_queue.put(msg)

    def join(self):
        self.reader.join()

    def terminate(self):
        self.running = False

    def read_thread(self):
        while self.running:
            ready = select.select([self.sock], [], [], 1)
            if ready[0]:
                data, addr = self.sock.recvfrom(1024)
                for msg in self.mav.parse_buf(data):
                    if msg is not None:
                        self.sysid = msg.sysid()
                        self.out_queue.put(msg)

    def write_thread(self):
        while self.running:
            try:
                msg = self.send_queue.get(timeout=1)
                self.sock.sendto(msg.serialize(), (self.target_host, self.target_port))
            except:
                pass # Timeout?

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
            if ep.get_sysid() != msg.sysid():
                ep.queue_msg(msg)
