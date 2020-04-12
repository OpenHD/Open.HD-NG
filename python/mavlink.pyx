
import time
import serial
import socket
import threading
import select
import queue
import logging

import numpy as np

from libcpp cimport bool
from libcpp.string cimport string
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

cdef extern from 'wifibroadcast/transfer_stats.hh':
    ctypedef struct transfer_stats_t:
        transfer_stats_t(uint32_t _sequences, uint32_t _blocks_in, uint32_t _blocks_out,
                         uint32_t _bytes_in, uint32_t _bytes_out, uint32_t _block_errors,
                         uint32_t _sequence_errors, uint32_t _inject_errors,
                         float _encode_time, float _send_time, float _pkt_time,
                         float _latency, float _rssi)
        uint32_t sequences
        uint32_t blocks_in
        uint32_t blocks_out
        uint32_t sequence_errors
        uint32_t block_errors
        uint32_t inject_errors
        uint32_t bytes_in
        uint32_t bytes_out
        float encode_time
        float send_time
        float pkt_time
        float latency
        float rssi
    cdef cppclass TransferStats:
        TransferStats(string name)
        string name()
        transfer_stats_t get_stats()
        bool update(string s)
        void timeout()
        string serialize()
    ctypedef struct wifi_adapter_rx_status_forward_t:
        uint32_t received_packet_cnt
        int8_t current_signal_dbm
        int8_t type
        int8_t signal_good
    ctypedef struct wifibroadcast_rx_status_forward_t:
        uint32_t damaged_block_cnt
        uint32_t lost_packet_cnt
        uint32_t skipped_packet_cnt
        uint32_t injection_fail_cnt
        uint32_t received_packet_cnt
        uint32_t kbitrate
        uint32_t kbitrate_measured
        uint32_t kbitrate_set
        uint32_t lost_packet_cnt_telemetry_up
        uint32_t lost_packet_cnt_telemetry_down
        uint32_t lost_packet_cnt_msp_up
        uint32_t lost_packet_cnt_msp_down
        uint32_t lost_packet_cnt_rc
        int8_t current_signal_joystick_uplink
        int8_t current_signal_telemetry_uplink
        int8_t joystick_connected
        float HomeLat
        float HomeLon
        uint8_t cpuload_gnd
        uint8_t temp_gnd
        uint8_t cpuload_air
        uint8_t temp_air
        uint32_t wifi_adapter_cnt
        wifi_adapter_rx_status_forward_t *adapter

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
        MavlinkMsg.__init__(self,sysid, compid)
        mavlink_msg_radio_status_pack(sysid, compid, &self.m_msg, rssi, remrssi, txbuf,
                                      noise, remnoise, rxerrors, fixed)

class Mavlink:

    def __init__(self, sysid=0, compid=0):
        self.m_msg = MavlinkMsg()
        self.sysid = sysid
        self.compid = compid
        self.peers = []

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

class Endpoint(object):

    def __init__(self, name, max_queue_size):
        self.name = name
        self.send_queue = queue.Queue(maxsize=max_queue_size)
        self.outputs = []
        self.peers = {}
        self.lock = threading.Lock()

    def queue_msg(self, msg):
        self.send_queue.put(msg)

    def add_route(self, to_endpoint):
        self.outputs.append(to_endpoint)

    def output_msg(self, msg):
        for endpoint in self.outputs:
            endpoint.queue_msg(msg)

    def log_peer(self, sysid):
        self.lock.acquire()
        ret = sysid in self.peers
        self.peers[sysid] = time.time()
        self.lock.release()
        return ret

    def timeout_peers(self, timeout):
        self.lock.acquire()
        timedout = [sysid for sysid in self.peers if (time.time() - self.peers[sysid]) > timeout]
        for sysid in timedout:
            del self.peers[sysid]
        self.lock.release()
        return timedout

class SerialEndpoint(Endpoint):
    
    def __init__(self, name, uart, baudrate, timeout=0.01, blocksize=1024, max_queue_size=100):
        Endpoint.__init__(self, name, max_queue_size)
        threading.Thread.__init__(self)
        self.mav = Mavlink()
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

    def join(self):
        self.reader.join()
        self.writer.join()

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
                    logging.debug("(%s) Read message %d from UART: %s of length %d  sysid: %d %d" %
                                  (self.name, msg.msgid(), self.uart, msg.len(), msg.sysid(), self.sysid))
                    if not self.log_peer(msg.sysid()):
                        logging.info("(%s) New connection from id: %d on UART: %s" % (msg.sysid(), self.uart))
                    self.output_msg(msg)
            for p in self.timeout_peers(10):
                logging.info("(%s) Timeout connection from id: %d on " % (p))

    def write_thread(self):
        while self.running:
            try:
                msg = self.send_queue.get(timeout=1)
                self.ser.write(msg.serialize())
            except:
                pass # Timeout?

class UDPEndpoint(Endpoint):
    
    def __init__(self, name, local_host, local_port, target_host, target_port,
                 bufsize=1024, max_queue_size=100):
        Endpoint.__init__(self, name, max_queue_size)
        threading.Thread.__init__(self)
        self.mav = Mavlink()
        self.send_queue = queue.Queue(maxsize=max_queue_size)
        self.local_host = local_host
        self.local_port = int(local_port)
        self.target_host = target_host
        self.target_port = int(target_port)
        self.orig_target_host = target_host
        self.orig_target_port = int(target_port)
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
                logging.fatal("(%s) Error binding to local port: (%s:%d)" % \
                              (self.name, self.local_host, self.local_port))
            self.sock.setblocking(0)
        logging.debug("(%s) Bound to: %s:%d" % (self.name, self.local_host, self.local_port))

        # Start the reader and writer threads
        self.running = True
        self.reader = threading.Thread(target=self.read_thread)
        self.reader.start()
        self.writer = threading.Thread(target=self.write_thread)
        self.writer.start()

    def join(self):
        self.reader.join()
        self.writer.join()

    def terminate(self):
        self.running = False

    def read_thread(self):

        # Keep track of when the last message was received so we can switch broadcast back on
        last_msg_time = 0
        while self.running:

            # Turn broadcast back on if we timeout (10 seconds)
            if last_msg_time != 0 and (time.time() - last_msg_time) > 10 and \
               (self.target_host != self.orig_target_host or self.target_port != self.orig_target_port):
                logging.info("(%s) Turning broadcast back on after timeout" % (self.name))
                self.target_host = self.orig_target_host
                self.target_port = self.orig_target_port
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                
            # Wait until a message is received so we don't block for too long
            ready = select.select([self.sock], [], [], 1)
            if ready[0]:

                # Receive the message
                data, addr = self.sock.recvfrom(1024)
                #logging.debug("(%s) Received patcket: (length=%d)" % (self.name, len(data)))

                # Parse the message
                for msg in self.mav.parse_buf(data):

                    # Relay the message
                    if msg is not None:
                        last_msg_time = time.time()
                        if not self.log_peer(msg.sysid()):
                            logging.info("(%s) New connection from id: %d" % (self.name, msg.sysid()))

                        # We should ensure that we're sending to that host and not broadcasting
                        if self.target_host == "" or self.target_host == "<broadcast>":
                            logging.info("(%s) Turning off broadcast (%s:%d) -> (%s:%d)" %
                                         (self.name, self.target_host, self.target_port, addr[0], addr[1]))
                            self.target_host = addr[0]
                            self.target_port = addr[1]
                            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 0)

                        # Pass on the message to the router
                        self.output_msg(msg)
                        
            for p in self.timeout_peers(10):
                logging.info("(%s) Timeout connection from id: %d" % (self.name,
                                                                      p))

    def write_thread(self):
        while self.running:
            try:
                msg = self.send_queue.get(timeout=1)
                if self.target_host != '' and self.target_port > 0:
                    buf = msg.serialize()
                    logging.debug("(%s) Sending message %d of length %d to (%s:%d)",
                                  self.name, msg.msgid(), len(buf), self.target_host, self.target_port)
                    self.sock.sendto(buf, (self.target_host, self.target_port))
            except:
                pass # Timeout?


class UDPSource(UDPEndpoint):
    '''
    A UDP endpoint that broadcasts to the specified port until a response
    message is received, then it switches to sending directly to the host/ip
    of the sender
    '''

    def __init__(self, name, source_port, bufsize=1024, max_queue_size=100):
        UDPEndpoint.__init__(self, name, '0.0.0.0', 0, '<broadcast>', source_port, bufsize=bufsize,
                             max_queue_size=max_queue_size)


class UDPSink(UDPEndpoint):
    '''
    A UDP endpoint that waits to receive a message on the specified port, then
    uses the address of the received message to communicate back with the sender
    '''

    def __init__(self, name, sink_port, sink_host='0.0.0.0', bufsize=1024, max_queue_size=100):
        UDPEndpoint.__init__(self, name, sink_host, sink_port, '', 0,
                             bufsize=bufsize, max_queue_size=max_queue_size)


class WFBStatusEndpoint(Endpoint):
    '''
    Receive and parse wifibroadcast status messages
    and generate mavlink radio status messages from them
    '''

    def __init__(self, name, host='', port=5800, max_queue_size=100):
        Endpoint.__init__(self, name, max_queue_size)
        threading.Thread.__init__(self)
        self.mav = Mavlink()
        self.send_queue = queue.Queue(maxsize=max_queue_size)
        self.host = host
        self.port = port
        self.sysid = 1 # We're pretending to be a flight controller

        # Create and initialize the socket as required
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

        # Default the socket to broadcast
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        # Start the reader thread
        self.running = True
        self.reader = threading.Thread(target=self.read_thread)
        self.reader.start()
        self.writer = threading.Thread(target=self.write_thread)
        self.writer.start()

        # Bind to the status receive port
        try:
            self.sock.bind((self.host, self.port))
        except:
            logging.fatal("(%s) Error binding to local port: (%s:%d)" % (self.name, self.host, self.port))
            self.sock.setblocking(0)
        logging.debug("Bound to: %s:%d" % (self.host, self.port))

    def queue_msg(self, msg):
        self.send_queue.put(msg)

    def join(self):
        self.reader.join()
        self.reader.join()

    def terminate(self):
        self.running = False

    def read_thread(self):
        cdef wifibroadcast_rx_status_forward_t stat;
        cdef char *statp = <char*>&stat;

        while self.running:

            # Wait until a message is received so we don't block for too long
            ready = select.select([self.sock], [], [], 1)
            if ready[0]:

                # Receive the message
                data, addr = self.sock.recvfrom(1024)
                logging.debug("(%s) Received patcket: (length=%d %d from %s)" %
                              (self.name, len(data), sizeof(wifibroadcast_rx_status_forward_t), str(addr)))

                # Parse the message
                if len(data) == sizeof(wifibroadcast_rx_status_forward_t):

                    # Create the mavlink radio status message
                    for i in range(len(data)):
                        statp[i] = data[i]
                    rssi = min(max(0, 2.5 * (stat.adapter[0].current_signal_dbm + 80), 0), 255)
                    remrssi = min(max(0, 2.5 * (stat.current_signal_telemetry_uplink + 80), 0), 100)
                    remrssi = 100
                    radio_status = RadioStatusMsg(1, 1, rssi, remrssi, 100, 10, 10,
                                                  stat.damaged_block_cnt, stat.lost_packet_cnt)

                    # Pass on the message to the router
                    self.output_msg(radio_status)

    def write_thread(self):
        '''The write thread just empties the queue'''
        while self.running:
            try:
                msg = self.send_queue.get(timeout=1)
            except:
                pass # Timeout?
