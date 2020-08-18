
from ctypes import *
import logging
import os
import numpy as np
import time



path = os.path.dirname(__file__)
logging.basicConfig()
logger = logging.getLogger('PyHackRf')
logger.setLevel(logging.DEBUG)


libhackrf = CDLL('/usr/local/lib/libhackrf.so.0')


def enum(*sequential, **named):
    enums = dict(zip(sequential, range(len(sequential))), **named)
    return type('Enum', (), enums)


HackRfVendorRequest = enum(
    HACKRF_VENDOR_REQUEST_SET_TRANSCEIVER_MODE=1,
    HACKRF_VENDOR_REQUEST_MAX2837_WRITE=2,
    HACKRF_VENDOR_REQUEST_MAX2837_READ=3,
    HACKRF_VENDOR_REQUEST_SI5351C_WRITE=4,
    HACKRF_VENDOR_REQUEST_SI5351C_READ=5,
    HACKRF_VENDOR_REQUEST_SAMPLE_RATE_SET=6,
    HACKRF_VENDOR_REQUEST_BASEBAND_FILTER_BANDWIDTH_SET=7,
    HACKRF_VENDOR_REQUEST_RFFC5071_WRITE=8,
    HACKRF_VENDOR_REQUEST_RFFC5071_READ=9,
    HACKRF_VENDOR_REQUEST_SPIFLASH_ERASE=10,
    HACKRF_VENDOR_REQUEST_SPIFLASH_WRITE=11,
    HACKRF_VENDOR_REQUEST_SPIFLASH_READ=12,
    HACKRF_VENDOR_REQUEST_CPLD_WRITE=13,
    HACKRF_VENDOR_REQUEST_BOARD_ID_READ=14,
    HACKRF_VENDOR_REQUEST_VERSION_STRING_READ=15,
    HACKRF_VENDOR_REQUEST_SET_FREQ=16,
    HACKRF_VENDOR_REQUEST_AMP_ENABLE=17,
    HACKRF_VENDOR_REQUEST_BOARD_PARTID_SERIALNO_READ=18,
    HACKRF_VENDOR_REQUEST_SET_LNA_GAIN=19,
    HACKRF_VENDOR_REQUEST_SET_VGA_GAIN=20,
    HACKRF_VENDOR_REQUEST_SET_TXVGA_GAIN=21)

HackRfConstants = enum(
    LIBUSB_ENDPOINT_IN=0x80,
    LIBUSB_ENDPOINT_OUT=0x00,
    HACKRF_DEVICE_OUT=0x40,
    HACKRF_DEVICE_IN=0xC0,
    HACKRF_USB_VID=0x1d50,
    HACKRF_USB_PID=0x6089)

HackRfError = enum(
    HACKRF_SUCCESS=0,
    HACKRF_TRUE=1,
    HACKRF_ERROR_INVALID_PARAM=-2,
    HACKRF_ERROR_NOT_FOUND=-5,
    HACKRF_ERROR_BUSY=-6,
    HACKRF_ERROR_NO_MEM=-11,
    HACKRF_ERROR_LIBUSB=-1000,
    HACKRF_ERROR_THREAD=-1001,
    HACKRF_ERROR_STREAMING_THREAD_ERR=-1002,
    HACKRF_ERROR_STREAMING_STOPPED=-1003,
    HACKRF_ERROR_STREAMING_EXIT_CALLED=-1004,
    HACKRF_ERROR_OTHER=-9999,
    # Python defaults to returning none
    HACKRF_ERROR=None)

HackRfTranscieverMode = enum(
    HACKRF_TRANSCEIVER_MODE_OFF=0,
    HACKRF_TRANSCEIVER_MODE_RECEIVE=1,
    HACKRF_TRANSCEIVER_MODE_TRANSMIT=2)


HackRfsweep_style = enum(
    HACKRF_LINEAR=0,
    HACKRF_INTERLEAVED=1)


# Data structures
_libusb_device_handle = c_void_p
_pthread_t = c_ulong

p_hackrf_device = c_void_p


class hackrf_transfer(Structure):
    _fields_ = [("device", p_hackrf_device),
                ("buffer", POINTER(c_byte)),
                ("buffer_length", c_int),
                ("valid_length", c_int),
                ("rx_ctx", c_void_p),
                ("tx_ctx", c_void_p)]


class read_partid_serialno_t(Structure):
    _fields_ = [("part_id", c_uint32*2),
                ("serial_no", c_uint32*4)]


class hackrf_device_list_t(Structure):
    _fields_ = [("serial_numbers", POINTER(c_char_p)),
                ("usb_board_ids", c_void_p),
                ("usb_device_index", POINTER(c_int)),
                ("devicecount", c_int),
                ("usb_devices", POINTER(c_void_p)),
                ("usb_devicecount", c_int)]



_callback = CFUNCTYPE(c_int, POINTER(hackrf_transfer))


def read_samples_cb_wrapper(hackrf_transfer):

    # let's access the contents
    c = hackrf_transfer.contents

    # c.device is an int representing the pointer to the hackrf device
    # we can get the pointer with p_hackrf_device(c.device)
    this_hackrf = HackRF._hackrf_dict[c.device]
    this_hackrf.read_samples_cb(c)
    return 0


def wrap_lib_call_function(lib, funcname, restype, argtypes):
    """Simplify wrapping ctypes functions"""
    func = lib.__getattr__(funcname)
    func.restype = restype
    func.argtypes = argtypes
    return func


class HackRF(object):

    _center_freq = 100e6
    _sample_rate = 20e6
    device_opened = False

    # dictionary containing all hackrf_devices in use
    _hackrf_dict = dict()

    fnct_hackrf_device_list = wrap_lib_call_function(libhackrf, "hackrf_device_list", POINTER(hackrf_device_list_t), [])
    fnct_hackrf_init = wrap_lib_call_function(libhackrf, "hackrf_init", c_int, [])
    fnct_hackrf_exit = wrap_lib_call_function(libhackrf, "hackrf_exit", c_int, [])
    fnct_hackrf_open = wrap_lib_call_function(libhackrf, "hackrf_open", c_int, [POINTER(p_hackrf_device)])
    fnct_hackrf_open_by_serial = wrap_lib_call_function(libhackrf, "hackrf_open_by_serial", c_int, [POINTER(p_hackrf_device)])
    fnct_hackrf_device_list_open = wrap_lib_call_function(libhackrf, "hackrf_device_list_open", c_int, [POINTER(hackrf_device_list_t), c_int, POINTER(p_hackrf_device)])
    fnct_hackrf_close = wrap_lib_call_function(libhackrf, "hackrf_close", c_int, [p_hackrf_device])
    fnct_hackrf_set_sample_rate = wrap_lib_call_function(libhackrf, "hackrf_set_sample_rate", c_int, [p_hackrf_device, c_double])
    fnct_hackrf_set_sample_rate_manual = wrap_lib_call_function(libhackrf, "hackrf_set_sample_rate_manual", c_int, [p_hackrf_device, c_uint32, c_uint32])
    fnct_hackrf_set_lna_gain = wrap_lib_call_function(libhackrf, "hackrf_set_lna_gain", c_int, [p_hackrf_device, c_uint32])
    fnct_hackrf_set_vga_gain = wrap_lib_call_function(libhackrf, "hackrf_set_vga_gain", c_int, [p_hackrf_device, c_uint32])
    fnct_hackrf_start_rx = wrap_lib_call_function(libhackrf, "hackrf_start_rx", c_int, [p_hackrf_device, _callback, c_void_p])
    fnct_hackrf_stop_rx = wrap_lib_call_function(libhackrf, "hackrf_stop_rx", c_int, [p_hackrf_device])
    fnct_hackrf_is_streaming = wrap_lib_call_function(libhackrf, "hackrf_is_streaming", c_int, [p_hackrf_device])
    fnct_hackrf_set_freq = wrap_lib_call_function(libhackrf, "hackrf_set_freq", c_int, [p_hackrf_device, c_uint64])
    fnct_hackrf_set_amp_enable = wrap_lib_call_function(libhackrf, "hackrf_set_amp_enable", c_int, [p_hackrf_device, c_uint8])
    fnct_hackrf_board_partid_serialno_read = wrap_lib_call_function(libhackrf, "hackrf_board_partid_serialno_read", c_int, [p_hackrf_device, POINTER(read_partid_serialno_t)])
    fnct_hackrf_set_baseband_filter_bandwidth = wrap_lib_call_function(libhackrf, "hackrf_set_baseband_filter_bandwidth", c_int, [p_hackrf_device, c_uint32])
    fnct_hackrf_init_sweep = wrap_lib_call_function(libhackrf, "hackrf_init_sweep", c_int, [p_hackrf_device, POINTER(c_uint16), c_int, c_uint32, c_uint32, c_uint32])
    fnct_hackrf_start_rx_sweep = wrap_lib_call_function(libhackrf, "hackrf_start_rx_sweep", c_int, [p_hackrf_device, _callback, c_void_p])
    fnct_hackrf_is_streaming = wrap_lib_call_function(libhackrf, "hackrf_is_streaming", c_int, [p_hackrf_device])
    fnct_hackrf_library_version = wrap_lib_call_function(libhackrf, "hackrf_library_version", c_char_p, [])
    fnct_hackrf_library_release = wrap_lib_call_function(libhackrf, "hackrf_library_release", c_char_p, [])
    fnct_hackrf_usb_api_version_read=wrap_lib_call_function(libhackrf,"hackrf_usb_api_version_read",c_int,[p_hackrf_device,POINTER(c_uint16)])

    rs_callback = _callback(read_samples_cb_wrapper)

    @staticmethod
    def get_dict():
        return HackRF._hackrf_dict

    @staticmethod
    def hackrf_device_list():
        devices = HackRF.fnct_hackrf_device_list()
        return devices

    @staticmethod
    def list_devices():
        devices = HackRF.fnct_hackrf_device_list()
        logger.debug("Serial Number:{}".format(devices.contents.serial_numbers.contents.value))

    @staticmethod
    def library_version():
        version=HackRF.fnct_hackrf_library_version()
        return version

    @staticmethod
    def library_release():
        release=HackRF.fnct_hackrf_library_release()
        return release
        
    
    def __init__(self, device_index=0):
        self.open(device_index)

        self.disable_amp()
        self.set_lna_gain(16)
        self.set_vga_gain(16)

        self.buffer = bytearray()
        self.num_bytes = 16*262144

    def open(self, device_index=0):
        self.dev_p = p_hackrf_device(None)

        hdl = HackRF.fnct_hackrf_device_list()
        result = HackRF.fnct_hackrf_device_list_open(
            hdl, device_index, pointer(self.dev_p))
        if result != 0:
            raise IOError('Error code %d when opening HackRF' % (result))

        HackRF._hackrf_dict[self.dev_p.value] = self
        self.device_opened = True

    def close(self):
        if not self.device_opened:
            return

        HackRF.fnct_hackrf_close(self.dev_p)
        self.device_opened = False

    def __del__(self):
        logger.debug("del function is being called")
        self.close()

    def read_samples_cb(self, c):
        """read samples call back function"""
        if len(self.buffer) == self.num_bytes:
            self.still_sampling = False
            return 0


        if len(self.buffer) > self.num_bytes:
            self.still_sampling = False
            self.buffer = self.buffer[0:self.num_bytes]
            return 0

     
        values = cast(c.buffer, POINTER(c_byte*c.buffer_length)).contents
        self.buffer = self.buffer + bytearray(values)
        return 0


    def read_samples(self, num_samples=131072, sleep_time=0.05):
        """read samples"""
        num_bytes = 2*num_samples
        self.num_bytes = int(num_bytes)

        self.buffer = bytearray()

        # start receiving
        result = HackRF.fnct_hackrf_start_rx(self.dev_p, HackRF.rs_callback, None)
        if result != 0:
            raise IOError("Error in hackrf_start_rx")
        self.still_sampling = True     

        while self.still_sampling:
            if sleep_time:
                time.sleep(sleep_time)

       
        result = HackRF.fnct_hackrf_stop_rx(self.dev_p)
        if result != 0:
            raise IOError("Error in hackrf_stop_rx")

      
        iq = HackRF.bytes2iq(self.buffer)

        return iq

    
    def set_freq(self, freq):
        """set the center frequency [hz]"""
        freq = int(freq)
        result = HackRF.fnct_hackrf_set_freq(self.dev_p, freq)
        if result != 0:
            raise IOError('Error code %d when setting frequency to %d Hz'
                          % (result, freq))

        self._center_freq = freq
        return

    def get_freq(self):
        return self._center_freq

    def set_sample_rate(self, rate):
        """set the sample rate [hz]"""
        result = HackRF.fnct_hackrf_set_sample_rate(self.dev_p, rate)
        if result != 0:
            # TODO: make this error message better
            raise IOError('Sample rate set failure')
        self._sample_rate = rate
        return

    def get_sample_rate(self):
        return self._sample_rate

    def set_sample_rate_manual(self, freq_hz, divider):
        return HackRF.fnct_hackrf_set_sample_rate_manual(self.dev_p, freq_hz, divider)

    def set_baseband_filter_bandwidth(self, bw_hz):
        HackRF.fnct_hackrf_set_baseband_filter_bandwidth(self.dev_p, bw_hz)

    def get_serial_no(self):
        return HackRF.get_serial_no_by_dev(self.dev_p)

    def enable_amp(self):
        result = HackRF.fnct_hackrf_set_amp_enable(self.dev_p, 1)
        if result != 0:
            raise IOError("error enabling amp")
        return 0

    def disable_amp(self):
        result = HackRF.fnct_hackrf_set_amp_enable(self.dev_p, 0)
        if result != 0:

            raise IOError("error disabling amp")
        return 0

    def set_lna_gain(self, gain):
        """set_lna_gain:rounds down to multiple of 8 (15 -> 8, 39 -> 32), etc.
            internally, hackrf_set_lna_gain does the same thing
            But we take care of it so we can keep track of the correct gain"""
        gain -= (gain % 8)    # round DOWN to multiple of 8
        result = HackRF.fnct_hackrf_set_lna_gain(self.dev_p, gain)
        if result != 0:
            raise IOError("error setting lna gain")
        self._lna_gain = gain
        logger.debug("LNA gain set to {} dB".format(gain))
        return 0

    def get_lna_gain(self):
        return self._lna_gain

    def set_vga_gain(self, gain):
        """set vga gain"""
        gain -= (gain % 2)
        result = HackRF.fnct_hackrf_set_vga_gain(self.dev_p, gain)
        if result != 0:
            raise IOError("error setting vga gain")
        self._vga_gain = gain
        logger.debug("VGA gain set to {} dB".format(gain))
        return 0

    def get_vga_gain(self):
        return self._vga_gain



    def start_rx(self, rx_cb_fn):
        rx_cb = _callback(rx_cb_fn)
        result = HackRF.fnct_hackrf_start_rx(self.dev_p, rx_cb, None)
        if result != 0:
            raise IOError("start_rx failure")

    def stop_rx(self):
        result = HackRF.fnct_hackrf_stop_rx(self.dev_p)
        if result != 0:
            raise IOError("stop_rx failure")

    @staticmethod
    def bytes2iq(data):
        """converts byte array to iq values"""
        values = np.array(data).astype(np.int8)
        iq = values.astype(np.float64).view(np.complex128)
        iq /= 127.5
        iq -= (1 + 1j)

        return iq

    @staticmethod
    def get_serial_no_by_dev(dev_p):
        """
        returns serial number as a string
        it is too big to be a single number, so make it a string
        the returned string matches the hackrf_info output
        """
        sn = read_partid_serialno_t()
        result = HackRF.fnct_hackrf_board_partid_serialno_read(dev_p, sn)
        if result != 0:
            raise IOError("Error %d while getting serial number" % (result))

        sn_str = ""
        for i in xrange(0, 4):
            sni = sn.serial_no[i]
            if sni == 0:
                sn_str += "00000000"
            else:
                sn_str += hex(sni)[2:-1]

        return sn_str

    def init_sweep(self, frequency_list, num_ranges, num_bytes, step_width, offset, sweep_style):
        """
        Initialize sweep mode:
        frequency_list is a list of start/stop pairs of frequencies in MHz.
        num_ranges is the number of pairs in frequency_list (1 to 10)
        num_bytes is the number of sample bytes to capture after each tuning.
        step_width is the width in Hz of the tuning step.
        offset is a number of Hz added to every tuning frequency.
        Use to select center frequency based on the expected usable bandwidth.
        sweep_mode
        LINEAR means step_width is added to the current frequency at each step.
        INTERLEAVED invokes a scheme in which each step is divided into two
        interleaved sub-steps, allowing the host to select the best portions
        of the FFT of each sub-step and discard the rest.
        """

        array_type = c_uint16*len(frequency_list)

        return HackRF.fnct_hackrf_init_sweep(self.dev_p, array_type(*frequency_list), num_ranges, num_bytes, int(step_width), offset, sweep_style)

    def start_rx_sweep(self, rx_cb_fn):
        fnct_cb = _callback(rx_cb_fn)
        return HackRF.fnct_hackrf_start_rx_sweep(self.dev_p, fnct_cb, None)

    def is_streaming(self):
        return HackRF.fnct_hackrf_is_streaming(self.dev_p)


    def api_version(self):
        version=c_uint16(0) 
        res=HackRF.fnct_hackrf_usb_api_version_read(self.dev_p,version)
        return int(version.value)

    # properties
    sample_rate = property(get_sample_rate, set_sample_rate)
    center_freq = property(get_freq, set_freq)
    lna_gain = property(get_lna_gain, set_lna_gain)
    vga_gain = property(get_vga_gain, set_vga_gain)


# really, user shouldn't have to call this function at all
result = HackRF.fnct_hackrf_init()
if result != 0:
    logger.debug("error initializing the hackrf library")



if __name__=="__main__":
    print(HackRF.library_version())
    print(HackRF.library_release())
    