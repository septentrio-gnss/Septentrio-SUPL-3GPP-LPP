#include "sbf.hpp"
#include <fcntl.h>
#include <iostream>
#include <termio.h>
#include <unistd.h>
/**
 * Calculate buffer CRC16
 */
uint16_t crc16(const uint8_t* data_p, uint32_t length) {
    uint8_t  x;
    uint16_t crc = 0;

    while (length--) {
        x = crc >> 8 ^ *data_p++;
        x ^= x >> 4;
        crc = static_cast<uint16_t>((crc << 8) ^ (x << 12) ^ (x << 5) ^ x);
    }

    return crc;
}

/**
 * Add payload rx byte
 */
// -1 = error, 0 = ok, 1 = payload completed
int SBF_parse::payloadRxAdd(const uint8_t b) {
    int      ret   = 0;
    uint8_t* p_buf = reinterpret_cast<uint8_t*>(&_buf);

    p_buf[_rx_payload_index++] = b;

    if ((_rx_payload_index > 7 && _rx_payload_index >= _buf.length) ||
        _rx_payload_index >= sizeof(_buf)) {
        ret = 1;  // payload received completely
    }

    return ret;
}

/**
 * Finish payload rx
 */
// 0 = no message handled, 1 = message handled, 2 = sat info message handled
int SBF_parse::payloadRxDone() {
    SBF_Information sbf_data = get_sbf_data();

    int ret = 0;

    if (_buf.length <= 4 || _buf.length > _rx_payload_index ||
        _buf.crc16 != crc16(reinterpret_cast<uint8_t*>(&_buf) + 4, _buf.length - 4)) {
        return 0;
    }

    // handle message
    switch (_buf.msg_id) {
    case SBF_ID_PVTGeodetic:
        _msg_status |= 1;

        sbf_data.altitude            = _buf.payload_pvt_geodetic.height;
        sbf_data.latitude            = _buf.payload_pvt_geodetic.latitude;
        sbf_data.longitude           = _buf.payload_pvt_geodetic.longitude;
        sbf_data.horizontal_accuracy = _buf.payload_pvt_geodetic.h_accuracy;
        sbf_data.vertical_accuracy   = _buf.payload_pvt_geodetic.v_accuracy;

        sbf_data.ve = _buf.payload_pvt_geodetic.ve;
        sbf_data.vn = _buf.payload_pvt_geodetic.vn;
        sbf_data.vu = _buf.payload_pvt_geodetic.vu;

        sbf_data.nr_sv = _buf.payload_pvt_geodetic.nr_sv;

        switch (_buf.payload_pvt_geodetic.mode_type) {
        default:
        case 0: sbf_data.fixq = SBF_FIX::INVALID; break;
        case 1: sbf_data.fixq = SBF_FIX::STANDALONE; break;
        case 2: sbf_data.fixq = SBF_FIX::DGPS_FIX; break;
        case 4: sbf_data.fixq = SBF_FIX::RTK_FIX; break;
        case 5: sbf_data.fixq = SBF_FIX::RTK_FLOAT; break;
        }

        break;

    case SBF_ID_VelCovGeodetic:
        _msg_status |= 2;
        sbf_data.cov_ve_ve = _buf.payload_vel_col_geodetic.cov_ve_ve;
        sbf_data.cov_vn_vn = _buf.payload_vel_col_geodetic.cov_vn_vn;
        sbf_data.cov_vu_vu = _buf.payload_vel_col_geodetic.cov_vu_vu;

        break;

    case SBF_ID_DOP:
        _msg_status |= 4;
        sbf_data.nr_sv = _buf.payload_dop.nr_sv;
        sbf_data.hDOP  = _buf.payload_dop.hDOP;
        sbf_data.pDOP  = _buf.payload_dop.pDOP;
        sbf_data.vDOP  = _buf.payload_dop.vDOP;
        sbf_data.tDOP  = _buf.payload_dop.tDOP;

        break;

    case SBF_ID_AttEuler:
        sbf_data.heading = _buf.payload_att_euler.heading;
        sbf_data.nr_sv   = _buf.payload_att_euler.nr_sv;
        break;

    case SBF_ID_AttCovEuler: break;

    default: break;
    }


    set_sbf_data(sbf_data);

    return ret;
}

// 0 = decoding, 1 = message handled, 2 = sat info message handled
int SBF_parse::parseChar(const uint8_t b) {
    int ret = 0;

    switch (_decode_state) {
    // Expecting Sync1
    case SBF_DECODE_SYNC1:
        if (b == SBF_SYNC1) {  // Sync1 found --> expecting Sync2

            payloadRxAdd(b);   // add a payload byte
            _decode_state = SBF_DECODE_SYNC2;
        }

        break;

    // Expecting Sync2
    case SBF_DECODE_SYNC2:
        if (b == SBF_SYNC2) {  // Sync2 found --> expecting CRC

            payloadRxAdd(b);   // add a payload byte
            _decode_state = SBF_DECODE_PAYLOAD;
        } else {               // Sync1 not followed by Sync2: reset parser
            decodeInit();
        }

        break;

    // Expecting payload
    case SBF_DECODE_PAYLOAD:

        ret = payloadRxAdd(b);  // add a payload byte

        if (ret < 0) {
            // payload not handled, discard message
            ret = 0;
            decodeInit();
        } else if (ret > 0) {
            ret = payloadRxDone();  // finish payload processing
            decodeInit();
        } else {
            // expecting more payload, stay in state SBF_DECODE_PAYLOAD
            ret = 0;
        }

        break;

    default: break;
    }

    return ret;
}

void SBF_parse::decodeInit() {
    _decode_state     = SBF_DECODE_SYNC1;
    _rx_payload_index = 0;
}

int SBF_parse::start_receive(const std::string serial_port) {
    _serialFd = open(serial_port.data(), O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (_serialFd == -1) {
        perror("Error opening serial port");
        return -1;
    }

    struct termios serialOptions;
    tcgetattr(_serialFd, &serialOptions);

    cfsetispeed(&serialOptions, B115200);       // Set input baud rate
    cfsetospeed(&serialOptions, B115200);       // Set output baud rate

    serialOptions.c_cflag |= (CLOCAL | CREAD);  // Enable receiver and set local mode

    // Set serial port settings
    serialOptions.c_cflag &= ~PARENB;  // No parity
    serialOptions.c_cflag &= ~CSTOPB;  // 1 stop bit
    serialOptions.c_cflag &= ~CSIZE;
    serialOptions.c_cflag |= CS8;      // 8 data bits

    tcsetattr(_serialFd, TCSANOW, &serialOptions);

    // start thread
    receive_running = true;
    receive_thread  = std::thread(&SBF_parse::receive_worker, this);

    return 0;
}

int SBF_parse::receive_worker() {
    fd_set readFds;
    char   buffer[4048];

    while (receive_running) {
        FD_ZERO(&readFds);
        FD_SET(_serialFd, &readFds);

        // Set timeout
        struct timeval timeout;
        timeout.tv_sec  = 1;  // Timeout seconds
        timeout.tv_usec = 0;  // Timeout microseconds

        int ready = select(_serialFd + 1, &readFds, NULL, NULL, &timeout);

        if (ready == -1) {
            receive_running = false;
            return -1;
        } else if (ready > 0) {
            if (FD_ISSET(_serialFd, &readFds)) {
                ssize_t bytesRead = read(_serialFd, buffer, sizeof(buffer));
                if (bytesRead > 0) {
                    for (int i = 0; i < bytesRead; i++) {
                        parseChar(buffer[i]);
                    }
                }
            }
        }
    }

    return 0;
}

SBF_parse::~SBF_parse() {
    receive_running = false;
    receive_thread.join();

    if (_serialFd >= 0) {
        close(_serialFd);
    }
}

SBF_Information SBF_parse::get_sbf_data() {
    const std::lock_guard<std::mutex> lock(_sbf_data_mutex);
    return this->current_sbf_info;
};

void SBF_parse::set_sbf_data(SBF_Information SBF_data) {
    const std::lock_guard<std::mutex> lock(_sbf_data_mutex);
    this->current_sbf_info = SBF_data;
};