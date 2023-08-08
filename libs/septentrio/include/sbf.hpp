#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

enum class SBF_FIX : uint8_t {
    // 0: No PVT available (the Error field indicates the cause of the absence of the PVT solution)
    INVALID = 0,
    // 1: Stand-Alone PVT
    STANDALONE = 1,
    // 2: Differential PVT
    DGPS_FIX = 2,
    // 4: RTK with fixed ambiguities
    RTK_FIX = 4,
    // 5: RTK with float ambiguities
    RTK_FLOAT = 5,
};

typedef struct {
    double latitude;

    double longitude;

    double altitude;

    double horizontal_accuracy;

    double vertical_accuracy;

    float vn;        /**< Velocity in the North direction */
    float ve;        /**< Velocity in the East direction */
    float vu;        /**< Velocity in the Up direction */

    float cov_vn_vn; /**< Variance of the north-velocity estimate */
    float cov_ve_ve; /**< Variance of the east-velocity estimate */
    float cov_vu_vu; /**< Variance of the up - velocity estimate */

    double heading;

    std::uint8_t nr_sv; /**< Total number of satellites used in the PVT computation. */

    std::uint16_t pDOP;
    std::uint16_t tDOP;
    std::uint16_t hDOP;
    std::uint16_t vDOP;

    SBF_FIX fixq;

} SBF_Information;

#define SBF_SYNC1 0x24
#define SBF_SYNC2 0x40

/* Block IDs */
#define SBF_ID_DOP 4001
#define SBF_ID_PVTGeodetic 4007
#define SBF_ID_ChannelStatus 4013
#define SBF_ID_GPSUtc 4031
#define SBF_ID_VelCovGeodetic 5908
#define SBF_ID_AttEuler 5938
#define SBF_ID_AttCovEuler 5939

#pragma pack(push, 1)

typedef struct {
    std::uint8_t mode_type : 4;       /**< Bit field indicating the PVT mode type, as follows:
                                      0: No PVT available (the Error field indicates the cause of the
                                      absence of the PVT solution)       1: Stand-Alone PVT       2: Differential PVT
                                      3: Fixed location
                                      4: RTK with fixed ambiguities
                                      5: RTK with float ambiguities
                                      6: SBAS aided PVT
                                      7: moving-base RTK with fixed ambiguities
                                      8: moving-base RTK with float ambiguities
                                      10:Precise Point Positioning (PPP) */
    std::uint8_t mode_reserved : 2;   /**< Reserved */
    std::uint8_t mode_base_fixed : 1; /**< Set if the user has entered the command
                                      setPVTMode,base,auto and the receiver is still in the process
                                      of determining its fixed position. */
    std::uint8_t
        mode_2d : 1; /**< 2D/3D flag: set in 2D mode(height assumed constant and not computed). */
    std::uint8_t error; /**< PVT error code. The following values are defined:
                         0: No Error
                         1: Not enough measurements
                         2: Not enough ephemerides available
                         3: DOP too large (larger than 15)
                         4: Sum of squared residuals too large
                         5: No convergence
                         6: Not enough measurements after outlier rejection
                         7: Position output prohibited due to export laws
                         8: Not enough differential corrections available
                         9: Base station coordinates unavailable
                         10:Ambiguities not fixed and user requested to only output RTK-fixed
                         positions Note: if this field has a non-zero value, all following fields
                         are set to their Do-Not-Use value. */
    double latitude;    /**< Marker latitude, from -PI/2 to +PI/2, positive North of Equator */
    double longitude;   /**< Marker longitude, from -PI to +PI, positive East of Greenwich */
    double
        height; /**< Marker ellipsoidal height (with respect to the ellipsoid specified by Datum) */
    float undulation; /**< Geoid undulation computed from the global geoid model defined in
                              the document 'Technical Characteristics of the NAVSTAR GPS, NATO, June
                         1991' */
    float vn;         /**< Velocity in the North direction */
    float ve;         /**< Velocity in the East direction */
    float vu;         /**< Velocity in the Up direction */
    float cog; /**< Course over ground: this is defined as the angle of the vehicle with respect
                       to the local level North, ranging from 0 to 360, and increasing towards east.
                       Set to the do-not-use value when the speed is lower than 0.1m/s. */
    double rx_clk_bias; /**< Receiver clock bias relative to system time reported in the Time System
                           field. To transfer the receiver time to the system time, use:
                           tGPS/GST=trx-RxClkBias */
    float
        RxClkDrift; /**< Receiver clock drift relative to system time (relative frequency error) */
    std::uint8_t time_system;  /**< Time system of which the offset is provided in this sub-block:
                                0:GPStime
                                1:Galileotime
                                3:GLONASStime */
    std::uint8_t datum;        /**< This field defines in which datum the coordinates are expressed:
                                0: WGS84/ITRS
                                19: Datum equal to that used by the DGNSS/RTK basestation
                                30: ETRS89(ETRF2000 realization)
                                31: NAD83(2011), North American Datum(2011)
                                32: NAD83(PA11), North American Datum, Pacificplate (2011)
                                33: NAD83(MA11), North American Datum, Marianas plate(2011)
                                34: GDA94(2010), Geocentric Datum of Australia (2010)
                                250:First user-defined datum
                                251:Second user-defined datum */
    std::uint8_t nr_sv;        /**< Total number of satellites used in the PVT computation. */
    std::uint8_t wa_corr_info; /**< Bit field providing information about which wide area
                                corrections have been applied: Bit 0: set if orbit and satellite
                                clock correction information is used Bit 1: set if range correction
                                information is used Bit 2: set if ionospheric information is used
                                Bit 3: set if orbit accuracy information is used(UERE/SISA)
                                Bit 4: set if DO229 Precision Approach mode is active
                                Bits 5-7: Reserved */
    std::uint16_t reference_id; /**< In case of DGPS or RTK operation, this field is to be
                                interpreted as the base station identifier. In SBAS operation, this
                                field is to be interpreted as the PRN of the geostationary satellite
                                used (from 120 to 158). If multiple base stations or multiple
                                geostationary satellites are used the value is set to 65534.*/
    std::uint16_t
        mean_corr_age; /**< In case of DGPS or RTK, this field is the mean age of the differential
                       corrections. In case of SBAS operation, this field is the mean age of the
                       'fast corrections' provided by the SBAS satellites */
    std::uint32_t signal_info; /**< Bit field indicating the type of GNSS signals having been used
                               in the PVT computations. If a bit i is set, the signal type having
                               index i has been used. */
    std::uint8_t alert_flag;   /**< Bit field indicating integrity related information */

    // Revision 1
    std::uint8_t  nr_bases;
    std::uint16_t ppp_info;
    // Revision 2
    std::uint16_t latency;
    std::uint16_t h_accuracy;
    std::uint16_t v_accuracy;
} sbf_payload_pvt_geodetic_t;

typedef struct {
    unsigned char mode_type : 4;       /**< Bit field indicating the PVT mode type, as follows:
                                      0: No PVT available (the Error field indicates the cause of the
                                      absence of the PVT solution)       1: Stand-Alone PVT       2: Differential PVT
                                      3: Fixed location
                                      4: RTK with fixed ambiguities
                                      5: RTK with float ambiguities
                                      6: SBAS aided PVT
                                      7: moving-base RTK with fixed ambiguities
                                      8: moving-base RTK with float ambiguities
                                      10:Precise Point Positioning (PPP) */
    unsigned char mode_reserved : 2;   /**< Reserved */
    unsigned char mode_base_fixed : 1; /**< Set if the user has entered the command
                                       setPVTMode,base,auto and the receiver is still in the process
                                       of determining its fixed position. */
    unsigned char
        mode_2d : 1; /**< 2D/3D flag: set in 2D mode(height assumed constant and not computed). */
    std::uint8_t error; /**< PVT error code. The following values are defined:
                         0: No Error
                         1: Not enough measurements
                         2: Not enough ephemerides available
                         3: DOP too large (larger than 15)
                         4: Sum of squared residuals too large
                         5: No convergence
                         6: Not enough measurements after outlier rejection
                         7: Position output prohibited due to export laws
                         8: Not enough differential corrections available
                         9: Base station coordinates unavailable
                         10:Ambiguities not fixed and user requested to only output RTK-fixed
                         positions Note: if this field has a non-zero value, all following fields
                         are set to their Do-Not-Use value. */
    float cov_vn_vn;    /**< Variance of the north-velocity estimate */
    float cov_ve_ve;    /**< Variance of the east-velocity estimate */
    float cov_vu_vu;    /**< Variance of the up - velocity estimate */
    float cov_dt_dt;    /**< Variance of the clock drift estimate */
    float cov_vn_ve;    /**< Covariance between the north - and east - velocity estimates */
    float cov_vn_vu;    /**< Covariance between the north - and up - velocity estimates */
    float cov_vn_dt;    /**< Covariance between the north - velocity and clock drift estimates */
    float cov_ve_vu;    /**< Covariance between the east - and up - velocity estimates */
    float cov_ve_dt;    /**< Covariance between the east - velocity and clock drift estimates */
    float cov_vu_dt;    /**< Covariance between the up - velocity and clock drift estimates */
} sbf_payload_vel_cov_geodetic_t;

typedef struct {
    std::uint8_t  nr_sv; /**< Total number of satellites used in the PVT computation. */
    std::uint8_t  reserved;
    std::uint16_t pDOP;
    std::uint16_t tDOP;
    std::uint16_t hDOP;
    std::uint16_t vDOP;
    float         hpl; /**< Horizontal Protection Level (see the DO229 standard). */
    float         vpl; /**< Vertical Protection Level (see the DO229 standard). */
} sbf_payload_dop_t;

typedef struct {
    std::uint8_t  antenna;
    std::uint8_t  reserved;
    std::uint16_t tracking_status;
    std::uint16_t pvt_status;
    std::uint16_t pvt_info;
} sbf_payload_channel_state_info_t;

typedef struct {
    std::uint8_t nr_sv; /**< The average over all antennas of the number of satellites currently
                           included in the attitude calculations. */
    std::uint8_t error_aux1 : 2; /**< Bits 0-1: Error code for Main-Aux1 baseline:
                                    0: No error
                                    1: Not enough measurements
                                    2: Reserved
                                    3: Reserved */
    std::uint8_t error_aux2 : 2; /**< Bits 2-3: Error code for Main-Aux2 baseline, same definition
                                    as bit 0-1. */
    std::uint8_t error_reserved : 3;      /**< Bits 4-6: Reserved */
    std::uint8_t error_not_requested : 1; /**< Bit 7: Set when GNSS-based attitude not requested by
                                             user. In that case, the other bits are all zero. */

    std::uint16_t
        mode;               /**< Attitude mode code:
                               0: No attitude
                               1: Heading, pitch (roll = 0), aux antenna positions obtained with float
                               ambiguities
                               2: Heading, pitch (roll = 0), aux antenna positions obtained with fixed
                               ambiguities
                               3: Heading, pitch, roll, aux antenna positions obtained with float ambiguities
                               4: Heading, pitch, roll, aux antenna positions obtained with fixed ambiguities */
    std::uint16_t reserved; /**< Reserved for future use, to be ignored by decoding software */

    float heading;          /**< Heading */
    float pitch;            /**< Pitch */
    float roll;             /**< Roll */
    float pitch_dot;        /**< Rate of change of the pitch angle */
    float roll_dot;         /**< Rate of change of the roll angle */
    float heading_dot;      /**< Rate of change of the heading angle */
} sbf_payload_att_euler;

typedef struct {
    std::uint8_t reserved;       /**< Reserved for future use, to be ignored by decoding software */

    std::uint8_t error_aux1 : 2; /**< Bits 0-1: Error code for Main-Aux1 baseline:
                                    0: No error
                                    1: Not enough measurements
                                    2: Reserved
                                    3: Reserved */
    std::uint8_t error_aux2 : 2; /**< Bits 2-3: Error code for Main-Aux2 baseline, same definition
                                    as bit 0-1. */
    std::uint8_t error_reserved : 3;      /**< Bits 4-6: Reserved */
    std::uint8_t error_not_requested : 1; /**< Bit 7: Set when GNSS-based attitude not requested by
                                             user. In that case, the other bits are all zero. */

    float cov_headhead;                   /**< Variance of the heading estimate */
    float cov_pitchpitch;                 /**< Variance of the pitch estimate */
    float cov_rollroll;                   /**< Variance of the roll estimate */
    float cov_headpitch;                  /**< Covariance between Euler angle estimates.
                                               Future functionality. The values are currently set to their Do-Not-Use
                                             values. */
    float cov_headroll;                   /**< Covariance between Euler angle estimates.
                                               Future functionality. The values are currently set to their Do-Not-Use
                                             values. */
    float cov_pitchroll;                  /**< Covariance between Euler angle estimates.
                                               Future functionality. The values are currently set to their Do-Not-Use
                                             values. */
} sbf_payload_att_cov_euler;

/* General message and payload buffer union */

typedef struct {
    std::uint16_t
        sync; /** The Sync field is a 2-byte array always set to 0x24, 0x40. The first byte of every
             SBF block has hexadecimal value 24 (decimal 36, ASCII '$'). The second byte of every
             SBF block has hexadecimal value 40 (decimal 64, ASCII '@'). */
    std::uint16_t
        crc16; /** The CRC field is the 16-bit CRC of all the bytes in an SBF block from and
           including the ID field to the last byte of the block. The generator polynomial for this
           CRC is the so-called CRC-CCITT polynomial: x 16 + x 12 + x 5 + x 0 . The CRC is computed
           in the forward direction using a seed of 0, no reverse and no final XOR. */
    std::uint16_t msg_id : 13; /** The ID field is a 2-byte block ID, which uniquely identifies the
                                  block type and its contents */
    std::uint8_t msg_revision : 3; /** block revision number, starting from 0 at the initial block
                                  definition, and incrementing each time backwards - compatible
                                  changes are performed to the block  */
    std::uint16_t length; /** The Length field is a 2-byte unsigned integer containing the size of
                         the SBF block. It is the total number of bytes in the SBF block including
                         the header. It is always a multiple of 4. */
    std::uint32_t TOW;    /**< Time-Of-Week: Time-tag, expressed in whole milliseconds from
                          the beginning of the current Galileo/GPSweek. */
    std::uint16_t WNc;    /**< The GPS week number associated with the TOW. WNc is a continuous
                          weekcount (hence the "c"). It is not affected by GPS week roll overs,
                          which occur every 1024 weeks. By definition of the Galileo system time,
                          WNc is also the Galileo week number + 1024. */
    union {
        sbf_payload_pvt_geodetic_t     payload_pvt_geodetic;
        sbf_payload_vel_cov_geodetic_t payload_vel_col_geodetic;
        sbf_payload_dop_t              payload_dop;
        sbf_payload_att_euler          payload_att_euler;
        sbf_payload_att_cov_euler      payload_att_cov_euler;
    };

    std::uint8_t padding[16];
} sbf_buf_t;
#pragma pack(pop)

/* Decoder state */
typedef enum {
    SBF_DECODE_SYNC1 = 0,
    SBF_DECODE_SYNC2,
    SBF_DECODE_PAYLOAD,
    SBF_DECODE_RTCM3
} sbf_decode_state_t;

class SBF_parse {
public:
    SBF_parse() = default;

    virtual ~SBF_parse();

    /**
     * @brief Open the given serial port and start the receiver worker on the serial_port
     */
    int start_receive(const std::string serial_port);

    /**
     * @brief Return the current SBF data
     */
    SBF_Information get_sbf_data();

    /**
     * @brief Return if the parser is running
     */
    bool is_running() const { return this->receive_running; };

private:
    /**
     * @brief Add payload rx byte
     */
    int payloadRxAdd(const std::uint8_t b);

    /**
     * @brief Parses incoming SBF blocks
     */
    int payloadRxDone(void);

    /**
     * @brief Reset the parse state machine for a fresh start
     */
    void decodeInit(void);

    /**
     * @brief Parse the binary SBF packet
     */
    int parseChar(const std::uint8_t b);

    /**
     * @brief Worker that read data from the receiver and call the parseChar function
     */
    int receive_worker();

    /**
     * @brief Set the sbf info
     */
    void set_sbf_data(SBF_Information SBF_data);

    std::uint8_t       _msg_status{0};
    sbf_decode_state_t _decode_state{SBF_DECODE_SYNC1};

    std::uint8_t  _dynamic_model{7};
    uint64_t      _last_timestamp_time{0};
    std::uint16_t _rx_payload_index{0};

    sbf_buf_t _buf;
    int       _serialFd;
    bool      receive_running{false};

    SBF_Information current_sbf_info;
    std::thread     receive_thread;
    std::mutex      _sbf_data_mutex;
};

std::uint16_t crc16(const std::uint8_t* buf, std::uint32_t len);