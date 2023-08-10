#include "location_information.h"
#include <modem.h>
#include <sbf.hpp>
#include "lpp/location_information.h"
#include "utility/types.h"
#include <cmath>

bool provide_location_information_callback(LocationInformation& location, HaGnssMetrics& metrics,
                                           void* userdata) {
    auto sbfParse = reinterpret_cast<SBF_parse*>(userdata);

    // check if the parser is currently running
    if (sbfParse && sbfParse->is_running()) {
        // get last value from parser
        auto receiver_info = sbfParse->get_sbf_data();

        // fill the struct with the parser value
        location.time                      = time(NULL);
        location.latitude                  = receiver_info.latitude;
        location.longitude                 = receiver_info.longitude;
        location.altitude                  = receiver_info.altitude;
        location.bearing                   = receiver_info.heading;
        location.horizontal_accuracy       = receiver_info.horizontal_accuracy;
        location.horizontal_speed          = sqrt(pow(receiver_info.ve,2) + pow(receiver_info.vn,2));
        location.horizontal_speed_accuracy = receiver_info.cov_vn_vn + receiver_info.cov_ve_ve;
        location.vertical_accuracy         = receiver_info.vertical_accuracy;
        location.vertical_speed            = receiver_info.vu;
        location.vertical_speed_accuracy   = receiver_info.cov_vu_vu;

        if (receiver_info.vu >= 0)
            location.vertical_velocity_direction = VerticalDirection::UP;
        else
            location.vertical_velocity_direction = VerticalDirection::DOWN;

        metrics.sats = receiver_info.nr_sv;
        metrics.age  = 0;
        metrics.hdop = receiver_info.hDOP;
        metrics.pdop = receiver_info.pDOP;

        switch (receiver_info.fixq) {
        case SBF_FIX::INVALID:
        default: metrics.fixq = FixQuality::INVALID; break;

        case SBF_FIX::DGPS_FIX: metrics.fixq = FixQuality::DGPS_FIX; break;

        case SBF_FIX::STANDALONE: metrics.fixq = FixQuality::STANDALONE; break;

        case SBF_FIX::RTK_FIX: metrics.fixq = FixQuality::RTK_FIX; break;

        case SBF_FIX::RTK_FLOAT: metrics.fixq = FixQuality::RTK_FLOAT; break;
        }

        return true;
    } else {
        return false;
    }
}

bool provide_ecid_callback(ECIDInformation& ecid, void* userdata) {
    auto modem = reinterpret_cast<Modem_AT*>(userdata);
    if (!modem) return false;

    auto neighbors = modem->neighbor_cells();
    auto cell      = modem->cell();
    if (!cell.initialized()) return false;

    ecid.cell           = cell.value();
    ecid.neighbor_count = 0;

    for (auto& neighbor_cell : neighbors) {
        if (ecid.neighbor_count < 16) {
            ecid.neighbors[ecid.neighbor_count++] = {
                .id     = neighbor_cell.id,
                .earfcn = neighbor_cell.EARFCN,
                .rsrp   = neighbor_cell.rsrp,
                .rsrq   = neighbor_cell.rsrq,
            };
        }
    }

    return true;
}