#include "ViconRecorder.h"

#include <iomanip>
#include <sstream>

namespace {

constexpr int kViconFormat = 1;
// Fixed-point, not exact round-trip: 4 decimal places is 0.1 mm for a
// metre-valued position, well inside Vicon's own measurement noise, and
// far more readable than a 17-significant-digit double. See the
// "Precision policy" note above Task 4 Step 1 for the full rationale.
constexpr int kDecimalPlaces = 4;

std::string QuaternionField(double value) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(kDecimalPlaces) << value;
    return oss.str();
}

void WriteEntityRow(std::ostream& out, unsigned int frame_number,
                     const std::string& kind, const std::string& subject,
                     const std::string& name, double x, double y, double z,
                     const std::string& qx, const std::string& qy,
                     const std::string& qz, const std::string& qw, bool valid,
                     const std::string& invalid_reason) {
    out << frame_number << "," << kind << "," << subject << "," << name << ","
        << x << "," << y << "," << z << "," << qx << "," << qy << "," << qz
        << "," << qw << "," << (valid ? 1 : 0) << "," << invalid_reason << "\n";
}

}  // namespace

ViconRecorder::ViconRecorder(std::ostream& frames_csv, std::ostream& entities_csv,
                              std::string host, std::string subject)
    : frames_csv_(frames_csv), entities_csv_(entities_csv),
      host_(std::move(host)), subject_(std::move(subject)) {
    frames_csv_ << std::fixed << std::setprecision(kDecimalPlaces);
    entities_csv_ << std::fixed << std::setprecision(kDecimalPlaces);
}

void ViconRecorder::WriteHeader() {
    frames_csv_ << "# vicon controller/planner recording -- parsers skip '#' lines\n";
    frames_csv_ << "# vicon_format = " << kViconFormat << "\n";
    frames_csv_ << "# host = " << host_ << "\n";
    frames_csv_ << "# subject = " << subject_ << "\n";
    frames_csv_ << "frame_number,host_time_s,frame_rate_hz,latency_total_s\n";

    entities_csv_ << "# vicon_format = " << kViconFormat << "\n";
    entities_csv_
        << "frame_number,kind,subject,name,x_m,y_m,z_m,qx,qy,qz,qw,valid,invalid_reason\n";
}

void ViconRecorder::Write(const ViconSnapshot& snapshot) {
    frames_csv_ << snapshot.frame_number << "," << snapshot.host_time_s << ","
                << snapshot.frame_rate_hz << "," << snapshot.latency_total_s
                << "\n";

    for (const auto& marker : snapshot.markers) {
        WriteEntityRow(entities_csv_, snapshot.frame_number, "marker", "",
                       marker.name, marker.position_m.x(), marker.position_m.y(),
                       marker.position_m.z(), "", "", "", "", marker.valid,
                       marker.invalid_reason);
    }
    for (const auto& segment : snapshot.segments) {
        WriteEntityRow(
            entities_csv_, snapshot.frame_number, "segment", segment.subject_name,
            segment.segment_name, segment.position_m.x(), segment.position_m.y(),
            segment.position_m.z(), QuaternionField(segment.orientation.x()),
            QuaternionField(segment.orientation.y()),
            QuaternionField(segment.orientation.z()),
            QuaternionField(segment.orientation.w()), segment.valid,
            segment.invalid_reason);
    }
}
