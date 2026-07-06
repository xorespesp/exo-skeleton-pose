#include "cli_options.hh"

namespace app
{
    void add_source_options(CLI::App& app, source_options& o)
    {
        app.add_option("-d,--device", o.device_index, "Device index to open")->default_val(0);
        app.add_option("-i,--input", o.input_path, "MKV recording file path to open");
        app.add_option("-s,--tag-size", o.tag_size_m, "AprilTag black-square edge length [m]")->default_val(0.05);
        app.add_option("-e,--exposure-us", o.exposure_us, "Manual color exposure [us] (default: auto)");
        app.add_option("-g,--gain", o.gain, "Manual color gain (default: auto)");
    }

} // namespace app
