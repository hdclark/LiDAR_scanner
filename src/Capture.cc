//Capture.cc -- Written by hal clark, 2020.

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <array>

#include <functional>
#include <algorithm>

#include <thread>
#include <mutex>
#include <chrono>

#include <librealsense2/rs.hpp>

#include <YgorMisc.h>
#include <YgorMath.h>

struct pc_datum {
    double t = 0;
    std::array<uint8_t, 3> c; // r, g, b.
    uint8_t i = 0; // infrared intensity.
    vec3<float> v; // x, y, z.
};

int main(int argc, char **argv){

    // Point cloud data.
    constexpr uint64_t mem_buffer_size = 2'000'000'000UL; // 2 GB.
    constexpr uint64_t max_datum_count = mem_buffer_size / sizeof(pc_datum);
    //std::vector<pc_datum> pc_bulk(max_datum_count, pc_datum());
    std::vector<pc_datum> pc_bulk;
    pc_bulk.reserve(max_datum_count);
    std::mutex pc_bulk_m;

    // Positioning / transform data.
    std::mutex transform_m;

    std::mutex frame_number_m;
    long int frame_number = 0;

    try{
        //rs2::log_to_console(RS2_LOG_SEVERITY_DEBUG);
        //rs2::log_to_console(RS2_LOG_SEVERITY_INFO);
        rs2::log_to_console(RS2_LOG_SEVERITY_WARN);
        //rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);

        auto frame_handler = [&](const rs2::frame& mframe) -> void {

            // If this is a set of synchronized frames, extract a point cloud.
            if(rs2::frameset sframes = mframe.as<rs2::frameset>()){

                // Discard the first frames to allow auto-exposure to stabilize.
                {
                    std::lock_guard<std::mutex> l(frame_number_m);
                    ++frame_number;
                    if(frame_number < 30) return;
                }

                auto colour = sframes.get_color_frame();
                if(!colour) colour = sframes.get_infrared_frame();
                if(!colour) FUNCERR("Unable to get synchronized colour frame");

                auto infra = sframes.get_infrared_frame();
                if(!infra) infra = sframes.get_color_frame();

                auto depth = sframes.get_depth_frame();
                if(!depth) FUNCERR("Unable to get synchronized depth frame");

                //long int frame_time = 0;
                //if(false){
                ////}else if(depth.supports_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP)){
                ////    frame_time = depth.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP);
                ////}else if(depth.supports_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP)){
                ////    frame_time = depth.get_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP);
                //}else if(mframe.supports_frame_metadata(RS2_FRAME_METADATA_BACKEND_TIMESTAMP)){
                //    frame_time = mframe.get_frame_metadata(RS2_FRAME_METADATA_BACKEND_TIMESTAMP);
                ////}else if(mframe.supports_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL)){
                ////    frame_time = mframe.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL);
                //}else{
                //    FUNCERR("Unable to get synchronized frame's time");
                //}
                const auto frame_time = depth.get_timestamp();

                //// Get a copy of sensor position/orientation transformation at relevant time.
                //{
                //    std::lock_guard<std::mutex> l(m);
                //    ...
                //}

                rs2::pointcloud pc;
                //pc.map_to(colour);
                pc.map_to(infra);
                rs2::points ps = pc.calculate(depth);
                if(!ps) FUNCERR("Unable to get texture-mapped point set from synchronized frame");

                auto N_verts = ps.size();
                auto verts = ps.get_vertices();
                auto tex_coords = ps.get_texture_coordinates();

                const uint8_t* c_texture_data = reinterpret_cast<const uint8_t*>(colour.get_data());
                const int32_t c_tex_w = colour.get_width();
                const int32_t c_tex_h = colour.get_height();
                const int32_t c_tex_bpp = colour.get_bytes_per_pixel();
                const int32_t c_tex_stride = colour.get_stride_in_bytes();

                const uint8_t* i_texture_data = reinterpret_cast<const uint8_t*>(infra.get_data());
                const int32_t i_tex_w = infra.get_width();
                const int32_t i_tex_h = infra.get_height();
                const int32_t i_tex_bpp = infra.get_bytes_per_pixel();
                const int32_t i_tex_stride = infra.get_stride_in_bytes();

        std::lock_guard<std::mutex> l(pc_bulk_m);
                if(max_datum_count <= (pc_bulk.size() + N_verts)){
                    FUNCWARN("Buffer filled, unable to collect more data");
                    return;
                }
                const float min_distance = 1.0E-6;
                for(long int i = 0; i < N_verts; ++i){
                    if( (min_distance <= std::abs(verts[i].x))
                    ||  (min_distance <= std::abs(verts[i].y))
                    ||  (min_distance <= std::abs(verts[i].z)) ){
                        // Note: negative here as per librealsense conventions. See librealsense2's 'export_to_ply()'.
                        const vec3<float> v(verts[i].x, -verts[i].y, -verts[i].z);

                        // Query the colour frame to get the proper colour.
                        const int32_t c_x = std::min( std::max(static_cast<int32_t>(tex_coords[i].u * c_tex_w + 0.5f), 0), c_tex_w - 1);
                        const int32_t c_y = std::min( std::max(static_cast<int32_t>(tex_coords[i].v * c_tex_h + 0.5f), 0), c_tex_h - 1);
                        const int32_t c_index = c_x * c_tex_bpp + c_y * c_tex_stride;

                        const int32_t i_x = std::min( std::max(static_cast<int32_t>(tex_coords[i].u * i_tex_w + 0.5f), 0), i_tex_w - 1);
                        const int32_t i_y = std::min( std::max(static_cast<int32_t>(tex_coords[i].v * i_tex_h + 0.5f), 0), i_tex_h - 1);
                        const int32_t i_index = i_x * i_tex_bpp + i_y * i_tex_stride;


                        //// Apply a copy of the current transformation.
                        // ...

                        // Store a copy of the point cloud data somewhere.
                        // ...
                        //FO_pc << frame_time 
                        //      << " " << v.x << " " << v.y << " " << v.z
                        //      << " " << tex.x << " " << tex.y 
                        //      << "\n"; // Let OS manage write flushing.
                        pc_bulk.emplace_back();
                        pc_bulk.back().t = frame_time;
                        pc_bulk.back().c = { c_texture_data[c_index + 0],
                                             c_texture_data[c_index + 1],
                                             c_texture_data[c_index + 2] };
                        pc_bulk.back().i = i_texture_data[i_index];
                        pc_bulk.back().v = v;
                    }
                }

            // Otherwise, this is an unsynchronized frame.
            }else{
                // Check which sensor this is and update the sensor position/orientation transformation at the moment of
                // this frame.
                //long int frame_time = 0;
                //if(false){
                ////}else if(mframe.supports_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP)){
                ////    frame_time = mframe.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP);
                ////}else if(mframe.supports_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP)){
                ////    frame_time = mframe.get_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP);
                //}else if(mframe.supports_frame_metadata(RS2_FRAME_METADATA_BACKEND_TIMESTAMP)){
                //    frame_time = mframe.get_frame_metadata(RS2_FRAME_METADATA_BACKEND_TIMESTAMP);
                ////}else if(mframe.supports_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL)){
                ////    frame_time = mframe.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL);
                //}else{
                //    FUNCERR("Unable to get un-synchronized frame's time");
                //}
                const auto frame_time = mframe.get_timestamp();

                // mframe ...

                //// Get a copy of sensor position/orientation transformation at relevant time.
                //{
                //    std::lock_guard<std::mutex> l(m);
                //    ...
                //}

        std::lock_guard<std::mutex> l(transform_m);
                // Do something here...
            }

            return;
        };


        // The default profile provides a recommended set of data streams.
        //
        // For the L515, this includes depth, image (colour and infrared), and IMU (accelerometer and gyroscope) at
        // recommended frame rates and frequencies.
        //
        // NOTE: Reduced-quality settings may be needed to avoid overloading the system over long periods.
        rs2::pipeline pl;
        rs2::config conf;
        rs2::pipeline_profile profiles = pl.start(conf, frame_handler);


        // Collect frames for a preset time and then terminate.
        //
        // NOTE: Instead, periodically check shutter button state.
        FUNCINFO("Collecting frames now..");
        for(auto i = 0; i < 10; ++i){
            std::this_thread::sleep_for(std::chrono::seconds(1));

            std::lock_guard<std::mutex> l(pc_bulk_m);
            std::cout << "  Filled "
                      << 0.1 * static_cast<double>(1000 * (1UL + pc_bulk.size()) / max_datum_count)
                      << "\% of buffer ("
                      << 0.1 * static_cast<double>(10 * sizeof(pc_datum) * pc_bulk.size() / 1'000'000'000)
                      << " GB)..." << std::endl;
        }

        FUNCINFO("Done collecting frames");
        pl.stop();


        
        // Transform the data and write it to disk.
        //
        // Note: Not sure if stop() clears queue, or how to reap any queued callbacks.
        //       The easiest solution is to claim all locks before processing anything.
        FUNCINFO("Transforming data and writing to disk..");
        {
            std::lock_guard<std::mutex> pc_bulk_l(pc_bulk_m);
            std::lock_guard<std::mutex> transform_l(transform_m);

            std::map<double, uint64_t> points_at_time;
            for(const auto &d : pc_bulk) ++points_at_time[d.t];

            std::set<double> time_has_been_processed;
            std::ofstream os;

            size_t num_written = 0;
            double notification_freq = 0.05;
            size_t num_notify = static_cast<size_t>(notification_freq * pc_bulk.size());
            for(const auto &d : pc_bulk){
                if(time_has_been_processed.count(d.t) == 0){
                    time_has_been_processed.insert(d.t);
                    FUNCINFO("Processing timestamp " << static_cast<uint64_t>(d.t));

                    if(os){
                        os.flush();
                        os.close();
                    }

                    const auto fname = std::string("pointcloud_") + std::to_string( static_cast<uint64_t>(d.t) ) + ".ply";
                    os.open(fname);
                    os << "ply" << std::endl
                       << "format ascii 1.0" << std::endl
		       << "comment metadata time = " << static_cast<uint64_t>(d.t) << std::endl
                       << "element vertex " << points_at_time[d.t] << std::endl
                       << "property float x" << std::endl
                       << "property float y" << std::endl
                       << "property float z" << std::endl
                       << "property uchar red" << std::endl
                       << "property uchar green" << std::endl
                       << "property uchar blue" << std::endl
                       << "property uchar intensity" << std::endl
                       << "end_header" << std::endl;
                }

                // Transform v here (TODO).
                // v' = f(v, d.t);
                os << d.v.x << " " << d.v.y << " " << d.v.z
                   << " " << static_cast<int>(d.c[0]) 
                   << " " << static_cast<int>(d.c[1])
                   << " " << static_cast<int>(d.c[2])
		   << " " << static_cast<int>(d.i)
                   << "\n"; // Let OS manage write flushing at the end.

                ++num_written;
                if( (num_written % num_notify) == 0 ){
                    std::cout << "  "
                              << 0.1 * static_cast<double>(1000 * (2UL + num_written) / pc_bulk.size())
                              << "\% of buffer has been written..."
                              << std::endl;
                }
            }
            os.flush();

            //std::ofstream FO_tr("transformation.txt");
            //FO_tr << frame_time
            //      //<< " " << mframe.stream_name()
            //      << "\n"; // Let OS manage write flushing.
        }

    }catch(const std::exception& e){
        FUNCERR("Encountered error: '" << e.what() << "'");
    }

    FUNCINFO("Done");
    return 0;
}

